#include "ADCMeasure.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#define TAG "ADCMeasure"

uint16_t raw_voltage[4] = {0}; // [board_id]
uint16_t raw_current[4] = {0}; // [board_id]

ADCMeasure::ADCMeasure(float vref, spi_host_device_t spi_host)
    : adc(vref, spi_host, CS_PIN_1, CS_PIN_2, CS_PIN_3, CS_PIN_4, RESET_PIN_1,
          RESET_PIN_2, RESET_PIN_3, RESET_PIN_4) {
  adcTaskHandle = nullptr;
  // Khởi tạo offset
  for (int i = 0; i < 4; i++) {
    currentOffset[i] = 0;
  }
  // Khởi tạo NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "Erasing NVS partition...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  for (int i = 0; i < 4; i++) {
    currentSlope[i] = 0.0001; // Giá trị mặc định nhỏ để có tín hiệu ban đầu
    voltageValueScale[i] = 24.0f; // Mặc định điểm scale là 24V
  }
}

esp_err_t ADCMeasure::init(uint8_t board_id, gpio_num_t cs_pin,
                           gpio_num_t reset_pin) {
  esp_err_t ret = ESP_OK;

  // Reset và khởi tạo các kênh ADC
  switch (board_id) {
  case BOARD_1:
    adc.resetHard(AD770X::CS_1);
    adc.reset(AD770X::CS_1);
    adc.init(AD770X::CHN_AIN1, AD770X::CS_1);
    adc.init(AD770X::CHN_AIN2, AD770X::CS_1);
    break;
  case BOARD_2:
    adc.resetHard(AD770X::CS_2);
    adc.reset(AD770X::CS_2);
    adc.init(AD770X::CHN_AIN1, AD770X::CS_2);
    adc.init(AD770X::CHN_AIN2, AD770X::CS_2);
    break;
  case BOARD_3:
    adc.resetHard(AD770X::CS_3);
    adc.reset(AD770X::CS_3);
    adc.init(AD770X::CHN_AIN1, AD770X::CS_3);
    adc.init(AD770X::CHN_AIN2, AD770X::CS_3);
    break;
  case BOARD_4:
    adc.resetHard(AD770X::CS_4);
    adc.reset(AD770X::CS_4);
    adc.init(AD770X::CHN_AIN1, AD770X::CS_4);
    adc.init(AD770X::CHN_AIN2, AD770X::CS_4);
    break;
  default:
    ESP_LOGE("ADCMeasure", "Invalid board ID: %d", board_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Cấu hình bộ lọc Kalman
  setupKalmanFilter();

  ret = loadVoltageCalibFromFlash(); // Hàm này trong code đã bao gồm cả
                                     // load Offset và Slope
  if (ret != ESP_OK) {
    ESP_LOGW(
        TAG,
        "Failed to load currentOffset from flash for Board %d, using default",
        board_id);
  }
  return ESP_OK;
}

void ADCMeasure::setupKalmanFilter() {
  // Ma trận chuyển trạng thái (giả sử hệ thống tuyến tính đơn giản)
  A[0] = 1.0;
  A[1] = 0.0;
  A[2] = 0.0;
  A[3] = 1.0;

  // Ma trận hiệp phương sai ban đầu
  P[0] = 0.1;
  P[1] = 0.0;
  P[2] = 0.0;
  P[3] = 0.1;

  // Nhiễu quá trình
  Q[0] = 0.01;
  Q[1] = 0.0;
  Q[2] = 0.0;
  Q[3] = 0.01;

  // Ma trận quan sát
  H[0] = 1.0;
  H[1] = 0.0;

  // Nhiễu quan sát
  R[0] = 0.1;

  // Khởi tạo trạng thái
  xp[0] = 0.0;
  xp[1] = 0.0;
  xc[0] = 0.0;
  xc[1] = 0.0;

  // Ma trận chuyển trạng thái (1 state, không có velocity)
  A_current[0] = 1.0; // State không đổi
  A_current[1] = 0.0;
  A_current[2] = 0.0;
  A_current[3] = 1.0;

  // Ma trận hiệp phương sai ban đầu - TĂNG LÊN để khởi tạo tốt hơn
  P_current[0] = 1.0; // Tăng từ 0.1 → 1.0
  P_current[1] = 0.0;
  P_current[2] = 0.0;
  P_current[3] = 1.0;

  // Nhiễu quá trình - GIẢM XUỐNG để ít thay đổi
  Q_current[0] = 0.001; // Giảm từ 0.01 → 0.001
  Q_current[1] = 0.0;
  Q_current[2] = 0.0;
  Q_current[3] = 0.001;

  // Ma trận quan sát
  H_current[0] = 1.0;
  H_current[1] = 0.0;

  // Nhiễu quan sát - TĂNG LÊN để filter mạnh hơn với nhiễu
  R_current[0] = 0.5; // Tăng từ 0.1 → 0.5 (filter mạnh hơn)

  // Khởi tạo trạng thái
  xp_current[0] = 0.0;
  xp_current[1] = 0.0;
  xc_current[0] = 0.0;
  xc_current[1] = 0.0;

  // Khởi tạo bộ lọc Kalman
  voltageFilter.init(1, 1, A, P, Q, H, R, xp, xc);
  currentFilter.init(1, 1, A_current, P_current, Q_current, H_current,
                     R_current, xp_current, xc_current);
}

esp_err_t ADCMeasure::startADCTask() {
  // Tạo task FreeRTOS để quét ADC
  BaseType_t result = xTaskCreate(adcScanTask, "ADC_Scan_Task",
                                  4096, // Stack size
                                  this, // Tham số truyền vào task
                                  5,    // Độ ưu tiên
                                  &adcTaskHandle);

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create ADC scan task");
    return ESP_FAIL;
  }
  return ESP_OK;
}

void ADCMeasure::adcScanTask(void *arg) {
  ADCMeasure *adcMeasure = static_cast<ADCMeasure *>(arg);
  uint8_t countBoardCalib[4] = {0, 0, 0, 0};
  uint32_t sumOffset[4] = {0, 0, 0, 0};
  bool boardCalibrated[4] = {false, false, false, false};
  bool lastCalibState = adcMeasure->isBoardCalibDone;
  bool wasCalibrating = false;
  uint8_t countCurrent[4] = {0, 0, 0, 0};
  uint8_t countVoltage[4] = {0, 0, 0, 0};
  uint32_t sumCurrent[4] = {0, 0, 0, 0};
  uint32_t sumVoltage[4] = {0, 0, 0, 0};
  uint16_t calibTimeout = 0; // Biến đếm thời gian chờ Calib
  while (1) {
    if (adcMeasure->isCalibratingZero && !wasCalibrating) {
      ESP_LOGW(TAG, "DANG RESET DU LIEU DE CALIB ZERO 10 MAU ...");
      for (int i = 0; i < 4; i++) {
        countCurrent[i] = 0;
        countVoltage[i] = 0;
        sumCurrent[i] = 0;
        sumVoltage[i] = 0;
        boardCalibrated[i] = false;
      }
      wasCalibrating = true;
      calibTimeout = 200;
    }

    for (int board = 0; board < 4; board++) {
      // 1. Đọc Dòng điện (AIN1)
      if (adcMeasure->adc.dataReady(AD770X::CHN_AIN1,
                                    static_cast<AD770X::CS_t>(board))) {
        uint16_t rawI = adcMeasure->adc.readADResultRaw(
            AD770X::CHN_AIN1, static_cast<AD770X::CS_t>(board));
        // adcMeasure->setCurrentOffset(BOARD_4, 0); // 38720 - 38594 -
        // adcMeasure->saveCurrentOffsetToFlash();
        raw_current[board] = rawI;
        ESP_LOGI(TAG, "\nraw_current[%d] = %d", board, raw_current[board]);

        if (adcMeasure->isCalibratingZero && countCurrent[board] < 10) {
          if (rawI != 0 && rawI != 65535) {
            sumCurrent[board] += rawI;
            countCurrent[board]++;
          }
        }

        // Cập nhật giá trị hiển thị (dùng offset mới nhất)
        float currentVal = adcMeasure->convertToCurrent(board, rawI);
        adcMeasure->setCurrent(board, currentVal);
      }

      // 2. Đọc Điện áp (AIN2)
      if (adcMeasure->adc.dataReady(AD770X::CHN_AIN2,
                                    static_cast<AD770X::CS_t>(board))) {
        uint16_t rawV = adcMeasure->adc.readADResultRaw(
            AD770X::CHN_AIN2, static_cast<AD770X::CS_t>(board));
        raw_voltage[board] = rawV;

        if (adcMeasure->isCalibratingZero && countVoltage[board] < 10) {
          if (rawV != 0 && rawV != 65535) {
            sumVoltage[board] += rawV;
            countVoltage[board]++;
          }
        }
        // float voltageVal = adcMeasure->convertToVoltage(board, rawV);
        adcMeasure->setVoltage(board,
                               adcMeasure->convertToVoltage(board, rawV));
        ESP_LOGI(TAG, "adcScanTask - Board %d Raw_Voltage: %d", board,
                 raw_voltage[board]);
      }

      // --- 4. KIỂM TRA HOÀN THÀNH CHO RIÊNG BOARD NÀY ---
      if (adcMeasure->isCalibratingZero && !boardCalibrated[board]) {
        if (countCurrent[board] >= 10 && countVoltage[board] >= 10) {

          // Tính trung bình
          adcMeasure->currentRaw0[board] = sumCurrent[board] / 10;
          adcMeasure->currentVal0[board] = 0.0f;
          adcMeasure->currentOffset[board] =
              adcMeasure->currentRaw0[board]; // Cập nhật offset

          adcMeasure->voltageRaw0[board] = sumVoltage[board] / 10;

          // Cập nhật lại hệ số
          adcMeasure->calculateCurrentSlope(board);

          boardCalibrated[board] = true;
          ESP_LOGI(TAG, "Board %d: Calib Zero xong. V0: %d, I0: %d", board,
                   adcMeasure->voltageRaw0[board],
                   adcMeasure->currentRaw0[board]);

          // LƯU FLASH NGAY LẬP TỨC CHO MODULE NÀY (Không chờ module khác)
          ESP_LOGW(TAG, "BOARD %d DA LAY MAU XONG -> LUU FLASH...", board);
          adcMeasure->saveCurrentOffsetToFlash();
          ESP_LOGI(
              TAG,
              "\n\n\n*********************HERE***********************\n\n\n");
        }
      }
    }
    if (wasCalibrating) {
      if (calibTimeout > 0) {
        calibTimeout--;
      } else {
        // Zero
        adcMeasure->isCalibratingZero = false;
        wasCalibrating = false;
        adcMeasure->isBoardCalibDone = true;
        ESP_LOGW(TAG,
                 "KET THUC PHIEN CALIB ZERO. San sang cho lenh tiep theo.");
        gpio_set_level(LED_CALIB_STATUS, 0);
      }
    }
    // adcMeasure->printFlashData();
    static int print_count = 0;
    if (++print_count >= 10) {
      adcMeasure->printFlashData();
      print_count = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(25)); // Delay ngắn để không chiếm dụng CPU
  }
}

void ADCMeasure::printFlashData() {
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("adc_storage", NVS_READONLY, &nvs_handle);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Khong the mo Flash de doc: %s", esp_err_to_name(ret));
    return;
  }

  ESP_LOGW(TAG, "========== DU LIEU CALIB TRONG FLASH ==========");
  for (int i = 0; i < 4; i++) {
    char k0[15], kS[15], kV[15];
    char kRaw0[15], kVal0[15], kRawScale[15], kValScale[15];

    // CHÚ Ý: Đặt tên Key y hệt như trong hàm LƯU (saveCurrentOffsetToFlash)
    snprintf(k0, 15, "v0_b%d", i);
    snprintf(kS, 15, "vs_b%d", i);
    snprintf(kV, 15, "vv_b%d", i);

    snprintf(kRaw0, 15, "iraw0_b%d", i);
    snprintf(kVal0, 15, "ival0_b%d", i);
    snprintf(kRawScale, 15, "irawScale_b%d", i);
    snprintf(kValScale, 15, "ivalScale_b%d", i);

    uint16_t v_r0 = 0, v_rS = 0;
    float v_vS = 0.0f;
    uint32_t v_bits = 0;

    uint16_t i_r0 = 0, i_rS = 0;
    float i_v0 = 0.0f, i_vS = 0.0f;
    uint32_t iv0_bits = 0, ivS_bits = 0;

    // --- Đọc Áp ---
    nvs_get_u16(nvs_handle, k0, &v_r0);
    nvs_get_u16(nvs_handle, kS, &v_rS);
    if (nvs_get_u32(nvs_handle, kV, &v_bits) == ESP_OK) {
      memcpy(&v_vS, &v_bits, sizeof(float));
    }

    // --- Đọc Dòng ---
    nvs_get_u16(nvs_handle, kRaw0, &i_r0);
    nvs_get_u16(nvs_handle, kRawScale, &i_rS);

    if (nvs_get_u32(nvs_handle, kVal0, &iv0_bits) == ESP_OK) {
      memcpy(&i_v0, &iv0_bits, sizeof(float));
    }
    if (nvs_get_u32(nvs_handle, kValScale, &ivS_bits) == ESP_OK) {
      memcpy(&i_vS, &ivS_bits, sizeof(float));
    }

    // --- IN RA LOG ---
    ESP_LOGI(TAG, "[BOARD %d]", i);
    ESP_LOGI(TAG,
             "  + Voltage : Raw0 = %5d | RawScale = %5d | ValScale = %.2f V",
             v_r0, v_rS, v_vS);
    ESP_LOGI(TAG,
             "  + Current : Raw0 = %5d | Val0 = %.2f A | RawScale = %5d | "
             "ValScale = %.2f A",
             i_r0, i_v0, i_rS, i_vS);
  }
  ESP_LOGW(TAG, "===============================================");

  nvs_close(nvs_handle);
}

esp_err_t ADCMeasure::saveCurrentOffsetToFlash() {

  ESP_LOGI(TAG, "\n\n\n*********************||***********************\n\n\n");
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("adc_storage", NVS_READWRITE, &nvs_handle);
  if (ret != ESP_OK)
    return ret;

  for (int i = 0; i < 4; i++) {

    char k0[15], kS[15], kV[15];
    char kRaw0[15], kVal0[15], kRawScale[15], kValScale[15];
    snprintf(k0, 15, "v0_b%d", i);
    snprintf(kS, 15, "vs_b%d", i);
    snprintf(kV, 15, "vv_b%d", i); // Nhãn lưu giá trị 15.0 hoặc 24.0

    snprintf(kRaw0, 15, "iraw0_b%d", i);
    snprintf(kVal0, 15, "ival0_b%d", i);
    snprintf(kRawScale, 15, "irawScale_b%d", i);
    snprintf(kValScale, 15, "ivalScale_b%d", i);

    nvs_set_u16(nvs_handle, k0, voltageRaw0[i]);
    ESP_LOGI(TAG, "Save Voltage Raw0 for Board %d: %d", i, voltageRaw0[i]);
    nvs_set_u16(nvs_handle, kS, voltageRawScale[i]);
    ESP_LOGI(TAG, "Save Voltage Raw Scale for Board %d: %d", i,
             voltageRawScale[i]);
    uint32_t v_bits;
    memcpy(&v_bits, &voltageValueScale[i], sizeof(float));
    nvs_set_u32(nvs_handle, kV, v_bits);
    ESP_LOGI(TAG, "Save Voltage Val Scale for Board %d: %f", i,
             voltageValueScale[i]);
    // Lưu mảng Dòng điện (2 Mốc)
    nvs_set_u16(nvs_handle, kRaw0, currentRaw0[i]);
    ESP_LOGI(TAG, "Save Current Raw0 for Board %d: %d", i, currentRaw0[i]);
    nvs_set_u16(nvs_handle, kRawScale, currentRawScale[i]);
    ESP_LOGI(TAG, "Save Current Raw Scale for Board %d: %d", i,
             currentRawScale[i]);
    // Lưu Double bằng cách ép kiểu sang uint64_t (Bitwise copy)

    uint32_t iv1_bits, iv2_bits;
    memcpy(&iv1_bits, &currentVal0[i], sizeof(float));
    memcpy(&iv2_bits, &currentValueScale[i], sizeof(float));
    nvs_set_u32(nvs_handle, kVal0, iv1_bits);
    ESP_LOGI(TAG, "Save Current Val0 for Board %d: %f", i, currentVal0[i]);
    nvs_set_u32(nvs_handle, kValScale, iv2_bits);
    ESP_LOGI(TAG, "Save Current Val Scale for Board %d: %f", i,
             currentValueScale[i]);
  }
  ret = nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  return ret;
}

esp_err_t ADCMeasure::loadCurrentOffsetFromFlash() {
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("adc_storage", NVS_READONLY, &nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS handle: %d. Saving default offsets...",
             ret);
    // Nếu mở NVS lỗi, lưu giá trị mặc định (0) vào flash
    for (int board = BOARD_1; board <= BOARD_4; board++) {
      currentOffset[board] = 0;
    }
    saveCurrentOffsetToFlash();
    return ret;
  }
  ESP_LOGI(TAG, "loadCurrentOffsetFromFlash - Mo flash thanh cong");
  // Đọc currentOffset cho từng board
  char key[16];
  for (uint8_t board = BOARD_1; board <= BOARD_4; board++) {
    snprintf(key, sizeof(key), "offset_board_%d", board);
    uint16_t offset = 0;
    ret = nvs_get_u16(
        nvs_handle, key,
        &offset); // Read offset from flash (lưu giá trị đọc được vào offset)
    if (ret == ESP_OK) {
      currentOffset[board] = offset;
    } else {
      ESP_LOGW(TAG, "No offset found for Board %d, using default 0", board);
      currentOffset[board] = 0;
    }
  }

  nvs_close(nvs_handle);
  return ESP_OK;
}

// Calib 0V cho cả 4 mạch (Kịch bản dùng chung)
void ADCMeasure::captureVoltageRaw0() {
  for (int i = 0; i < 4; i++) {
    voltageRaw0[i] =
        raw_voltage[i]; // raw_voltage là mảng cập nhật từ task scan
  }

  saveCurrentOffsetToFlash();
  ESP_LOGI(TAG, "Calib 0V hoàn tất cho 4 mạch");
}

void ADCMeasure::captureVoltageRaw24() {
  for (int i = 0; i < 4; i++) {
    voltageRawScale[i] = raw_voltage[i];
  }
  saveVoltageCalibToFlash();
  ESP_LOGI(TAG, "Captured Raw 24V for all boards");
}

// Calib 24V cho cả 4 mạch (Kịch bản 1)
void ADCMeasure::captureVoltageRaw24_All() {
  for (int i = 0; i < 4; i++) {
    voltageRawScale[i] = raw_voltage[i];
  }
  saveVoltageCalibToFlash(); // Ghi mảng r24 vào NVS
  ESP_LOGI(TAG, "Calib 24V hoàn tất cho TẤT CẢ mạch");
}

// Calib 24V cho từng mạch đơn lẻ (Kịch bản 2)
void ADCMeasure::captureVoltageRaw24_Single(uint8_t board_idx) {
  if (board_idx < 4) {
    voltageRawScale[board_idx] = raw_voltage[board_idx];
    saveVoltageCalibToFlash();
    ESP_LOGI(TAG, "Calib 24V hoàn tất cho MẠCH %d", board_idx + 1);
  }
}

esp_err_t ADCMeasure::loadVoltageCalibFromFlash() {
  nvs_handle_t nvs_handle;
  // Mở NVS ở chế độ READONLY
  if (nvs_open("adc_storage", NVS_READONLY, &nvs_handle) != ESP_OK) {
    return ESP_FAIL;
  }

  for (int i = 0; i < 4; i++) {
    char k0[15], kS[15], kV[15];
    char kRaw0[15], kVal0[15], kRawScale[15], kValScale[15];

    snprintf(k0, 15, "v0_b%d", i);
    snprintf(kS, 15, "vs_b%d", i);
    snprintf(kV, 15, "vv_b%d", i);

    snprintf(kRaw0, 15, "iraw0_b%d", i);
    snprintf(kVal0, 15, "ival0_b%d", i);
    snprintf(kRawScale, 15, "irawScale_b%d", i);
    snprintf(kValScale, 15, "ivalScale_b%d", i);

    // 1. Đọc các giá trị uint16_t (Ok)
    nvs_get_u16(nvs_handle, k0, &voltageRaw0[i]);
    nvs_get_u16(nvs_handle, kS, &voltageRawScale[i]);

    // 2. Đọc Float (voltageValueScale) bằng uint32_t
    uint32_t v_bits;
    if (nvs_get_u32(nvs_handle, kV, &v_bits) == ESP_OK) {
      memcpy(&voltageValueScale[i], &v_bits, sizeof(float));
    } else {
      voltageValueScale[i] = 24.0f; // Giá trị mặc định nếu chưa có
    }

    // 3. Đọc Double (currentSlope) bằng uint64_t
    if (nvs_get_u16(nvs_handle, kRaw0, &currentRaw0[i]) != ESP_OK)
      currentRaw0[i] = 0;
    if (nvs_get_u16(nvs_handle, kRawScale, &currentRawScale[i]) != ESP_OK)
      currentRawScale[i] = 0;

    uint32_t iv1_bits, iv2_bits;
    if (nvs_get_u32(nvs_handle, kVal0, &iv1_bits) == ESP_OK) {
      memcpy(&currentVal0[i], &iv1_bits, sizeof(float));
    } else
      currentVal0[i] = 0.0f;

    if (nvs_get_u32(nvs_handle, kValScale, &iv2_bits) == ESP_OK) {
      memcpy(&currentValueScale[i], &iv2_bits, sizeof(float));
    } else
      currentValueScale[i] = 0.0f;

    // Phục hồi lại hệ số góc m cho RAM
    calculateCurrentSlope(i);

    ESP_LOGD("NVS_LOAD", "Board %d: R0=%u, RS=%u, V_Scale=%.2f, I_Slope=%.6f",
             i, voltageRaw0[i], voltageRawScale[i], voltageValueScale[i],
             currentSlope[i]);
  }

  nvs_close(nvs_handle);
  return ESP_OK;
}

double ADCMeasure::getVoltage(uint8_t board_id) { return voltage[board_id]; }

void ADCMeasure::setVoltage(uint8_t board_id, float voltageValue) {
  if (board_id <= BOARD_4) {
    this->voltage[board_id] = voltageValue;
  }
}

double ADCMeasure::getCurrent(uint8_t board_id) { return current[board_id]; }

void ADCMeasure::setCurrent(uint8_t board_id, float currentValue) {
  if (board_id <= BOARD_4) {
    current[board_id] = currentValue;
  }
}

void ADCMeasure::setCurrentOffset(uint8_t board_id, uint16_t offset) {
  // ESP_LOGI(TAG, "Vao setCurrentOffset");
  if (board_id <= BOARD_4) {
    if (currentOffset[board_id] == 0) {
      currentOffset[board_id] = offset; // Lưu giá trị offset ban đầu
    } else {
      // Tính giá trị offset trung bình
      currentOffset[board_id] = (currentOffset[board_id] + offset) /
                                2; // Cập nhật giá trị offset trung bình
    }
  }
}

double ADCMeasure::convertToVoltage(uint8_t board_id, uint16_t raw) {
  if (board_id > BOARD_4)
    return 0.0;
  ESP_LOGI(TAG, "\n\n-----Gia tri Voltage raw doc duoc tai board %d: %d",
           board_id, raw);
  uint16_t r0 = voltageRaw0[board_id];
  ESP_LOGI(TAG, "\n\n-----Gia tri Voltage raw0 doc duoc tai board %d: %d",
           board_id, r0);
  uint16_t rS = voltageRawScale[board_id];
  ESP_LOGI(TAG, "\n\n-----Gia tri Voltage raw Scale doc duoc tai board %d: %d",
           board_id, rS);
  float vS = voltageValueScale[board_id]; // <--- Giá trị này lấy từ bộ nhớ
  ESP_LOGI(TAG,
           "\n\n-----Gia tri dien ap chuan de calib doc duoc tu flash tai "
           "board %d: %f",
           board_id, vS);
  if (rS <= r0)
    return 0.0;
  double slope = (double)vS / (rS - r0);
  double result = slope * ((int32_t)raw - (int32_t)r0);
  ESP_LOGI(TAG, "\n\n-----Gia tri dien ap tai board %d: %f", board_id, result);
  return (result < 0.0) ? 0.0 : result;
}

double ADCMeasure::convertToCurrent(uint8_t board_id, uint16_t raw) {
  if (board_id >= 4)
    return 0.0;

  uint16_t r0 = currentRaw0[board_id];
  float v0 = currentVal0[board_id];
  uint16_t rS = currentRawScale[board_id];
  float vS = currentValueScale[board_id];
  double slope = currentSlope[board_id];

  if (slope == 0.0) {
    return 0.0;
  }

  // Áp dụng công thức: I = slope * (Raw - Raw0)
  int32_t diff = (int32_t)raw - (int32_t)r0;
  // double result = slope * ((int32_t)raw - (int32_t)r0);
  double result = fabs(slope * (double)diff);
  ESP_LOGI(TAG, "Gia tri slope doc duoc tai board %d: %f", board_id, slope);
  ESP_LOGI(TAG, "Gia tri current doc duoc tai board %d: %f", board_id, result);
  // Chặn giá trị âm do nhiễu khi dòng thực tế = 0A (Raw đọc được < Raw0)
  return (result < 0.0) ? 0.0 : result;
}

void ADCMeasure::calibrateDynamicScale(uint8_t board_id, float actualV,
                                       float actualI) {
  if (board_id >= 4)
    return;

  // 1. Calib Áp (Giữ nguyên logic cũ của bạn)
  voltageRawScale[board_id] = raw_voltage[board_id];
  voltageValueScale[board_id] = actualV;

  // 2. Calib Dòng (Two-point)
  currentRawScale[board_id] = raw_current[board_id];
  currentValueScale[board_id] = actualI;

  calculateCurrentSlope(board_id);
  // 3. Lưu vào Flash
  saveCurrentOffsetToFlash();

  ESP_LOGW(
      TAG,
      "BOARD %d CALIB SCALE OK: V=%.2fV, I=%.2fA (ADC_I: %d), Slope_I: %.6f",
      board_id, actualV, actualI, currentRawScale[board_id],
      currentSlope[board_id]);
}

void ADCMeasure::CalibCurrentZero(void) {
  // Chỉ cần hạ cờ này xuống, adcScanTask sẽ tự động reset mảng đếm và lấy 10
  // mẫu mới cho cả Áp và Dòng
  this->isBoardCalibDone = false;
  this->isCalibratingZero = true;

  ESP_LOGI(
      TAG,
      "Bắt đầu Calib Zero (Lấy trung bình 10 mẫu) - isBoardCalibDone = FALSE");
}

void ADCMeasure::calculateCurrentSlope(uint8_t board_id) {
  if (board_id >= 4)
    return;
  int32_t diffRaw =
      (int32_t)currentRawScale[board_id] - (int32_t)currentRaw0[board_id];

  // Đề phòng chia cho 0 nếu chưa Calib hoặc Calib lỗi
  if (diffRaw <= 0) {
    currentSlope[board_id] = 0.0; // Đặt bằng 0 để báo hiệu chưa calib hợp lệ
    return;
  }

  currentSlope[board_id] =
      (double)(currentValueScale[board_id] - currentVal0[board_id]) /
      (double)diffRaw;
}