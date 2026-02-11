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
    // for (int j = 0; j < 2; j++) {
    currentOffset[i] = 0;
    // }
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
  // ESP_LOGI(TAG, "NVS initialized");
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

  ret = loadCurrentOffsetFromFlash();
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

  uint8_t countBoard1Calib = 0;
  uint8_t countBoard2Calib = 0;
  uint8_t countBoard3Calib = 0;
  uint8_t countBoard4Calib = 0;
  int measurement_count = 0;
  // uint8_t filter = 0;
  bool isBoard1Calib = false;
  bool isBoard2Calib = false;
  bool isBoard3Calib = false;
  bool isBoard4Calib = false;
  uint8_t isFirstReadCurrent[4] = {
      0, 0, 0, 0}; // Biến để kiểm tra lần đọc đầu tiên của dòng điện
  // Kiểm tra xem đã có offset trong flash chưa cho từng board
  bool hasOffsetFromFlash[4] = {false, false, false, false};
  for (int i = 0; i < 4; i++) {
    if (adcMeasure->currentOffset[i] != 0) {
      // ESP_LOGI(TAG, "\ncurrentOffset[%d] = %d", i,
      //          adcMeasure->currentOffset[i]);
      hasOffsetFromFlash[i] = true;
    }
  }
  // Kiểm tra offset trong flash
  // if (adcMeasure->currentOffset[1] != 0) {
  //   ESP_LOGI(TAG, "currentOffset[1] = %d", adcMeasure->currentOffset[1]);
  //   hasOffsetFromFlash[1] = true;
  // }
  // Nếu đã có offset từ flash cho board nào, bỏ qua calibration ban đầu cho
  // board đó
  if (hasOffsetFromFlash[BOARD_1]) {
    isBoard1Calib = true;
    // ESP_LOGI(TAG, "Board 1: Using offset from flash, skipping initial
    // calibration");
  }
  if (hasOffsetFromFlash[BOARD_2]) {
    isBoard2Calib = true;
    // ESP_LOGI(TAG, "Board 2: Using offset from flash, skipping initial
    // calibration");
  }
  if (hasOffsetFromFlash[BOARD_3]) {
    isBoard3Calib = true;
    // ESP_LOGI(TAG, "Board 3: Using offset from flash, skipping initial
    // calibration");
  }
  if (hasOffsetFromFlash[BOARD_4]) {
    isBoard4Calib = true;
    // ESP_LOGI(TAG, "Board 4: Using offset from flash, skipping initial
    // calibration");
  }

  // Nếu tất cả các board đều có offset từ flash, đánh dấu calibration đã hoàn
  // thành
  if (hasOffsetFromFlash[BOARD_1] && hasOffsetFromFlash[BOARD_2] &&
      hasOffsetFromFlash[BOARD_3] && hasOffsetFromFlash[BOARD_4]) {
    adcMeasure->isBoardCalibDone = true;
    // ESP_LOGI(TAG, "All boards have offset from flash, skipping initial
    // calibration");
  }

  // if (hasOffsetFromFlash[BOARD_2]) {
  //   adcMeasure->isBoardCalibDone = true;
  // }

  while (1) {
    for (int board = BOARD_1; board <= BOARD_4; board++) {
      // Kiểm tra kênh AIN1 (dòng điện)
      if (adcMeasure->adc.dataReady(AD770X::CHN_AIN1,
                                    static_cast<AD770X::CS_t>(board))) {
        raw_current[board] = adcMeasure->adc.readADResultRaw(
            AD770X::CHN_AIN1, static_cast<AD770X::CS_t>(board));
        adcMeasure->setCurrentOffset(BOARD_3, 38719); // 38720 - 38594 - 38719
        adcMeasure->saveCurrentOffsetToFlash();
        if (isFirstReadCurrent[board] < 3) {

          isFirstReadCurrent[board]++;

        } else {
          // if (raw_current[board] - adcMeasure->currentOffset[board] >= 0) {
          //   //   adcMeasure->setCurrent(
          //   //       board, adcMeasure->convertToCurrent(
          //   //                  board, raw_current[board] -
          //   // adcMeasure->currentOffset[board]));
          //   // } else {
          //   //   adcMeasure->setCurrent(board,
          //   //                          adcMeasure->convertToCurrent(
          //   //                              board,
          //   //                              adcMeasure->currentOffset[board]
          //   -
          //   //                                         raw_current[board]));
          //   // }

          //   // Sử dụng offset ngay lập tức (từ flash hoặc calibration)
          //   uint16_t offset = adcMeasure->currentOffset[board];
          //   if (raw_current[board] >= offset) {
          //     adcMeasure->setCurrent(board,
          //                            adcMeasure->convertToCurrent(
          //                                board, raw_current[board] -
          //                                offset));
          //   } else {
          //     adcMeasure->setCurrent(board,
          //                            adcMeasure->convertToCurrent(
          //                                board, offset -
          //                                raw_current[board]));
          //   }
          // }
          // Sử dụng offset ngay lập tức (từ flash hoặc calibration)
          uint16_t offset = adcMeasure->currentOffset[board];
          float currentValue = 0.0f;
          ESP_LOGI(TAG, "\nraw_current[%d] = %d, offset[%d] = %d", board,
                   raw_current[board], board, offset);
          if (raw_current[board] >= offset) {
            currentValue = adcMeasure->convertToCurrent(
                board, raw_current[board] - offset);
            adcMeasure->setCurrent(board, currentValue);
          } else {
            currentValue = adcMeasure->convertToCurrent(
                board, offset - raw_current[board]);
            adcMeasure->setCurrent(board, currentValue);
          }

          ESP_LOGI(TAG, "adcScanTask - Board %d Raw_Current: %d - I = %f",
                   board, raw_current[board], currentValue);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
        // Kiểm tra kênh AIN2 (điện áp)
        if (adcMeasure->adc.dataReady(AD770X::CHN_AIN2,
                                      static_cast<AD770X::CS_t>(board))) {
          raw_voltage[board] = adcMeasure->adc.readADResultRaw(
              AD770X::CHN_AIN2, static_cast<AD770X::CS_t>(board));
          ESP_LOGI(TAG, "adcScanTask - Board %d Raw_Voltage: %d", board,
                   raw_voltage[board]);
          adcMeasure->setVoltage(
              board,
              adcMeasure->convertToVoltage(
                  board,
                  raw_voltage[board])); // Cập nhật giá trị điện áp đã lọc
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms

        if (!isBoard1Calib && board == BOARD_1 &&
            !adcMeasure->isBoardCalibDone) {
          if (raw_current[board] != 0) {
            adcMeasure->setCurrentOffset(board, raw_current[board]);
          }

          if (countBoard1Calib < 10) {
            countBoard1Calib++;
          } else {
            isBoard1Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
          }
        }
        // ==============
        // Nếu calib chưa xong thì thực hiện trong hàm này
        if (!isBoard2Calib && board == BOARD_2 &&
            !adcMeasure->isBoardCalibDone) {
          if (raw_current[board] != 0) {
            adcMeasure->setCurrentOffset(board, raw_current[board]);
          }
          if (countBoard2Calib < 10) {
            countBoard2Calib++;
          } else {
            isBoard2Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
          }
        }
        // =============
        if (!isBoard3Calib && board == BOARD_3 &&
            !adcMeasure->isBoardCalibDone) {
          if (raw_current[board] != 0) {
            adcMeasure->setCurrentOffset(board, raw_current[board]);
          }
          if (countBoard3Calib < 10) {
            countBoard3Calib++;
          } else {
            isBoard3Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
          }
        }
        if (!isBoard4Calib && board == BOARD_4 &&
            !adcMeasure->isBoardCalibDone) {
          if (raw_current[board] != 0) {
            adcMeasure->setCurrentOffset(board, raw_current[board]);
          }
          if (countBoard4Calib < 10) {
            countBoard4Calib++;
          } else {
            isBoard4Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
          }
        }
        if (isBoard1Calib && isBoard2Calib && isBoard3Calib && isBoard4Calib &&
            !adcMeasure->isBoardCalibDone) {
          // if (isBoard2Calib && !adcMeasure->isBoardCalibDone) {
          adcMeasure->isBoardCalibDone =
              true; // Đã hoàn thành hiệu chỉnh cho tất cả các board
          isBoard1Calib = false;
          isBoard2Calib = false;
          isBoard3Calib = false;
          isBoard4Calib = false;
          countBoard1Calib = 0;
          countBoard2Calib = 0;
          countBoard3Calib = 0;
          countBoard4Calib = 0;
          // Lưu giá trị offset vào flash
          if (adcMeasure->saveCurrentOffsetToFlash() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save current offset to flash");
          } else {
            ESP_LOGI(TAG, "Current offset saved to flash successfully - %d",
                     adcMeasure->currentOffset[board]);
          }
          gpio_set_level(LED_CALIB_STATUS, 1);
          // ESP_LOGI(TAG,
          //          "\n\n\nCalibration completed for all
          //          boards.\n\n\n\n\n\n");

          // Đo VREF2 cho Board 2 (current channel)
          // uint16_t offset3 = adcMeasure->getCurrentOffset(BOARD_3);
          // if (offset3 > 0) {
          //   float vref3_calc = (1.65f * 65536.0f) / offset3;
          //   ESP_LOGI(TAG, "========================================");
          //   ESP_LOGI(TAG, "VREF3 Measurement (Board 3 - Current)");
          //   ESP_LOGI(TAG, "----------------------------------------");
          //   ESP_LOGI(TAG, "currentOffset[BOARD_3]: %d", offset3);
          //   ESP_LOGI(TAG, "VIOUT(Q) = 1.65V (VCC/2 = 3.3V/2)");
          //   ESP_LOGI(TAG, "VREF3 calculated: %.3fV", vref3_calc);
          //   ESP_LOGI(TAG, "VREF3 in code: %.3fV", VREF3);
          //   ESP_LOGI(TAG, "Difference: %.3fV (%.2f%%)", vref3_calc - VREF3,
          //            ((vref3_calc - VREF3) / VREF3) * 100.0f);
          //   ESP_LOGI(TAG, "========================================");
          //   ESP_LOGI(TAG, ">>> Cập nhật: #define VREF3 %.3ff <<<",
          //   vref3_calc);
          // }
        }
      }
    }
    if (measurement_count > 0 && measurement_count % 10 == 0 &&
        adcMeasure->isBoardCalibDone) {

      // // Khởi tạo cho board 1
      if (adcMeasure->init(BOARD_1, CS_PIN_1, RESET_PIN_1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 1");
        return;
      }
      // Khởi tạo cho board 2
      if (adcMeasure->init(BOARD_2, CS_PIN_2, RESET_PIN_2) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 2");
        return;
      }
      // // Khởi tạo cho board 3
      if (adcMeasure->init(BOARD_3, CS_PIN_3, RESET_PIN_3) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 3");
        return;
      }
      // // Khởi tạo cho board 4
      if (adcMeasure->init(BOARD_4, CS_PIN_4, RESET_PIN_4) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 4");
        return;
      }
      isFirstReadCurrent[BOARD_1] =
          0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 1
      isFirstReadCurrent[BOARD_2] =
          0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 2
      isFirstReadCurrent[BOARD_3] =
          0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 3
      isFirstReadCurrent[BOARD_4] =
          0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 4
      vTaskDelay(300 / portTICK_PERIOD_MS); // Delay 300ms
    }
    measurement_count++;
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
  }
}

esp_err_t ADCMeasure::saveCurrentOffsetToFlash() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  nvs_handle_t nvs_handle;
  ret = nvs_open("adc_storage", NVS_READWRITE, &nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS handle: %d", ret);
    return ret;
  }

  // Lưu currentOffset cho từng board
  char key[16];
  for (int board = BOARD_1; board <= BOARD_4; board++) {
    snprintf(key, sizeof(key), "offset_board_%d", board);
    ret = nvs_set_u16(nvs_handle, key, currentOffset[board]);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save currentOffset for Board %d: %d", board,
               ret);
    } else {
      ESP_LOGI(TAG, "Saved currentOffset for Board %d: %d", board,
               currentOffset[board]);
    }
  }

  // Commit dữ liệu vào flash
  ret = nvs_commit(nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit NVS: %d", ret);
  }

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
      // ESP_LOGI(TAG, "Read currentOffset from flash for Board %d: %d", board,
      //          offset);
    } else {
      ESP_LOGW(TAG, "No offset found for Board %d, using default 0", board);
      currentOffset[board] = 0;
    }
  }

  nvs_close(nvs_handle);
  return ESP_OK;
}

double ADCMeasure::getVoltage(uint8_t board_id) { return voltage[board_id]; }

void ADCMeasure::setVoltage(uint8_t board_id, float voltageValue) {
  if (board_id <= BOARD_4) {
    this->voltage[board_id] = voltageValue;
    // ESP_LOGI(TAG, "Set voltage for Board %d: %f V", board_id,
    // voltageValue);
  }
}

double ADCMeasure::getCurrent(uint8_t board_id) { return current[board_id]; }

void ADCMeasure::setCurrent(uint8_t board_id, float currentValue) {
  if (board_id <= BOARD_4) {
    // Moving average filter với 10 mẫu (đơn giản và ổn định)
    // static float currentBuffer[4][10] = {{0}};
    // static int bufferIndex[4] = {0};
    // static int bufferCount[4] = {0};

    // currentBuffer[board_id][bufferIndex[board_id]] = currentValue;
    // bufferIndex[board_id] = (bufferIndex[board_id] + 1) % 10;
    // if (bufferCount[board_id] < 10) {
    //   bufferCount[board_id]++;
    // }

    // float sum = 0.0f;
    // for (int i = 0; i < bufferCount[board_id]; i++) {
    //   sum += currentBuffer[board_id][i];
    // }
    // current[board_id] = sum / bufferCount[board_id];
    current[board_id] = currentValue;
  }
}

void ADCMeasure::setCurrentOffset(uint8_t board_id, uint16_t offset) {
  // ESP_LOGI(TAG, "Vao setCurrentOffset");
  if (board_id <= BOARD_4) {
    // ESP_LOGI(TAG, "Truoc khi chia trung binh, currentOffset for Board %d :
    // %d",
    //          board_id, offset);
    if (currentOffset[board_id] == 0) {
      currentOffset[board_id] = offset; // Lưu giá trị offset ban đầu
    } else {
      // Tính giá trị offset trung bình
      currentOffset[board_id] = (currentOffset[board_id] + offset) /
                                2; // Cập nhật giá trị offset trung bình
    }
    // ESP_LOGI(TAG, "Sau khi chia trung binh, currentOffset for Board %d : %d",
    //          board_id, offset);
  }
}

/**
 * @brief Converts the raw ADC value to voltage based on the board ID.
 *
 * @param[in] board_id The board ID (BOARD_2, BOARD_2, or BOARD_3)
 * @param[in] raw The raw ADC value
 * @return The voltage value in Volts
 */
double ADCMeasure::convertToVoltage(uint8_t board_id, uint16_t raw) {
  switch (board_id) {
  case BOARD_1: {
    // ESP_LOGI(TAG, "convertToVoltage - Board %d V =  %f", board_id,
    //          CONVERT_ADC2_TO_VOL(raw));
    // return CONVERT_ADC1_TO_VOL(raw);
    const double raw0 = 28926;
    const double raw24 = 31256;
    const double slope = 24.0 / (raw24 - raw0);
    double result = (raw - raw0) * slope;
    if (result < 0.0)
      return 0.0; // Chặn nhiễu âm
    return result;
  }
  // case BOARD_1:
  //   return CONVERT_ADC1_TO_VOL(raw);
  case BOARD_2: {
    // ESP_LOGI(TAG, "convertToVoltage - Board %d V =  %f", board_id,
    //          CONVERT_ADC2_TO_VOL(raw));
    // return CONVERT_ADC2_TO_VOL(raw);
    const double raw0 = 28925;
    const double raw24 = 31267;
    const double slope = 24.0 / (raw24 - raw0);
    double result = (raw - raw0) * slope;
    if (result < 0.0)
      return 0.0; // Chặn nhiễu âm
    return result;
  }

  case BOARD_3: {
    // ESP_LOGI(TAG, "convertToVoltage - Board %d V =  %f", board_id,
    //          CONVERT_ADC2_TO_VOL(raw));
    // return CONVERT_ADC3_TO_VOL(raw);
    const double raw0 = 28925; // Điều chỉnh raw0 cho board 3 dựa trên giá trị
                               // thực tế (offset khác với các board khác)
    const double raw24 = 31234; // Giữ nguyên raw12, slope sẽ tự động điều chỉnh
    const double slope = 24.0 / (raw24 - raw0);
    double result = (raw - raw0) * slope;
    if (result < 0.0)
      return 0.0; // Chặn nhiễu âm
    return result;
  }

  case BOARD_4: {
    // ESP_LOGI(TAG, "Hello");
    // return CONVERT_ADC4_TO_VOL(raw);
    const double raw0 = 28926;
    const double raw24 = 31260;
    const double slope = 24.0 / (raw24 - raw0);
    double result = (raw - raw0) * slope;
    if (result < 0.0)
      return 0.0; // Chặn nhiễu âm
    return result;
  }
  // case BOARD_3:
  //   return CONVERT_ADC3_TO_VOL(raw);
  // case BOARD_4:
  //   return CONVERT_ADC4_TO_VOL(raw);
  default:
    return 0.0;
  }
}

/**
 * Chuyển raw ADC sang current (A).
 * 0.025 là độ nhạy của ACS758 (25mV/A).
 */
double ADCMeasure::convertToCurrent(uint8_t board_id, uint16_t raw) {
  switch (board_id) {
  case BOARD_1:
    // ESP_LOGI(TAG, "convertToCurrent - Board %d I =  %f", board_id,
    //          (((raw * VREF1) / 65536.0) / 0.0076));
    return (((raw * VREF1 * 2) / 65536.0) / 0.0264) * 2.34;
  case BOARD_2:
    // ESP_LOGI(TAG, "convertToCurrent - Board %d I =  %f", board_id,
    //          (((raw * VREF2) / 65536.0) / 0.0076));
    return (((raw * VREF2 * 2) / 65536.0) / 0.0264) * 2.218;
  case BOARD_3:
    // ESP_LOGI(TAG, "convertToCurrent - Board %d I =  %f", board_id,
    //          (((raw * VREF3) / 65536.0) / 0.0076));
    return (((raw * VREF3 * 2) / 65536.0) / 0.0264) * 2.26;
  case BOARD_4:
    // ESP_LOGI(TAG, "convertToCurrent - Board %d I =  %f", board_id,
    //          (((raw * VREF4) / 65536.0) / 0.0076));
    return (((raw * VREF4 * 2) / 65536.0) / 0.0264) * 2.3;
  default:
    return 0.0;
  }
}

uint16_t ADCMeasure::getCurrentOffset(uint8_t board_id) {
  uint16_t offset = 0;
  if (board_id <= BOARD_4) {
    offset = currentOffset[board_id];
  }
  return offset; // Trả về giá trị offset nếu bộ lọc thất bại
}