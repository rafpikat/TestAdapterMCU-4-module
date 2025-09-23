#include "ADCMeasure.h"
#define TAG "ADCMeasure"

uint16_t raw_voltage[3] = {{0}}; // [board_id]
uint16_t raw_current[3] = {{0}}; // [board_id]

ADCMeasure::ADCMeasure(float vref, spi_host_device_t spi_host)
    : adc(vref, spi_host, CS_PIN_1, CS_PIN_2, CS_PIN_3, RESET_PIN_1, RESET_PIN_2, RESET_PIN_3) {
    adcTaskHandle = nullptr;
    // Khởi tạo offset
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            currentOffset[i] = 0;
        }
    }
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
}

esp_err_t ADCMeasure::init(uint8_t board_id, gpio_num_t cs_pin, gpio_num_t reset_pin) {
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
        default:
            ESP_LOGE("ADCMeasure", "Invalid board ID: %d", board_id);
            return ESP_ERR_INVALID_ARG;
    }


    // Cấu hình bộ lọc Kalman
    setupKalmanFilter();

    ret = loadCurrentOffsetFromFlash();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load currentOffset from flash for Board %d, using default", board_id);
    }
    return ESP_OK;
}

void ADCMeasure::setupKalmanFilter() {
    // Ma trận chuyển trạng thái (giả sử hệ thống tuyến tính đơn giản)
    A[0] = 1.0; A[1] = 0.0;
    A[2] = 0.0; A[3] = 1.0;

    // Ma trận hiệp phương sai ban đầu
    P[0] = 0.1; P[1] = 0.0;
    P[2] = 0.0; P[3] = 0.1;

    // Nhiễu quá trình
    Q[0] = 0.01; Q[1] = 0.0;
    Q[2] = 0.0; Q[3] = 0.01;

    // Ma trận quan sát
    H[0] = 1.0; H[1] = 0.0;

    // Nhiễu quan sát
    R[0] = 0.1;

    // Khởi tạo trạng thái
    xp[0] = 0.0; xp[1] = 0.0;
    xc[0] = 0.0; xc[1] = 0.0;

    // Khởi tạo bộ lọc Kalman
    voltageFilter.init(1, 1, A, P, Q, H, R, xp, xc);
    currentFilter.init(1, 1, A, P, Q, H, R, xp, xc);
}

esp_err_t ADCMeasure::startADCTask() {
    // Tạo task FreeRTOS để quét ADC
    BaseType_t result = xTaskCreate(
        adcScanTask,
        "ADC_Scan_Task",
        4096, // Stack size
        this, // Tham số truyền vào task
        5,    // Độ ưu tiên
        &adcTaskHandle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC scan task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ADCMeasure::adcScanTask(void* arg) {
    ADCMeasure* adcMeasure = static_cast<ADCMeasure*>(arg);

    uint8_t countBoard1Calib = 0;
    uint8_t countBoard2Calib = 0;
    uint8_t countBoard3Calib = 0;
    int measurement_count = 0;
    uint8_t filter = 0;
    bool isBoard1Calib = false;
    bool isBoard2Calib = false;
    bool isBoard3Calib = false;
    uint8_t isFirstReadCurrent[3] = {0, 0, 0}; // Biến để kiểm tra lần đọc đầu tiên của dòng điện
    while (1) {
        for (uint8_t board = BOARD_1; board <= BOARD_3; board++) {
            // Kiểm tra kênh AIN2 (dòng điện)
            if (adcMeasure->adc.dataReady(AD770X::CHN_AIN2, static_cast<AD770X::CS_t>(board))) {
                raw_current[board] = adcMeasure->adc.readADResultRaw(AD770X::CHN_AIN2, static_cast<AD770X::CS_t>(board));
                    if(isFirstReadCurrent[board] < 3) {
                        // Lần đọc đầu tiên, lưu giá trị offset
                        isFirstReadCurrent[board] ++;
            
                    }else{
                        ESP_LOGI(TAG, "B %d RW_C: %d", board, raw_current[board]);
                        if(raw_current[board] - adcMeasure->currentOffset[board] >= 0 )
                        {
                            adcMeasure->setCurrent(board, adcMeasure->convertToCurrent(board, raw_current[board] - adcMeasure->currentOffset[board]));
                        }else{
                            adcMeasure->setCurrent(board, adcMeasure->convertToCurrent(board, adcMeasure->currentOffset[board]-raw_current[board]));
                        }
                    } 
            }
            vTaskDelay(50 / portTICK_PERIOD_MS); // Delay 100ms
            // Kiểm tra kênh AIN1 (điện áp)
            if (adcMeasure->adc.dataReady(AD770X::CHN_AIN1, static_cast<AD770X::CS_t>(board))) {
                raw_voltage[board] = adcMeasure->adc.readADResultRaw(AD770X::CHN_AIN1, static_cast<AD770X::CS_t>(board));
                    ESP_LOGI(TAG, "B %d RW_V: %d", board, raw_voltage[board]);
                    adcMeasure->setVoltage(board, adcMeasure->convertToVoltage(board, raw_voltage[board])); // Cập nhật giá trị điện áp đã lọc
            }
            vTaskDelay(50 / portTICK_PERIOD_MS); // Delay 100ms

            
            if(!isBoard1Calib && board == BOARD_1 && !adcMeasure->isBoardCalibDone){
                if(raw_current[board]!=0)
                {
                    adcMeasure->setCurrentOffset(board, raw_current[board]);
                }

                if(countBoard1Calib < 20) {
                    countBoard1Calib++;
                } else {
                    isBoard1Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
                }
            }
            if(!isBoard2Calib && board == BOARD_2&& !adcMeasure->isBoardCalibDone){
                if(raw_current[board]!=0)
                {
                    adcMeasure->setCurrentOffset(board, raw_current[board]);
                }
                if(countBoard2Calib < 20) {
                    countBoard2Calib++;
                } else {
                    isBoard2Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
                }   
            }
            if(!isBoard3Calib && board == BOARD_3&& !adcMeasure->isBoardCalibDone){
                if(raw_current[board]!=0)
                {
                    adcMeasure->setCurrentOffset(board, raw_current[board]);
                }
                if(countBoard3Calib < 20) {
                    countBoard3Calib++;
                } else {
                    isBoard3Calib = true; // Đã đủ lần đo, không cần hiệu chỉnh nữa
                }   
            }
            if(isBoard1Calib && isBoard2Calib && isBoard3Calib && !adcMeasure->isBoardCalibDone) {
                adcMeasure->isBoardCalibDone = true; // Đã hoàn thành hiệu chỉnh cho tất cả các board
                isBoard1Calib = false;
                isBoard2Calib = false;
                isBoard3Calib = false;
                countBoard1Calib = 0;
                countBoard2Calib = 0;
                countBoard3Calib = 0;
                // Lưu giá trị offset vào flash
                if (adcMeasure->saveCurrentOffsetToFlash() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to save current offset to flash");
                } else {
                    ESP_LOGI(TAG, "Current offset saved to flash successfully");
                }
                gpio_set_level(LED_CALIB_STATUS, 1);
                ESP_LOGI(TAG, "\n\n\nCalibration completed for all boards.\n\n\n\n\n\n");
            }
        }
        if(measurement_count > 0 && measurement_count % 10 == 0 && adcMeasure->isBoardCalibDone) {
            
            // Khởi tạo cho board 1
            if (adcMeasure->init(BOARD_1, CS_PIN_1, RESET_PIN_1) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC for Board 1");
            return;
            }
            // Khởi tạo cho board 2
            if (adcMeasure->init(BOARD_2, CS_PIN_2, RESET_PIN_2) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize ADC for Board 2");
                return;
            }
            // Khởi tạo cho board 3
            if (adcMeasure->init(BOARD_3, CS_PIN_3, RESET_PIN_3) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize ADC for Board 3");
                return;
            }
            isFirstReadCurrent[BOARD_1] = 0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 1
            isFirstReadCurrent[BOARD_2] = 0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 2
            isFirstReadCurrent[BOARD_3] = 0; // Đặt lại biến kiểm tra lần đọc đầu tiên cho board 3
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 300ms
            
        }
        measurement_count++;
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
        
    }
}
esp_err_t ADCMeasure::saveCurrentOffsetToFlash() {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("adc_storage", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %d", ret);
        return ret;
    }

    // Lưu currentOffset cho từng board
    char key[16];
    for (uint8_t board = BOARD_1; board <= BOARD_3; board++) {
        snprintf(key, sizeof(key), "offset_board_%d", board);
        ret = nvs_set_u16(nvs_handle, key, currentOffset[board]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save offset for Board %d: %d", board, ret);
        } else {
            ESP_LOGI(TAG, "Saved offset for Board %d: %d", board, currentOffset[board]);
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
        ESP_LOGE(TAG, "Failed to open NVS handle: %d. Saving default offsets...", ret);
        // Nếu mở NVS lỗi, lưu giá trị mặc định (0) vào flash
        for (uint8_t board = BOARD_1; board <= BOARD_3; board++) {
            currentOffset[board] = 0;
        }
        saveCurrentOffsetToFlash();
        return ret;
    }

    // Đọc currentOffset cho từng board
    char key[16];
    for (uint8_t board = BOARD_1; board <= BOARD_3; board++) {
        snprintf(key, sizeof(key), "offset_board_%d", board);
        uint16_t offset = 0;
        ret = nvs_get_u16(nvs_handle, key, &offset);
        if (ret == ESP_OK) {
            currentOffset[board] = offset;
            ESP_LOGI(TAG, "Loaded offset for Board %d: %d", board, offset);
        } else {
            ESP_LOGW(TAG, "No offset found for Board %d, using default 0", board);
            currentOffset[board] = 0;
        }
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}
double ADCMeasure::getVoltage(uint8_t board_id) {
    return voltage[board_id]; // Trả về giá trị thô nếu bộ lọc thất bại
}

void ADCMeasure::setVoltage(uint8_t board_id, float voltageValue) {
    if (board_id >= BOARD_1 && board_id <= BOARD_3) {
        this->voltage[board_id] = voltageValue;
        // ESP_LOGI(TAG, "Set voltage for Board %d: %f V", board_id, voltageValue);
    }
}

double ADCMeasure::getCurrent(uint8_t board_id) {
    return current[board_id]; // Trả về giá trị thô nếu bộ lọc thất bại
}
void ADCMeasure::setCurrent(uint8_t board_id, float currentValue) {
    if (board_id >= BOARD_1 && board_id <= BOARD_3) {
        current[board_id] = currentValue;
    }
}
void ADCMeasure::setCurrentOffset(uint8_t board_id, uint16_t offset) {
    if (board_id >= BOARD_1 && board_id <= BOARD_3) {
        if(currentOffset[board_id] == 0)
        {
            currentOffset[board_id] = offset; // Lưu giá trị offset ban đầu
        }
        else
        {
            // Tính giá trị offset trung bình
            currentOffset[board_id] = (currentOffset[board_id] + offset) / 2; // Cập nhật giá trị offset trung bình
        }
        ESP_LOGI(TAG, "Set current offset for Board %d : %d", board_id, offset);
    }
}

/**
 * @brief Converts the raw ADC value to voltage based on the board ID.
 *
 * @param[in] board_id The board ID (BOARD_1, BOARD_2, or BOARD_3)
 * @param[in] raw The raw ADC value
 * @return The voltage value in Volts
 */
double ADCMeasure::convertToVoltage(uint8_t board_id, uint16_t raw) {
    switch (board_id) {
        case BOARD_1:
            return CONVERT_ADC1_TO_VOL(raw);
        case BOARD_2:
            return CONVERT_ADC2_TO_VOL(raw);
        case BOARD_3:
            return CONVERT_ADC3_TO_VOL(raw);
        default:
            return 0.0;
    }
}

double ADCMeasure::convertToCurrent(uint8_t board_id, uint16_t raw) {
    switch (board_id) {
        case BOARD_1:
            return (((raw * VREF1) / 65536.0) / 0.025);
        case BOARD_2:
            return (((raw * VREF2) / 65536.0) / 0.025);
        case BOARD_3:
            return (((raw * VREF3) / 65536.0) / 0.025);
        default:
            return 0.0;
    }
}

uint16_t ADCMeasure::getCurrentOffset(uint8_t board_id) { 
    uint16_t offset = 0;
    if (board_id >= BOARD_1 && board_id <= BOARD_3) {
        offset = currentOffset[board_id];
    }
    return offset; // Trả về giá trị offset nếu bộ lọc thất bại
}