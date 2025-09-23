#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ad770x.h"
#include "esp_log.h"
#include "esp_err.h"
#include "uartDriver.h"
#include "ADCMeasure.h"
#include <driver/uart.h>

// #define SPI_HOST SPI2_HOST

static const char* TAG = "MAIN";
// Hàm callback
void onValidMessageReceived(void* object) {
    ADCMeasure* adcMeasure = static_cast<ADCMeasure*>(object);
    adcMeasure->CalibCurrentZero();
    // gpio_set_level(LED_CALIB_STATUS, 0);

}
extern "C" void app_main(void) {
    
    // Khởi tạo ADCMeasure
    ADCMeasure adcMeasure(2.5, SPI3_HOST);
    
    // Khởi tạo cho board 1
    if (adcMeasure.init(BOARD_1, CS_PIN_1, RESET_PIN_1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 1");
        return;
    }
    // Khởi tạo cho board 2
    if (adcMeasure.init(BOARD_2, CS_PIN_2, RESET_PIN_2) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 2");
        return;
    }
    // Khởi tạo cho board 3
    if (adcMeasure.init(BOARD_3, CS_PIN_3, RESET_PIN_3) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for Board 3");
        return;
    }
    // Bắt đầu task quét ADC
    if (adcMeasure.startADCTask() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC task");
        return;
    }

    // Đợi một chút để ADC ổn định
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Khởi tạo UART
    UartDriver uart;
    uart.init();
    // Thiết lập callback bằng hàm thông thường
    uart.setCallback(onValidMessageReceived, &adcMeasure);
    // Vòng lặp chính để đọc giá trị
    while (1) {
        // vTaskDelay(200 / portTICK_PERIOD_MS);
        for (uint8_t board = BOARD_1; board <= BOARD_3; board++)
        {
            // uint8_t board = BOARD_1;
            float voltage = adcMeasure.getVoltage(board);
            uart.send(voltage, 0x01, board+1);
            // ESP_LOGI(TAG, "Board %d Voltage: %.6f V", board, voltage);

            float current = adcMeasure.getCurrent(board);
            uart.send(current, 0x02, board+1);
            // ESP_LOGI(TAG, "Board %d Current: %.6f mA", board, current * 1000.0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}