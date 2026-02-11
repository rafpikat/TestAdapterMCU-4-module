#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <driver/uart.h>

#include "ADCMeasure.h"
#include "ad770x.h"
#include "uartDriver.h"

// #define SPI_HOST SPI2_HOST

static const char *TAG = "MAIN";
// Hàm callback
void onValidMessageReceived(void *object) {
  ADCMeasure *adcMeasure = static_cast<ADCMeasure *>(object);
  adcMeasure->CalibCurrentZero();
  // gpio_set_level(LED_CALIB_STATUS, 0);
}
extern "C" void app_main(void) {

  // Khởi tạo ADCMeasure
  ADCMeasure adcMeasure(2.5, SPI3_HOST);

  // // Khởi tạo cho board 1
  if (adcMeasure.init(BOARD_1, CS_PIN_1, RESET_PIN_1) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC for Board 1");
    return;
  }
  // Khởi tạo cho board 2
  if (adcMeasure.init(BOARD_2, CS_PIN_2, RESET_PIN_2) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC for Board 2");
    return;
  }
  // // Khởi tạo cho board 3
  if (adcMeasure.init(BOARD_3, CS_PIN_3, RESET_PIN_3) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC for Board 3");
    return;
  }
  // // Khởi tạo cho board 4
  if (adcMeasure.init(BOARD_4, CS_PIN_4, RESET_PIN_4) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC for Board 4");
    return;
  }

  // ========================================
  // TEST PHẦN CỨNG VÀ GIAO TIẾP VỚI AD7705
  // ========================================
  ESP_LOGI(TAG, "\n\n=== BẮT ĐẦU TEST PHẦN CỨNG VÀ GIAO TIẾP AD7705 ===\n");

  // Test từng board
  for (int board = BOARD_1; board <= BOARD_4; board++) {
    ESP_LOGI(TAG, "\n>>> Testing Board %d <<<", board + 1);

    // Test hardware trước (GPIO, SPI, CS pin, Reset pin)
    ESP_LOGI(TAG, "--- Hardware Debug ---");
    adcMeasure.adc.debugHardware(static_cast<AD770X::CS_t>(board));
    vTaskDelay(300 / portTICK_PERIOD_MS);

    // Sau đó test communication
    ESP_LOGI(TAG, "--- Communication Test ---");
    bool result =
        adcMeasure.adc.testCommunication(static_cast<AD770X::CS_t>(board));
    if (result) {
      ESP_LOGI(TAG, "✅ Board %d: Communication OK", board + 1);
    } else {
      ESP_LOGE(TAG, "❌ Board %d: Communication FAILED", board + 1);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  ESP_LOGI(TAG, "\n=== KẾT THÚC TEST PHẦN CỨNG VÀ GIAO TIẾP ===\n\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // ========================================

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
    for (int board = BOARD_1; board <= BOARD_4; board++) {
      // if (board == BOARD_4) {
      //   ESP_LOGI(TAG, "Processing Board 4");
      // }
      // uint8_t board = BOARD_1;
      float voltage = adcMeasure.getVoltage(board);
      uart.send(voltage, 0x01, board + 1);
      // ESP_LOGI(TAG, "Board %d Voltage: %.6f V", board, voltage);

      float current = adcMeasure.getCurrent(board);
      uart.send(current, 0x02, board + 1);
      // ESP_LOGI(TAG, "Board %d Current: %.6f mA", board, current * 1000.0);
      // if (board == BOARD_4) {
      //   ESP_LOGI(TAG, "Board 4 - Voltage: %.6f V, Current: %.6f mA", voltage,
      //            current * 1000.0);
      // }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}