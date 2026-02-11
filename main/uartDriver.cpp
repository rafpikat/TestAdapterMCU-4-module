
// #include "esp_log.h"
// #include "esp_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "uartDriver.h"
// /**
//  * @brief Chuyển đổi giá trị float sang 4 bytes big-endian
//  *
//  * @param value Giá trị float cần chuyển đổi
//  * @param bytes Mảng 4 bytes nhận giá trị sau khi chuyển đổi
//  */

// void float_to_bytes(float value, uint8_t *bytes) {
//     union {
//         float f;
//         uint32_t u;
//     } float_union;
//     float_union.f = value;

//     // Chuyển sang big-endian
//     bytes[0] = (float_union.u >> 24) & 0xFF;
//     bytes[1] = (float_union.u >> 16) & 0xFF;
//     bytes[2] = (float_union.u >> 8) & 0xFF;
//     bytes[3] = float_union.u & 0xFF;
// }

// void SendUART(float value, uint8_t cmd_id, uint8_t id_board) {
//     uint8_t message[10]={0};
//     uint8_t index = 0;

//     // Start Byte: 4C 4D
//     message[index++] = START_BYTE_1;
//     message[index++] = START_BYTE_2;

//     // Cmd ID
//     message[index++] = cmd_id;

//     // Length Data: 5 bytes (1 byte ID board + 4 bytes float)
//     message[index++] = 0x05;

//     // Data: ID board
//     message[index++] = id_board;

//     // Data: Giá trị float
//     float_to_bytes(value, &message[index]);
//     index += 4;

//     // Tính Check XOR (chỉ trên phần Data: ID board + float)
//     uint8_t check_xor = 0;
//     for (uint8_t i = 4; i < index; i++) { // Bắt đầu từ index 4 (phần Data)
//         check_xor ^= message[i];
//     }
//     message[index++] = check_xor;

//     // Gửi bản tin UART
//     uart_write_bytes(UART_NUM_0, message, index);
// }
// void uartDriverInit(void)
// {
//     // Cấu hình UART
//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };

//     // Cấu hình UART0
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
//                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

//     // Cài đặt driver cho UART0
//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0,
//     NULL, 0));
// }
#include "uartDriver.h"
#include "esp_err.h"
#include "esp_log.h"
#include <array>

static const char *TAG = "UART_DRIVER";

UartDriver::UartDriver() : uart_queue_(nullptr), callback_(nullptr) {}

UartDriver::~UartDriver() {
  // Xóa hàng đợi nếu cần
  if (uart_queue_) {
    vQueueDelete(uart_queue_);
  }
}

void UartDriver::floatToBytes(float value, uint8_t *bytes) {
  union {
    float f;
    uint32_t u;
  } float_union;
  float_union.f = value;

  // Chuyển sang big-endian
  bytes[0] = (float_union.u >> 24) & 0xFF;
  bytes[1] = (float_union.u >> 16) & 0xFF;
  bytes[2] = (float_union.u >> 8) & 0xFF;
  bytes[3] = float_union.u & 0xFF;
}

uint8_t UartDriver::calculateChecksum(const uint8_t *message, size_t data_start,
                                      size_t data_end) {
  uint8_t check_xor = 0;
  for (size_t i = data_start; i < data_end; ++i) {
    check_xor ^= message[i];
  }
  return check_xor;
}

bool UartDriver::send(float value, uint8_t cmd_id, uint8_t id_board) {
  uint8_t message[10] = {0};
  uint8_t index = 0;

  // Start Bytes: 4C 4D
  message[index++] = START_BYTE_1;
  message[index++] = START_BYTE_2;

  // Cmd ID
  message[index++] = cmd_id;

  // Length Data: 5 bytes (1 byte ID board + 4 bytes float)
  message[index++] = 0x05;

  // Data: ID Board
  message[index++] = id_board;

  // Data: Float value
  floatToBytes(value, &message[index]);
  index += 4;

  // Tính Checksum
  uint8_t checksum = calculateChecksum(message, 4, index); // Tính trước
  message[index++] = checksum;

  // Gửi bản tin UART
  int sent_bytes = uart_write_bytes(UART_PORT_NUM, message, index);
  if (sent_bytes != static_cast<int>(index)) {
    ESP_LOGE(TAG, "Failed to send all bytes, sent: %d, expected: %d",
             sent_bytes, index);
    return false;
  }
  return true;
}

void UartDriver::init() {
  // Cấu hình UART
  uart_config_t uart_config = {};
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.rx_flow_ctrl_thresh = 0;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  // Cấu hình UART0
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Cài đặt driver với hàng đợi sự kiện
  uart_queue_ = xQueueCreate(20, sizeof(uart_event_t));
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2,
                                      20, &uart_queue_, 0));
  // // ESP_LOGI(TAG, "UART initialized successfully");

  // Tạo task để xử lý sự kiện UART
  xTaskCreate(uartEventTask, "uart_event_task", 2048, this, 12, nullptr);
}

void UartDriver::setCallback(UARTCallback callback, void *object) {
  this->callback_ = callback;
  this->callback_object_ = object; // Lưu đối tượng để sử dụng trong callback
}

void UartDriver::uartEventTask(void *pvParameters) {
  UartDriver *driver = static_cast<UartDriver *>(pvParameters);
  std::array<uint8_t, 16> buffer = {0}; // Buffer đủ lớn để chứa bản tin tối đa
  size_t index = 0;
  bool start_found = false;
  uart_event_t event;

  while (true) {
    if (xQueueReceive(driver->uart_queue_, &event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {
        while (true) {
          uint8_t byte;
          int len =
              uart_read_bytes(UART_PORT_NUM, &byte, 1, 20 / portTICK_PERIOD_MS);
          if (len <= 0)
            break;

          // Tìm start bytes
          if (!start_found) {
            if (byte == START_BYTE_1) {
              buffer[index++] = byte;
              start_found = true;
            }
            continue;
          }

          if (index == 1 && byte != START_BYTE_2) {
            index = 0;
            start_found = false;
            continue;
          }

          buffer[index++] = byte;

          // Kiểm tra khi nhận đủ Length
          if (index == 4) {
            // Length là độ dài phần data
            uint8_t data_length = buffer[3];
            if (data_length == 0 ||
                data_length > 12) { // Giới hạn để tránh tràn buffer
              ESP_LOGE(TAG, "Invalid data length: %d", data_length);
              index = 0;
              start_found = false;
              continue;
            }
          }

          // Kiểm tra khi nhận đủ bản tin (2 start bytes + 1 CmdID + 1 Length +
          // data_length + 1 Checksum)
          if (index == 4 + buffer[3] + 1) {
            // Kiểm tra Checksum
            uint8_t received_checksum = buffer[index - 1];
            uint8_t calculated_checksum =
                driver->calculateChecksum(buffer.data(), 4, 4 + buffer[3]);
            if (received_checksum != calculated_checksum) {
              ESP_LOGE(TAG,
                       "Checksum mismatch: received 0x%02X, calculated 0x%02X",
                       received_checksum, calculated_checksum);
              index = 0;
              start_found = false;
              continue;
            }

            // Kiểm tra CmdID và Data
            if (buffer[2] == 0x03 && buffer[3] == 0x01 && buffer[4] == 0x00) {
              if (driver->callback_) {
                driver->callback_(driver->callback_object_);
              }
            }

            index = 0;
            start_found = false;
          }

          // Reset nếu buffer đầy
          if (index >= buffer.size()) {
            ESP_LOGE(TAG, "Buffer overflow");
            index = 0;
            start_found = false;
          }
        }
      }
    }
  }
}