
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <driver/gpio.h>
// #include <driver/uart.h>
// #define UART_PORT_NUM      UART_NUM_0   // Sử dụng UART0
// #define UART_TX_PIN        GPIO_NUM_1   // TX mặc định của UART0
// #define UART_RX_PIN        GPIO_NUM_3   // RX mặc định của UART0
// #define BUF_SIZE           1024
// #define START_BYTE_1       0x4C
// #define START_BYTE_2        0x4D
// void float_to_bytes(float value, uint8_t *bytes);
// void SendUART(float value, uint8_t cmd_id, uint8_t id_board);
// void uartDriverInit(void);
#pragma once

#include <array>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <functional>

class UartDriver {
public:
  static constexpr uart_port_t UART_PORT_NUM = UART_NUM_0;
  static constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_1;
  static constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_3;
  static constexpr size_t BUF_SIZE = 1024;
  static constexpr uint8_t START_BYTE_1 = 0x4C;
  static constexpr uint8_t START_BYTE_2 = 0x4D;

  // Callback type for handling received messages
  typedef void (*UARTCallback)(void *object);

  UartDriver();
  ~UartDriver();

  // Khởi tạo UART driver
  void init();

  // Gửi bản tin UART
  bool send(float value, uint8_t cmd_id, uint8_t id_board);

  // Thiết lập callback cho bản tin cmdId = 0x03, data = 0x00
  void setCallback(UARTCallback callback, void *object);

private:
  // Chuyển đổi float sang 4 bytes big-endian
  void floatToBytes(float value, uint8_t *bytes);

  // Tính checksum XOR cho phần dữ liệu
  uint8_t calculateChecksum(const uint8_t *message, size_t data_start,
                            size_t data_end);

  // Task để xử lý sự kiện UART và bản tin
  static void uartEventTask(void *pvParameters);

  QueueHandle_t uart_queue_; // Hàng đợi sự kiện UART
  UARTCallback callback_;    // Callback cho bản tin cmdId = 0x03, data = 0x00
  void *callback_object_;    // Đối tượng để truyền vào callback
};