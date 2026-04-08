#include "ad770x.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "AD770X";

// Helper function to convert byte to binary string
static void byteToBinary(uint8_t byte, char *binary_str) {
  for (int i = 7; i >= 0; i--) {
    binary_str[7 - i] = (byte & (1 << i)) ? '1' : '0';
  }
  binary_str[8] = '\0';
}

AD770X::AD770X(double vref, spi_host_device_t spi_host, gpio_num_t cs_pin_1,
               gpio_num_t cs_pin_2, gpio_num_t cs_pin_3, gpio_num_t cs_pin_4,
               gpio_num_t reset_pin_1, gpio_num_t reset_pin_2,
               gpio_num_t reset_pin_3, gpio_num_t reset_pin_4)
    : VRef(vref), spi_host(spi_host), cs_pin_1(cs_pin_1), cs_pin_2(cs_pin_2),
      cs_pin_3(cs_pin_3), cs_pin_4(cs_pin_4), reset_pin_1(reset_pin_1),
      reset_pin_2(reset_pin_2), reset_pin_3(reset_pin_3),
      reset_pin_4(reset_pin_4) {
  // THÊM ĐOẠN NÀY: Reset GPIO trước khi config
  gpio_reset_pin(cs_pin_1);
  gpio_reset_pin(cs_pin_2);
  gpio_reset_pin(cs_pin_3);
  gpio_reset_pin(cs_pin_4);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // Configure GPIO for CS
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << cs_pin_1),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  gpio_config_t io_conf_2 = {.pin_bit_mask = (1ULL << cs_pin_2),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_ENABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_conf_2));

  gpio_config_t io_conf_3 = {.pin_bit_mask = (1ULL << cs_pin_3),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_ENABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_conf_3));

  gpio_config_t io_conf_4 = {.pin_bit_mask = (1ULL << cs_pin_4),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_ENABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_conf_4));

  // Configure GPIO for Reset
  gpio_config_t io_rs_conf = {.pin_bit_mask = (1ULL << reset_pin_1),
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_ENABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_rs_conf));

  gpio_config_t io_rs_conf_2 = {.pin_bit_mask = (1ULL << reset_pin_2),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_ENABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_rs_conf_2));

  gpio_config_t io_rs_conf_3 = {.pin_bit_mask = (1ULL << reset_pin_3),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_ENABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_rs_conf_3));

  gpio_config_t io_rs_conf_4 = {.pin_bit_mask = (1ULL << reset_pin_4),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_ENABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_rs_conf_4));

  // Configure GPIO for LED Calibration Status
  gpio_config_t io_led_conf = {.pin_bit_mask = (1ULL << LED_CALIB_STATUS),
                               .mode = GPIO_MODE_OUTPUT,
                               .pull_up_en = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io_led_conf));
  gpio_set_level(cs_pin_1, 1); // CS 1 high (inactive)
  gpio_set_level(cs_pin_2, 1); // CS 2 high (inactive)
  gpio_set_level(cs_pin_3, 1); // CS 3 high (inactive)
  gpio_set_level(cs_pin_4, 1); // CS 4 high (inactive)

  gpio_set_level(reset_pin_1, 1);      // CS high (inactive)
  gpio_set_level(reset_pin_2, 1);      // CS high (inactive)
  gpio_set_level(reset_pin_3, 1);      // CS high (inactive)
  gpio_set_level(reset_pin_4, 1);      // CS high (inactive)
  gpio_set_level(LED_CALIB_STATUS, 0); // LED Calibration Status low (inactive)
  // Initialize SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = DEFAULT_MOSI_PIN;
  buscfg.miso_io_num = DEFAULT_MISO_PIN;
  buscfg.sclk_io_num = DEFAULT_SCK_PIN;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.data4_io_num = -1;
  buscfg.data5_io_num = -1;
  buscfg.data6_io_num = -1;
  buscfg.data7_io_num = -1;
  buscfg.max_transfer_sz = 32;
  buscfg.flags = 0;

  ESP_ERROR_CHECK(spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO));

  // Add SPI device
  spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.dummy_bits = 0;
  devcfg.mode = 3;
  devcfg.clock_speed_hz = 1 * 5 * 1000; // 500kHz
  devcfg.spics_io_num = -1;             // CS handled manually
  devcfg.queue_size = 1;

  ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &devcfg, &spi_device));
}

AD770X::~AD770X() {
  spi_bus_remove_device(spi_device);
  spi_bus_free(spi_host);
}

void AD770X::readAndDisplaySPIPins() {
  // Cấu hình GPIO để đọc (tạm thời)
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << DEFAULT_MOSI_PIN) |
                         (1ULL << DEFAULT_MISO_PIN) | (1ULL << DEFAULT_SCK_PIN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Đọc giá trị GPIO
  int mosi_level = gpio_get_level(DEFAULT_MOSI_PIN);
  int miso_level = gpio_get_level(DEFAULT_MISO_PIN);
  int sck_level = gpio_get_level(DEFAULT_SCK_PIN);
}

static void spiPinMonitorTask(void *arg) {
  AD770X *adc = static_cast<AD770X *>(arg);

  while (1) {
    adc->readAndDisplaySPIPins();
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
  }
}

// Implementation của hàm startSPIPinMonitorTask
void AD770X::startSPIPinMonitorTask() {
  xTaskCreate(spiPinMonitorTask, "spi_pin_monitor", 2048, this, 5, NULL);
}

uint8_t AD770X::spiTransfer(uint8_t data) {
  uint8_t ret = 0;
  spi_transaction_t t = {};
  t.length = 8; // 8 bits
  t.rxlength = 8;
  t.tx_buffer = &data;
  t.rx_buffer = &ret;

  esp_err_t err = spi_device_polling_transmit(spi_device, &t);

  if (err != ESP_OK) {
    ESP_LOGE("SPI_TRANSFER", "SPI transfer error: %s", esp_err_to_name(err));
    ESP_LOGE("SPI_TRANSFER",
             "TX: 0x%02X, CS states: CS1=%d, CS2=%d, CS3=%d, CS4=%d", data,
             gpio_get_level(cs_pin_1), gpio_get_level(cs_pin_2),
             gpio_get_level(cs_pin_3), gpio_get_level(cs_pin_4));
  }

  char tx_binary[9], rx_binary[9];
  byteToBinary(data, tx_binary);
  byteToBinary(ret, rx_binary);
  ESP_LOGI("SPI_TRANSFER", "TX: 0x%02X (0b%s) -> RX: 0x%02X (0b%s)", data,
           tx_binary, ret, rx_binary);

  return ret;
}

void AD770X::setNextOperation(uint8_t reg, uint8_t channel, uint8_t readWrite,
                              CS_t cs) {
  gpio_num_t cs_pin = (cs == CS_1)   ? this->cs_pin_1
                      : (cs == CS_2) ? this->cs_pin_2
                      : (cs == CS_3) ? this->cs_pin_3
                                     : this->cs_pin_4;
  uint8_t r = (reg << 4) | (readWrite << 3) | channel;

  const char *reg_name = (reg == REG_CMM)      ? "REG_CMM"
                         : (reg == REG_SETUP)  ? "REG_SETUP"
                         : (reg == REG_CLOCK)  ? "REG_CLOCK"
                         : (reg == REG_DATA)   ? "REG_DATA"
                         : (reg == REG_TEST)   ? "REG_TEST"
                         : (reg == REG_NOP)    ? "REG_NOP"
                         : (reg == REG_OFFSET) ? "REG_OFFSET"
                         : (reg == REG_GAIN)   ? "REG_GAIN"
                                               : "UNKNOWN";
  const char *ch_name = (channel == CHN_AIN1)   ? "CHN_AIN1"
                        : (channel == CHN_AIN2) ? "CHN_AIN2"
                        : (channel == CHN_COMM) ? "CHN_COMM"
                        : (channel == CHN_AIN3) ? "CHN_AIN3"
                                                : "UNKNOWN";

  ESP_LOGI("setNextOperation", "=== setNextOperation ===");
  ESP_LOGI("setNextOperation",
           "Register: %s (0x%02X), Channel: %s (0x%02X), R/W: %s (%d), CS: %d",
           reg_name, reg, ch_name, channel, (readWrite == 0) ? "WRITE" : "READ",
           readWrite, cs);
  char cmd_binary[9];
  byteToBinary(r, cmd_binary);

  // Đảm bảo tất cả CS pin về HIGH trước khi set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Sau đó mới set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(2 / portTICK_PERIOD_MS);
  uint8_t rx = spiTransfer(r);
  char rx_binary[9];
  byteToBinary(rx, rx_binary);
  ESP_LOGI("setNextOperation", "Response: 0x%02X (0b%s)", rx, rx_binary);
  ESP_LOGI("setNextOperation", "========================\n");
  // Set tất cả CS pin về HIGH sau khi hoàn thành
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(
      2 / portTICK_PERIOD_MS); // Tăng delay để đảm bảo CS HIGH được nhận diện
}

void AD770X::writeClockRegister(uint8_t CLKDIS, uint8_t CLKDIV,
                                uint8_t outputUpdateRate, CS_t cs) {
  uint8_t r = (CLKDIS << 4) | (CLKDIV << 3) | outputUpdateRate;
  r &= ~(1 << 2); // Clear CLK bit

  const char *rate_name = (outputUpdateRate == UPDATE_RATE_20)    ? "20Hz"
                          : (outputUpdateRate == UPDATE_RATE_25)  ? "25Hz"
                          : (outputUpdateRate == UPDATE_RATE_100) ? "100Hz"
                          : (outputUpdateRate == UPDATE_RATE_200) ? "200Hz"
                          : (outputUpdateRate == UPDATE_RATE_50)  ? "50Hz"
                          : (outputUpdateRate == UPDATE_RATE_60)  ? "60Hz"
                          : (outputUpdateRate == UPDATE_RATE_250) ? "250Hz"
                          : (outputUpdateRate == UPDATE_RATE_500) ? "500Hz"
                                                                  : "UNKNOWN";

  ESP_LOGI("writeClockRegister", "=== writeClockRegister ===");
  ESP_LOGI("writeClockRegister",
           "CLKDIS: %d, CLKDIV: %d, UpdateRate: %s (0x%02X), CS: %d", CLKDIS,
           CLKDIV, rate_name, outputUpdateRate, cs);
  char clock_binary[9];
  byteToBinary(r, clock_binary);
  ESP_LOGI("writeClockRegister", "Clock register byte: 0x%02X (0b%s)", r,
           clock_binary);

  // Đảm bảo tất cả CS pin về HIGH trước khi set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Sau đó mới set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(2 / portTICK_PERIOD_MS);
  uint8_t rx = spiTransfer(r);
  char rx_binary[9];
  byteToBinary(rx, rx_binary);
  ESP_LOGI("writeClockRegister", "Response: 0x%02X (0b%s)", rx, rx_binary);
  ESP_LOGI("writeClockRegister", "==========================\n");
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(
      2 / portTICK_PERIOD_MS); // Tăng delay để đảm bảo CS HIGH được nhận diện
}

void AD770X::writeSetupRegister(uint8_t operationMode, uint8_t gain,
                                uint8_t unipolar, uint8_t buffered,
                                uint8_t fsync, CS_t cs) {
  gpio_num_t cs_pin = (cs == CS_1)   ? this->cs_pin_1
                      : (cs == CS_2) ? this->cs_pin_2
                      : (cs == CS_3) ? this->cs_pin_3
                                     : this->cs_pin_4;
  uint8_t r = (operationMode << 6) | (gain << 3) | (unipolar << 2) |
              (buffered << 1) | fsync;

  const char *mode_name =
      (operationMode == MODE_NORMAL)           ? "NORMAL"
      : (operationMode == MODE_SELF_CAL)       ? "SELF_CAL"
      : (operationMode == MODE_ZERO_SCALE_CAL) ? "ZERO_SCALE_CAL"
      : (operationMode == MODE_FULL_SCALE_CAL) ? "FULL_SCALE_CAL"
                                               : "UNKNOWN";
  const char *gain_name = (gain == GAIN_1)     ? "1x"
                          : (gain == GAIN_2)   ? "2x"
                          : (gain == GAIN_4)   ? "4x"
                          : (gain == GAIN_8)   ? "8x"
                          : (gain == GAIN_16)  ? "16x"
                          : (gain == GAIN_32)  ? "32x"
                          : (gain == GAIN_64)  ? "64x"
                          : (gain == GAIN_128) ? "128x"
                                               : "UNKNOWN";

  ESP_LOGI("writeSetupRegister", "=== writeSetupRegister ===");
  ESP_LOGI("writeSetupRegister",
           "Mode: %s (0x%02X), Gain: %s (0x%02X), Unipolar: %d, Buffered: %d,"
           "FSYNC: %d, CS: %d",
           mode_name, operationMode, gain_name, gain, unipolar, buffered, fsync,
           cs);
  char setup_binary[9];
  byteToBinary(r, setup_binary);
  ESP_LOGI("writeSetupRegister", "Setup register byte: 0x%02X (0b%s)", r,
           setup_binary);

  // Đảm bảo tất cả CS pin về HIGH trước khi set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Sau đó mới set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(2 / portTICK_PERIOD_MS);
  uint8_t rx = spiTransfer(r);
  char rx_binary[9];
  byteToBinary(rx, rx_binary);
  ESP_LOGI("writeSetupRegister", "Response: 0x%02X (0b%s)", rx, rx_binary);
  ESP_LOGI("writeSetupRegister", "===========================\n");
  // Set tất cả CS pin về HIGH sau khi hoàn thành
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(
      2 / portTICK_PERIOD_MS); // Tăng delay để đảm bảo CS HIGH được nhận diện
}

// Đọc kết quả ADC 16 - bit.
// Quản lý CS : set tất cả HIGH, chỉ CS được chọn LOW, đọc 2 bytes(MSB, LSB),
// ghép thành 16 - bit,rồi set tất cả HIGH lại.

uint16_t AD770X::readADResult(CS_t cs) {
  uint8_t b1, b2;

  ESP_LOGI("readADResult", "=== readADResult ===");
  ESP_LOGI("readADResult", "CS: %d", cs);

  // Set tất cả CS pin: chỉ pin được chọn xuống LOW, các pin khác lên HIGH
  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(1 / portTICK_PERIOD_MS);
  ESP_LOGI("readADResult", "Reading MSB (sending 0x00)...");
  b1 = spiTransfer(0x0);
  ESP_LOGI("readADResult", "MSB received: 0x%02X (0b%08b)", b1, b1);
  ESP_LOGI("readADResult", "Reading LSB (sending 0x00)...");
  b2 = spiTransfer(0x0);
  ESP_LOGI("readADResult", "LSB received: 0x%02X (0b%08b)", b2, b2);
  uint16_t result = (b1 << 8) | b2;
  ESP_LOGI("readADResult", "Combined result: 0x%04X (%u decimal)", result,
           result);
  ESP_LOGI("readADResult", "===================\n");

  // Set tất cả CS pin về HIGH sau khi đọc xong để đảm bảo an toàn
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  return result;
}

uint16_t AD770X::readADResultRaw(uint8_t channel, CS_t cs) {
  TickType_t timeout =
      xTaskGetTickCount() + pdMS_TO_TICKS(300); // 500ms timeout

  while (!dataReady(channel, cs)) {

    if (xTaskGetTickCount() > timeout) {

      ESP_LOGE(TAG, "DRDY timeout on channel %d, CS %d. Resetting...", channel,
               cs);
      return 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); // Yield to avoid blocking
  }
  setNextOperation(REG_DATA, channel, 1, cs);
  return readADResult(cs);
}

double AD770X::readADResult(CS_t cs, uint8_t channel, float refOffset) {
  return ((readADResultRaw(channel, cs) * 1.0 / 65536.0) * VRef) - refOffset;
}

bool AD770X::dataReady(uint8_t channel, CS_t cs) {
  ESP_LOGI("dataReady", "=== dataReady ===");
  ESP_LOGI("dataReady", "Channel: %d, CS: %d", channel, cs);

  // Đảm bảo tất cả CS pin về HIGH trước khi set CS cần thiết xuống LOW
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Set CS cần thiết xuống LOW và giữ LOW trong suốt quá trình đọc
  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Gửi command byte để set next operation (đọc Communication Register)
  uint8_t cmd = (REG_CMM << 4) | (1 << 3) | channel; // READ operation
  uint8_t cmd_rx = spiTransfer(cmd);
  ESP_LOGI("dataReady", "Command byte: 0x%02X, Response: 0x%02X", cmd, cmd_rx);

  vTaskDelay(5 / portTICK_PERIOD_MS);
  ESP_LOGI("dataReady", "Reading Communication Register (sending 0x00)...");
  uint8_t b1 = spiTransfer(0x0);
  char b1_binary[9];
  byteToBinary(b1, b1_binary);
  ESP_LOGI("dataReady", "Response: 0x%02X (0b%s)", b1, b1_binary);
  bool ready = (b1 & 0x80) == 0x0;
  ESP_LOGI("dataReady", "DRDY bit (bit 7): %s (%s)",
           (b1 & 0x80) ? "HIGH (not ready)" : "LOW (ready)",
           ready ? "READY" : "NOT READY");
  ESP_LOGI("dataReady", "================\n");
  // Set tất cả CS pin về HIGH sau khi hoàn thành
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(
      5 / portTICK_PERIOD_MS); // Tăng delay để đảm bảo CS HIGH được nhận diện
  return ready;
}

void AD770X::reset(CS_t cs) {

  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);

  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(130 / portTICK_PERIOD_MS);
  for (int i = 0; i < 100; i++) {
    spiTransfer(0xFF);
  }
  // Set tất cả CS pin về HIGH sau khi reset
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

void AD770X::resetHard(CS_t cs) {
  gpio_num_t reset_pin = (cs == CS_1)   ? this->reset_pin_1
                         : (cs == CS_2) ? this->reset_pin_2
                         : (cs == CS_3) ? this->reset_pin_3
                                        : this->reset_pin_4;
  gpio_set_level(reset_pin, 1);
  vTaskDelay(15 / portTICK_PERIOD_MS);
  gpio_set_level(reset_pin, 0);
  vTaskDelay(15 / portTICK_PERIOD_MS);
  gpio_set_level(reset_pin, 1);
  vTaskDelay(15 / portTICK_PERIOD_MS);
}

void AD770X::init(uint8_t channel, CS_t cs) {
  init(channel, CLK_DIV_1, BIPOLAR, GAIN_1, UPDATE_RATE_50, cs);
}

/**
 * @brief Initializes the AD770X ADC for a specified channel with given
 * parameters.
 *
 * @param channel The ADC channel to initialize (e.g., AIN1, AIN2, etc.).
 * @param clkDivider The clock divider setting for the ADC.
 * @param polarity The polarity setting (bipolar or unipolar) for the ADC input.
 * @param gain The gain setting for the ADC input.
 * @param updRate The update rate for the ADC data output.
 *
 * This function sets up the clock and setup registers of the AD770X for
 * the specified channel, and performs a self-calibration. It waits until
 * the ADC is ready to provide data.
 */

void AD770X::init(uint8_t channel, uint8_t clkDivider, uint8_t polarity,
                  uint8_t gain, uint8_t updRate, CS_t cs) {
  TickType_t timeout =
      xTaskGetTickCount() + pdMS_TO_TICKS(1000); // 1s timeout for calibration
  bool timedOut = false;
  do {
    setNextOperation(REG_CLOCK, channel, 0, cs);
    writeClockRegister(0, clkDivider, updRate, cs);
    setNextOperation(REG_SETUP, channel, 0, cs);
    writeSetupRegister(MODE_SELF_CAL, gain, polarity, 1, 0, cs);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Waiting for self-calibration to complete...Channel%s %s",
             channel == CHN_AIN1 ? "1" : "2",
             cs == CS_1   ? "CS_1"
             : cs == CS_2 ? "CS_2"
             : cs == CS_3 ? "CS_3"
                          : "CS_4");
    if (xTaskGetTickCount() > timeout) {
      timedOut = true;
      ESP_LOGE(TAG, "Self-calibration timeout on channel %d, CS %d", channel,
               cs);
      break;
    }
  } while (!dataReady(channel, cs));
  if (!timedOut) {
    ESP_LOGI(TAG, "Self-calibration completed for channel %d, CS %d", channel,
             cs); // chanel 1 == AIN 1 (dòng điện), chanel 2 == AIN 2 (điện áp)
  } else {
    // Reset AD7705 sau calibration timeout để tránh stuck state
    ESP_LOGW(TAG,
             "Resetting AD7705 after calibration timeout (channel %d, CS %d)",
             channel, cs);
    resetHard(cs);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Đảm bảo tất cả CS pin về HIGH
    gpio_set_level(cs_pin_1, 1);
    gpio_set_level(cs_pin_2, 1);
    gpio_set_level(cs_pin_3, 1);
    gpio_set_level(cs_pin_4, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void AD770X::deselectAll() {
  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);
}

/**
 * @brief Test giao tiếp với AD7705
 *
 * Hàm này kiểm tra xem ESP32 có thể giao tiếp với AD7705 không bằng cách:
 * 1. Đọc Communication Register nhiều lần
 * 2. Kiểm tra giá trị nhận được có hợp lệ không
 * 3. Thử write/read Setup register
 *
 * @param cs Chip Select pin (CS_1, CS_2, CS_3, CS_4)
 * @return true nếu giao tiếp thành công, false nếu thất bại
 */
bool AD770X::testCommunication(CS_t cs) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Testing communication with AD7705");
  ESP_LOGI(TAG, "CS: %d", cs);
  ESP_LOGI(TAG, "========================================");

  bool testPassed = false;
  uint8_t responses[10];
  bool allZero = true;
  bool allFF = true;
  bool valuesChange = false;

  // Test 1: Đọc Communication Register nhiều lần
  ESP_LOGI(TAG, "\n--- Test 1: Reading Communication Register ---");
  resetHard(cs);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  for (int i = 0; i < 10; i++) {
    setNextOperation(REG_CMM, CHN_AIN1, 1, cs);

    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
    gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

    vTaskDelay(1 / portTICK_PERIOD_MS);
    responses[i] = spiTransfer(0x00);

    gpio_set_level(cs_pin_1, 1);
    gpio_set_level(cs_pin_2, 1);
    gpio_set_level(cs_pin_3, 1);
    gpio_set_level(cs_pin_4, 1);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "  Read %d: 0x%02X (0b%08b)", i + 1, responses[i],
             responses[i]);

    if (responses[i] != 0x00)
      allZero = false;
    if (responses[i] != 0xFF)
      allFF = false;
    if (i > 0 && responses[i] != responses[i - 1])
      valuesChange = true;
  }

  ESP_LOGI(TAG, "\n--- Analysis ---");
  ESP_LOGI(TAG, "All zeros: %s",
           allZero ? "YES (BAD - No communication)" : "NO");
  ESP_LOGI(TAG, "All 0xFF: %s", allFF ? "YES (BAD - No communication)" : "NO");
  ESP_LOGI(TAG, "Values change: %s", valuesChange ? "YES (GOOD)" : "NO");

  if (allZero || allFF) {
    ESP_LOGE(TAG, "❌ FAILED: Always receiving 0x%02X - No SPI communication!",
             allZero ? 0x00 : 0xFF);
    ESP_LOGE(TAG, "   Check: MOSI, MISO, SCK connections");
    ESP_LOGE(TAG, "   Check: CS pin is working");
    ESP_LOGE(TAG, "   Check: Power supply to AD7705");
    return false;
  }

  if (!valuesChange) {
    ESP_LOGW(TAG, "⚠️  WARNING: All reads return same value (0x%02X)",
             responses[0]);
    ESP_LOGW(TAG, "   ADC might be stuck or not responding properly");
  }

  // Test 2: Write và Read lại Setup Register
  ESP_LOGI(TAG, "\n--- Test 2: Write/Read Setup Register ---");
  resetHard(cs);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  // Write test value
  uint8_t testValue = 0x45; // MODE_NORMAL, GAIN_2, BIPOLAR, etc.
  setNextOperation(REG_SETUP, CHN_AIN1, 0, cs);

  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(1 / portTICK_PERIOD_MS);
  uint8_t writeResponse = spiTransfer(testValue);

  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);

  vTaskDelay(10 / portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "  Wrote: 0x%02X, Response: 0x%02X", testValue, writeResponse);

  // Read back
  setNextOperation(REG_SETUP, CHN_AIN1, 1, cs);

  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(1 / portTICK_PERIOD_MS);
  uint8_t readBack = spiTransfer(0x00);

  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);

  vTaskDelay(10 / portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "  Read back: 0x%02X", readBack);

  // Test 3: Kiểm tra DRDY bit
  ESP_LOGI(TAG, "\n--- Test 3: Check DRDY bit ---");
  bool drdyStatus = dataReady(CHN_AIN1, cs);
  ESP_LOGI(TAG, "  DRDY status: %s", drdyStatus ? "READY" : "NOT READY");

  // Tổng kết
  ESP_LOGI(TAG, "\n========================================");
  ESP_LOGI(TAG, "Test Summary:");
  ESP_LOGI(TAG, "  SPI Communication: %s",
           (allZero || allFF) ? "❌ FAILED" : "✅ OK");
  ESP_LOGI(TAG, "  Register Write/Read: %s",
           (readBack == testValue || readBack != 0x00) ? "✅ OK" : "⚠️  CHECK");
  ESP_LOGI(TAG, "  DRDY Check: %s", drdyStatus ? "✅ READY" : "⚠️  NOT READY");

  if (!allZero && !allFF) {
    testPassed = true;
    ESP_LOGI(TAG, "\n✅ Communication test PASSED!");
    ESP_LOGI(TAG, "   ESP32 can communicate with AD7705");
  } else {
    ESP_LOGE(TAG, "\n❌ Communication test FAILED!");
    ESP_LOGE(TAG, "   ESP32 cannot communicate with AD7705");
    ESP_LOGE(TAG, "   Please check hardware connections");
  }
  ESP_LOGI(TAG, "========================================\n");

  return testPassed;
}

/**
 * @brief Debug kiểm tra phần cứng SPI
 *
 * Hàm này kiểm tra các vấn đề phần cứng có thể gây ra lỗi giao tiếp:
 * 1. Kiểm tra CS pin có hoạt động không
 * 2. Kiểm tra MISO pin có kết nối không
 * 3. Kiểm tra SPI bus có hoạt động không
 * 4. Đề xuất các bước khắc phục
 */
void AD770X::debugHardware(CS_t cs) {
  ESP_LOGI(TAG, "\n========================================");
  ESP_LOGI(TAG, "HARDWARE DEBUG - Board %d", cs);
  ESP_LOGI(TAG, "========================================\n");

  gpio_num_t cs_pin = (cs == CS_1)   ? this->cs_pin_1
                      : (cs == CS_2) ? this->cs_pin_2
                      : (cs == CS_3) ? this->cs_pin_3
                                     : this->cs_pin_4;
  gpio_num_t reset_pin = (cs == CS_1)   ? this->reset_pin_1
                         : (cs == CS_2) ? this->reset_pin_2
                         : (cs == CS_3) ? this->reset_pin_3
                                        : this->reset_pin_4;

  // Test 1: Kiểm tra CS pin
  ESP_LOGI(TAG, "--- Test 1: CS Pin Control ---");
  ESP_LOGI(TAG, "CS Pin: GPIO %d", cs_pin);

  // Kiểm tra xem pin có được cấu hình đúng không
  // Lưu ý: gpio_get_level() trên OUTPUT pin sẽ trả về giá trị đã set,
  // không phải giá trị thực tế trên pin. Điều này là bình thường.

  // Set HIGH
  gpio_set_level(cs_pin, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS); // Tăng delay để đảm bảo pin đã set
  int level_high = gpio_get_level(cs_pin);
  ESP_LOGI(TAG, "  Set HIGH -> Read: %d (expected: 1)", level_high);
  ESP_LOGI(TAG, "  Note: gpio_get_level() on OUTPUT pin returns set value");

  // Set LOW
  gpio_set_level(cs_pin, 0);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  int level_low = gpio_get_level(cs_pin);
  ESP_LOGI(TAG, "  Set LOW  -> Read: %d (expected: 0)", level_low);

  // Set HIGH lại
  gpio_set_level(cs_pin, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  // Lưu ý: Trong ESP32, gpio_get_level() trên OUTPUT pin sẽ trả về giá trị đã
  // set. Nếu pin được cấu hình đúng, nó sẽ trả về giá trị đúng. Nếu trả về 0
  // khi đã set HIGH, có thể pin không được cấu hình đúng hoặc có conflict.
  if (level_high != 1 || level_low != 0) {
    ESP_LOGW(TAG, "  ⚠️  CS Pin level read mismatch!");
    ESP_LOGW(TAG, "     This may be normal if pin is configured correctly");
    ESP_LOGW(TAG, "     Check hardware connection with multimeter");
  } else {
    ESP_LOGI(TAG, "  ✅ CS Pin level read OK (software check)");
    ESP_LOGI(TAG,
             "     Note: Verify with multimeter for hardware confirmation");
  }

  // Test 2: Kiểm tra Reset pin
  ESP_LOGI(TAG, "\n--- Test 2: Reset Pin Control ---");
  ESP_LOGI(TAG, "Reset Pin: GPIO %d", reset_pin);

  gpio_set_level(reset_pin, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  int reset_high = gpio_get_level(reset_pin);
  ESP_LOGI(TAG, "  Set HIGH -> Read: %d (expected: 1)", reset_high);

  gpio_set_level(reset_pin, 0);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  int reset_low = gpio_get_level(reset_pin);
  ESP_LOGI(TAG, "  Set LOW  -> Read: %d (expected: 0)", reset_low);

  gpio_set_level(reset_pin, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  if (reset_high != 1 || reset_low != 0) {
    ESP_LOGW(TAG, "  ⚠️  Reset Pin level read mismatch!");
    ESP_LOGW(TAG, "     This may be normal if pin is configured correctly");
    ESP_LOGW(TAG, "     Check hardware connection with multimeter");
  } else {
    ESP_LOGI(TAG, "  ✅ Reset Pin level read OK (software check)");
    ESP_LOGI(TAG,
             "     Note: Verify with multimeter for hardware confirmation");
  }

  // Test 3: Kiểm tra MISO pin (có thể đọc được không)
  ESP_LOGI(TAG, "\n--- Test 3: MISO Pin Check ---");
  ESP_LOGI(TAG, "MISO Pin: GPIO %d", DEFAULT_MISO_PIN);
  ESP_LOGI(TAG, "  Note: MISO should be connected to AD7705 DOUT pin");
  ESP_LOGI(TAG, "  If MISO is floating, ESP32 will read 0x00");

  // Test 4: Thử reset AD7705 và đọc lại
  ESP_LOGI(TAG, "\n--- Test 4: AD7705 Reset Test ---");
  ESP_LOGI(TAG, "Performing hard reset...");
  resetHard(cs);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Thử đọc Communication Register sau reset
  setNextOperation(REG_CMM, CHN_AIN1, 1, cs);

  gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);
  gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);
  gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
  gpio_set_level(cs_pin_4, cs == CS_4 ? 0 : 1);

  vTaskDelay(5 / portTICK_PERIOD_MS);
  uint8_t test_read = spiTransfer(0x00);

  gpio_set_level(cs_pin_1, 1);
  gpio_set_level(cs_pin_2, 1);
  gpio_set_level(cs_pin_3, 1);
  gpio_set_level(cs_pin_4, 1);

  ESP_LOGI(TAG, "  After reset, read: 0x%02X", test_read);
  if (test_read == 0x00) {
    ESP_LOGE(TAG, "  ❌ Still reading 0x00 - No communication!");
  } else {
    ESP_LOGI(TAG, "  ✅ Got response: 0x%02X", test_read);
  }

  // Tổng kết và đề xuất
  ESP_LOGI(TAG, "\n========================================");
  ESP_LOGI(TAG, "DEBUG SUMMARY & RECOMMENDATIONS");
  ESP_LOGI(TAG, "========================================");

  if (test_read == 0x00) {
    ESP_LOGE(TAG, "\n❌ CRITICAL: ESP32 always receives 0x00 from AD7705");
    ESP_LOGE(TAG, "\n📋 DIAGNOSIS:");
    ESP_LOGE(TAG, "   This indicates NO SPI communication with AD7705");
    ESP_LOGE(TAG, "   MISO pin is likely floating (not connected)");
    ESP_LOGE(TAG, "\n🔍 MOST LIKELY CAUSES (in order):");
    ESP_LOGE(TAG, "   1. MISO pin (GPIO %d) NOT connected to AD7705 DOUT",
             DEFAULT_MISO_PIN);
    ESP_LOGE(TAG, "   2. AD7705 NOT powered (VDD missing or wrong voltage)");
    ESP_LOGE(TAG, "   3. AD7705 chip damaged or wrong model");
    ESP_LOGE(TAG, "   4. CS pin (GPIO %d) not properly controlling AD7705",
             cs_pin);
    ESP_LOGE(TAG, "\n✅ HARDWARE CHECKLIST:");
    ESP_LOGE(TAG, "   [1] Measure MISO (GPIO %d) voltage with multimeter",
             DEFAULT_MISO_PIN);
    ESP_LOGE(TAG, "       - Should be ~1.65V if floating (pulled to mid-rail)");
    ESP_LOGE(TAG, "       - Should change during SPI transaction if connected");
    ESP_LOGE(TAG, "   [2] Verify MISO (GPIO %d) → AD7705 DOUT (pin 15)",
             DEFAULT_MISO_PIN);
    ESP_LOGE(TAG, "   [3] Verify MOSI (GPIO %d) → AD7705 DIN (pin 16)",
             DEFAULT_MOSI_PIN);
    ESP_LOGE(TAG, "   [4] Verify SCK  (GPIO %d) → AD7705 SCLK (pin 17)",
             DEFAULT_SCK_PIN);
    ESP_LOGE(TAG, "   [5] Verify CS   (GPIO %d) → AD7705 CS (pin 18)", cs_pin);
    ESP_LOGE(TAG, "   [6] Measure AD7705 VDD (pin 1) = 3.3V or 5V");
    ESP_LOGE(TAG, "   [7] Verify AD7705 GND (pin 9) connected to ESP32 GND");
    ESP_LOGE(TAG, "   [8] Verify AD7705 VREF (pin 2) is connected");
    ESP_LOGE(TAG,
             "   [9] Check AD7705 chip marking (should be AD7705, not AD7706)");
    ESP_LOGE(TAG, "\n⚙️  SOFTWARE TRIES:");
    ESP_LOGE(TAG, "   [ ] Try slower SPI clock (100kHz instead of 500kHz)");
    ESP_LOGE(TAG, "   [ ] Try SPI Mode 3 instead of Mode 2");
    ESP_LOGE(TAG, "   [ ] Add longer delays between SPI transactions");
  } else {
    ESP_LOGI(TAG, "\n✅ Hardware communication detected!");
    ESP_LOGI(TAG, "   Received: 0x%02X (not 0x00)", test_read);
    ESP_LOGI(TAG, "   SPI connection seems OK");
    ESP_LOGI(TAG, "   If still having issues, check:");
    ESP_LOGI(TAG, "   - SPI mode (currently Mode 2)");
    ESP_LOGI(TAG, "   - Timing delays");
    ESP_LOGI(TAG, "   - AD7705 register configuration");
  }

  ESP_LOGI(TAG, "========================================\n");
}