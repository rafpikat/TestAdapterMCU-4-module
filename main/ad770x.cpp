#include "ad770x.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "AD770X";

AD770X::AD770X(double vref, spi_host_device_t spi_host, 
    gpio_num_t cs_pin_1, 
    gpio_num_t cs_pin_2,
    gpio_num_t cs_pin_3,
    gpio_num_t reset_pin_1,
    gpio_num_t reset_pin_2,
    gpio_num_t reset_pin_3) 
    : VRef(vref), spi_host(spi_host), 
    cs_pin_1(cs_pin_1), cs_pin_2(cs_pin_2), cs_pin_3(cs_pin_3),
    reset_pin_1(reset_pin_1), reset_pin_2(reset_pin_2), reset_pin_3(reset_pin_3) {
    // Configure GPIO for CS
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << cs_pin_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_config_t io_conf_2 = {
        .pin_bit_mask = (1ULL << cs_pin_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_2));

    gpio_config_t io_conf_3 = {
        .pin_bit_mask = (1ULL << cs_pin_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_3));
    // Configure GPIO for Reset
    gpio_config_t io_rs_conf = {
        .pin_bit_mask = (1ULL << reset_pin_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_rs_conf));

    gpio_config_t io_rs_conf_2 = {
        .pin_bit_mask = (1ULL << reset_pin_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_rs_conf_2));

    gpio_config_t io_rs_conf_3 = {
        .pin_bit_mask = (1ULL << reset_pin_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_rs_conf_3));
    // Configure GPIO for LED Calibration Status
    gpio_config_t io_led_conf = {
        .pin_bit_mask = (1ULL << LED_CALIB_STATUS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_led_conf));
    gpio_set_level(cs_pin_1, 1); // CS 1 high (inactive)
    gpio_set_level(cs_pin_2, 1); // CS 2 high (inactive)
    gpio_set_level(cs_pin_3, 1); // CS 3 high (inactive)
    gpio_set_level(reset_pin_1, 1); // CS high (inactive)
    gpio_set_level(reset_pin_2, 1); // CS high (inactive)
    gpio_set_level(reset_pin_3, 1); // CS high (inactive)
    gpio_set_level(LED_CALIB_STATUS, 0); // LED Calibration Status low (inactive)
    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = DEFAULT_MOSI_PIN,
        .miso_io_num = DEFAULT_MISO_PIN,
        .sclk_io_num = DEFAULT_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    ESP_ERROR_CHECK(spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO));

    // Add SPI device
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 2, 
        .clock_speed_hz = 1 * 5 * 1000, // 500kHz
        .spics_io_num = -1, // CS handled manually
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &devcfg, &spi_device));
}

AD770X::~AD770X() {
    spi_bus_remove_device(spi_device);
    spi_bus_free(spi_host);
}

uint8_t AD770X::spiTransfer(uint8_t data) {
    uint8_t ret = 0;
    spi_transaction_t t = {
        .length = 8, // 8 bits
        .rxlength = 8,
        .tx_buffer = &data,
        .rx_buffer = &ret
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device, &t));
    return ret;
}

void AD770X::setNextOperation(uint8_t reg, uint8_t channel, uint8_t readWrite, CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;
    uint8_t r = (reg << 4) | (readWrite << 3) | channel;

    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);


	vTaskDelay(1 / portTICK_PERIOD_MS);
    spiTransfer(r);
    gpio_set_level(cs_pin, 1); // CS high
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void AD770X::writeClockRegister(uint8_t CLKDIS, uint8_t CLKDIV, uint8_t outputUpdateRate, CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;
    uint8_t r = (CLKDIS << 4) | (CLKDIV << 3) | outputUpdateRate;
    r &= ~(1 << 2); // Clear CLK bit
    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
    spiTransfer(r);
    gpio_set_level(cs_pin, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void AD770X::writeSetupRegister(uint8_t operationMode, uint8_t gain, uint8_t unipolar, uint8_t buffered, uint8_t fsync, CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;
    uint8_t r = (operationMode << 6) | (gain << 3) | (unipolar << 2) | (buffered << 1) | fsync;
    
    
    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);

	vTaskDelay(1 / portTICK_PERIOD_MS);
    spiTransfer(r);
    gpio_set_level(cs_pin, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

uint16_t AD770X::readADResult(CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;
    uint8_t b1, b2;
    
    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);

	vTaskDelay(1 / portTICK_PERIOD_MS);
    b1 = spiTransfer(0x0);
    b2 = spiTransfer(0x0);
    gpio_set_level(cs_pin, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
    return (b1 << 8) | b2;
}

uint16_t AD770X::readADResultRaw(uint8_t channel, CS_t cs) {
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(300); // 500ms timeout


    while (!dataReady(channel, cs)) {
        
        if (xTaskGetTickCount() > timeout) {

            ESP_LOGE(TAG, "DRDY timeout on channel %d, CS %d. Resetting...", channel, cs);
            return 0;

        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to avoid blocking
    }
    setNextOperation(REG_DATA, channel, 1, cs);
    return readADResult(cs);
}


double AD770X::readADResult(CS_t cs, uint8_t channel, float refOffset) {
    return ((readADResultRaw(channel, cs) * 1.0 / 65536.0) * VRef) - refOffset;
}

bool AD770X::dataReady(uint8_t channel, CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;
    setNextOperation(REG_CMM, channel, 1, cs);

    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);

	vTaskDelay(1 / portTICK_PERIOD_MS);
    uint8_t b1 = spiTransfer(0x0);
    gpio_set_level(cs_pin, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
    return (b1 & 0x80) == 0x0;
}

void AD770X::reset(CS_t cs) {
    gpio_num_t cs_pin = (cs == CS_1) ? this->cs_pin_1 : 
                          (cs == CS_2) ? this->cs_pin_2 : 
                          this->cs_pin_3;

    gpio_set_level(cs_pin_1, cs == CS_1 ? 0 : 1);

    gpio_set_level(cs_pin_2, cs == CS_2 ? 0 : 1);

    gpio_set_level(cs_pin_3, cs == CS_3 ? 0 : 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
    for (int i = 0; i < 100; i++) {
        spiTransfer(0xFF);
    }
    gpio_set_level(cs_pin, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void AD770X::resetHard(CS_t cs) {
    gpio_num_t reset_pin = (cs == CS_1) ? this->reset_pin_1 : 
                            (cs == CS_2) ? this->reset_pin_2 : 
                            this->reset_pin_3;
    gpio_set_level(reset_pin, 1);
	vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(reset_pin, 0);
	vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(reset_pin, 1);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

void AD770X::init(uint8_t channel, CS_t cs) {
    init(channel, CLK_DIV_1, BIPOLAR, GAIN_2, UPDATE_RATE_50, cs);
}

/**
 * @brief Initializes the AD770X ADC for a specified channel with given parameters.
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

void AD770X::init(uint8_t channel, uint8_t clkDivider, uint8_t polarity, uint8_t gain, uint8_t updRate, CS_t cs) {
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(1000); // 1s timeout for calibration
    bool timedOut = false;
    do {
        setNextOperation(REG_CLOCK, channel, 0, cs);
        writeClockRegister(0, clkDivider, updRate, cs);
        setNextOperation(REG_SETUP, channel, 0, cs);
        writeSetupRegister(MODE_SELF_CAL, gain, polarity, 0, 0, cs);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for self-calibration to complete...Channel%s %s", channel == CHN_AIN1 ? "1" : "2", cs == CS_1 ? "CS_1" : 
                          cs == CS_2 ? "CS_2" : "CS_3");
        if (xTaskGetTickCount() > timeout) {
            timedOut = true;
            ESP_LOGE(TAG, "Self-calibration timeout on channel %d, CS %d", channel, cs);
            break;
        }
    } while (!dataReady(channel, cs));
    if (!timedOut) {
        ESP_LOGI(TAG, "Self-calibration completed for channel %d, CS %d", channel, cs);
    }
}