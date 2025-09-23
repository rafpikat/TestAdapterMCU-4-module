#ifndef AD770X_H
#define AD770X_H

#include <driver/spi_master.h>
#include <driver/gpio.h>

/*
 * AD7705/AD7706 Library for ESP-IDF
 * Adapted from Kerry D. Wong's Arduino library
 * http://www.kerrywong.com
 * Initial Arduino version 1.0 3/2011, Updated 1.1 4/2012
 * ESP-IDF conversion 2025
 */

// Pin definitions (configurable via constructor)
#define DEFAULT_MOSI_PIN GPIO_NUM_23
#define DEFAULT_MISO_PIN GPIO_NUM_19
#define DEFAULT_SCK_PIN GPIO_NUM_18
#define DEFAULT_CS_PIN GPIO_NUM_5
#define DEFAULT_RESET_PIN GPIO_NUM_4
#define LED_CALIB_STATUS GPIO_NUM_2
class AD770X {
public:
    // Register selection (RS2 RS1 RS0)
    static const uint8_t REG_CMM = 0x0; // Communication register (8-bit)
    static const uint8_t REG_SETUP = 0x1; // Setup register (8-bit)
    static const uint8_t REG_CLOCK = 0x2; // Clock register (8-bit)
    static const uint8_t REG_DATA = 0x3; // Data register (16-bit, conversion result)
    static const uint8_t REG_TEST = 0x4; // Test register (8-bit)
    static const uint8_t REG_NOP = 0x5; // No operation
    static const uint8_t REG_OFFSET = 0x6; // Offset register (24-bit)
    static const uint8_t REG_GAIN = 0x7; // Gain register (24-bit)

    // Channel selection (CH1 CH0)
    static const uint8_t CHN_AIN1 = 0x0; // AIN1; calibration register pair 0
    static const uint8_t CHN_AIN2 = 0x1; // AIN2; calibration register pair 1
    static const uint8_t CHN_COMM = 0x2; // Common; calibration register pair 0
    static const uint8_t CHN_AIN3 = 0x3; // AIN3; calibration register pair 2

    // Output update rate (CLK FS1 FS0)
    static const uint8_t UPDATE_RATE_20 = 0x0; // 20 Hz
    static const uint8_t UPDATE_RATE_25 = 0x1; // 25 Hz
    static const uint8_t UPDATE_RATE_100 = 0x2; // 100 Hz
    static const uint8_t UPDATE_RATE_200 = 0x3; // 200 Hz
    static const uint8_t UPDATE_RATE_50 = 0x4; // 50 Hz
    static const uint8_t UPDATE_RATE_60 = 0x5; // 60 Hz
    static const uint8_t UPDATE_RATE_250 = 0x6; // 250 Hz
    static const uint8_t UPDATE_RATE_500 = 0x7; // 500 Hz

    // Operating mode options (MD1 MD0)
    static const uint8_t MODE_NORMAL = 0x0; // Normal mode
    static const uint8_t MODE_SELF_CAL = 0x1; // Self-calibration
    static const uint8_t MODE_ZERO_SCALE_CAL = 0x2; // Zero-scale system calibration
    static const uint8_t MODE_FULL_SCALE_CAL = 0x3; // Full-scale system calibration

    // Gain settings
    static const uint8_t GAIN_1 = 0x0;
    static const uint8_t GAIN_2 = 0x1;
    static const uint8_t GAIN_4 = 0x2;
    static const uint8_t GAIN_8 = 0x3;
    static const uint8_t GAIN_16 = 0x4;
    static const uint8_t GAIN_32 = 0x5;
    static const uint8_t GAIN_64 = 0x6;
    static const uint8_t GAIN_128 = 0x7;

    static const uint8_t UNIPOLAR = 0x0;
    static const uint8_t BIPOLAR = 0x1;

    static const uint8_t CLK_DIV_1 = 0x1;
    static const uint8_t CLK_DIV_2 = 0x2;

    
    typedef enum {
        CS_1 = 0,  ///< Chip Select 1
        CS_2 = 1,  ///< Chip Select 2
        CS_3 = 2   ///< Chip Select 3
    } CS_t;
    AD770X(double vref, spi_host_device_t spi_host, 
        gpio_num_t cs_pin_1, gpio_num_t cs_pin_2, gpio_num_t cs_pin_3, 
        gpio_num_t reset_pin_1, gpio_num_t reset_pin_2, gpio_num_t reset_pin_3);
    ~AD770X();
    void setNextOperation(uint8_t reg, uint8_t channel, uint8_t readWrite, CS_t cs);
    void writeClockRegister(uint8_t CLKDIS, uint8_t CLKDIV, uint8_t outputUpdateRate, CS_t cs);
    void writeSetupRegister(uint8_t operationMode, uint8_t gain, uint8_t unipolar, uint8_t buffered, uint8_t fsync, CS_t cs);
    uint16_t readADResultRaw(uint8_t channel, CS_t cs);
    double readADResult(CS_t cs, uint8_t channel, float refOffset= 0.0);
    void reset(CS_t cs);
    void resetHard(CS_t cs);
    bool dataReady(uint8_t channel, CS_t cs);
    void init(uint8_t channel, CS_t cs);
    void init(uint8_t channel, uint8_t clkDivider, uint8_t polarity, uint8_t gain, uint8_t updRate, CS_t cs);

private:
    uint8_t spiTransfer(uint8_t data);
    uint16_t readADResult(CS_t cs);
    double VRef;
    spi_host_device_t spi_host;
    spi_device_handle_t spi_device;
    gpio_num_t cs_pin_1;
    gpio_num_t cs_pin_2;
    gpio_num_t cs_pin_3;
    gpio_num_t reset_pin_1;
    gpio_num_t reset_pin_2;
    gpio_num_t reset_pin_3;
};

#endif