#ifndef __ADC_MEASURE_H__
#define __ADC_MEASURE_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ad770x.h"
#include "kalman.h"
#include "esp_log.h"
#include "nvs_flash.h" 

// Định nghĩa board và chân GPIO
#define BOARD_1         0x00
#define BOARD_2         0x01
#define BOARD_3         0x02
#define CS_PIN_1        GPIO_NUM_5
#define CS_PIN_2        GPIO_NUM_16
#define CS_PIN_3        GPIO_NUM_14
#define RESET_PIN_1     GPIO_NUM_4
#define RESET_PIN_2     GPIO_NUM_15
#define RESET_PIN_3     GPIO_NUM_22

#define RES_TD_1        145.5f
#define RES_TD_2        145.5f
#define RES_TD_3        145.5f
#define RES_FB_1        2.362f
#define RES_FB_2        2.368f
#define RES_FB_3        2.368f
// #define VREF3           2.475f
#define VREF3           2.489f
// #define VREF2           2.48f
#define VREF2           2.56f
// #define VREF1           2.368f // Sử dụng VREF1 là 2.5V-90
#define VREF1           5.649f

// #define VREF3_VOLTAGE   2.53f
#define VREF3_VOLTAGE   2.489f
// #define VREF2_VOLTAGE   2.51f
#define VREF2_VOLTAGE   2.489f
// #define VREF1_VOLTAGE   2.5f
#define VREF1_VOLTAGE   2.495f

// Macro chuyển đổi ADC sang điện áp/dòng điện
#define CONVERT_ADC1_TO_VOL(raw) (((raw * VREF1_VOLTAGE) / 65536.0) / (RES_FB_1 / RES_TD_1))
#define CONVERT_ADC2_TO_VOL(raw) (((raw * VREF2_VOLTAGE) / 65536.0) / (RES_FB_2 / RES_TD_2))
#define CONVERT_ADC3_TO_VOL(raw) (((raw * VREF3_VOLTAGE) / 65536.0) / (RES_FB_3 / RES_TD_3))
#define CONVERT_ADC1_TO_CURRENT(raw) (((raw * VREF1) / 65536.0) / 0.025)
#define CONVERT_ADC2_TO_CURRENT(raw) (((raw * VREF2) / 65536.0) / 0.025)
#define CONVERT_ADC3_TO_CURRENT(raw) (((raw * VREF3) / 65536.0) / 0.025)

class ADCMeasure {
public:
    // Constructor
    ADCMeasure(float vref, spi_host_device_t spi_host);

    // Khởi tạo ADC và các chân
    esp_err_t init(uint8_t board_id, gpio_num_t cs_pin, gpio_num_t reset_pin);

    // Bắt đầu task quét ADC
    esp_err_t startADCTask();

    // Đọc giá trị điện áp (đã lọc Kalman)
    double getVoltage(uint8_t board_id);

    // Đặt giá trị điện áp (chưa lọc)
    void setVoltage(uint8_t board_id, float voltageValue);

    // Đọc giá trị dòng điện (đã lọc Kalman)
    double getCurrent(uint8_t board_id);

    // Đặt giá trị dòng điện
    void setCurrent(uint8_t board_id, float current);

    // Đặt offset cho dòng điện
    void setCurrentOffset(uint8_t board_id, uint16_t offset);
    
    uint16_t getCurrentOffset(uint8_t board_id);
    // Đối tượng AD770X
    AD770X adc;
    
    TaskHandle_t adcTaskHandle;

    // Lưu currentOffset vào flash
    esp_err_t saveCurrentOffsetToFlash();

    // Đọc currentOffset từ flash
    esp_err_t loadCurrentOffsetFromFlash();

    void CalibCurrentZero(void)
    {
        isBoardCalibDone = false;
    }
private:

    // Bộ lọc Kalman cho điện áp và dòng điện
    KalmanFilter voltageFilter;
    KalmanFilter currentFilter;
    
    bool isBoardCalibDone = true;
    // Ma trận và vector cho bộ lọc Kalman
    float A[4]; // Ma trận chuyển trạng thái (2x2)
    float P[4]; // Ma trận hiệp phương sai (2x2)
    float Q[4]; // Nhiễu quá trình (2x2)
    float H[2]; // Ma trận quan sát (1x2)
    float R[1]; // Nhiễu quan sát (1x1)
    float xp[2]; // Trạng thái dự đoán
    float xc[2]; // Trạng thái hiện tại

    // Offset cho dòng điện
    uint16_t currentOffset[3]; // [board_id][channel]
    // Lưu giá trị thô của điện áp và dòng điện
    double current[3]= {0.0, 0.0, 0.0}; // Lưu giá trị dòng điện đã lọc
    double voltage[3]= {0.0, 0.0, 0.0}; // Lưu giá trị điện áp đã lọc
    // Task handle cho FreeRTOS
    // TaskHandle_t adcTaskHandle;

    // Hàm task quét ADC
    static void adcScanTask(void* arg);

    // Hàm chuyển đổi giá trị thô thành điện áp/dòng điện
    double convertToVoltage(uint8_t board_id, uint16_t raw);
    double convertToCurrent(uint8_t board_id, uint16_t raw);

    // Cấu hình bộ lọc Kalman
    void setupKalmanFilter();
};

#endif // __ADC_MEASURE_H__