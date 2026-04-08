#ifndef __ADC_MEASURE_H__
#define __ADC_MEASURE_H__

#include "ad770x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include "nvs_flash.h"
#include <cmath>
#include <cstring>

// Định nghĩa board và chân GPIOs
#define BOARD_1 0x00
#define BOARD_2 0x01
#define BOARD_3 0x02
#define BOARD_4 0x03

#define CS_PIN_1 GPIO_NUM_5
#define CS_PIN_2 GPIO_NUM_14
#define CS_PIN_3 GPIO_NUM_16
#define CS_PIN_4 GPIO_NUM_32

#define RESET_PIN_1 GPIO_NUM_4
#define RESET_PIN_2 GPIO_NUM_22
#define RESET_PIN_3 GPIO_NUM_15
#define RESET_PIN_4 GPIO_NUM_27

#define R_TOTAL1 (99.1 + 42.7 + 2.35) // kOhm
#define R_TOTAL2 (98.6 + 42.9 + 2.35) // kOhm
#define R_TOTAL3 (99.4 + 42.6 + 2.35) // kOhm
#define R_TOTAL4 (99.5 + 42.6 + 2.68) // kOhm
#define R_BOTTOM1 (2.35)              // kOhm
#define R_BOTTOM2 (2.35)              // kOhm
#define R_BOTTOM3 (2.35)              // kOhm
#define R_BOTTOM4 (2.68)              // kOhm

// #define VREF3           2.475f, 2.530f
#define VREF3 2.52f
// #define VREF2           2.79f
#define VREF2 2.52f
// #define VREF1           2.368f // Sử dụng VREF1 là 2.5V-90
#define VREF1 2.526f
// #define VREF4
#define VREF4 2.52f

#define VREF4_VOLTAGE 2.489f
// #define VREF3_VOLTAGE   2.53f
#define VREF3_VOLTAGE 2.489f
// #define VREF2_VOLTAGE   2.51f
#define VREF2_VOLTAGE 2.54f
// #define VREF1_VOLTAGE   2.5f
#define VREF1_VOLTAGE 2.52f

// Macro chuyển đổi ADC sang điện áp/dòng điện
#define CONVERT_ADC1_TO_VOL(raw)                                               \
  ((((float)raw / 65536.0) * VREF1_VOLTAGE) * (R_TOTAL1 / R_BOTTOM1))
#define CONVERT_ADC2_TO_VOL(raw)                                               \
  ((((float)raw / 65536.0) * VREF2_VOLTAGE) * (R_TOTAL2 / R_BOTTOM2))
#define CONVERT_ADC3_TO_VOL(raw)                                               \
  ((((float)raw / 65536.0) * VREF3_VOLTAGE) * (R_TOTAL3 / R_BOTTOM3))
#define CONVERT_ADC4_TO_VOL(raw)                                               \
  ((((float)raw / 65536.0) * VREF4_VOLTAGE) * (R_TOTAL4 / R_BOTTOM4))

#define CONVERT_ADC1_TO_CURRENT(raw)                                           \
  (((raw * VREF1 * 2.0) / 65536.0) / 0.0264) // 39069
#define CONVERT_ADC2_TO_CURRENT(raw) (((raw * VREF2 * 2.0) / 65536.0) / 0.0264)
#define CONVERT_ADC3_TO_CURRENT(raw) (((raw * VREF3 * 2.0) / 65536.0) / 0.0264)
#define CONVERT_ADC4_TO_CURRENT(raw) (((raw * VREF4 * 2.0) / 65536.0) / 0.0264)
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
  esp_err_t saveVoltageCalibToFlash();
  void captureVoltageRaw24_Single(uint8_t board_idx);
  void captureVoltageRaw24_All();
  // Đọc currentOffset từ flash
  esp_err_t loadCurrentOffsetFromFlash();
  void calibrateDynamicScale(uint8_t board_id, float actualV, float actualI);
  esp_err_t loadCurrentConfigFromFlash(); // Đọc lại toàn bộ thông số
  void captureVoltageRaw0();              // Tool gửi lệnh Calib 0V
  void captureVoltageRaw24();             // Tool gửi lệnh Calib 24V
  esp_err_t loadVoltageCalibFromFlash();  // Cần hàm load khi khởi động

  void CalibCurrentZero(void);

private:
  // Bộ lọc Kalman cho điện áp và dòng điện
  KalmanFilter voltageFilter;
  KalmanFilter currentFilter;

  bool isBoardCalibDone = false;
  // bool isBoardCalibDone = false;
  // Ma trận và vector cho bộ lọc Kalman
  float A[4];  // Ma trận chuyển trạng thái (2x2)
  float P[4];  // Ma trận hiệp phương sai (2x2)
  float Q[4];  // Nhiễu quá trình (2x2)
  float H[2];  // Ma trận quan sát (1x2)
  float R[1];  // Nhiễu quan sát (1x1)
  float xp[2]; // Trạng thái dự đoán
  float xc[2]; // Trạng thái hiện tại

  // State vectors riêng cho current filter
  float A_current[4];
  float P_current[4];
  float Q_current[4];
  float H_current[2];
  float R_current[1];
  float xp_current[2];
  float xc_current[2];

  // Offset cho dòng điện
  uint16_t currentOffset[4]; // [board_id][channel]
  // Lưu giá trị thô của điện áp và dòng điện
  double current[4] = {0.0, 0.0, 0.0, 0.0}; // Lưu giá trị dòng điện đã lọc
  double voltage[4] = {0.0, 0.0, 0.0, 0.0}; // Lưu giá trị điện áp đã lọc
  // Task handle cho FreeRTOS
  // TaskHandle_t adcTaskHandle;
  uint16_t voltageRaw0[4]; // Giá trị raw tại 0V cho 4 board
  uint16_t
      voltageRawScale[4]; // Lưu ADC thô tại điểm Scale (thay cho voltageRaw24)
  float voltageValueScale[4]; // Lưu giá trị thực tế (V) lúc Calib
                              // (VD: 21.0, 15.0)
  uint16_t currentRaw0[4];    // ADC thô tại 0A cho 4 board
  float currentVal0[4];       // Mốc 1: Dòng thực tế (Thường = 0.0f)

  uint16_t currentRawScale[4]; // ADC thô tại điểm chuẩn (VD: 3A)
  float currentValueScale[4]; // Giá trị dòng điện chuẩn thực tế (A)

  double currentSlope[4];         // Lưu hệ số dòng điện (A/ADC)
  bool isCalibratingZero = false; // THÊM: Flag phân biệt loại calib
  // Hàm task quét ADC
  static void adcScanTask(void *arg);
  void calculateCurrentSlope(uint8_t board_id);
  // Hàm chuyển đổi giá trị thô thành điện áp/dòng điện
  double convertToVoltage(uint8_t board_id, uint16_t raw);
  double convertToCurrent(uint8_t board_id, uint16_t raw);
  void printFlashData();
  // Cấu hình bộ lọc Kalman
  void setupKalmanFilter();
};

#endif // __ADC_MEASURE_H__