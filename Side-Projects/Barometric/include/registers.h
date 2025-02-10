// BMP180 Registers
#define BMP180_CALIBRATION_REG 0xAA
#define BMP180_CTRL_MEAS_REG   0xF4
#define BMP180_OUT_MSB_REG     0xF6
#define BMP180_SOFT_RESET_REG  0xE0

// Commands
#define BMP180_TEMP_MEASURE    0x2E
#define BMP180_PRESS_MEASURE   0x34

// Calibration coefficients count
#define BMP180_CALIBRATION_DATA_LEN 22

// Oversampling configurations (OSS)
#define BMP180_OSS_ULTRA_LOW_POWER      0  // 1 sample,  4.5ms delay
#define BMP180_OSS_STANDARD             1  // 2 samples, 7.5ms delay
#define BMP180_OSS_HIGH_RESOLUTION      2  // 4 samples, 13.5ms delay
#define BMP180_OSS_ULTRA_HIGH_RESOLUTION 3 // 8 samples, 25.5ms delay

// Measurement modes
#define BMP180_MODE_TEMPERATURE         0x2E
#define BMP180_MODE_PRESSURE            0x34