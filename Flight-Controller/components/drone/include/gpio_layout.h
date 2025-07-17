#pragma once

// PWM Output sets (GND, SIG)
#define PWM1_PIN        GPIO_NUM_5
#define PWM2_PIN        GPIO_NUM_18
#define PWM3_PIN        GPIO_NUM_19
#define PWM4_PIN        GPIO_NUM_23

// Onboard LEDs
#define LED1_PIN        GPIO_NUM_12
#define LED2_PIN        GPIO_NUM_13
#define LED3_PIN        GPIO_NUM_14

// Buzzer
#define BUZZER_PIN      GPIO_NUM_15

// I2C Pins
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22

// SPI Pins
#define SPI_MOSI_PIN    GPIO_NUM_32
#define SPI_MISO_PIN    GPIO_NUM_25
#define SPI_SCLK_PIN    GPIO_NUM_33
#define SPI_CS_PIN      GPIO_NUM_16
#define CARD_IN_PIN     GPIO_NUM_34

// UARTS
#define UART1_TX        GPIO_NUM_27
#define UART1_RX        GPIO_NUM_4
#define UART2_TX        GPIO_NUM_17
#define UART2_RX        GPIO_NUM_35

// Voltage Divider
#define VOLTAGE_ADC_CH  ADC_CHANNEL_0  // GPIO36
#define VOLTAGE_ADC_UNIT ADC_UNIT_1

// Ultrasonic Sensor
#define US_ECHO_PIN     GPIO_NUM_39
#define US_TRIG_PIN     GPIO_NUM_26