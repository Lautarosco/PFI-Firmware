#ifndef COMPENSATION_WORDS_H
#define COMPENSATION_WORDS_H

#include <stdint.h>
#include <esp_err.h>
#include <data_types/bmp280_data_types.h>


typedef struct compensation_words {
    unsigned short int dig_T1;
    signed short int dig_T2;
    signed short int dig_T3;

    unsigned short int dig_P1;
    signed short int dig_P2;
    signed short int dig_P3;
    signed short int dig_P4;
    signed short int dig_P5;
    signed short int dig_P6;
    signed short int dig_P7;
    signed short int dig_P8;
    signed short int dig_P9;
} compensation_words_t;

typedef enum _signed {
    UNSIGNED,
    SIGNED
} _signed_t;


/**
 * @brief Read compensation words stored in registers
 * 
 * @param slave_addr: Address of sensor (bmp280)
 * @param comp_words: Variable containing all compensation words
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_InitCompWords(uint8_t slave_addr, compensation_words_t * comp_words);

#endif
