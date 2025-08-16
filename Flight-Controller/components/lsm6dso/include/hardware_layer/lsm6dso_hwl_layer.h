#pragma once

#include <hardware_layer/registers/lsm6dso_registers.h>
#include <hardware_layer/modes/accelerometer/lsm6dso_acc_modes.h>
#include <hardware_layer/modes/gyroscope/lsm6dso_gyro_modes.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <stdbool.h>

inline uint8_t LSM6DSO_SET_BITS(unsigned char n_bits, unsigned char starting_bit_pos, uint8_t reg_value, uint8_t mode) {
    /**
     * i.e
     * 
     * reg = x??x xxxx
     * n = 2d
     * s = 5d
     * mode = 10b
     * 
     * a = 1u << 2d = 0000 0100b = 0x04 = 4d
     * b = a - 1 = 3d = 0000 0011b
     * C = b << s = 0110 0000b
     * d = ~c = 1001 1111
     * 
     * reg & d =
     * x??x xxxx
     * 1001 1111 &
     * ------------
     * x00x xxxx = e
     * 
     * f = e | (mode << s) =
     * x00x xxxx
     * 0100 0000 |
     * ------------
     * x10x xxxx
     */

    return (~(((1U << n_bits) - 1) << starting_bit_pos) & reg_value) | (mode << starting_bit_pos);
}

inline uint16_t LSM6DSO_CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
    return ((uint16_t) msb << 8) | ((uint16_t) lsb);
}

typedef struct lsm6dso_data_ready {
    bool temp_ready;
    bool acc_ready;
    bool gyro_ready;
} lsm6dso_data_ready_t;

/**
 * @brief Get LSM6DSO ID
 * 
 * @param i2c_lsm_handler: Device handler
 * @param id: Pointer to variable to store device ID
 * @param msg: Pointer to buffer to store function response
 * @param msg_len: Length of buffer
 * 
 * @param
 *      - ESP_OK: Success
 *      - ESP_FAIL
 */
esp_err_t lsm6dso_hwl_get_chip_id(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t *id, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_read_reg(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t *reg_addr, unsigned int reg_len, uint8_t *reg_value, unsigned int bytes, const char *func_caller);
esp_err_t lsm6dso_hwl_write_reg(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t reg_addr, uint8_t reg_value, const char *func_caller);
esp_err_t lsm6dso_hwl_set_odr_acc(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_odr_acc_t odr, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_set_fs_acc(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_fs_acc_t fs, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_en_lpf2_acc(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_dis_lpf2_acc(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_set_odr_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_odr_gyro_t odr, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_set_fs_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_fs_gyro_t fs, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_sw_reset(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_en_lpf1_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_lpf1_gyro_t lpf_mode, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_dis_lpf1_gyro(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_en_hpf_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_hpf_gyro_t hpf_mode, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_dis_hpf_gyro(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_hwl_data_ready(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_data_ready_t *data_ready, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_read_gyro_and_acc(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_gyro_t *gyro, float gyro_so, lsm6dso_gyro_unit_t gyro_unit, lsm6dso_acc_t *acc, float acc_so, lsm6dso_acc_unit_t acc_unit, float acc_x_off, float acc_y_off, float acc_z_off, float gyro_x_off, float gyro_y_off, float gyro_z_off, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_read_gyro(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_gyro_t *gyro, float gyro_so, lsm6dso_gyro_unit_t gyro_unit, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_read_acc(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_acc_t *acc, float acc_so, lsm6dso_acc_unit_t acc_unit, char *msg, unsigned int msg_len);
esp_err_t lsm6dso_read_temp(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, char *msg, unsigned int msg_len);
