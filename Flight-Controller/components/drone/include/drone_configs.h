#ifndef DRONE_CONFIGS_H
#define DRONE_CONFIGS_H

#include <drone_structs.h>

/**
 * @brief Drone Class generic configs
 */
static drone_cfg_t DroneConfigs = {
    .roll = {
        .P = 1.0f,
        .Q = 0.1f,
        .R = 0.5f,
    },
    .pitch = {
        .P = 0.0f,
        .Q = 0.0f,
        .R = 0.0f,
    },
    .yaw = {
        .P = 0.0f,
        .Q = 0.0f,
        .R = 0.0f,
    },
    .IIR_coeff_roll_dot  = 0.0f,
    .IIR_coeff_pitch_dot = 0.0f,
    .IIR_coeff_yaw_dot   = 0.0f,
    .imu_cfg = {
        .imu_i2c_cfg = {
            .address = BMI160_ADDR,
            .scl     = 22,
            .sda     = 21,
        },
        .acc_mode    = BMI160_CMD_ACC_NORMAL_MODE,
        .acc_freq    = BMI160_ACC_CONF_100HZ_NORMAL,
        .acc_range   = BMI160_ACC_RANGE_4G,
        .gyro_mode   = BMI160_CMD_GYRO_NORMAL_MODE,
        .gyro_freq   = BMI160_GYRO_CONF_100HZ_NORMAL,
        .gyro_range  = BMI160_GYRO_RANGE_250DPS,
        .gyro_offset = {
            .x = 0,
            .y = 0,
            .z = 0,
        },
    },
    .esp_mac_addr = { 0xf0, 0xf0, 0x02, 0x43, 0x53, 0x53 },
    .pwm_cfg = {
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_26,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_0,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_5,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_1,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_15,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_2,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_18,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_3,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        }
    },
    .pid_cfgs = {
        {
            .tag = Z,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 1.0f
            },
            .integral_limits = { .min = -125.0f, .max = 125.0f },
            .pid_output_limits = { .min = -500.0f, .max = 500.0f }
        },
        {
            .tag = ROLL,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 1.0f
            },
            .integral_limits = { .min = -125.0f, .max = 125.0f },
            .pid_output_limits = { .min = -500.0f, .max = 500.0f }
        },
        {
            .tag = PITCH,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 0.0f
            },
            .integral_limits = { .min = 0.0f, .max = 0.0f },
            .pid_output_limits = { .min = 0.0f, .max = 0.0f }
        },
        {
            .tag = YAW,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 0.0f
            },
            .integral_limits = { .min = 0.0f, .max = 0.0f },
            .pid_output_limits = { .min = 0.0f, .max = 0.0f }
        },
        {
            .tag = ROLL_D,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 1.0f
            },
            .integral_limits = { .min = -100000.0f, .max = 100000.0f },
            .pid_output_limits = { .min = -100000.0f, .max = 100000.0f }
        },
        {
            .tag = PITCH_D,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 0.0f
            },
            .integral_limits = { .min = 0.0f, .max = 0.0f },
            .pid_output_limits = { .min = 0.0f, .max = 0.0f }
        },
        {
            .tag = YAW_D,
            .pid_gains = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .Kb = 0.0f
            },
            .integral_limits = { .min = 0.0f, .max = 0.0f },
            .pid_output_limits = { .min = 0.0f, .max = 0.0f }
        }
    },
    .mma_out_limits = {
        .upper = 0.8f,
        .lower = 1.3f
    }
};

#endif
