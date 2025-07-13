#include <hardware_layer/bmp390_hw_layer.h>
#include <hardware_layer/bmp390_registers.h>

#include <hardware_layer/bmp390_modes_names.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>
#include <stdbool.h>

#include <string.h>


/* ========== Private structs ========== */

typedef enum bmp390_calib_data_index {
    NVM_PAR_T1_LSB,
    NVM_PAR_T1_MSB,
    NVM_PAR_T2_LSB,
    NVM_PAR_T2_MSB,
    NVM_PAR_T3,
    NVM_PAR_P1_LSB,
    NVM_PAR_P1_MSB,
    NVM_PAR_P2_LSB,
    NVM_PAR_P2_MSB,
    NVM_PAR_P3,
    NVM_PAR_P4,
    NVM_PAR_P5_LSB,
    NVM_PAR_P5_MSB,
    NVM_PAR_P6_LSB,
    NVM_PAR_P6_MSB,
    NVM_PAR_P7,
    NVM_PAR_P8,
    NVM_PAR_P9_LSB,
    NVM_PAR_P9_MSB,
    NVM_PAR_P10,
    NVM_PAR_P11
} bmp390_calib_data_index_t;

typedef struct bmp390_reg_modes {
    const char *mode_name;                                          /* Name of register mode */
    uint8_t reg;                                                    /* Register address */
    uint8_t totals;                                                 /* Total bits used for this mode */
    uint8_t mode;                                                   /* Starting bit position */
    const char* (*get_name_func)(unsigned int mode_value);          /* [bmp390_get_<register>_<name/status>] function --> See src/hardware_layer/bmp390_modes_names.c */
} bmp390_reg_modes_t;

/* ========== Private variables ========== */

/* Compute constants for compensation coefficients, to ensure precision */

static const double POW_2_N8  = 0.00390625;                         /* 2^(-8) */
static const double POW_2_P30 = 1073741824.0;                       /* 2^(30) */
static const double POW_2_P48 = 281474976710656.0;                  /* 2^(48) */
static const double POW_2_P14 = 16384.0;                            /* 2^(14) */
static const double POW_2_P20 = 1048576.0;                          /* 2^(20) */
static const double POW_2_P29 = 536870912.0;                        /* 2^(29) */
static const double POW_2_P32 = 4294967296.0;                       /* 2^(32) */
static const double POW_2_P37 = 137438953472.0;                     /* 2^(37) */
static const double POW_2_N3  = 0.125;                              /* 2^(-3) */
static const double POW_2_P6  = 64.0;                               /* 2^(6) */
static const double POW_2_P8  = 256.0;                              /* 2^(8) */
static const double POW_2_P15 = 32768.0;                            /* 2^(15) */
static const double POW_2_P65 = 36893488147419103232.0;             /* 2^(65) */

const char *bmp390_hwl_tag = "[BMP390_HW_LAYER]";       /* Hardware layer TAG */

static const bmp390_reg_modes_t bmp390_reg_modes_arr[] = {
    {.reg = BMP390_CONFIG_RW_REG,   .mode_name = "iir_filter",  .totals = 3, .mode = BMP390_CONFIG_IIR,          .get_name_func = bmp390_get_iir_coef_name},
    {.reg = BMP390_ODR_RW_REG,      .mode_name = "odr_sel",     .totals = 5, .mode = BMP390_ODR_ODR_SEL,         .get_name_func = bmp390_get_odr_sel_name},
    {.reg = BMP390_OSR_RW_REG,      .mode_name = "osr_p",       .totals = 3, .mode = BMP390_OSR_P,               .get_name_func = bmp390_get_osr_press_name},
    {.reg = BMP390_OSR_RW_REG,      .mode_name = "osr_t",       .totals = 3, .mode = BMP390_OSR_T,               .get_name_func = bmp390_get_osr_temp_name},
    {.reg = BMP390_PWR_CTRL_RW_REG, .mode_name = "mode",        .totals = 2, .mode = BMP390_PWR_CTRL_MODE,       .get_name_func = bmp390_get_pwr_mode_name},
    {.reg = BMP390_PWR_CTRL_RW_REG, .mode_name = "press_en",    .totals = 1, .mode = BMP390_PWR_CTRL_PRESS_EN,   .get_name_func = bmp390_get_press_status},
    {.reg = BMP390_PWR_CTRL_RW_REG, .mode_name = "temp_en",     .totals = 1, .mode = BMP390_PWR_CTRL_TEMP_EN,    .get_name_func = bmp390_get_temp_status},
    {.reg = BMP390_IF_CONF_RW_REG,  .mode_name = "spi3",        .totals = 1, .mode = BMP390_IF_CONF_SPI3,        .get_name_func = bmp390_get_spi_mode_name},
    {.reg = BMP390_IF_CONF_RW_REG,  .mode_name = "i2c_wdt_en",  .totals = 1, .mode = BMP390_IF_CONF_I2C_WDT_EN,  .get_name_func = bmp390_get_i2c_wdt_en_status},
    {.reg = BMP390_IF_CONF_RW_REG,  .mode_name = "i2c_wdt_sel", .totals = 1, .mode = BMP390_IF_CONF_I2C_WDT_SEL, .get_name_func = bmp390_get_i2c_wdt_tout_name},
};

/* ========== Public functions ========== */

int bmp390_hwl_get_mode_val(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t reg_addr, uint8_t mode, char *msg, size_t msg_length) {
    for(int i = 0; i < ((sizeof(bmp390_reg_modes_arr)) / (sizeof(bmp390_reg_modes_arr[0]))); i++) {
        /* Match register and mode */
        if((bmp390_reg_modes_arr[i].mode == mode) && (bmp390_reg_modes_arr[i].reg == reg_addr)) {
            /* Read DATA register content and check for errors */
            uint8_t reg_value = 0;

            esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &reg_value, 1, 100);
            if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
                ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
            }

            uint8_t mask = ((1U << bmp390_reg_modes_arr[i].totals) - 1) << bmp390_reg_modes_arr[i].mode;
            uint8_t mode_value = (reg_value & mask) >> bmp390_reg_modes_arr[i].mode;

            // ESP_LOGI(bmp390_hwl_tag, "<%s> --> Value: <0x%X>, Status: <%s>", bmp390_reg_modes_arr[i].mode_name, mode_value, bmp390_reg_modes_arr[i].get_name_func(mode_value));
            memset(msg, 0, msg_length);
            snprintf(msg, msg_length, "<%s> --> Value: <0x%X>, Status: <%s>", bmp390_reg_modes_arr[i].mode_name, mode_value, bmp390_reg_modes_arr[i].get_name_func(mode_value));

            return mode_value;
        }
    }

    ESP_LOGE(bmp390_hwl_tag, "Mode not found");
    return -1;
}

esp_err_t bmp390_hwl_get_chip_id(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t *read_data) {
    /* Check if <read_data> is a valid pointer */
    if(read_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t reg_addr = BMP390_CHIP_ID_RO_REG;

    /* Clear <read_data> parameter for security */
    *read_data = 0;

    /* Read CHIP_ID register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, read_data, 1, 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    return ESP_OK;
}

esp_err_t bmp390_hwl_get_rev_id(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t *read_data) {
    /* Check if <read_data> is a valid pointer */
    if(read_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = BMP390_REV_ID_RO_REG;

    /* Clear <read_data> parameter for security */
    *read_data = 0;

    /* Read REV_ID register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, read_data, 1, 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }
    
    return ESP_OK;
}

esp_err_t bmp390_hwl_err(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t err_reg = 0;
    uint8_t reg_addr = BMP390_ERR_REG_RO_REG;

    /* Read ERR_REG register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, sizeof(reg_addr), &err_reg, sizeof(err_reg), 100);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    if(err_reg & (1U << BMP390_ERR_REG_FATAL_ERR)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Fatal error", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    if(err_reg & (1U << BMP390_ERR_REG_CMD_ERR)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Command execution failed", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    if(err_reg & (1U << BMP390_ERR_REG_CONF_ERR)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Sensor configuration error detected", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmp390_hwl_cmd_rdy_status(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t status = 0;
    uint8_t reg_addr = BMP390_STATUS_RO_REG;

    /* Read STATUS register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &status, sizeof(status), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    if(status & (1U << BMP390_STATUS_CMD_RDY)) {
        return ESP_OK;
    }

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t bmp390_hwl_drdy_press_status(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t status = 0;
    uint8_t reg_addr = BMP390_STATUS_RO_REG;

    /* Read STATUS register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &status, sizeof(status), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    if(status & (1U << BMP390_STATUS_DRDY_PRESS)) {
        return ESP_OK;
    }

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t bmp390_hwl_drdy_temp_status(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t status = 0;
    uint8_t reg_addr = BMP390_STATUS_RO_REG;

    /* Read STATUS register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &status, 1, 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }
    
    if(status & (1U << BMP390_STATUS_DRDY_TEMP)) {
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FINISHED;
}

int bmp390_hwl_detect_soft_reset(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t event = 0;
    uint8_t reg_addr = BMP390_EVENT_RO_REG;

    /* Read EVENT register and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &event, sizeof(event), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    if(event & (1U << BMP390_EVENT_POR_DETECTED)) {
        return true;
    }

    return false;
}

esp_err_t bmp390_hwl_spi_en(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_if_conf_reg_spi_t spi_mode) {
    uint8_t if_conf = 0;
    uint8_t reg_addr = BMP390_IF_CONF_RW_REG;

    /* Read IF_CONF register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &if_conf, sizeof(if_conf), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bit in position <BMP390_IF_CONF_SPI3> of <if_conf>. Then, set <spi_mode> bit as its new value */
    uint8_t mask = SET_BITS(if_conf, 1U, spi_mode, BMP390_IF_CONF_SPI3);
    // uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_SPI3)) | (spi_mode << BMP390_IF_CONF_SPI3);
    
    /* Write new register value to IF_CONF register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set SPI mode --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual IF_CONF register <mode> mode value */
    char msg[256];

    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_IF_CONF_SPI3, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set SPI mode --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != spi_mode) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set SPI mode --> FAILED", __func__, __LINE__, spi_mode, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_i2c_en_wdt(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout) {
    uint8_t if_conf = 0;
    uint8_t reg_addr = BMP390_IF_CONF_RW_REG;

    /* Read IF_CONF register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &if_conf, sizeof(if_conf), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }
    
    /* Clear i2c_wdt_en and i2c_wdt_sel bits, then enable I2C wdt timeout and define its period <i2c_wdt_tout> */
    uint8_t mask = SET_BITS(if_conf, 0b11, ((BMP390_IF_CONF_I2C_WDT_EN << 0) | (i2c_wdt_tout << 1)), 1);
    // uint8_t new_if_conf = (if_conf & ~((1U << BMP390_IF_CONF_I2C_WDT_EN) | (1U << BMP390_IF_CONF_I2C_WDT_SEL))) | ((1U << BMP390_IF_CONF_I2C_WDT_EN) | (i2c_wdt_tout << BMP390_IF_CONF_I2C_WDT_SEL));
    
    /* Write IF_CONF register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable and configure I2C watchdog timeout --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check if I2C watchdog timeout is enabled */
    char msg[256];

    bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_IF_CONF_I2C_WDT_EN, msg, sizeof(msg));

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);

    /* Check actual IF_CONF register <mode> mode value */
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, BMP390_IF_CONF_RW_REG, BMP390_IF_CONF_I2C_WDT_SEL, msg, sizeof(msg));
    
    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable and configure I2C watchdog timeout --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != i2c_wdt_tout) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Enable and configure I2C watchdog timeout --> FAILED", __func__, __LINE__, i2c_wdt_tout, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_i2c_dis_wdt(i2c_master_dev_handle_t i2c_bmp_handler) {
    /* Read IF_CONF register content and check for errors */
    uint8_t if_conf = 0;
    uint8_t reg_addr = BMP390_IF_CONF_RW_REG;

    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &if_conf, sizeof(if_conf), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }
        
    /* Clear <BMP390_IF_CONF_I2C_WDT_EN> bit of IF_CONF register and disable watchdog timeout */
    uint8_t mask = SET_BITS(if_conf, 1U, BMP390_IF_CONF_I2C_WDT_OFF, BMP390_IF_CONF_I2C_WDT_EN);
    // uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_I2C_WDT_EN)) | (BMP390_IF_CONF_I2C_WDT_OFF << BMP390_IF_CONF_I2C_WDT_EN);
        
    /* Write IF_CONF register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Disable I2C watchdog timeout --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    char msg[256];
    bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_IF_CONF_I2C_WDT_EN, msg, sizeof(msg));

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_set_pwr_mode(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_pwr_ctrl_mode_t pwr_mode) {
    uint8_t pwr_ctrl = 0;
    uint8_t reg_addr = BMP390_PWR_CTRL_RW_REG;

    /* Read PWR_CTRL register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &pwr_ctrl, sizeof(pwr_ctrl), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits 4 and 5 of PWR_CTRL register, then set the power mode */
    uint8_t mask = SET_BITS(pwr_ctrl, 0b11, pwr_mode, BMP390_PWR_CTRL_MODE);
    // uint8_t mask = (pwr_ctrl & ~(0b11 << BMP390_PWR_CTRL_MODE)) | (pwr_mode << BMP390_PWR_CTRL_MODE);

    /* Write mask to PWR_CTRL register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set power mode --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }
    
    /* Check actual PWR_CTRL register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_PWR_CTRL_MODE, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set power mode --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != pwr_mode) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set power mode --> FAILED", __func__, __LINE__, pwr_mode, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_press_en(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t pwr_ctrl = 0;
    uint8_t reg_addr = BMP390_PWR_CTRL_RW_REG;

    /* Read PWR_CTRL register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &pwr_ctrl, sizeof(pwr_ctrl), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits 4 and 5 of PWR_CTRL register, then enable pressure sensor */
    uint8_t mask = SET_BITS(pwr_ctrl, 1U, BMP390_PWR_CTRL_PRESS_ON, BMP390_PWR_CTRL_PRESS_EN);
    // uint8_t mask = (pwr_ctrl & ~(1U << BMP390_PWR_CTRL_PRESS_EN)) | (BMP390_PWR_CTRL_PRESS_ON << BMP390_PWR_CTRL_PRESS_EN);

    /* Write mask to PWR_CTRL register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable pressure sensor --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual PWR_CTRL register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_PWR_CTRL_PRESS_EN, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable pressure sensor --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != BMP390_PWR_CTRL_PRESS_ON) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Enable pressure sensor --> FAILED", __func__, __LINE__, BMP390_PWR_CTRL_PRESS_ON, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_temp_en(i2c_master_dev_handle_t i2c_bmp_handler) {
    uint8_t pwr_ctrl = 0;
    uint8_t reg_addr = BMP390_PWR_CTRL_RW_REG;

    /* Read PWR_CTRL register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &pwr_ctrl, sizeof(pwr_ctrl), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits[5:4] of PWR_CTRL register, then enable temperature sensor */
    uint8_t mask = SET_BITS(pwr_ctrl, 1U, BMP390_PWR_CTRL_TEMP_ON, BMP390_PWR_CTRL_TEMP_EN);
    // uint8_t mask = (pwr_ctrl & ~(1U << BMP390_PWR_CTRL_TEMP_EN)) | (BMP390_PWR_CTRL_TEMP_ON << BMP390_PWR_CTRL_TEMP_EN);

    /* Write mask to PWR_CTRL register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable pressure sensor --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual PWR_CTRL register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_PWR_CTRL_TEMP_EN, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Enable and configure I2C watchdog timeout --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != BMP390_PWR_CTRL_TEMP_ON) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Enable pressure sensor --> FAILED", __func__, __LINE__, BMP390_PWR_CTRL_TEMP_ON, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_set_osr_press(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_osr_press_t osr_press) {
    if((osr_press < BMP390_OSR_P_X1) || (osr_press > BMP390_OSR_P_X32)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Invalid <osr_press> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t osr = 0;
    uint8_t reg_addr = BMP390_OSR_RW_REG;

    /* Read OSR register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &osr, sizeof(osr), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits[2:0] of OSR register and set oversampling rate for pressure measurements */
    uint8_t mask = SET_BITS(osr, 0b111, osr_press, BMP390_OSR_P);
    // uint8_t mask = (osr & ~(0b111 << BMP390_OSR_P)) | (osr_press << BMP390_OSR_P);

    /* Write mask to OSR register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set pressure oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual OSR register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_OSR_P, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set pressure oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != osr_press) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set pressure oversampling rate --> FAILED", __func__, __LINE__, osr_press, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_set_osr_temp(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_osr_temp_t osr_temp) {
    if((osr_temp < BMP390_OSR_T_X1) || (osr_temp > BMP390_OSR_T_X32)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Invalid <osr_temp> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t osr = 0;
    uint8_t reg_addr = BMP390_OSR_RW_REG;

    /* Read OSR register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &osr, sizeof(osr), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits[2:0] of OSR register and set oversampling rate for temperature measurements */
    uint8_t mask = SET_BITS(osr, 0b111, osr_temp, BMP390_OSR_T);
    // uint8_t mask = (osr & ~(0b111 << BMP390_OSR_T)) | (osr_temp << BMP390_OSR_T);

    /* Write mask to OSR register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set temperature oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual OSR register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_OSR_T, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set temperature oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != osr_temp) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set temperature oversampling rate --> FAILED", __func__, __LINE__, osr_temp, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_set_odr(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_odr_sel_t odr_sel) {
    if((odr_sel < BMP390_ODR_SEL_200_HZ) || (odr_sel > BMP390_ODR_SEL_0P0015_HZ)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Invalid <osr_temp> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t odr = 0;
    uint8_t reg_addr = BMP390_ODR_RW_REG;

    /* Read ODR register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &odr, sizeof(odr), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits[4:0] of ODR register and set output data rates */
    uint8_t mask = SET_BITS(odr, 0b1111, odr_sel, BMP390_ODR_ODR_SEL);
    // uint8_t mask = (odr & ~(0b1111 << BMP390_ODR_ODR_SEL)) | (odr_sel << BMP390_ODR_ODR_SEL);

    /* Write mask to ODR register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};
    
    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set output data rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual OSR register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_ODR_ODR_SEL, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set output data rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != odr_sel) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set output data rate --> FAILED", __func__, __LINE__, odr_sel, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_set_iir_coef(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_config_coef_t iir_coef) {
    if((iir_coef < BMP390_CONFIG_COEF_0) || (iir_coef > BMP390_CONFIG_COEF_127)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Invalid <iir_coef> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config = 0;
    uint8_t reg_addr = BMP390_CONFIG_RW_REG;

    /* Read CONFIG register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &config, sizeof(config), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear bits[3:1] of CONFIG register and set IIR filter coefficient */
    // uint8_t mask = (config & ~(0b111 << BMP390_CONFIG_IIR)) | (iir_coef << BMP390_CONFIG_IIR);
    uint8_t mask = SET_BITS(config, 0b111, iir_coef, BMP390_CONFIG_IIR);

    /* Write mask to CONFIG register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set IIR filter coefficient --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    /* Check actual OSR register <mode> mode value */
    char msg[256];
    int mode_value = bmp390_hwl_get_mode_val(i2c_bmp_handler, reg_addr, BMP390_CONFIG_IIR, msg, sizeof(msg));

    if(mode_value == -1) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set IIR filter coefficient --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else if(mode_value != iir_coef) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Desired mode <0x%X> does not match actual mode <0x%X>. Set IIR filter coefficient --> FAILED", __func__, __LINE__, iir_coef, mode_value);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hwl_tag, "%s", msg);
    return ESP_OK;
}

esp_err_t bmp390_hwl_exec_cmd(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_cmd_t cmd_sel) {
    uint8_t cmd = 0;
    uint8_t reg_addr = BMP390_CMD_RW_REG;

    /* Read CMD register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, &cmd, sizeof(cmd), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Clear all bits[7:0] of CONFIG register and set cmd to be executed */
    uint8_t mask = SET_BITS(cmd, 0xFF, cmd_sel, 0);
    // uint8_t mask = (cmd & ~0b11111111) | cmd_sel;

    /* Write mask to CMD register and check for errors */
    uint8_t write_data[2] = {reg_addr, mask};

    ret = i2c_master_transmit(i2c_bmp_handler, write_data, sizeof(write_data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Set CMD --> FAILED", __func__, __LINE__);
        return ESP_FAIL;    
    }

    ESP_LOGI(bmp390_hwl_tag, "Execute CMD <%s> --> OK", bmp390_get_cmd_name(cmd_sel));
    return ESP_OK;
}

esp_err_t bmp390_hwl_read_raw_data(i2c_master_dev_handle_t i2c_bmp_handler, uint32_t *adc_temp, uint32_t *adc_press) {
    uint8_t data[6] = {0};
    uint8_t reg_addr = BMP390_DATA_0_RO_REG;

    /* Check if adc_temp and adc_press are valid pointers */
    if((adc_temp == NULL) || (adc_press == NULL)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Either temperature or pressure pointers are NULL", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    /* Clear both pointers for security */
    *adc_temp = 0;
    *adc_press = 0;

    /* Read DATA register content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, data, sizeof(data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }
    
    /* Note: To ensure aritmetic consistency in bit-operationes, MSB, LSB and XLSB bytes are converted to 32-bit numbers */

    /* Raw temperature = MSB[23:16] | LSB[15:8] | XLSB[7:0] */
    *adc_temp = ((uint32_t) data[5] << 16) | ((uint32_t) data[4] << 8) | ((uint32_t) data[3]);

    /* Raw pressure = MSB[23:16] | LSB[15:8] | XLSB[7:0] */
    *adc_press = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | ((uint32_t) data[0]);

    return ESP_OK;
}

esp_err_t bmp390_hwl_get_comp_coefs(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_calib_data_t *calib_data) {
    uint8_t data[21] = {0};
    uint8_t reg_addr = BMP390_CALIBRATION_DATA_INITIAL_ADDR;

    /* Read CALIBRATION_DATA registers content and check for errors */
    esp_err_t ret = i2c_master_transmit_receive(i2c_bmp_handler, &reg_addr, 1, data, sizeof(data), 100);
    if((ret != ESP_OK) || (bmp390_hwl_err(i2c_bmp_handler) != ESP_OK)) {
        ESP_LOGE(bmp390_hwl_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);    
    }

    /* Note: Compensation formula is implemented in floating point. Thus, coefficients must be converted into floating point numbers */

    calib_data->par_t1  = (double) CONCAT_BYTES(data[NVM_PAR_T1_MSB], data[NVM_PAR_T1_LSB]) / POW_2_N8;
    calib_data->par_t2  = (double) CONCAT_BYTES(data[NVM_PAR_T2_MSB], data[NVM_PAR_T2_LSB]) / POW_2_P30;
    calib_data->par_t3  = (double) ((int8_t) data[NVM_PAR_T3]) / POW_2_P48;
    calib_data->par_p1  = (double) ((int16_t) CONCAT_BYTES(data[NVM_PAR_P1_MSB], data[NVM_PAR_P1_LSB]) - POW_2_P14) / POW_2_P20;
    calib_data->par_p2  = (double) ((int16_t) CONCAT_BYTES(data[NVM_PAR_P2_MSB], data[NVM_PAR_P2_LSB]) - POW_2_P14) / POW_2_P29;
    calib_data->par_p3  = (double) ((int8_t) (data[NVM_PAR_P3])) / POW_2_P32;
    calib_data->par_p4  = (double) ((int8_t) (data[NVM_PAR_P4])) / POW_2_P37;
    calib_data->par_p5  = (double) CONCAT_BYTES(data[NVM_PAR_P5_MSB], data[NVM_PAR_P5_LSB]) / POW_2_N3;
    calib_data->par_p6  = (double) CONCAT_BYTES(data[NVM_PAR_P6_MSB], data[NVM_PAR_P6_LSB]) / POW_2_P6;
    calib_data->par_p7  = (double) ((int8_t) (data[NVM_PAR_P7])) / POW_2_P8;
    calib_data->par_p8  = (double) ((int8_t) (data[NVM_PAR_P8])) / POW_2_P15;
    calib_data->par_p9  = (double) ((int16_t) CONCAT_BYTES(data[NVM_PAR_P9_MSB], data[NVM_PAR_P9_LSB])) / POW_2_P48;
    calib_data->par_p10 = (double) ((int8_t) (data[NVM_PAR_P10])) / POW_2_P48;
    calib_data->par_p11 = (double) ((int8_t) (data[NVM_PAR_P11])) / POW_2_P65;

    return ESP_OK;
}
