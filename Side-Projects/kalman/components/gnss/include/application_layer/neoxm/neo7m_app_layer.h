#ifndef NEO7M_APP_LAYER_H
#define NEO7M_APP_LAYER_H

/**
 * @file neo7m_app_layer.h
 * @brief NEO-7M Applcation Layer
 */

#include <application_layer/gnss_app_layer.h>


/* ========== Public functions ========== */

/**
 * @brief Send messages to receiver and wait for response. Then, update Gnss object attributes with it
 * 
 * @param gnss: Instance of Gnss Class
 * 
 * @retval
 *      - ESP_OK
 */
esp_err_t gnss_neo7m_measure(gnss_t *gnss);


/**
 * @brief Initialize Gnss object
 * 
 * @param gnss: Pointer to the Gnss instance
 * @param gnss_params: GNSS parameters needed to initialize a Gnss instance
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t gnss_neo7m_init(gnss_t *gnss, gnss_params_t gnss_params);

/**
 * @brief Update satellite vehicles data
 * 
 * @param gnss: Instance of Gnss Class
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to read UBX-NAV-SVINFO message
 */
esp_err_t gnss_neo7m_update_SVs_data(gnss_t *gnss);

/**
 * @brief Check if vertical dilution of precision (vDOP) is valid
 * 
 * @param vDOP: Vertical dilution of precision value
 * 
 * @retval
 *      - true: vDOP is acceptable
 *      - false: vDOP is not acceptable
 */
bool gnss_neo7m_vDOP_is_valid(double vDOP);

#endif
