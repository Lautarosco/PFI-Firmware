#ifndef GNSS_APP_LAYER_H
#define GNSS_APP_LAYER_H

#include <hardware_layer/gnss_hwl_layer.h>

typedef enum gnss_protocol_enable {
    GNSS_ENABLE_UBX_ONLY,           /* Enable only UBX protocol */
    GNSS_ENABLE_NMEA_ONLY,          /* Enable only NMEA protocol */
    GNSS_ENABLE_ALL                 /* Enable both NMEA and UBX protocols */
} gnss_protocol_enable_t;

typedef struct gnss_params {
    uart_port_t uart_port;                      /* UART port number */
    gnss_protocol_enable_t gnss_protocol;       /* GNSS protocol to be used */
} gnss_params_t;

typedef struct gnss gnss_t;

typedef struct gnss {
    uart_port_t uart_port;      /* UART port number */

    /**
     * @brief Disable NMEA messages
     * 
     * @param gnss: Pointer to the Gnss instance
     * @param uart_port: UART port number to be used for communication
     * 
     * @retval
     *      - ESP_OK: Success
     *      - ESP_ERR_INVALID_ARG: Invalid argument
     */
    esp_err_t (*init)(gnss_t *gnss, gnss_params_t gnss_params);
} gnss_t;

/**
 * @brief Make an instance of Gnss Class
 * 
 * @param gnss: Pointer to the Gnss instance
 * 
 * @retval none
 */
void Gnss(gnss_t *gnss);

#endif
