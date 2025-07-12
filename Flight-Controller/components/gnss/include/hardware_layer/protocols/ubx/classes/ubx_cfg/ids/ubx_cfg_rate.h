/**
 * @file ubx_cfg_rate.h
 * @brief UBX-CFG-RATE message
 */


#ifndef UBX_CFG_RATE_H
#define UBX_CFG_RATE_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>


typedef struct ubx_cfg_rate {
    unsigned short measRate;            /* Measurement Rate, GPS measurements are taken every measRate milliseconds */
    unsigned short navRate;             /* Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
    unsigned short timeRef;             /* Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_cfg_rate_t;


/**
 * @brief Parse an UBX-CFG-RATE message
 * 
 * @param paylaod: Pointer to payload of UBX-CFG-RATE message
 * @param payload_len: Length of payload
 * @param cfg_rate: Pointer to store each UBX-CFG-RATE field
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid payload length
 */
esp_err_t gnss_parse_cfg_rate(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_rate_t *cfg_rate);

#endif
