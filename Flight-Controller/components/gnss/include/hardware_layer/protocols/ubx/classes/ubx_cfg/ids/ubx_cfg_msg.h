/**
 * @file ubx_cfg_msg.h
 * @brief UBX-CFG-MSG Message
 */

#ifndef UBX_CFG_MSG_H
#define UBX_CFG_MSG_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>

typedef struct ubx_cfg_msg {
    unsigned char msgClass;         /* Message Class */
    unsigned char msgID;            /* Message Identifier */
    unsigned char rate[6];          /* Send rate on I/O Port (6 Ports) */
} ubx_cfg_msg_t;

/**
 * @brief Parses UBX-CFG-MSG message payload
 * 
 * @param payload Pointer to the payload data
 * @param payload_len Length of the payload data
 * @param ubx_cfg_msg Pointer to the ubx_cfg_msg_t structure to fill
 * 
 * @retval
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG Invalid payload length
 */
esp_err_t gnss_parse_cfg_msg(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_msg_t *ubx_cfg_msg);

#endif
