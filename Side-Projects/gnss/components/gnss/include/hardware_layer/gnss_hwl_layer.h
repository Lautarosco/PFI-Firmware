/**
 * @file gnss_hwl_layer.h
 * @brief GNSS Hardware Layer
 */


#ifndef GNSS_HWL_LAYER_H
#define GNSS_HWL_LAYER_H

#include <stdint.h>                                                     /* Standard integer types */

#include <hardware_layer/protocols/ubx/neo7_ubx_classes.h>              /* UBX classes */
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>             /* UBX messages */
#include <hardware_layer/protocols/ubx/neo7_ubx_packet.h>               /* UBX packet structure */
#include <hardware_layer/protocols/nmea/neo7_nmea_messages.h>           /* NMEA messages */
#include <hardware_layer/gnss_types.h>                                  /* Data types */

#include <driver/uart.h>                                                /* ESP-IDF UART driver */

#include <esp_err.h>                                                    /* ESP-ERR library */
#include <esp_log.h>                                                    /* ESP-IDF logging library */


/* ========== GNSS General Specifications ========== */

#define GNSS_MIN_SVS                        0x03            /* Minimum amount of Satellite Vehicles needed by receiver to calculate a navigation solution */
#define GNSS_UBX_DEFAULT_PACKET_LEN         0x08            /* UBX default length is 8 bytes (if payload = 0 bytes) */
#define GNSS_STATIC_HOLD_DEFAULT            0x00            /* Default static hold threshold value */
#define GNSS_MEASRATE_DEFAULT               1000            /* Default GPS measurement rate in ms */

typedef enum ubx_msg_status {
    UBX_MSG_INVALID = -1,
    UBX_MSG_NACK,
    UBX_MSG_VALID,
    UBX_MSG_ACK
} ubx_msg_status_t;

typedef enum ubx_msg_mode {
    UBX_MSG_POLLING,            /* UBX message is obtained by polling it */
    UBX_MSG_PERIODIC            /* Device will send UBX message regularly */
} ubx_msg_mode_t;


/* ========== Hardware Layer functions ========== */

/**
 * @brief Calculate the UBX checksum for a given payload
 * 
 * @param ck_a: Pointer to the first checksum byte
 * @param ck_b: Pointer to the second checksum byte
 * @param packet: UBX packet
 * @param len: Payload length in bytes
 * 
 * @retval none
 */
esp_err_t ubx_get_payload_checksum(gnss_checksum_t *ck_a, gnss_checksum_t *ck_b, const gnss_packet_t *packet, gnss_payload_len_t len);

/**
 * @brief Get the most significant byte of a 16-bit integer
 * 
 * @param x: 16-bit integer value
 * 
 * @retval 8-bit MSB of x
 */
inline uint8_t MSB_16B(uint16_t x) {
    return (x & (0xFF << 8)) >> 8;
}

/**
 * @brief Get the least significant byte of a 16-bit integer
 * 
 * @param x: 16-bit integer value
 * 
 * @retval 8-bit LSB of x
 */
inline uint8_t LSB_16B(uint16_t x) {
    return x & 0xFF;
}

/**
 * @brief Get the payload length from an UBX packet
 * 
 * @param packet: Pointer to the UBX packet
 * 
 * @retval 16-bit payload length
 */
static inline gnss_payload_len_t ubx_get_payload_length(const gnss_packet_t *packet) {
    return ((gnss_payload_len_t) (packet[UBX_PACKET_POS_LENGTH_MSB] << 8)) | ((gnss_payload_len_t) packet[UBX_PACKET_POS_LENGTH_LSB]);
}

/**
 * @brief Check if UBX packet header is valid
 * 
 * @param packet: Pointer to the UBX packet
 * @param packet_len: Length of the UBX packet
 * 
 * @retval
 *      - true: UBX packet header is valid
 *      - false: UBX packet header is not valid or packet is NULL
 */
bool check_ubx_header(const gnss_packet_t *packet, gnss_packet_len_t packet_len);

/**
 * @brief Check if an UBX packet is valid
 * 
 * @param packet: Pointer to the UBX packet
 * @param packet_len: Length of the UBX packet
 * 
 * @retval
 *     - true: UBX packet is valid
 *      - false: UBX packet is not valid or packet is NULL
 */
bool is_packet_valid(const gnss_packet_t *packet, gnss_packet_len_t packet_len);

/**
 * @brief Process an UBX packet from receiver response and check if its an ACK/NACK/MSG-ACK or an invalid packet
 * 
 * @param packet: Pointer to the UBX packet
 * @param packet_len: Length of the UBX packet
 * @param class: Class ID to be checked
 * @param id: Message ID to be checked
 * @param func_caller: Name of the fuction which is requesting a polling to some UBX message
 * 
 * @retval
 *      - UBX_MSG_INVALID: Invalid UBX packet
 *      - UBX_MSG_NACK: Receiver replied with a NACK
 *      - UBX_MSG_VALID: Receiver replied with an echo (but filling the payload with measured data) or with a MSG-ACK response
 *      - UBX_MSG_ACK: Receiver replied with a ACK
 */
ubx_msg_status_t check_ubx_msg(const gnss_packet_t *packet, gnss_packet_len_t packet_len, gnss_class_t class, gnss_id_t id, const char *func_caller);

/**
 * @brief Make an UBX packet with the given payload
 * 
 * @param packet: Pointer to the buffer where the packet will be stored
 * @param packet_len: Length of the packet buffer
 * @param class: Class of the UBX message
 * @param id: ID of the UBX message
 * @param payload: Pointer to the payload data
 * @param payload_len: Length of the payload in bytes
 * @param final_packet_len: Pointer to store the final packet length (including header, class, id, payload, and both checksums)
 * 
 * @retval
 *      - ESP_OK: Packet created successfully
 *      - ESP_ERR_INVALID_SIZE: Payload length exceeds the size of a 16-bit integer or the final packet size exceeds the maximum size of a UBX packet
 */
esp_err_t gnss_hwl_make_packet(gnss_packet_t *packet, gnss_packet_len_t packet_len, const gnss_class_t class, const gnss_id_t id,
    const gnss_payload_t *payload, gnss_payload_len_t payload_len, gnss_packet_len_t *final_packet_len);

/**
 * @brief Send an UBX packet to the GNSS receiver via UART
 * 
 * @param uart_port: UART port number to be used for communication
 * @param packet: Pointer to the UBX packet to be sent
 * @param packet_len: Length of the UBX packet
 * 
 * @retval
 *      - ESP_OK: Packet sent successfully
 *      - ESP_ERR_INVALID_ARG: UBX packet is not valid
 *      - ESP_FAIL: Failed to write UBX packet to UART port
 */
esp_err_t gnss_hwl_send_packet(uart_port_t uart_port, gnss_packet_t *packet, gnss_packet_len_t packet_len);

/**
 * @brief Receive an UBX packet from the GNSS receiver via UART
 * 
 * @param uart_port: UART port number to be used for communication
 * @param class: Class ID of the UBX message to be received
 * @param id: Message ID of the UBX message to be received
 * @param response: Pointer to buffer to store the payload from the received UBX packet
 * @param payload_len: Length of the payload in bytes
 * @param func_caller: Name of the fuction which is requesting a polling to some UBX message
 * @param should_wait: Flag to determine if the function must wait until receiver reply, or just read ONCE and dont mind if data is ready or not
 * 
 * @retval
 *      - ESP_OK: Packet received successfully
 *      - ESP_FAIL: Failed to read UBX packet from UART port
 *      - ESP_ERR_TIMEOUT: Timeout waiting for receiver response
 */
esp_err_t gnss_hwl_receive_packet(uart_port_t uart_port, gnss_class_t class, gnss_id_t id, gnss_payload_t *response, gnss_payload_len_t payload_len, const char *func_caller, bool should_wait);

/**
 * @brief Make an UBX packet for disabling a specific NMEA message
 * 
 * @param packet: Pointer to the buffer where the UBX packet will be stored
 * @param packet_len: Length of the packet buffer
 * @param nmea_msg: NMEA message ID to disable
 * 
 * @retval
 *      - ESP_OK: NMEA message disabled successfully
 *     - ESP_ERR_INVALID_SIZE: Payload length exceeds the size of a 16-bit integer or the final packet size exceeds the maximum size of a UBX packet
 */
esp_err_t gnss_hwl_disable_nmea_msg_packet(gnss_packet_t packet[UBX_CFG_MSG_LEN_8 + 8], gnss_packet_len_t packet_len, const gnss_id_t nmea_msg);

/**
     * @brief Disable NMEA messages
     * 
     * @note Before exiting the function, UART Rx buffer is flushed in order to remove any pending data
     * 
     * @param uart_port: UART port number to be used for communication
     * 
     * @retval
     *      - ESP_OK: Success
     *      - ESP_FAIL
     */
esp_err_t gnss_hwl_disable_nmea(uart_port_t uart_port);

/**
 * @brief Read UART Rx buffer until receiver respond with an ACK or NACK
 * 
 * @param uart_port: UART port number to be used for communication 
 * @param ack_class: Class ID of ACK/NACK response
 * @param ack_id: Message ID of ACK/NACK response
 * 
 * @retval
 *      - ESP_OK: Command was acknowledged
 *      - ESP_FAIL: UART read failed
 *      - ESP_ERR_INVALID_RESPONSE: Command was not-acknowledged
 */
esp_err_t gnss_hwl_ack_nack_detect(uart_port_t uart_port, gnss_class_t *ack_class, gnss_id_t *ack_id);

/**
 * @brief Makes an UBX poll/request message based on given <class> and <id>. Then write it to UART Tx buffer and read UART Rx buffer until receiver responds
 * 
 * @note <response> buffer contains ONLY the payload from receiver response
 * 
 * @param uart_port: UART port number to be used for communication
 * @param payload_len: Length of payload (Expected payload from receiver response)
 * @param class: Class ID to be checked
 * @param id: Message ID to be checked
 * @param response: Pointer to buffer to store the payload from receiver response
 * @param func_caller: Name of the fuction which is requesting a polling to some UBX message
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to make UBX packet
 *      - ESP_ERR_TIMEOUT: Receiver response timeout
 */
esp_err_t gnss_hwl_poll_request(uart_port_t uart_port, gnss_payload_len_t payload_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *response, const char *func_caller);

/**
 * @brief Send an UBX-NAV-PVT message to receiver and wait for response. Then parse received paylaod
 * 
 * @param uart_port: UART port number to be used for communication
 * @param nav_pvt: Pointer to ubx_nav_pvt_t instance to update it with UBX-NAV-PVT receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-NAV-PVT message failed
 */
esp_err_t gnss_hwl_read_nav_pvt(uart_port_t uart_port, ubx_nav_pvt_t *nav_pvt);

/**
 * @brief Send an UBX-CFG-RATE message to receiver and wait for response. Then parse received payload
 * 
 * @param uart_port: UART port number to be used for communication
 * @param cfg_rate: Pointer to ubx_cfg_rate_t instance to update it with UBX-CFG-RATE receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-CFG-RATE message failed
 */
esp_err_t gnss_hwl_read_cfg_rate(uart_port_t uart_port, ubx_cfg_rate_t *cfg_rate);

/**
 * @brief Set the measurement rate of the GNSS receiver
 * 
 * @param uart_port: UART port number to be used for communication
 * @param measure_rate_ms: Measurement rate in milliseconds
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to set measurement rate
 */
esp_err_t gnss_hwl_set_meas_rate(uart_port_t uart_port, unsigned short measure_rate_ms);

/**
 * @brief Read the UBX-NAV-SVINFO message from the GNSS receiver
 * 
 * @param uart_port: UART port number to be used for communication
 * @param ubx_nav_svinfo: Pointer to unx_nav_svinfo_t instance to update it with UBX-NAV-SVINFO receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-NAV-SVINFO message failed
 */
esp_err_t gnss_hwl_read_nav_svinfo(uart_port_t uart_port, ubx_nav_svinfo_t *ubx_nav_svinfo);

/**
 * @brief Read the UBX-CFG-NAV5 message from the GNSS receiver
 * 
 * @param uart_port: UART port number to be used for communication
 * @param ubx_cfg_nav5: Pointer to ubx_cfg_nav5_t instance to update it with UBX-CFG-NAV5 receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-CFG-NAV5 message failed
 */
esp_err_t gnss_hwl_read_cfg_nav5(uart_port_t uart_port, ubx_cfg_nav5_t *ubx_cfg_nav5);

/**
 * @brief Set the dynamic platform model of the GNSS receiver
 * 
 * @param uart_port: UART port number to be used for communication
 * @param dynModel: Dynamic platform model to be set
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to set dynamic platform model
 */
esp_err_t gnss_hwl_set_dynModel(uart_port_t uart_port, ubx_cfg_nav5_dynModel_t dynModel);

/**
 * @brief Set the static hold threshold [m/s] of the GNSS receiver
 * 
 * @note: Below <static_hold_threshold> velocity, the receiver will increase accuracy of measurements
 * 
 * @param uart_port: UART port number to be used for communication
 * @param static_hold_threshold: Static hold threshold in cm/s
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to set static hold threshold
 */
esp_err_t gnss_hwl_set_static_hold_threshold(uart_port_t uart_port, unsigned char static_hold_threshold);

/**
 * @brief Read the UBX-NAV-DOP message from the GNSS receiver
 * 
 * @param uart_port: UART port number to be used for communication
 * @param ubx_nav_dop: Pointer to ubx_nav_dop_t instance to update it with UBX-NAV-DOP receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-NAV-DOP message failed
 */
esp_err_t gnss_hwl_read_nav_dop(uart_port_t uart_port, ubx_nav_dop_t *ubx_nav_dop);

/**
 * @brief Set an UBX message as periodic or disable if its already enabled
 * 
 * @note: UBX message mode is only enabled/disabled in UART port (this function will not have effect on I2C, SPI nor USB)
 * 
 * @param uart_port: UART port number to be used for communication
 * @param class: Class ID to be checked
 * @param id: Message ID to be checked
 * @param mode: Mode of transaction of UBX message (polling or periodic)
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL
 */
esp_err_t gnss_hwl_set_uart_msg_mode(uart_port_t uart_port, gnss_class_t class, gnss_id_t id, ubx_msg_mode_t mode);

/**
 * @brief Loop through data and seek for an UBX message with Class = <class> and ID = <id>. If found, copy payload into <payload> buffer
 * 
 * @note If payload length es greater than buffer length <payload_len>, then an error will be returned
 * 
 * @param data: Data to be looped
 * @param data_len: Length of data
 * @param class: Class of UBX message
 * @param id: ID of UBX message
 * @param payload: Empty buffer to store UBX payload
 * @param payload_len: Length of buffer
 * @param rx_pyld_len: Pointer to variable to store actual payload length
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG
 *      - ESP_FAIL
 */
esp_err_t gnss_hwl_get_msg_payload(uint8_t *data, unsigned int data_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *payload, gnss_payload_len_t payload_len, gnss_payload_len_t *rx_pyld_len);

#endif
