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

/**
 * @file gnss_hwl_layer.h
 * @brief GNSS Hardware Layer
 */

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
inline uint8_t msb_16b(uint16_t x) {
    return (x & (0xFF << 8)) >> 8;
}

/**
 * @brief Get the least significant byte of a 16-bit integer
 * 
 * @param x: 16-bit integer value
 * 
 * @retval 8-bit LSB of x
 */
inline uint8_t lsb_16b(uint16_t x) {
    return x & 0xFF;
}

/**
 * @brief Make an UBX packet with the given payload
 * 
 * @param packet: Pointer to the buffer where the packet will be stored
 * @param packet_len: Length of the packet buffer
 * @param class: Class of the UBX message
 * @param id: ID of the UBX message
 * @param payload: Pointer to the payload data
 * @param len: Length of the payload in bytes
 * @param final_packet_len: Pointer to store the final packet length (including header, class, id, payload, and both checksums)
 * 
 * @retval
 *      - ESP_OK: Packet created successfully
 *      - ESP_ERR_INVALID_SIZE: Payload length exceeds the size of a 16-bit integer or the final packet size exceeds the maximum size of a UBX packet
 */
esp_err_t gnss_hwl_make_packet(gnss_packet_t *packet, gnss_packet_len_t packet_len, const gnss_class_t class, const gnss_id_t id,
    const gnss_payload_t *payload, gnss_payload_len_t payload_len, gnss_packet_len_t *final_packet_len);

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
 * @brief Makes an UBX poll/request message based on given <class> and <id>, and write it to UART Tx buffer. Then Read UART Rx buffer until receiver respond
 * 
 * @note <response> buffer contains ONLY the payload from receiver response
 * 
 * @param uart_port: UART port number to be used for communication
 * @param payload_len: Length of payload (Expected payload from receiver response)
 * @param class: Class ID to be checked
 * @param id: Message ID to be checked
 * @param response: Pointer to buffer to store receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to make UBX packet
 *      - ESP_ERR_TIMEOUT: Receiver response timeout
 */
esp_err_t gnss_hwl_read_ubx_msg(uart_port_t uart_port, gnss_payload_len_t payload_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *response);

/**
 * @brief Check if given Class ID and Message ID matches with Class ID and Messsage ID of given packet
 * 
 * @param packet: Pointer to UBX packet
 * @param packet_len: Length of UBX packet
 * @param class: Class ID to be checked
 * @param id: Message ID to be checked
 * 
 * @retval
 *      - 1: Match
 *      - 0: No matches
 *      - (-1): Error
 */
int gnss_hwl_msg_check(gnss_packet_t *packet, gnss_packet_len_t packet_len, gnss_class_t class, gnss_id_t id);

/**
 * @brief Send an UBX-NAV-PVT message to receiver and wait for response. Then parse received paylaod
 * 
 * @param uart_port: UART port number to be used for communication
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Read UBX-NAV-PVT message failed
 */
esp_err_t gnss_hwl_read_nav_pvt(uart_port_t uart_port);

#endif
