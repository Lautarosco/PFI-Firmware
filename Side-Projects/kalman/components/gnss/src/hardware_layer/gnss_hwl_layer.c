/**
 * @file gnss_hwl_layer.c
 * @brief GNSS Hardware Layer
 */

#include <hardware_layer/gnss_hwl_layer.h>      /* GNSS hardware layer */

#include <stdbool.h>

#include <string.h>

/**
 * @brief Detect ACK receiver response
 * 
 * @param class: Class ID of the acknowledged message
 * @param id: Message ID of the acknowledged message
 * 
 * @retval
 *      - true: ACK response
 *      - false: NACK response or other message
 */
static inline bool GNSS_ACK(gnss_class_t class, gnss_id_t id) {return (class == UBX_CLASS_ACK) && (id == UBX_ACK_ACK_ID);}

/**
 * @brief Detect NACK receiver response
 * 
 * @param class: Class ID of the not-acknowledged message
 * @param id: Message ID of the not-acknowledged message
 * 
 * @retval
 *      - true: NACK response
 *      - false: ACK response or other message
 */
static inline bool GNSS_NACK(gnss_class_t class, gnss_id_t id) {return (class == UBX_CLASS_ACK) && (id == UBX_ACK_NACK_ID);}


/* ========== Private variables ========== */

const char *gnss_hwl_tag = "[GNSS_HW_LAYER]";


/* ========== Public functions ========== */

/* ========== Useful functions ========== */

esp_err_t ubx_get_payload_checksum(gnss_checksum_t *ck_a, gnss_checksum_t *ck_b, const gnss_packet_t *packet, gnss_payload_len_t len) {
    *ck_a = 0;
    *ck_b = 0;

    if(sizeof(len) > sizeof(uint16_t)) {
        return ESP_ERR_INVALID_SIZE;  /* Payload length exceeds the size of a 16-bit integer */
    }

    for(int i = UBX_PACKET_POS_CLASS; i < (UBX_PACKET_POS_PAYLOAD + len); i++) {
        *ck_a += packet[i];
        *ck_b += *ck_a;
    }

    return ESP_OK;
}

bool check_ubx_header(const gnss_packet_t *packet, gnss_packet_len_t packet_len) {
    if((packet_len < 2) || (packet == NULL)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Invalid UBX packet", __func__, __LINE__);
        return false;
    }

    return (packet[UBX_PACKET_POS_SYNC1] == UBX_SYNC1) && (packet[UBX_PACKET_POS_SYNC2] == UBX_SYNC2);
}

bool is_packet_valid(const gnss_packet_t *packet, gnss_packet_len_t packet_len) {
    if(packet_len < 8) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Invalid UBX packet length", __func__, __LINE__);
        return false;
    }
    
    if(!check_ubx_header(packet, packet_len)) {
        return false;
    }

    gnss_payload_len_t payload_len = ubx_get_payload_length(packet);
    if(packet_len < (payload_len + 8)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Actual packet length is more than expected", __func__, __LINE__);
        return false;  /* Packet length is more than the expected length */
    }

    gnss_checksum_t ck_a = 0;
    gnss_checksum_t ck_b = 0;

    ubx_get_payload_checksum(&ck_a, &ck_b, packet, payload_len);
    
    return (packet[UBX_PACKET_POS_PAYLOAD + payload_len] == ck_a) && (packet[UBX_PACKET_POS_PAYLOAD + (payload_len + 1)] == ck_b);
}

ubx_msg_status_t check_ubx_msg(const gnss_packet_t *packet, gnss_packet_len_t packet_len, gnss_class_t class, gnss_id_t id, const char *func_caller) {
    // printf("Receiver response (%d bytes): ", packet_len);
    // for(int i = 0; i < packet_len; i++) {
    //     printf("0x%X ", packet[i]);
    // }
    // printf("\n\n");

    /* 1. Check if data is at least 8 bytes (default UBX packet length if payload = 0 bytes) */
    if(packet_len < GNSS_UBX_DEFAULT_PACKET_LEN) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: [Caller: %s] UBX packet must be at least 8 bytes", __func__, __LINE__, func_caller);
        return UBX_MSG_INVALID;
    }

    /* 2. Loop through received data */
    for(int k = 0; k < (packet_len - UBX_PACKET_POS_PAYLOAD); k++) {
        /* a. Wait until UBX packet header is found */
        if((packet[k] == UBX_SYNC1) && (packet[k + UBX_PACKET_POS_SYNC2] == UBX_SYNC2)) {
            /* c. Check if remain data is at least 8 bytes long */
            if(k + GNSS_UBX_DEFAULT_PACKET_LEN > packet_len) {
                ESP_LOGE(
                    gnss_hwl_tag,
                    "{Function %s in line %d}: [Caller: %s] Not enough bytes for an UBX packet (8 bytes at least if payload = 0 bytes)",
                    __func__, __LINE__, func_caller
                );
                return UBX_MSG_INVALID;
            }

            gnss_checksum_t ck_a = 0;
            gnss_checksum_t ck_b = 0;

            /* NEO-7M sends payload length in little-endian format (LSB-MSB). Thus, it must be converted to big-endian format (just the normal MSB-LSB) */
            gnss_payload_len_t payload_len = ((gnss_payload_len_t) (packet[k + UBX_PACKET_POS_LENGTH_MSB] << 8)) | ((gnss_payload_len_t) packet[k + UBX_PACKET_POS_LENGTH_LSB]);

            /* Check if there are enough bytes for UBX default length + received payload bytes */
            if(payload_len + GNSS_UBX_DEFAULT_PACKET_LEN > packet_len) {
                ESP_LOGE(
                    gnss_hwl_tag,
                    "{Function %s in line %d}: Received UBX response (%d bytes) is greater than expected (%d bytes)",
                    __func__, __LINE__,
                    payload_len + GNSS_UBX_DEFAULT_PACKET_LEN, packet_len
                );
                return UBX_MSG_INVALID;
            }

            /* d. Check if its an ACK response --> UBX-CLASS-ID = UBX-ACK-ACK */
            if((packet[k + UBX_PACKET_POS_CLASS] == UBX_CLASS_ACK) && (packet[k + UBX_PACKET_POS_ID] == UBX_ACK_ACK_ID)) {
                /* Calculate checksum over aparent UBX packet */
                ubx_get_payload_checksum(&ck_a, &ck_b, &(packet[k]), payload_len);

                /* Check if checksum bytes match with bytes in the buffer */
                if((packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)] != ck_a) || (packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)] != ck_b)) {
                    ESP_LOGE(
                        gnss_hwl_tag,
                        "{Function %s in line %d}: [Caller: %s] Invalid checksum bytes <0x%X 0x%x> for ACK response. Calculated checksum <0x%X 0x%X>",
                        __func__, __LINE__,
                        func_caller,
                        packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)], packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)],
                        ck_a, ck_b
                    );
                    return UBX_MSG_INVALID;
                }

                /* Checksum is correct, thus UBX receiver response packet structure is correct */
                ESP_LOGI(
                    gnss_hwl_tag,
                    "{Function %s in line %d}: [Caller: %s] Receiver replied with an ACK for message Class: <0x%x>, ID: <0x%X>",
                    __func__, __LINE__,
                    func_caller,
                    class, id
                );
                return UBX_MSG_ACK;
            }

            /* e. Check if its an NACK response --> UBX-CLASS-ID = UBX-ACK-NACK */
            if((packet[k + UBX_PACKET_POS_CLASS] == UBX_CLASS_ACK) && (packet[k + UBX_PACKET_POS_ID] == UBX_ACK_NACK_ID)) {
                /* Calculate checksum over aparent UBX packet */
                ubx_get_payload_checksum(&ck_a, &ck_b, &(packet[k]), payload_len);

                /* Check if checksum bytes match with bytes in the buffer */
                if((packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)] != ck_a) || (packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)] != ck_b)) {
                    ESP_LOGE(
                        gnss_hwl_tag,
                        "{Function %s in line %d}: [Caller: %s] Invalid checksum bytes <0x%X 0x%x> for NACK response. Calculated checksum <0x%X 0x%X>",
                        __func__, __LINE__,
                        func_caller,
                        packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)], packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)],
                        ck_a, ck_b
                    );
                    return UBX_MSG_INVALID;
                }

                /* Checksum is correct, thus UBX receiver response packet structure is correct */
                ESP_LOGE(
                    gnss_hwl_tag,
                    "{Function %s in line %d}: [Caller: %s] Receiver replied with a NACK for message Class: <0x%X>, ID: <0x%X>",
                    __func__, __LINE__,
                    func_caller,
                    class, id
                );
                return UBX_MSG_NACK;
            }

            /* f. Check if its an UBX-<class>-<id> response --> UBX-CLASS-ID = UBX-<class>-<id> */
            if((packet[k + UBX_PACKET_POS_CLASS] == class) && (packet[k + UBX_PACKET_POS_ID] == id)) {
                /* First, calculate checksum over the aparent UBX packet */
                ubx_get_payload_checksum(&ck_a, &ck_b, &(packet[k]), payload_len);

                /* Check if checksum bytes match with bytes in the buffer */
                if((packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)] != ck_a) || (packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)] != ck_b)) {
                    ESP_LOGE(
                        gnss_hwl_tag,
                        "{Function %s in line %d}: [Caller: %s] Invalid checksum bytes <0x%X 0x%x> for <Class: 0x%X>, <ID: 0x%X> response. Calculated checksum <0x%X 0x%X>",
                        __func__, __LINE__,
                        func_caller,
                        packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len)], packet[k + (UBX_PACKET_POS_PAYLOAD + payload_len + 1)],
                        class, id,
                        ck_a, ck_b
                    );
                    return UBX_MSG_INVALID;
                }

                /* Check if receiver replied with an UBX-CFG-XXX message */
                if(class == UBX_CLASS_CFG) {
                    /* In this case, receiver responds with the same message UBX-<CFG>-<id>, and then with an UBX-ACK-ACK */
                    /* Hence, we should check if there are at least 10 bytes extra related to the ACK respond */
                    if(packet_len - (payload_len + GNSS_UBX_DEFAULT_PACKET_LEN + k) >= (GNSS_UBX_DEFAULT_PACKET_LEN + UBX_ACK_ACK_LEN_2)) {
                        /* Now we have to process remain ACK message */

                        /* Calculate starting position of UBX-ACK-ACK response */
                        unsigned int ack_init_pos = k + payload_len + GNSS_UBX_DEFAULT_PACKET_LEN;

                        /* Check if UBX header is correct */
                        if((packet[ack_init_pos] == UBX_SYNC1) && (packet[ack_init_pos + 1] == UBX_SYNC2)) {
                            /* Calculate checksum over the UBX-ACK-ACK message response  */
                            ubx_get_payload_checksum(&ck_a, &ck_b, &(packet[ack_init_pos]), UBX_ACK_ACK_LEN_2);

                            /* Check if checksum bytes match with bytes in the buffer */
                            if((packet[ack_init_pos + 6 + UBX_ACK_ACK_LEN_2] != ck_a) || (packet[ack_init_pos + 7 + UBX_ACK_ACK_LEN_2] != ck_b)) {
                                ESP_LOGE(
                                    gnss_hwl_tag,
                                    "{Function %s in line %d}: [Caller: %s] Invalid checksum bytes <0x%X 0x%X> for NACK response. Calculated checksum <0x%X 0x%X>",
                                    __func__, __LINE__,
                                    func_caller,
                                    packet[ack_init_pos + 6 + UBX_ACK_ACK_LEN_2], packet[ack_init_pos + 7 + UBX_ACK_ACK_LEN_2],
                                    ck_a, ck_b
                                );
                                return UBX_MSG_INVALID;
                            }

                            /* UBX-ACK-ACK checksum is correct */
                            ESP_LOGI(
                                gnss_hwl_tag,
                                "{Function %s in line %d}: [Caller: %s] Receiver updated payload and replied with a MSG-ACK message for Class: 0x%x, ID: 0x%X",
                                __func__, __LINE__,
                                func_caller,
                                class, id
                            );
                            return UBX_MSG_VALID;
                        }

                        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: [Caller: %s] Invalid UBX-ACK-ACK header", __func__, __LINE__, func_caller);
                        return UBX_MSG_INVALID; 
                    }

                    /* Despite the fact that the receiver replied with the message, the ACK is missing so its considered as an invalid response */
                    ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: [Caller: %s] There are not enough bytes for an UBX-ACK-ACK message", __func__, __LINE__, func_caller);
                    return UBX_MSG_INVALID;        
                }

                /* For other UBX classes, receiver is just going to reply with same UBX message class and id */
                ESP_LOGI(
                    gnss_hwl_tag,
                    "{Function %s in line %d}: [Caller: %s] Receiver updated payload for message Class: 0x%x, ID: 0x%X",
                    __func__, __LINE__,
                    func_caller,
                    class, id
                );

                return UBX_MSG_VALID;
            }

            /* g. Up to this point, either its an invalid UBX packet or it could also be a valid UBX packet but its just not the one we want */
            // ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: [Caller: %s] Invalid receiver response", __func__, __LINE__, func_caller);
            // return UBX_MSG_INVALID;
            ESP_LOGW(gnss_hwl_tag, "Either its an invalid UBX packet or its not the packet we want. Thus, just moving on");
            continue;
        }

        /* b. Dummy data, just continue to next element */
        continue;
    }

    return UBX_MSG_INVALID;
}


/* ========== Hardware functions ========== */

esp_err_t gnss_hwl_make_packet(gnss_packet_t *packet, gnss_packet_len_t packet_len, const gnss_class_t class, const gnss_id_t id,
    const gnss_payload_t *payload, gnss_payload_len_t payload_len, gnss_packet_len_t *final_packet_len) {

    packet[UBX_PACKET_POS_SYNC1] = UBX_SYNC1;      /* First byte of the header of a UBX packet */
    packet[UBX_PACKET_POS_SYNC2] = UBX_SYNC2;      /* Second byte of the header of a UBX packet */

    packet[UBX_PACKET_POS_CLASS] = class;   /* Class of the message */
    packet[UBX_PACKET_POS_ID] = id;         /* ID of the message */

    if(sizeof(payload_len) > sizeof(uint16_t)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Payload length exceeds the size of a 16-bit integer", __func__, __LINE__);
        return ESP_ERR_INVALID_SIZE;
    }

    packet[UBX_PACKET_POS_LENGTH_LSB] = LSB_16B(payload_len);   /* LSB byte of the payload length */
    packet[UBX_PACKET_POS_LENGTH_MSB] = MSB_16B(payload_len);   /* MSB byte of the payload length */

    memcpy(&(packet[UBX_PACKET_POS_PAYLOAD]), payload, payload_len);    /* Copy the payload into the packet */

    uint8_t ck_a = 0;  /* Checksum first byte */
    uint8_t ck_b = 0;  /* Checksum second byte */

    ubx_get_payload_checksum(&ck_a, &ck_b, packet, payload_len);   /* Calculate the checksum for the payload */

    packet[UBX_PACKET_POS_PAYLOAD + payload_len] = ck_a;        /* First byte of the checksum */
    packet[UBX_PACKET_POS_PAYLOAD + (payload_len + 1)] = ck_b;  /* Second byte of the checksum */

    /* From header to payload there are 6 bytes, then payload length <len> its variable and finally we add the 2 bytes from checksum A and B */
    *final_packet_len = UBX_PACKET_POS_PAYLOAD + payload_len + 2;

    if(*final_packet_len > packet_len) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Total packet size <%d> exceeds the size of the given UBX packet buffer <%d>", __func__, __LINE__, packet_len, *final_packet_len);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_send_packet(uart_port_t uart_port, gnss_packet_t *packet, gnss_packet_len_t packet_len) {
    if(!is_packet_valid(packet, packet_len)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: UBX packet is not valid", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    if(uart_write_bytes(uart_port, packet, packet_len) < 0) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Write UBX packet to UART port <%d> --> FAILED", __func__, __LINE__, uart_port);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_receive_packet(uart_port_t uart_port, gnss_class_t class, gnss_id_t id, gnss_payload_t *response,
    gnss_payload_len_t payload_len, const char *func_caller, bool should_wait
) {
    /* Initialize some local variables */
    uint8_t rx_buffer[1024];
    unsigned int uart_read_ticks = 0;               /* RTOS ticks used by uart_read_bytes function */
    unsigned int xDelay = 0;                        /* vTaskDelay in ms */
    unsigned int retries = 500;                     /* Response timeout is 500 * 10 ms = 5000 ms = 5 s */
    int len = 0;                                    /* Length of data read in UART Rx buffer */
    int ack = 0;                                    /* Receiver response ACK flag */
    int nack = 0;                                   /* Receiver response NACK flag */
    bool check_rx_data = false;                     /* Flag to determine if data is an UBX packet and should be processed */
    int ubx_init_pos = 0;                           /* UBX packet initial position within the UART Rx buffer (it may be dummy data before the UBX packet) */
    gnss_payload_len_t rx_payload_len = 0;          /* Payload length reported by receiver */

    /* If <should_wait> is set to true, then we must wait until receiver responds or timeout */
    if(should_wait) {
        uart_read_ticks = 20;
        xDelay = 10;
    }

    while(retries--) {
        len = uart_read_bytes(uart_port, rx_buffer, sizeof(rx_buffer), uart_read_ticks);
        if(len > 0) {
            // printf("Recevied length: %d\n", len);

            /* Check if read data has enough bytes for the payload sent by the receiver */
            for(int i = 0; i < (len - UBX_PACKET_POS_LENGTH_MSB); i++) {
                /* Loop through data until finding the UBX header */
                if((rx_buffer[i] == UBX_SYNC1) && (rx_buffer[i + UBX_PACKET_POS_SYNC2] == UBX_SYNC2)) {
                    rx_payload_len = ((gnss_payload_len_t) (rx_buffer[i + UBX_PACKET_POS_LENGTH_MSB] << 8)) | ((gnss_payload_len_t) rx_buffer[i + UBX_PACKET_POS_LENGTH_LSB]);
                    if(len >= GNSS_UBX_DEFAULT_PACKET_LEN + rx_payload_len) {
                        ubx_init_pos = i;
                        check_rx_data = true;
                        break;
                    }
                }
            }

            if(check_rx_data) {
                int ret = check_ubx_msg(&(rx_buffer[ubx_init_pos]), len - ubx_init_pos, class, id, func_caller);

                if(ret == UBX_MSG_VALID) {
                    break;
                } else if(ret == UBX_MSG_ACK) {
                    ack = true;
                    break;
                } else if(ret == UBX_MSG_NACK) {
                    nack = true;
                    break;
                } else if(ret == UBX_MSG_INVALID) {
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
        }

        /* If function shouldn't wait, then leave */
        if(!should_wait) {
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(xDelay));
    }

    if(!retries) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Waiting for receiver [Class: <0x%X> ID: <0x%X>] response timeout", __func__, __LINE__, class, id);
        return ESP_ERR_TIMEOUT;
    }

    if(nack) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if(ack) {
        return ESP_OK;
    }

    /* If received bytes are less than expected. Then in order to avoid overflow by reading more bytes than what rx_buffer has, update payload length with received sensor data */
    if(payload_len > rx_payload_len) {
        ESP_LOGW(
            gnss_hwl_tag,
            "{Function %s in line %d}: Expected payload length (%d bytes) is greater than received payload (%d bytes). To avoid overflow, only received payload length bytes are copied",
            __func__, __LINE__,
            payload_len, rx_payload_len
        );
        payload_len = rx_payload_len;
    }
    
    memcpy(response, &(rx_buffer[ubx_init_pos + UBX_PACKET_POS_PAYLOAD]), payload_len);
    
    return ESP_OK;
}

esp_err_t gnss_hwl_disable_nmea_msg_packet(gnss_packet_t packet[UBX_CFG_MSG_LEN_8 + 8], gnss_packet_len_t packet_len, const gnss_id_t nmea_msg) {
    unsigned int payload_len = 0;
    uint8_t payload[UBX_CFG_MSG_LEN_8] = {
        NMEA_CLASS,   /* NMEA Class */
        nmea_msg,     /* NMEA Message ID */
        0x00,         /* Rate I2C disable */
        0x00,         /* Rate UART1 disable */
        0x00,         /* UNDEFINED */
        0x00,         /* Rate USB disable */
        0x00,         /* Rate SPI disable */
        0x00          /* RESERVED */
    };

    esp_err_t ret = gnss_hwl_make_packet(packet, packet_len, UBX_CLASS_CFG, UBX_CFG_MSG_ID, payload, UBX_CFG_MSG_LEN_8, &payload_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Disable NMEA message <0x%X> --> FAILED", __func__, __LINE__, nmea_msg);
        return ret;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_disable_nmea(uart_port_t uart_port) {
    uint8_t nmea_messages[] = {
        NMEA_DTM_ID,
        NMEA_GBS_ID,
        NMEA_GGA_ID,
        NMEA_GLL_ID,
        NMEA_GLQ_ID,
        NMEA_GNQ_ID,
        NMEA_GNS_ID,
        NMEA_GPQ_ID,
        NMEA_GRS_ID,
        NMEA_GSA_ID,
        NMEA_GST_ID,
        NMEA_GSV_ID,
        NMEA_RMC_ID,
        NMEA_TXT_ID,
        NMEA_VTG_ID,
        NMEA_ZDA_ID
    };

    for(int i = 0; i < ((sizeof(nmea_messages)) / (sizeof(nmea_messages[0]))); i++) {
        uint8_t ubx_packet[GNSS_UBX_DEFAULT_PACKET_LEN + 8];  /* UBX packet length = 8 bytes + <nmea message payload length> */
        gnss_hwl_disable_nmea_msg_packet(ubx_packet, sizeof(ubx_packet), nmea_messages[i]);

        uart_write_bytes(uart_port, ubx_packet, sizeof(ubx_packet));

        // vTaskDelay(pdMS_TO_TICKS(50));  /* Wait some time to ensure the packet is sent */
        gnss_payload_t response[UBX_ACK_ACK_LEN_2];
        if(gnss_hwl_receive_packet(uart_port, UBX_CLASS_ACK, UBX_ACK_ACK_ID, response, 16, __func__, true) != ESP_OK) {
            ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Disable NMEA message [%d] <0x%X> --> FAILED", __func__, __LINE__, i, nmea_messages[i]);
        } else {
            ESP_LOGI(gnss_hwl_tag, "Disable <0x%X> NMEA message --> OK", nmea_messages[i]);
        }
    }

    uart_flush_input(uart_port);    /* Flush the input buffer to remove any pending data */

    return ESP_OK;
}

esp_err_t gnss_hwl_ack_nack_detect(uart_port_t uart_port, gnss_class_t *ack_class, gnss_id_t *ack_id) {
    uint8_t packet[10];     /* Packet length is 8 bytes + payload. Thus, for ACK-ACK message payload length is 2 bytes */
    int data_len         = 0;       /* Length of data read from UART Rx buffer */
    // unsigned int retries = 10;      /* Max. retries for waiting receiver ACK response */

    bool read = true;
    
    while(read) {
        data_len = uart_read_bytes(uart_port, packet, sizeof(packet), 100);    /* Read UART Rx buffer */

        if(data_len < sizeof(packet)) {
            ESP_LOGW(gnss_hwl_tag, "Partial UBX response (%d bytes). Response ignored", data_len);
            continue;
        }

        printf("Datos recibidos: ");
        for(int i = 0; i < data_len; i++) {
            printf("%c", packet[i]);
        }
        printf("\n\n");

        if(GNSS_ACK(packet[UBX_PACKET_POS_CLASS], packet[UBX_PACKET_POS_ID])) {             /* Check if receiver replied with an ACK */
            ESP_LOGW(gnss_hwl_tag, "ACK detected for Class <0x%X> and Message <0x%X>", packet[UBX_PACKET_POS_PAYLOAD], packet[UBX_PACKET_POS_PAYLOAD + 1]);
            *ack_class = packet[UBX_PACKET_POS_PAYLOAD];
            *ack_id = packet[UBX_PACKET_POS_PAYLOAD + 1];
            read = true;

            return ESP_OK;
        } else if(GNSS_NACK(packet[UBX_PACKET_POS_CLASS], packet[UBX_PACKET_POS_ID])) {     /* Check if receiver replied with an NACK */
            ESP_LOGW(gnss_hwl_tag, "NACK detected for Class <0x%X> and Message <0x%X>", packet[UBX_PACKET_POS_PAYLOAD], packet[UBX_PACKET_POS_PAYLOAD + 1]);
            return ESP_ERR_INVALID_RESPONSE;
        }

        // vTaskDelay(pdMS_TO_TICKS(x));
    }

    ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Timeout waiting for ACK/NACK", __func__, __LINE__);
    return ESP_ERR_TIMEOUT;
}

esp_err_t gnss_hwl_poll_request(uart_port_t uart_port, gnss_payload_len_t payload_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *response, const char *func_caller) {

    gnss_packet_t packet[8 + payload_len];   /* Always 8 bytes + payload length */
    gnss_payload_t payload[] = {};
    gnss_packet_len_t packet_len = 0;

    esp_err_t ret = gnss_hwl_make_packet(packet, sizeof(packet), class, id, payload, sizeof(payload), &packet_len);

    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Make UBX <0x%X 0x%X> packet --> FAILED", __func__, __LINE__, class, id);
        return ESP_FAIL;
    }

    uart_flush_input(uart_port);
    ret = gnss_hwl_send_packet(uart_port, packet, packet_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Send UBX packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ret = gnss_hwl_receive_packet(uart_port, class, id, response, payload_len, func_caller, true);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Receive UBX packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_read_nav_pvt(uart_port_t uart_port, ubx_nav_pvt_t *nav_pvt) {
    gnss_payload_t response[UBX_NAV_PVT_LEN_84];

    /* Send UBX-NAV-PVT message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_poll_request(uart_port, UBX_NAV_PVT_LEN_84, UBX_CLASS_NAV, UBX_NAV_PVT_ID, response, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-NAV-PVT --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_parse_nav_pvt(response, sizeof(response), nav_pvt);

    return ESP_OK;
}

esp_err_t gnss_hwl_read_cfg_rate(uart_port_t uart_port, ubx_cfg_rate_t *cfg_rate) {
    gnss_payload_t response[UBX_CFG_RATE_LEN_6];

    /* Send UBX-CFG-RATE message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_poll_request(uart_port, UBX_CFG_RATE_LEN_6, UBX_CLASS_CFG, UBX_CFG_RATE_ID, response, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-CFG-RATE --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_parse_cfg_rate(response, sizeof(response), cfg_rate);

    return ESP_OK;
}

esp_err_t gnss_hwl_set_meas_rate(uart_port_t uart_port, unsigned short measure_rate_ms) {
    gnss_packet_t packet[8 + UBX_CFG_RATE_LEN_6];
    gnss_payload_t payload[UBX_CFG_RATE_LEN_6] = {
        LSB_16B(measure_rate_ms), MSB_16B(measure_rate_ms),     /* LSB-MSB of <measRate> field (little endian format) */
        LSB_16B(1), MSB_16B(1),                                 /* LSB-MSB of <navRate> field (little endian format) --> MUST BE SET TO 1 */
        LSB_16B(1), MSB_16B(1)                                  /* LSB-MSB of <timeRef> field (little endian format) --> 0 = UTC time, 1 = GPS time */
    };

    gnss_packet_len_t packet_len = 0;

    esp_err_t ret = gnss_hwl_make_packet(packet, sizeof(packet), UBX_CLASS_CFG, UBX_CFG_RATE_ID, payload, sizeof(payload), &packet_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    uart_flush_input(uart_port);
    ret = gnss_hwl_send_packet(uart_port, packet, packet_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Send UBX-CFG-RATE packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_payload_t response[UBX_CFG_RATE_LEN_6];
    ret = gnss_hwl_receive_packet(uart_port, UBX_CLASS_CFG, UBX_CFG_RATE_ID, response, sizeof(response), __func__, true);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Read UBX-CFG-RATE response --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_read_nav_svinfo(uart_port_t uart_port, ubx_nav_svinfo_t *ubx_nav_svinfo) {
    gnss_payload_t response[UBX_NAV_SVINFO_LEN_8 + (12 * GNSS_MAX_CHANNELS)];

    /* Send UBX-NAV-SVINFO message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_poll_request(uart_port, UBX_NAV_SVINFO_LEN_8 + (12 * GNSS_MAX_CHANNELS), UBX_CLASS_NAV, UBX_NAV_SVINFO_ID, response, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-NAV-SVINFO --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ret = gnss_parse_nav_svinfo(response, sizeof(response), ubx_nav_svinfo);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Parse UBX-NAV-SVINFO --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_read_cfg_nav5(uart_port_t uart_port, ubx_cfg_nav5_t *ubx_cfg_nav5) {
    gnss_payload_t response[UBX_CFG_NAV5_LEN_36];

    /* Send UBX-NAV-SVINFO message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_poll_request(uart_port, UBX_CFG_NAV5_LEN_36, UBX_CLASS_CFG, UBX_CFG_NAV5_ID, response, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-CFG-NAV5 --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ret = gnss_parse_cfg_nav5(response, sizeof(response), ubx_cfg_nav5);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Parse UBX-CFG-NAV5 --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_set_dynModel(uart_port_t uart_port, ubx_cfg_nav5_dynModel_t dynModel) {
    esp_err_t ret = ESP_OK;

    /* Get actual value of UBX-CFG-NAV5 */
    gnss_payload_t ubx_cfg_nav5_payload[UBX_CFG_NAV5_LEN_36];

    ret = gnss_hwl_poll_request(uart_port, UBX_CFG_NAV5_LEN_36, UBX_CLASS_CFG, UBX_CFG_NAV5_ID, ubx_cfg_nav5_payload, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-CFG-NAV5 --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ubx_cfg_nav5_payload[0] |= 1U << UBX_CFG_NAV5_MASK_DYN_BIT;         /* Apply dynamic model settings */
    ubx_cfg_nav5_payload[2] = dynModel;                                 /* Set the dynamic platform model */

    gnss_packet_t packet[8 + UBX_CFG_NAV5_LEN_36];
    gnss_packet_len_t packet_len = 0;

    ret = gnss_hwl_make_packet(packet, sizeof(packet), UBX_CLASS_CFG, UBX_CFG_NAV5_ID, ubx_cfg_nav5_payload, sizeof(ubx_cfg_nav5_payload), &packet_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    uart_flush_input(uart_port);
    ret = gnss_hwl_send_packet(uart_port, packet, packet_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Send UBX-CFG-NAV5 packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_payload_t response[UBX_CFG_NAV5_LEN_36];
    ret = gnss_hwl_receive_packet(uart_port, UBX_CLASS_CFG, UBX_CFG_NAV5_ID, response, sizeof(response), __func__, true);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Set UBX-CFG-NAV5 response --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_set_static_hold_threshold(uart_port_t uart_port, unsigned char static_hold_threshold) {
    esp_err_t ret = ESP_OK;

    /* Get actual value of UBX-CFG-NAV5 */
    gnss_payload_t ubx_cfg_nav5_payload[UBX_CFG_NAV5_LEN_36];

    ret = gnss_hwl_poll_request(uart_port, UBX_CFG_NAV5_LEN_36, UBX_CLASS_CFG, UBX_CFG_NAV5_ID, ubx_cfg_nav5_payload, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-CFG-NAV5 --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ubx_cfg_nav5_payload[0] |= 1U << UBX_CFG_NAV5_MASK_STATICHOLDMASK_BIT;              /* Apply dynamic model settings */
    ubx_cfg_nav5_payload[22] = static_hold_threshold;                                   /* Set the dynamic platform model */

    gnss_packet_t packet[8 + UBX_CFG_NAV5_LEN_36];
    gnss_packet_len_t packet_len = 0;

    ret = gnss_hwl_make_packet(packet, sizeof(packet), UBX_CLASS_CFG, UBX_CFG_NAV5_ID, ubx_cfg_nav5_payload, sizeof(ubx_cfg_nav5_payload), &packet_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    uart_flush_input(uart_port);
    ret = gnss_hwl_send_packet(uart_port, packet, packet_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Send UBX-CFG-NAV5 packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_payload_t response[UBX_CFG_NAV5_LEN_36];
    ret = gnss_hwl_receive_packet(uart_port, UBX_CLASS_CFG, UBX_CFG_NAV5_ID, response, sizeof(response), __func__, true);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Read UBX-CFG-NAV5 response --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_read_nav_dop(uart_port_t uart_port, ubx_nav_dop_t *ubx_nav_dop) {
    gnss_payload_t response[UBX_NAV_DOP_LEN_18];

    /* Send UBX-NAV-DOP message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_poll_request(uart_port, UBX_NAV_DOP_LEN_18, UBX_CLASS_NAV, UBX_NAV_DOP_ID, response, __func__);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Poll UBX-NAV-DOP --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ret = gnss_parse_nav_dop(response, sizeof(response), ubx_nav_dop);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Parse UBX-NAV-DOP --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_set_uart_msg_mode(uart_port_t uart_port, gnss_class_t class, gnss_id_t id, ubx_msg_mode_t mode) {
    esp_err_t ret = ESP_OK;

    gnss_packet_t packet[8 + UBX_CFG_MSG_LEN_8];
    gnss_payload_t payload[UBX_CFG_MSG_LEN_8] = {
        class,
        id,
        0x00,           /* Rate I2C disable */
        mode,           /* Rate UART1 disable */
        0x00,           /* UNDEFINED */
        0x00,           /* Rate USB disable */
        0x00,           /* Rate SPI disable */
        0x00            /* RESERVED */
    };
    gnss_packet_len_t packet_len = 0;

    ret = gnss_hwl_make_packet(packet, sizeof(packet), UBX_CLASS_CFG, UBX_CFG_MSG_ID, payload, sizeof(payload), &packet_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    uart_flush_input(uart_port);
    ret = gnss_hwl_send_packet(uart_port, packet, packet_len);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Send UBX-CFG-MSG packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    gnss_payload_t response[UBX_CFG_MSG_LEN_8];
    ret = gnss_hwl_receive_packet(uart_port, UBX_CLASS_CFG, UBX_CFG_MSG_ID, response, sizeof(response), __func__, true);
    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Read UBX-CFG-MSG response --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gnss_hwl_get_msg_payload(uint8_t *data, unsigned int data_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *payload,
    gnss_payload_len_t payload_len, gnss_payload_len_t *rx_pyld_len
) {
    /* 1. Check if parameters are valid */
    if((data == NULL) || (payload == NULL) || (rx_pyld_len == NULL)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Invalid params", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    /* 2. Initialize payload length to 0 */
    *rx_pyld_len = 0;

    /* 3. Loop through data */
    for(int k = 0; k < (data_len - UBX_PACKET_POS_PAYLOAD); k++) {
        /* Check if there are enough bytes to offset <UBX_PACKET_POS_PAYLOAD> positions */
        if((k + UBX_PACKET_POS_PAYLOAD) > data_len) {
            ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Not enough bytes to read", __func__, __LINE__);
            return ESP_ERR_INVALID_SIZE;
        }

        /* Search for UBX header */
        if((data[k] == UBX_SYNC1) && (data[k + UBX_PACKET_POS_SYNC2] == UBX_SYNC2)) {
            /* Check if message Class and ID matches with given parameters */
            if((data[k + UBX_PACKET_POS_CLASS] == class) && (data[k + UBX_PACKET_POS_ID] == id)) {
                /* Retrieve payload length from UBX packet */
                gnss_payload_len_t rx_payload_len = ((gnss_payload_len_t) (data[k + UBX_PACKET_POS_LENGTH_MSB] << 8)) | ((gnss_payload_len_t) data[k + UBX_PACKET_POS_LENGTH_LSB]);

                /* Check if UBX payload length does not exceeds length of given buffer */
                if(rx_payload_len > payload_len) {
                    ESP_LOGE(
                        gnss_hwl_tag,
                        "{Function %s in line %d}: Payload length of UBX packet (%d bytes) is greather than expected (%d bytes)",
                        __func__, __LINE__,
                        rx_payload_len, payload_len
                    );
                    return ESP_ERR_INVALID_ARG;
                }

                /* Clear buffer */
                memset(payload, 0, payload_len);

                /* Copy found payload into given buffer */
                memcpy(payload, &(data[k + UBX_PACKET_POS_PAYLOAD]), rx_payload_len);

                /* Update rx_pyld_len with actual paylaod length */
                *rx_pyld_len = rx_payload_len;

                return ESP_OK;
            }

            /* If Class and ID does not match, continue looping through data */
            continue;
        }

        /* If its not an UBX header, just move on */
        continue;
    }

    ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: UBX-<0x%X>-<0x%X> not found", __func__, __LINE__, class, id);
    return ESP_FAIL;
}
