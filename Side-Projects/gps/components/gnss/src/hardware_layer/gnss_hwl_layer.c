#include <hardware_layer/gnss_hwl_layer.h>      /* GNSS hardware layer */
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/classes/ubx_nav.h>

#include <stdbool.h>

#include <string.h>

/**
 * @brief Detect ACK receiver response
 * 
 * @param class: Class ID of the acknowledged message
 * @param id: Message ID of the acknowledged message
 */
static inline bool GNSS_ACK(gnss_class_t class, gnss_id_t id) {return (class == UBX_CLASS_ACK) && (id == UBX_ACK_ACK_ID);}

/**
 * @brief Detect NACK receiver response
 * 
 * @param class: Class ID of the not-acknowledged message
 * @param id: Message ID of the not-acknowledged message
 */
static inline bool GNSS_NACK(gnss_class_t class, gnss_id_t id) {return (class == UBX_CLASS_ACK) && (id == UBX_ACK_NACK_ID);}

/* ========== Private variables ========== */

const char *gnss_hwl_tag = "[GNSS_HWL_LAYER]";

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

    ESP_LOGW("DEBUG", "{Function %s in line %d}: Checksum computed: CK_A = 0x%02X, CK_B = 0x%02X", __func__, __LINE__, *ck_a, *ck_b);

    return ESP_OK;
}

/* ========== Hardware functions ========== */

esp_err_t gnss_hwl_make_packet(gnss_packet_t *packet, gnss_packet_len_t packet_len, const gnss_class_t class, const gnss_id_t id,
    const gnss_payload_t *payload, gnss_payload_len_t payload_len, gnss_packet_len_t *final_packet_len) {

    packet[UBX_PACKET_POS_SYNC1] = UBX_SYNC1;      /* First byte of the header of a UBX packet */
    packet[UBX_PACKET_POS_SYNC2] = UBX_SYNC2;      /* Second byte of the header of a UBX packet */

    ESP_LOGW("DEBUG", "{Function %s in line %d}: Class: 0x%02X", __func__, __LINE__, class);
    ESP_LOGW("DEBUG", "{Function %s in line %d}: ID: 0x%02X", __func__, __LINE__, id);
    ESP_LOGW("DEBUG", "{Function %s in line %d}: Payload length: %d", __func__, __LINE__, payload_len);

    packet[UBX_PACKET_POS_CLASS] = class;   /* Class of the message */
    packet[UBX_PACKET_POS_ID] = id;         /* ID of the message */

    if(sizeof(payload_len) > sizeof(uint16_t)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Payload length exceeds the size of a 16-bit integer", __func__, __LINE__);
        return ESP_ERR_INVALID_SIZE;
    }

    packet[UBX_PACKET_POS_LENGTH_LSB] = lsb_16b(payload_len);   /* LSB byte of the payload length */
    packet[UBX_PACKET_POS_LENGTH_MSB] = msb_16b(payload_len);   /* MSB byte of the payload length */

    ESP_LOGW("DEBUG", "{Function %s in line %d}: Payload length LSB: %d", __func__, __LINE__, packet[UBX_PACKET_POS_LENGTH_LSB]);
    ESP_LOGW("DEBUG", "{Function %s in line %d}: Payload length MSB: %d", __func__, __LINE__, packet[UBX_PACKET_POS_LENGTH_MSB]);

    memcpy(&(packet[UBX_PACKET_POS_PAYLOAD]), payload, payload_len);    /* Copy the payload into the packet */

    uint8_t ck_a = 0;  /* Checksum first byte */
    uint8_t ck_b = 0;  /* Checksum second byte */

    ubx_get_payload_checksum(&ck_a, &ck_b, packet, payload_len);   /* Calculate the checksum for the payload */

    packet[UBX_PACKET_POS_PAYLOAD + payload_len] = ck_a;        /* First byte of the checksum */
    packet[UBX_PACKET_POS_PAYLOAD + (payload_len + 1)] = ck_b;  /* Second byte of the checksum */

    /* From header to payload there are 6 bytes, then payload length <len> its variable and finally we add the 2 bytes from checksum A and B */
    *final_packet_len = UBX_PACKET_POS_PAYLOAD + payload_len + 2;

    printf("UBX packet:[");
    for(int i = 0; i < *final_packet_len; i++) {
        printf(" 0x%X", packet[i]);
    }
    printf("]\n");

    if(*final_packet_len > packet_len) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Total packet size <%d> exceeds the size of the given UBX packet buffer <%d>", __func__, __LINE__, packet_len, *final_packet_len);
        return ESP_ERR_INVALID_SIZE;
    }

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
        uint8_t ubx_packet[8 + 8];  /* UBX packet length = 8 bytes + <payload length> */
        esp_err_t ret = gnss_hwl_disable_nmea_msg_packet(ubx_packet, sizeof(ubx_packet), nmea_messages[i]);
        if(ret != ESP_OK) {
            ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Disable NMEA message <0x%X> --> FAILED", __func__, __LINE__, nmea_messages[i]);
            return ret;
        }

        printf("Packet for disabling NMEA message <0x%X>: ", nmea_messages[i]);
        for(int j = 0; j < sizeof(ubx_packet); j++) {
            printf("0x%X ", ubx_packet[j]);
        }
        printf("\n\n");

        uart_write_bytes(uart_port, ubx_packet, sizeof(ubx_packet));

        vTaskDelay(pdMS_TO_TICKS(50));  /* Wait some time to ensure the packet is sent */
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
        if(data_len < 0) {
            ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Read UART Rx buffer --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }

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

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Timeout waiting for ACK/NACK", __func__, __LINE__);
    return ESP_ERR_TIMEOUT;
}

esp_err_t gnss_hwl_read_ubx_msg(uart_port_t uart_port, gnss_payload_len_t payload_len, gnss_class_t class, gnss_id_t id, gnss_payload_t *response) {
    gnss_packet_t packet[8 + payload_len];   /* Always 8 bytes + payload length */
    gnss_payload_t payload[] = {};
    gnss_packet_len_t packet_len = 0;

    esp_err_t ret = gnss_hwl_make_packet(packet, sizeof(packet), class, id, payload, sizeof(payload), &packet_len);

    if(ret != ESP_OK) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Make NAV-PVT packet --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uart_write_bytes(uart_port, packet, packet_len);

    uint8_t rx_buffer[1024];
    int len = 0;
    int read_uart = false;
    unsigned int retries = 10;

    while((read_uart <= 0) && retries) {
        len = uart_read_bytes(uart_port, rx_buffer, sizeof(rx_buffer), 100);

        if(len >= 8) {
            read_uart = gnss_hwl_msg_check(rx_buffer, len, class, id);
        }
        
        retries--;

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if(!retries) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Waiting for receiver [Class: <%d> ID: <%d>] response timeout", __func__, __LINE__, class, id);
        return ESP_ERR_TIMEOUT;
    }

    memcpy(response, &rx_buffer[UBX_PACKET_POS_PAYLOAD], payload_len);

    return ESP_OK;
}

int gnss_hwl_msg_check(gnss_packet_t *packet, gnss_packet_len_t packet_len, gnss_class_t class, gnss_id_t id) {


    if(packet_len < 8) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Invalid packet length", __func__, __LINE__);
        return -1;
    }

    if((packet[UBX_PACKET_POS_SYNC1] != UBX_SYNC1) || (packet[UBX_PACKET_POS_SYNC2] != UBX_SYNC2)) {
        ESP_LOGE(gnss_hwl_tag, "{Function %s in line %d}: Invalid packet header", __func__, __LINE__);
        return -1;
    }

    if((packet[UBX_PACKET_POS_CLASS] == class) && (packet[UBX_PACKET_POS_ID] == id)) {
        return true;
    }

    return false;
}

esp_err_t gnss_hwl_read_nav_pvt(uart_port_t uart_port) {
    gnss_payload_t response[UBX_NAV_PVT_LEN_84];

    /* Send UBX-NAV-PVT message to receiver and wait for response */
    esp_err_t ret = gnss_hwl_read_ubx_msg(uart_port, UBX_NAV_PVT_LEN_84, UBX_CLASS_NAV, UBX_NAV_PVT_ID, response);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    printf("NAV-PVT response: ");
    for(int i = 0; i < sizeof(response); i++) {
        printf("0x%X ", response[i]);
    }
    printf("\n");

    ubx_nav_pvt_t nav_pvt;
    gnss_parse_nav_pvt(response, sizeof(response), &nav_pvt);

    ESP_LOGW(gnss_hwl_tag, "iTow: %lu ms", nav_pvt.iTow);
    ESP_LOGW(gnss_hwl_tag, "UTC Date: %04hu-%02hhu-%02hhu", nav_pvt.year, nav_pvt.month, nav_pvt.day);
    ESP_LOGW(gnss_hwl_tag, "UTC Time: %02hhu:%02hhu:%02hhu", nav_pvt.hour, nav_pvt.min, nav_pvt.sec);
    ESP_LOGW(gnss_hwl_tag, "validity flags: 0x%02X", nav_pvt.valid);
    ESP_LOGW(gnss_hwl_tag, "tAcc: %lu ns", nav_pvt.tAcc);
    ESP_LOGW(gnss_hwl_tag, "nano: %ld ns", nav_pvt.nano);
    ESP_LOGW(gnss_hwl_tag, "fixType: %hhu", nav_pvt.fixType);
    ESP_LOGW(gnss_hwl_tag, "flags: 0x%02X", nav_pvt.flags);
    ESP_LOGW(gnss_hwl_tag, "reserved1: 0x%02X", nav_pvt.reserved1);
    ESP_LOGW(gnss_hwl_tag, "numSV: %hhu", nav_pvt.numSV);
    ESP_LOGW(gnss_hwl_tag, "lon: %.7f°", nav_pvt.lon / 1e7);
    ESP_LOGW(gnss_hwl_tag, "lat: %.7f°", nav_pvt.lat / 1e7);
    ESP_LOGW(gnss_hwl_tag, "height: %.3f m", nav_pvt.height / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "hMSL: %.3f m", nav_pvt.hMSL / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "hAcc: %.3f m", nav_pvt.hAcc / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "vAcc: %.3f m", nav_pvt.vAcc / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "velN: %.3f m/s", nav_pvt.velN / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "velE: %.3f m/s", nav_pvt.velE / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "velD: %.3f m/s", nav_pvt.velD / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "gSpeed: %.3f m/s", nav_pvt.gSpeed / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "heading: %.5f°", nav_pvt.heading / 1e5);
    ESP_LOGW(gnss_hwl_tag, "sAcc: %.3f m/s", nav_pvt.sAcc / 1000.0);
    ESP_LOGW(gnss_hwl_tag, "headingAcc: %.5f°", nav_pvt.headingAcc / 1e5);
    ESP_LOGW(gnss_hwl_tag, "pDOP: %.2f", nav_pvt.pDOP / 100.0);
    ESP_LOGW(gnss_hwl_tag, "reserved2: 0x%04X", nav_pvt.reserved2);
    ESP_LOGW(gnss_hwl_tag, "reserved3: 0x%08lX", nav_pvt.reserved3);

    if(!gnss_nav_pvt_fixType(nav_pvt.fixType)) {
        ESP_LOGE(gnss_hwl_tag, "Invalid fixType");
    }

    if(!gnss_nav_pvt_fix_ok(nav_pvt.flags)) {
        ESP_LOGE(gnss_hwl_tag, "Invalid fix");
    }

    if(!gnss_nav_pvt_diffsoln(nav_pvt.flags)) {
        ESP_LOGE(gnss_hwl_tag, "No differential corrections were applied");
    }

    if(!gnss_nav_pvt_valid(nav_pvt.valid)) {
        ESP_LOGE(gnss_hwl_tag, "Invalid UTC date");
    }

    return ESP_OK;
}
