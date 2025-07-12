#include <hardware_layer/protocols/ubx/classes/ubx_cfg/ids/ubx_cfg_msg.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <string.h>

static const char *ubx_cfg_msg_tag = "[UBX-CFG-MSG]";

esp_err_t gnss_parse_cfg_msg(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_msg_t *ubx_cfg_msg) {
    if(payload_len != UBX_CFG_MSG_LEN_8) {
        ESP_LOGE(ubx_cfg_msg_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_CFG_MSG_LEN_8);
        return ESP_ERR_INVALID_ARG;
    }
    memset(ubx_cfg_msg, 0, sizeof(ubx_cfg_msg_t));      /* Initialize struct to 0 before accessing its members */
    
    unsigned int byte_offset = 0;

    /** @note: The order of memcpy must be according to byte offsets for UBX-CFG-NAV5 message */

    /* Copy payload <msgClass> field bytes into ubx_cfg_msg msgClass attribute */
    memcpy(&(ubx_cfg_msg->msgClass), &(payload[byte_offset]), sizeof(ubx_cfg_msg->msgClass));
    byte_offset += sizeof(ubx_cfg_msg->msgClass);

    /* Copy payload <msgID> field bytes into ubx_cfg_msg msgID attribute */
    memcpy(&(ubx_cfg_msg->msgID), &(payload[byte_offset]), sizeof(ubx_cfg_msg->msgID));
    byte_offset += sizeof(ubx_cfg_msg->msgID);

    /* Copy payload <rate> field bytes into ubx_cfg_msg rate attribute */
    memcpy(&(ubx_cfg_msg->rate), &(payload[byte_offset]), sizeof(ubx_cfg_msg->rate));
    byte_offset += sizeof(ubx_cfg_msg->rate);

    ESP_LOGI(ubx_cfg_msg_tag, "msgClass: <0x%X>, msgID: <0x%X>, msgRate: <0x%X>", (unsigned int) ubx_cfg_msg->msgClass, (unsigned int) ubx_cfg_msg->msgID, (unsigned int) ubx_cfg_msg->rate);

    return ESP_OK;
}
