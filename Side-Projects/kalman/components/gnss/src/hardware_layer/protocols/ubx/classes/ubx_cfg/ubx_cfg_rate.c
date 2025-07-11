#include <hardware_layer/protocols/ubx/classes/ubx_cfg/ids/ubx_cfg_rate.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <string.h>

static const char *ubx_cfg_rate_tag = "[UBX-CFG-RATE]";

esp_err_t gnss_parse_cfg_rate(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_rate_t *cfg_rate) {
    if(payload_len != UBX_CFG_RATE_LEN_6) {
        ESP_LOGE(ubx_cfg_rate_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_CFG_RATE_LEN_6);
        return ESP_ERR_INVALID_ARG;
    }
    
    unsigned int byte_offset = 0;

    /* Copy payload <measRate> field bytes into cfg_rate measRate attribute */
    memcpy(&(cfg_rate->measRate), &(payload[byte_offset]), sizeof(cfg_rate->measRate));
    byte_offset += sizeof(cfg_rate->measRate);

    /* Copy payload <navRate> field bytes into cfg_rate navRate attribute */
    memcpy(&(cfg_rate->navRate), &(payload[byte_offset]), sizeof(cfg_rate->navRate));
    byte_offset += sizeof(cfg_rate->navRate);

    /* Copy payload <timeRef> field bytes into cfg_rate timeRef attribute */
    memcpy(&(cfg_rate->timeRef), &(payload[byte_offset]), sizeof(cfg_rate->timeRef));
    byte_offset += sizeof(cfg_rate->timeRef);
    
    return ESP_OK;
}
