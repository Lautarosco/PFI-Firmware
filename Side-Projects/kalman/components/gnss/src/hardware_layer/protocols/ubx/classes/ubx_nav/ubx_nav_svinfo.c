/**
 * @file ubx_nav_svinfo.c
 * @brief UBX-NAV-SVINFO message
 */

#include <hardware_layer/protocols/ubx/classes/ubx_nav/ids/ubx_nav_svinfo.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>
#include <hardware_layer/gnss_types.h>

#include <esp_log.h>

#include <string.h>


/* ========== Private variables ========== */

const char *ubx_nav_svinfo_tag = "[UBX-NAV-SVINFO]";


/* ========== Public functions ========== */

esp_err_t gnss_parse_nav_svinfo(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_svinfo_t *ubx_nav_svinfo) {
    if(payload_len < UBX_NAV_SVINFO_LEN_8) {
        ESP_LOGE(ubx_nav_svinfo_tag, "{Function %s in line %d}: Invalid payload length", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    unsigned int byte_offset = 0;

    /* Copy payload <iTow> field bytes into ubx_nav_svinfo iTow attribute */
    memcpy(&(ubx_nav_svinfo->iTow), &(payload[byte_offset]), sizeof(ubx_nav_svinfo->iTow));
    byte_offset += sizeof(ubx_nav_svinfo->iTow);

    /* Copy payload <numCh> field bytes into ubx_nav_svinfo numCh attribute */
    memcpy(&(ubx_nav_svinfo->numCh), &(payload[byte_offset]), sizeof(ubx_nav_svinfo->numCh));
    byte_offset += sizeof(ubx_nav_svinfo->numCh);

    /* Copy payload <globalFlags> field bytes into ubx_nav_svinfo globalFlags attribute */
    memcpy(&(ubx_nav_svinfo->globalFlags), &(payload[byte_offset]), sizeof(ubx_nav_svinfo->globalFlags));
    byte_offset += sizeof(ubx_nav_svinfo->globalFlags);

    /* Copy payload <reserved2> field bytes into ubx_nav_svinfo reserved2 attribute */
    memcpy(&(ubx_nav_svinfo->reserved2), &(payload[byte_offset]), sizeof(ubx_nav_svinfo->reserved2));
    byte_offset += sizeof(ubx_nav_svinfo->reserved2);

    /* Check if number of channels exceeds #define number */
    if(GNSS_MAX_CHANNELS < ubx_nav_svinfo->numCh) {
        ESP_LOGE(ubx_nav_svinfo_tag, "{Function %s in line %d}: Number of channels (%d) exceeds maximum allowed (%d)", __func__, __LINE__, ubx_nav_svinfo->numCh, GNSS_MAX_CHANNELS);
        return ESP_FAIL;
    }

    /* Copy payload <SVs> field bytes into ubx_nav_svinfo SVs attribute */
    memcpy(&(ubx_nav_svinfo->SVs), &(payload[byte_offset]), ubx_nav_svinfo->numCh * sizeof(ubx_nav_svinfo->SVs[0]));
    byte_offset += ubx_nav_svinfo->numCh * sizeof(ubx_nav_svinfo->SVs[0]);

    return ESP_OK;
}

char *gnss_get_SV_gnss_type(unsigned char svid) {
    if((svid > 0) && (svid < 33)) {
        return "GPS";
    } else if((svid > 119) && (svid < 159)) {
        return "SBAS";
    } else if((svid > 192) && (svid < 198)) {
        return "QZSS";
    } else if((svid > 64) && (svid < 97)) {
        return "GLONASS";
    } else if(svid == 255) {
        return "UNTRACKED GLONASS";
    } else {
        return "UNKOWN SVID";
    }
}
