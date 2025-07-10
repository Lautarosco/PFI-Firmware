#include <hardware_layer/protocols/ubx/classes/ubx_cfg/ids/ubx_cfg_nav5.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <string.h>

static const char *ubx_cfg_nav5_tag = "[UBX-CFG-NAV5]";

esp_err_t gnss_parse_cfg_nav5(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_nav5_t *ubx_cfg_nav5) {
    if(payload_len != UBX_CFG_NAV5_LEN_36) {
        ESP_LOGE(ubx_cfg_nav5_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_CFG_NAV5_LEN_36);
        return ESP_ERR_INVALID_ARG;
    }
    memset(ubx_cfg_nav5, 0, sizeof(ubx_cfg_nav5_t));
    
    unsigned int byte_offset = 0;

    /** @note: The order of memcpy must be according to byte offsets for UBX-CFG-NAV5 message */

    /* Copy payload <mask> field bytes into ubx_cfg_nav5 mask attribute */
    memcpy(&(ubx_cfg_nav5->mask), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->mask));
    byte_offset += sizeof(ubx_cfg_nav5->mask);

    /* Copy payload <dynModel> field bytes into ubx_cfg_nav5 dynModel attribute */
    memcpy(&(ubx_cfg_nav5->dynModel), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->dynModel));
    byte_offset += sizeof(ubx_cfg_nav5->dynModel);

    /* Copy payload <fixMode> field bytes into ubx_cfg_nav5 fixMode attribute */
    memcpy(&(ubx_cfg_nav5->fixMode), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->fixMode));
    byte_offset += sizeof(ubx_cfg_nav5->fixMode);

    /* Copy payload <fixedAlt> field bytes into ubx_cfg_nav5 fixedAlt attribute */
    memcpy(&(ubx_cfg_nav5->fixedAlt), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->fixedAlt));
    byte_offset += sizeof(ubx_cfg_nav5->fixedAlt);

    /* Copy payload <fixedAltVar> field bytes into ubx_cfg_nav5 fixedAltVar attribute */
    memcpy(&(ubx_cfg_nav5->fixedAltVar), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->fixedAltVar));
    byte_offset += sizeof(ubx_cfg_nav5->fixedAltVar);

    /* Copy payload <minElev> field bytes into ubx_cfg_nav5 minElev attribute */
    memcpy(&(ubx_cfg_nav5->minElev), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->minElev));
    byte_offset += sizeof(ubx_cfg_nav5->minElev);

    /* Copy payload <drLimit> field bytes into ubx_cfg_nav5 drLimit attribute */
    memcpy(&(ubx_cfg_nav5->drLimit), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->drLimit));
    byte_offset += sizeof(ubx_cfg_nav5->drLimit);

    /* Copy payload <pDop> field bytes into ubx_cfg_nav5 pDop attribute */
    memcpy(&(ubx_cfg_nav5->pDop), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->pDop));
    byte_offset += sizeof(ubx_cfg_nav5->pDop);

    /* Copy payload <tDop> field bytes into ubx_cfg_nav5 tDop attribute */
    memcpy(&(ubx_cfg_nav5->tDop), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->tDop));
    byte_offset += sizeof(ubx_cfg_nav5->tDop);

    /* Copy payload <pAcc> field bytes into ubx_cfg_nav5 pAcc attribute */
    memcpy(&(ubx_cfg_nav5->pAcc), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->pAcc));
    byte_offset += sizeof(ubx_cfg_nav5->pAcc);

    /* Copy payload <tAcc> field bytes into ubx_cfg_nav5 tAcc attribute */
    memcpy(&(ubx_cfg_nav5->tAcc), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->tAcc));
    byte_offset += sizeof(ubx_cfg_nav5->tAcc);

    /* Copy payload <staticHoldThresh> field bytes into ubx_cfg_nav5 staticHoldThresh attribute */
    memcpy(&(ubx_cfg_nav5->staticHoldThresh), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->staticHoldThresh));
    byte_offset += sizeof(ubx_cfg_nav5->staticHoldThresh);

    /* Copy payload <dgspTimeOut> field bytes into ubx_cfg_nav5 dgspTimeOut attribute */
    memcpy(&(ubx_cfg_nav5->dgspTimeOut), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->dgspTimeOut));
    byte_offset += sizeof(ubx_cfg_nav5->dgspTimeOut);

    /* Copy payload <cnoThreshNumSVs> field bytes into ubx_cfg_nav5 cnoThreshNumSVs attribute */
    memcpy(&(ubx_cfg_nav5->cnoThreshNumSVs), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->cnoThreshNumSVs));
    byte_offset += sizeof(ubx_cfg_nav5->cnoThreshNumSVs);

    /* Copy payload <cnoThresh> field bytes into ubx_cfg_nav5 cnoThresh attribute */
    memcpy(&(ubx_cfg_nav5->cnoThresh), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->cnoThresh));
    byte_offset += sizeof(ubx_cfg_nav5->cnoThresh);

    /* Copy payload <reserved2> field bytes into ubx_cfg_nav5 reserved2 attribute */
    memcpy(&(ubx_cfg_nav5->reserved2), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->reserved2));
    byte_offset += sizeof(ubx_cfg_nav5->reserved2);

    /* Copy payload <reserved3> field bytes into ubx_cfg_nav5 reserved3 attribute */
    memcpy(&(ubx_cfg_nav5->reserved3), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->reserved3));
    byte_offset += sizeof(ubx_cfg_nav5->reserved3);

    /* Copy payload <reserved4> field bytes into ubx_cfg_nav5 reserved4 attribute */
    memcpy(&(ubx_cfg_nav5->reserved4), &(payload[byte_offset]), sizeof(ubx_cfg_nav5->reserved4));
    byte_offset += sizeof(ubx_cfg_nav5->reserved4);

    // ESP_LOGI(
    //     ubx_cfg_nav5_tag,
    //     "NAV5 Config:\n"
    //     "  Dynamic platform model: %d\n"
    //     "  Fix mode: %d\n"
    //     "  Fixed altitude for 2D fix mode: %.4f m\n"
    //     "  Fixed altitude variance for 2D mode: %.4f m^2\n"
    //     "  Position DOP mask to use: %.4f\n"
    //     "  Time DOP mask to use: %.4f\n"
    //     "  Position accuracy mask to use: %d m\n"
    //     "  Time accuracy mask to use: %d s\n"
    //     "  Static hold threshold: %d cm/s\n",
    //     ubx_cfg_nav5->dynModel,
    //     ubx_cfg_nav5->fixMode,
    //     ubx_cfg_nav5->fixedAlt / 100.0,
    //     ubx_cfg_nav5->fixedAltVar / 10000.0,
    //     ubx_cfg_nav5->pDop / 10.0,
    //     ubx_cfg_nav5->tDop / 10.0,
    //     ubx_cfg_nav5->pAcc,
    //     ubx_cfg_nav5->tAcc,
    //     ubx_cfg_nav5->staticHoldThresh
    // );

    return ESP_OK;
}
