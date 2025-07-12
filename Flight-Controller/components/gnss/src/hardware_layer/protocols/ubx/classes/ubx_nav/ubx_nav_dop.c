/**
 * @file ubx_nav_dop.c
 * @brief UBX-NAV-DOP message implementations
 */

#include <hardware_layer/protocols/ubx/classes/ubx_nav/ids/ubx_nav_dop.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <stdbool.h>
#include <string.h>

static const char *ubx_nav_dop_tag = "[UBX-NAV-DOP]";

esp_err_t gnss_parse_nav_dop(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_dop_t *ubx_nav_dop) {
    if(payload_len != UBX_NAV_DOP_LEN_18) {
        ESP_LOGE(ubx_nav_dop_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_NAV_DOP_LEN_18);
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ubx_nav_dop, 0, sizeof(ubx_nav_dop_t));

    unsigned int byte_offset = 0;

    /* Copy payload <iTow> field into nav_pvt iTow attribute */
    memcpy(&(ubx_nav_dop->iTow), &(payload[byte_offset]), sizeof(ubx_nav_dop->iTow));
    byte_offset += sizeof(ubx_nav_dop->iTow);

    /* Copy payload <gDOP> field into nav_pvt gDOP attribute */
    memcpy(&(ubx_nav_dop->gDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->gDOP));
    byte_offset += sizeof(ubx_nav_dop->gDOP);

    /* Copy payload <pDOP> field into nav_pvt pDOP attribute */
    memcpy(&(ubx_nav_dop->pDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->pDOP));
    byte_offset += sizeof(ubx_nav_dop->pDOP);

    /* Copy payload <tDOP> field into nav_pvt tDOP attribute */
    memcpy(&(ubx_nav_dop->tDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->tDOP));
    byte_offset += sizeof(ubx_nav_dop->tDOP);

    /* Copy payload <vDOP> field into nav_pvt vDOP attribute */
    memcpy(&(ubx_nav_dop->vDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->vDOP));
    byte_offset += sizeof(ubx_nav_dop->vDOP);

    /* Copy payload <hDOP> field into nav_pvt hDOP attribute */
    memcpy(&(ubx_nav_dop->hDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->hDOP));
    byte_offset += sizeof(ubx_nav_dop->hDOP);

    /* Copy payload <nDOP> field into nav_pvt nDOP attribute */
    memcpy(&(ubx_nav_dop->nDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->nDOP));
    byte_offset += sizeof(ubx_nav_dop->nDOP);

    /* Copy payload <eDOP> field into nav_pvt eDOP attribute */
    memcpy(&(ubx_nav_dop->eDOP), &(payload[byte_offset]), sizeof(ubx_nav_dop->eDOP));
    byte_offset += sizeof(ubx_nav_dop->eDOP);

    return ESP_OK;
}

