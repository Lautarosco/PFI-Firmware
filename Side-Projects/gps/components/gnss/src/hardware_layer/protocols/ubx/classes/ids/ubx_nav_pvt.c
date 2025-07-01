#include <hardware_layer/protocols/ubx/classes/ids/ubx_nav_pvt.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <stdbool.h>
#include <string.h>

static const char *nav_pvt_tag = "[NAV_PVT]";

esp_err_t gnss_parse_nav_pvt(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_pvt_t *nav_pvt) {
    if(payload_len != UBX_NAV_PVT_LEN_84) {
        ESP_LOGE(nav_pvt_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_NAV_PVT_LEN_84);
        return ESP_ERR_INVALID_ARG;
    }

    unsigned int byte_offset = 0;

    memcpy(&(nav_pvt->iTow), &(payload[byte_offset]), sizeof(nav_pvt->iTow));
    byte_offset += sizeof(nav_pvt->iTow);

    memcpy(&(nav_pvt->year), &(payload[byte_offset]), sizeof(nav_pvt->year));
    byte_offset += sizeof(nav_pvt->year);

    memcpy(&(nav_pvt->month), &(payload[byte_offset]), sizeof(nav_pvt->month));
    byte_offset += sizeof(nav_pvt->month);

    memcpy(&(nav_pvt->day), &(payload[byte_offset]), sizeof(nav_pvt->day));
    byte_offset += sizeof(nav_pvt->day);

    memcpy(&(nav_pvt->hour), &(payload[byte_offset]), sizeof(nav_pvt->hour));
    byte_offset += sizeof(nav_pvt->hour);

    memcpy(&(nav_pvt->min), &(payload[byte_offset]), sizeof(nav_pvt->min));
    byte_offset += sizeof(nav_pvt->min);

    memcpy(&(nav_pvt->sec), &(payload[byte_offset]), sizeof(nav_pvt->sec));
    byte_offset += sizeof(nav_pvt->sec);

    memcpy(&(nav_pvt->valid), &(payload[byte_offset]), sizeof(nav_pvt->valid));
    byte_offset += sizeof(nav_pvt->valid);

    memcpy(&(nav_pvt->tAcc), &(payload[byte_offset]), sizeof(nav_pvt->tAcc));
    byte_offset += sizeof(nav_pvt->tAcc);

    memcpy(&(nav_pvt->nano), &(payload[byte_offset]), sizeof(nav_pvt->nano));
    byte_offset += sizeof(nav_pvt->nano);

    memcpy(&(nav_pvt->fixType), &(payload[byte_offset]), sizeof(nav_pvt->fixType));
    byte_offset += sizeof(nav_pvt->fixType);

    memcpy(&(nav_pvt->flags), &(payload[byte_offset]), sizeof(nav_pvt->flags));
    byte_offset += sizeof(nav_pvt->flags);

    memcpy(&(nav_pvt->reserved1), &(payload[byte_offset]), sizeof(nav_pvt->reserved1));
    byte_offset += sizeof(nav_pvt->reserved1);

    memcpy(&(nav_pvt->numSV), &(payload[byte_offset]), sizeof(nav_pvt->numSV));
    byte_offset += sizeof(nav_pvt->numSV);

    memcpy(&(nav_pvt->lon), &(payload[byte_offset]), sizeof(nav_pvt->lon));
    byte_offset += sizeof(nav_pvt->lon);

    memcpy(&(nav_pvt->lat), &(payload[byte_offset]), sizeof(nav_pvt->lat));
    byte_offset += sizeof(nav_pvt->lat);

    memcpy(&(nav_pvt->height), &(payload[byte_offset]), sizeof(nav_pvt->height));
    byte_offset += sizeof(nav_pvt->height);

    memcpy(&(nav_pvt->hMSL), &(payload[byte_offset]), sizeof(nav_pvt->hMSL));
    byte_offset += sizeof(nav_pvt->hMSL);

    memcpy(&(nav_pvt->hAcc), &(payload[byte_offset]), sizeof(nav_pvt->hAcc));
    byte_offset += sizeof(nav_pvt->hAcc);

    memcpy(&(nav_pvt->vAcc), &(payload[byte_offset]), sizeof(nav_pvt->vAcc));
    byte_offset += sizeof(nav_pvt->vAcc);

    memcpy(&(nav_pvt->velN), &(payload[byte_offset]), sizeof(nav_pvt->velN));
    byte_offset += sizeof(nav_pvt->velN);

    memcpy(&(nav_pvt->velE), &(payload[byte_offset]), sizeof(nav_pvt->velE));
    byte_offset += sizeof(nav_pvt->velE);

    memcpy(&(nav_pvt->velD), &(payload[byte_offset]), sizeof(nav_pvt->velD));
    byte_offset += sizeof(nav_pvt->velD);

    memcpy(&(nav_pvt->gSpeed), &(payload[byte_offset]), sizeof(nav_pvt->gSpeed));
    byte_offset += sizeof(nav_pvt->gSpeed);

    memcpy(&(nav_pvt->heading), &(payload[byte_offset]), sizeof(nav_pvt->heading));
    byte_offset += sizeof(nav_pvt->heading);

    memcpy(&(nav_pvt->sAcc), &(payload[byte_offset]), sizeof(nav_pvt->sAcc));
    byte_offset += sizeof(nav_pvt->sAcc);

    memcpy(&(nav_pvt->headingAcc), &(payload[byte_offset]), sizeof(nav_pvt->headingAcc));
    byte_offset += sizeof(nav_pvt->headingAcc);

    memcpy(&(nav_pvt->pDOP), &(payload[byte_offset]), sizeof(nav_pvt->pDOP));
    byte_offset += sizeof(nav_pvt->pDOP);

    memcpy(&(nav_pvt->reserved2), &(payload[byte_offset]), sizeof(nav_pvt->reserved2));
    byte_offset += sizeof(nav_pvt->reserved2);

    memcpy(&(nav_pvt->reserved3), &(payload[byte_offset]), sizeof(nav_pvt->reserved3));
    byte_offset += sizeof(nav_pvt->reserved3);

    return ESP_OK;
}
