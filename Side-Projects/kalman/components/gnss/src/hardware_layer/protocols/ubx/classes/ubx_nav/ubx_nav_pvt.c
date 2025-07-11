#include <hardware_layer/protocols/ubx/classes/ubx_nav/ubx_nav.h>
#include <hardware_layer/gnss_types.h>
#include <hardware_layer/protocols/ubx/neo7_ubx_messages.h>

#include <esp_err.h>
#include <esp_log.h>

#include <stdbool.h>
#include <string.h>

static const char *ubx_nav_pvt_tag = "[UBX-NAV-PVT]";

esp_err_t gnss_parse_nav_pvt(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_pvt_t *nav_pvt) {
    if(payload_len != UBX_NAV_PVT_LEN_84) {
        ESP_LOGE(ubx_nav_pvt_tag, "{Function %s in line %d}: Payload length must be %d bytes", __func__, __LINE__, UBX_NAV_PVT_LEN_84);
        return ESP_ERR_INVALID_ARG;
    }

    memset(nav_pvt, 0, sizeof(ubx_nav_pvt_t));

    unsigned int byte_offset = 0;

    /* Copy payload <iTow> field into nav_pvt iTow attribute */
    memcpy(&(nav_pvt->iTow), &(payload[byte_offset]), sizeof(nav_pvt->iTow));
    byte_offset += sizeof(nav_pvt->iTow);

    /* Copy payload <year> field into nav_pvt year attribute */
    memcpy(&(nav_pvt->year), &(payload[byte_offset]), sizeof(nav_pvt->year));
    byte_offset += sizeof(nav_pvt->year);

    /* Copy payload <month> field into nav_pvt month attribute */
    memcpy(&(nav_pvt->month), &(payload[byte_offset]), sizeof(nav_pvt->month));
    byte_offset += sizeof(nav_pvt->month);

    /* Copy payload <day> field into nav_pvt day attribute */
    memcpy(&(nav_pvt->day), &(payload[byte_offset]), sizeof(nav_pvt->day));
    byte_offset += sizeof(nav_pvt->day);

    /* Copy payload <hour> field into nav_pvt hour attribute */
    memcpy(&(nav_pvt->hour), &(payload[byte_offset]), sizeof(nav_pvt->hour));
    byte_offset += sizeof(nav_pvt->hour);

    /* Copy payload <min> field into nav_pvt min attribute */
    memcpy(&(nav_pvt->min), &(payload[byte_offset]), sizeof(nav_pvt->min));
    byte_offset += sizeof(nav_pvt->min);

    /* Copy payload <sec> field into nav_pvt sec attribute */
    memcpy(&(nav_pvt->sec), &(payload[byte_offset]), sizeof(nav_pvt->sec));
    byte_offset += sizeof(nav_pvt->sec);

    /* Copy payload <valid> field into nav_pvt valid attribute */
    memcpy(&(nav_pvt->valid), &(payload[byte_offset]), sizeof(nav_pvt->valid));
    byte_offset += sizeof(nav_pvt->valid);

    /* Copy payload <tAcc> field into nav_pvt tAcc attribute */
    memcpy(&(nav_pvt->tAcc), &(payload[byte_offset]), sizeof(nav_pvt->tAcc));
    byte_offset += sizeof(nav_pvt->tAcc);

    /* Copy payload <nano> field into nav_pvt nano attribute */
    memcpy(&(nav_pvt->nano), &(payload[byte_offset]), sizeof(nav_pvt->nano));
    byte_offset += sizeof(nav_pvt->nano);

    /* Copy payload <fixType> field into nav_pvt fixType attribute */
    memcpy(&(nav_pvt->fixType), &(payload[byte_offset]), sizeof(nav_pvt->fixType));
    byte_offset += sizeof(nav_pvt->fixType);

    /* Copy payload <flags> field into nav_pvt flags attribute */
    memcpy(&(nav_pvt->flags), &(payload[byte_offset]), sizeof(nav_pvt->flags));
    byte_offset += sizeof(nav_pvt->flags);

    /* Copy payload <reserved1> field into nav_pvt reserved1 attribute */
    memcpy(&(nav_pvt->reserved1), &(payload[byte_offset]), sizeof(nav_pvt->reserved1));
    byte_offset += sizeof(nav_pvt->reserved1);

    /* Copy payload <numSV> field into nav_pvt numSV attribute */
    memcpy(&(nav_pvt->numSV), &(payload[byte_offset]), sizeof(nav_pvt->numSV));
    byte_offset += sizeof(nav_pvt->numSV);

    /* Copy payload <lon> field into nav_pvt lon attribute */
    memcpy(&(nav_pvt->lon), &(payload[byte_offset]), sizeof(nav_pvt->lon));
    byte_offset += sizeof(nav_pvt->lon);

    /* Copy payload <lat> field into nav_pvt lat attribute */
    memcpy(&(nav_pvt->lat), &(payload[byte_offset]), sizeof(nav_pvt->lat));
    byte_offset += sizeof(nav_pvt->lat);

    /* Copy payload <height> field into nav_pvt height attribute */
    memcpy(&(nav_pvt->height), &(payload[byte_offset]), sizeof(nav_pvt->height));
    byte_offset += sizeof(nav_pvt->height);

    /* Copy payload <hMSL> field into nav_pvt hMSL attribute */
    memcpy(&(nav_pvt->hMSL), &(payload[byte_offset]), sizeof(nav_pvt->hMSL));
    byte_offset += sizeof(nav_pvt->hMSL);

    /* Copy payload <hAcc> field into nav_pvt hAcc attribute */
    memcpy(&(nav_pvt->hAcc), &(payload[byte_offset]), sizeof(nav_pvt->hAcc));
    byte_offset += sizeof(nav_pvt->hAcc);

    /* Copy payload <vAcc> field into nav_pvt vAcc attribute */
    memcpy(&(nav_pvt->vAcc), &(payload[byte_offset]), sizeof(nav_pvt->vAcc));
    byte_offset += sizeof(nav_pvt->vAcc);

    /* Copy payload <velN> field into nav_pvt velN attribute */
    memcpy(&(nav_pvt->velN), &(payload[byte_offset]), sizeof(nav_pvt->velN));
    byte_offset += sizeof(nav_pvt->velN);

    /* Copy payload <velE> field into nav_pvt velE attribute */
    memcpy(&(nav_pvt->velE), &(payload[byte_offset]), sizeof(nav_pvt->velE));
    byte_offset += sizeof(nav_pvt->velE);

    /* Copy payload <velD> field into nav_pvt velD attribute */
    memcpy(&(nav_pvt->velD), &(payload[byte_offset]), sizeof(nav_pvt->velD));
    byte_offset += sizeof(nav_pvt->velD);

    /* Copy payload <gSpeed> field into nav_pvt gSpeed attribute */
    memcpy(&(nav_pvt->gSpeed), &(payload[byte_offset]), sizeof(nav_pvt->gSpeed));
    byte_offset += sizeof(nav_pvt->gSpeed);

    /* Copy payload <heading> field into nav_pvt heading attribute */
    memcpy(&(nav_pvt->heading), &(payload[byte_offset]), sizeof(nav_pvt->heading));
    byte_offset += sizeof(nav_pvt->heading);

    /* Copy payload <sAcc> field into nav_pvt sAcc attribute */
    memcpy(&(nav_pvt->sAcc), &(payload[byte_offset]), sizeof(nav_pvt->sAcc));
    byte_offset += sizeof(nav_pvt->sAcc);

    /* Copy payload <headingAcc> field into nav_pvt headingAcc attribute */
    memcpy(&(nav_pvt->headingAcc), &(payload[byte_offset]), sizeof(nav_pvt->headingAcc));
    byte_offset += sizeof(nav_pvt->headingAcc);

    /* Copy payload <pDOP> field into nav_pvt pDOP attribute */
    memcpy(&(nav_pvt->pDOP), &(payload[byte_offset]), sizeof(nav_pvt->pDOP));
    byte_offset += sizeof(nav_pvt->pDOP);

    /* Copy payload <reserved2> field into nav_pvt reserved2 attribute */
    memcpy(&(nav_pvt->reserved2), &(payload[byte_offset]), sizeof(nav_pvt->reserved2));
    byte_offset += sizeof(nav_pvt->reserved2);

    /* Copy payload <reserved3> field into nav_pvt reserved3 attribute */
    memcpy(&(nav_pvt->reserved3), &(payload[byte_offset]), sizeof(nav_pvt->reserved3));
    byte_offset += sizeof(nav_pvt->reserved3);

    return ESP_OK;
}
