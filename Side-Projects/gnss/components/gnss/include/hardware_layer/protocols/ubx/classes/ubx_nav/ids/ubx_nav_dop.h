/**
 * @file ubx_nav_dop.h
 * @brief UBX-NAV-DOP message
 */

#ifndef UBX_NAV_DOP_H
#define UBX_NAV_DOP_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>

#define GNSS_MAX_VDOP           10          /* Vertical dilution of precision (DOP) is acceptable when its lower than <GNSS_MAX_VDOP> value */

typedef struct ubx_nav_dop {
    unsigned long iTow;     /* GPS time of week [ms] of the navigation epoch */
    unsigned short gDOP;    /* Dimensionless geometric DOP (scaling 0.01) */
    unsigned short pDOP;    /* Dimensionless position DOP (Dilution of precision in position) (scaling 0.01) */
    unsigned short tDOP;    /* Dimensionless time DOP (Dilution of precision in time) (scaling 0.01) */
    unsigned short vDOP;    /* Dimensionless vertical DOP (Dilution of precision in altitude) (scaling 0.01) */
    unsigned short hDOP;    /* Dimensionless horizontal DOP (Dilution of precision in horizontal position) (scaling 0.01) */
    unsigned short nDOP;    /* Dimensionless northing DOP (Dilution of precision in north position) (scaling 0.01) */
    unsigned short eDOP;    /* Dimensionless easting DOP (Dilution of precision in east position) (scaling 0.01) */
} ubx_nav_dop_t;

/**
 * @brief Parse UBX-NAV-DOP message
 * 
 * @param payload Pointer to the payload data
 * @param payload_len Length of the payload data
 * @param ubx_nav_dop Pointer to the structure to fill with parsed data
 * 
 * @retval
 *      - ESP_OK: Parsing successful
 *      - ESP_ERR_INVALID_ARG: Invalid argument (e.g., incorrect payload length)
 */
esp_err_t gnss_parse_nav_dop(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_dop_t *ubx_nav_dop);

#endif
