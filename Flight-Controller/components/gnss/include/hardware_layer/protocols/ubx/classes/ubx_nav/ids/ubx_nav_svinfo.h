/**
 * @file ubx_nav_svinfo.h
 * @brief UBX-NAV-SVINFO message
 */

#ifndef UBX_NAV_SVINFO_H
#define UBX_NAV_SVINFO_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>
#include <stdbool.h>

#define GNSS_MAX_CHANNELS       20              /* Maximum number of channels */

typedef struct ubx_nav_svinfo {
    unsigned long iTow;             /* GPS time [ms] of week */
    unsigned char numCh;            /* Number of channels */
    unsigned char globalFlags;      /* Global flags */
    unsigned short reserved2;       /* Reserved */
    struct {
        unsigned char chn;          /* Channel number, 255 for SVs not assigned to a channel */
        unsigned char svid;         /* Satellite ID */
        unsigned char flags;        /* Flags */
        unsigned char quality;      /* Signal quality */
        unsigned char cno;          /* Carrier to noise ratio [dBHz] (signal strength) */
        signed char elev;           /* Elevation [deg] */
        signed short azim;          /* Azimuth [deg] */
        signed long prRes;          /* Pseudo range residual [cm] */
    } SVs[GNSS_MAX_CHANNELS];       /* Array of satellite vehicles information */
} ubx_nav_svinfo_t;

/**
 * @brief Check if the SV is used for navigation
 * 
 * @param flags: Flags byte from UBX-NAV-SVINFO message
 * 
 * @retval
 *      - true: SV is used for navigation
 *      - false: SV is not used for navigation
 */
inline bool gnss_nav_svinfo_svUsed(const unsigned char flags) {
    uint8_t mask = 0x01;        /* SV is used for navigation */
    return flags & mask;
}

/**
 * @brief Check if the SV is unhealthy
 * 
 * @param flags: Flags byte from UBX-NAV-SVINFO message
 * 
 * @retval
 *      - true: SV is unhealthy
 *      - false: SV is healthy
 */
inline bool gnss_nav_svinfo_unhealthy(const unsigned char flags) {
    uint8_t mask = 1U << 4;     /* SV is unhealthy */
    return flags & mask;
}

/**
 * @brief Check if the signal quality is good
 * 
 * @param quality: Signal quality byte from UBX-NAV-SVINFO message
 * 
 * @retval
 *      - true: Signal quality is good (>= 4)
 *      - false: Signal quality is poor (< 4)
 */
inline unsigned int gnss_nav_svinfo_quality(const unsigned char quality) {
    return quality >= 4;
}

/**
 * @brief Read UBX-NAV-SVINFO message from GNSS receiver and parse the payload
 * 
 * @param payload: Pointer to the UBX-NAV-SVINFO payload
 * @param payload_len: Length of the UBX-NAV-SVINFO payload
 * @param ubx_nav_svinfo: Pointer to unx_nav_svinfo_t instance to update it with UBX-NAV-SVINFO receiver response
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid payload length or number of channels exceeds maximum allowed
 */
esp_err_t gnss_parse_nav_svinfo(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_svinfo_t *ubx_nav_svinfo);

/**
 * @brief Get the GNSS type of a satellite vehicle based on its SVID
 * 
 * @param svid: Satellite Vehicle ID (SVID)
 * 
 * @retval
 *      - "GPS": If SVID is in the range of 1 to 32 (GPS satellites)
 *      - "SBAS": If SVID is in the range of 120 to 158 (SBAS satellites)
 *      - "QZSS": If SVID is in the range of 193 to 197 (QZSS satellites)
 *      - "GLONASS": If SVID is in the range of 65 to 96 (GLONASS satellites)
 *      - "UNTRACKED GLONASS": If SVID is 255 (untracked GLONASS satellite)
 *      - "UNKNOWN SVID": If SVID does not match any known GNSS type
 */
char *gnss_get_SV_gnss_type(unsigned char svid);

#endif
