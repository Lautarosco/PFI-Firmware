#ifndef UBX_NAV_PVT_H
#define UBX_NAV_PVT_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>
#include <stdbool.h>

/* ========== UBX-NAV-PVT ========== */

typedef enum ubx_nav_pvt_fixType {
    UBX_NAV_PVT_FIXTYPE_NO_FIX,                     /* No Fix */
    UBX_NAV_PVT_FIXTYPE_DEAD_RECK_ONLY,             /* Dead Reckoning only */
    UBX_NAV_PVT_FIXTYPE_2D_FIX,                     /* 2D-Fix */
    UBX_NAV_PVT_FIXTYPE_3D_FIX,                     /* 3D-Fix */
    UBX_NAV_PVT_FIXTYPE_GNSS_DEAD_RECK,             /* GNSS + dead reckoning combined */
    UBX_NAV_PVT_FIXTYPE_TIME_ONLY_FIX               /* Time only fix */
} ubx_nav_pvt_fixType_t;

typedef struct ubx_nav_pvt {
    unsigned long iTow;             /* GPS time [ms] of week of the navigation epoch */
    unsigned short year;            /* Year [y] (UTC) */
    unsigned char month;            /* Month [month], range 1..12 (UTC) */
    unsigned char day;              /* Day of month [day], range 1..31 (UTC) */
    unsigned char hour;             /* Hour of day [h], range 0..23 (UTC) */
    unsigned char min;              /* Minute of hour [min], range 0..59 (UTC) */
    unsigned char sec;              /* Seconds of minute [s], range 0..60 (UTC) */
    unsigned char valid;            /* Validity Flags */
    unsigned long tAcc;             /* Time accuracy [ns] estimate (UTC) */
    signed long nano;               /* Fraction of second [ns], range -1e9 .. 1e9 (UTC) */
    unsigned char fixType;          /* GNSSfix Type, range 0..5
                                       0x00 = No Fix
                                       0x01 = Dead Reckoning only
                                       0x02 = 2D-Fix
                                       0x03 = 3D-Fix
                                       0x04 = GNSS + dead reckoning combined
                                       0x05 = Time only fix
                                       0x06..0xff: reserved
                                    */
    unsigned char flags;            /* Fix Status Flags */
    unsigned char reserved1;        /* Reserved */
    unsigned char numSV;            /* Number of satellites used in Nav Solution */
    signed long lon;                /* Longitude [deg] (scaling 1e-7) */
    signed long lat;                /* Latitude [deg] (scaling 1e-7) */
    signed long height;             /* Height [mm] above Ellipsoid */
    signed long hMSL;               /* Height [mm] above mean sea level */
    unsigned long hAcc;             /* Horizontal Accuracy [mm] Estimate */
    unsigned long vAcc;             /* Vertical Accuracy [mm] Estimate */
    signed long velN;               /* NED north velocity [mm/s] */
    signed long velE;               /* NED east velocity [mm/s] */
    signed long velD;               /* NED down velocity [mm/s] */
    signed long gSpeed;             /* Ground Speed (2-D) [mm/s] */
    signed long heading;            /* Heading of motion 2-D [deg] (scaling 1e-5) */
    unsigned long sAcc;             /* Speed Accuracy [mm/s] Estimate */
    unsigned long headingAcc;       /* Heading Accuracy [deg] Estimate (scaling 1e-5) */
    unsigned short pDOP;            /* Position DOP (scaling 0.01) */
    unsigned short reserved2;       /* Reserved */
    unsigned long reserved3;        /* Reserved */
} ubx_nav_pvt_t;

/**
 * @brief Check fixType field of UBX-NAV-PVT message
 * 
 * @param fixType: Current value of receiver response
 * 
 * @retval
 *      - true: Fix is 2D, 3D or a combination of both
 *      - false
 */
static inline bool gnss_nav_pvt_fixType(const unsigned char fixType) {
    return (fixType >= UBX_NAV_PVT_FIXTYPE_2D_FIX) && (fixType <= UBX_NAV_PVT_FIXTYPE_GNSS_DEAD_RECK);
}

/**
 * @brief Check if receiver applied fixes are valid
 * 
 * @param flags: Current <flags> field value of receiver response
 * 
 * @retval
 *      - true: Applied fixes are valid
 *      - false: Fixes are invalid
 */
static inline bool gnss_nav_pvt_fix_ok(const unsigned char flags) {
    uint8_t mask = 0x1;     /* A valid fix */

    return flags & mask;
}

/**
 * @brief Check if receiver applied differential corrections
 * 
 * @param flags: Current <flags> field value of receiver response
 * 
 * @retval
 *      - true: Receiver applied differential corrections
 *      - false
 */
static inline bool gnss_nav_pvt_diffsoln(const unsigned char flags) {
    uint8_t mask = 1U << 1;     /* Differential corrections were applied */

    return flags & mask;
}

/**
 * @brief Check if received date is valid
 * 
 * @param valid: Current <valid> field value of receiver response
 * 
 * @retval
 *      - true: Time is valid
 *      - false: Time is invalid
 */
static inline bool gnss_nav_pvt_valid(const unsigned char valid) {
    uint8_t utc_date_ok   = 1U << 0;        /* Valid UTC date */
    uint8_t utc_time_ok   = 1U << 1;        /* Valid UTC time of day */
    uint8_t date_resolved = 1U << 2;        /* UTC time of day has been fully resolved (no seconds uncertainty) */

    uint8_t mask = utc_date_ok | utc_time_ok | date_resolved;

    return valid & mask;
}

/**
 * @brief Parse a NAV-PVT UBX message
 * 
 * @param paylaod: Pointer to payload of NAV-PVT message
 * @param payload_len: Length of payload
 * @param nav_pvt: Pointer to store each NAV-PVT field
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid payload length
 */
esp_err_t gnss_parse_nav_pvt(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_nav_pvt_t *nav_pvt);

#endif
