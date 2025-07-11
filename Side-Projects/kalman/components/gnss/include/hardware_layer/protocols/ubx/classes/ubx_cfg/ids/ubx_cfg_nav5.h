/**
 * @file ubx_cfg_nav5.h
 * @brief UBX-CFG-NAV5 Message
 */

#ifndef UBX_CFG_NAV5_H
#define UBX_CFG_NAV5_H

#include <hardware_layer/gnss_types.h>
#include <esp_err.h>

typedef struct ubx_cfg_nav5 {
    unsigned short mask;                /* Parameters Bitmask. Only the masked parameters will be applied */
    unsigned char dynModel;             /* Dynamic Platform model:
                                           0 Portable
                                           2 Stationary
                                           3 Pedestrian
                                           4 Automotive
                                           5 Sea
                                           6 Airborne with <1g Acceleration
                                           7 Airborne with <2g Acceleration
                                           8 Airborne with <4g Acceleration
                                        */
    unsigned char fixMode;              /* Position Fixing Mode.
                                           1: 2D only
                                           2: 3D only
                                           3: Auto 2D/3D
                                        */
    signed long fixedAlt;               /* Fixed altitude [m] (mean sea level) for 2D fix mode (scaling 0.01) */
    unsigned long fixedAltVar;          /* Fixed altitude [m^2] variance for 2D mode (scaling 0.0001) */
    signed char minElev;                /* Minimum Elevation [deg] for a GNSS satellite to be used in NAV */
    unsigned char drLimit;              /* Reserved [s] */
    unsigned short pDop;                /* Position DOP Mask to use (scaling 0.1) */
    unsigned short tDop;                /* Time DOP Mask to use (scaling 0.1) */
    unsigned short pAcc;                /* Position [m] Accuracy Mask */
    unsigned short tAcc;                /* Time [m] Accuracy Mask */
    unsigned char staticHoldThresh;     /* Static hold threshold [cm/s]. Once static hold mode has been entered, the position output is kept static and the velocity is set to 0 until there is evidence of moving again */
    unsigned char dgspTimeOut;          /* DGPS timeout [s] */
    unsigned char cnoThreshNumSVs;      /* Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted */
    unsigned char cnoThresh;            /* C/N0 threshold [dBHz] for deciding whether to attempt a fix */
    unsigned short reserved2;           /* Always set to 0 */
    unsigned long reserved3;           /* Always set to 0 */
    unsigned long reserved4;           /* Always set to 0 */
} ubx_cfg_nav5_t;

typedef enum ubx_cfg_nav5_dynModel {
    UBX_CFG_NAV5_DYNMODEL_PORTABLE,                     /* Dynamic platform model is portable */
    UBX_CFG_NAV5_DYNMODEL_STATIONARY = 2,               /* Dynamic platform model is stationary */
    UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN,                   /* Dynamic platform model is pedestrian */
    UBX_CFG_NAV5_DYNMODEL_AUTOMOTIVE,                   /* Dynamic platform model is automotive */
    UBX_CFG_NAV5_DYNMODEL_SEA,                          /* Dynamic platform model is sea */
    UBX_CFG_NAV5_DYNMODEL_AIRBONE_1G_ACC,               /* Dynamic platform model is airbone 1g acceleration */
    UBX_CFG_NAV5_DYNMODEL_AIRBONE_2G_ACC,               /* Dynamic platform model is airbone 2g acceleration */
    UBX_CFG_NAV5_DYNMODEL_AIRBONE_4G_ACC,               /* Dynamic platform model is airbone 4g acceleration */
} ubx_cfg_nav5_dynModel_t;

typedef enum ubx_cfg_nav5_mask_bits {
    UBX_CFG_NAV5_MASK_DYN_BIT,                          /* Apply dynamic model settings */
    UBX_CFG_NAV5_MASK_MINEL_BIT,                        /* Apply minimum elevation settings */
    UBX_CFG_NAV5_MASK_POSFIXMODE_BIT,                   /* Apply fix mode settings */
    UBX_CFG_NAV5_MASK_DRLIM_BIT,                        /* Reserved */
    UBX_CFG_NAV5_MASK_POSMASK_BIT,                      /* Apply position mask settings */
    UBX_CFG_NAV5_MASK_TIMEMASK_BIT,                     /* Apply time mask settings */
    UBX_CFG_NAV5_MASK_STATICHOLDMASK_BIT,               /* Apply static hold settings */
    UBX_CFG_NAV5_MASK_DGPSMASK_BIT,                     /* Apply DGPS settings */
    UBX_CFG_NAV5_MASK_RESERVEDBIT0_BIT = 15,            /* Reserved */
} ubx_cfg_nav5_mask_bits_t;

/**
 * @brief Parse UBX-CFG-NAV5 message payload
 * 
 * @param payload: Pointer to the payload data
 * @param payload_len: Length of the payload data
 * @param ubx_cfg_nav5: Pointer to the ubx_cfg_nav5_t structure to fill with parsed data
 * 
 * @retval
 *      - ESP_OK: Successfully parsed the payload
 *      - ESP_ERR_INVALID_ARG: Invalid payload length
 */
esp_err_t gnss_parse_cfg_nav5(const gnss_payload_t *payload, gnss_payload_len_t payload_len, ubx_cfg_nav5_t *ubx_cfg_nav5);

#endif
