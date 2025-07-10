#ifndef GNSS_APP_LAYER_H
#define GNSS_APP_LAYER_H

#include <hardware_layer/gnss_hwl_layer.h>



/* ========== Configuration ========== */

typedef enum gnss_protocol_enable {
    GNSS_ENABLE_UBX_ONLY,           /* Enable only UBX protocol */
    GNSS_ENABLE_NMEA_ONLY,          /* Enable only NMEA protocol */
    GNSS_ENABLE_ALL                 /* Enable both NMEA and UBX protocols */
} gnss_protocol_enable_t;

typedef enum gnss_neoxm {
    GNSS_NEO_6M,        /* Device is NEO-6M */
    GNSS_NEO_7M         /* Device is NEO-7M */
} gnss_neoxm_t;

typedef struct gnss_params {
    uart_port_t uart_port;                      /* UART port number */
    gnss_protocol_enable_t gnss_protocol;       /* GNSS protocol to be used */
    gnss_neoxm_t neoxm_version;                 /* Device version */
    unsigned short measRate;                    /* Measurement Rate, GPS measurements are taken every measRate milliseconds */
    ubx_cfg_nav5_dynModel_t dynModel;           /* Dynamic platform model (u-blox7 (V14) Receiver Description Protocol Specification, p. 2, Sec. 2.1):
                                                   0 Portable | Max position deviation: medium | Max vertical velocity [m/s]: 50
                                                   2 Stationary | Max position deviation: small | Max vertical velocity [m/s]: 6
                                                   3 Pedestrian | Max position deviation: small | Max vertical velocity [m/s]: 20
                                                   4 Automotive | Max position deviation: medium | Max vertical velocity [m/s]: 15
                                                   5 Sea | Max position deviation: medium | Max vertical velocity [m/s]: 5
                                                   6 Airborne with <1g Acceleration | Max position deviation: large | Max vertical velocity [m/s]: 100
                                                   7 Airborne with <2g Acceleration | Max position deviation: large | Max vertical velocity [m/s]: 100
                                                   8 Airborne with <4g Acceleration | Max position deviation: large | Max vertical velocity [m/s]: 100
                                                */
    unsigned char static_hold_threshold;        /* Static hold threshold [cm/s] (u-blox7 (V14) Receiver Description Protocol Specification, p. 3, Sec. 2.4) */
} gnss_params_t;



/* ========== Data structures ========== */

typedef struct gnss_position_data {
    double lon;                 /* Longitude [deg] (scaling 1e-7) */
    double lat;                 /* Latitude [deg] (scaling 1e-7) */
    double height;              /* Height [m] above Ellipsoid */
    double hMSL;                /* Height [m] above mean sea level */
    double hAcc;                /* Horizontal Accuracy [m] Estimate */
    double pDOP;                /* Position DOP (Dilution of precision in position) (scaling 0.01) */
    double vDOP;                /* Vertical DOP (Dilution of precision in altitude) (scaling 0.01) */
} gnss_position_data_t;

typedef struct gnss_svs_data {
    unsigned int numSV;                 /* Total satellite vehicles (SVs) used in last navigation solution */
    struct {
        unsigned char svid;             /* Satellite Vehicle ID */
        unsigned char chn;              /* Channel number */
        unsigned char cno;              /* Carrier to Noise Ratio (C/N0) [dB-Hz] */
        bool svUsed;                    /* SV is used for navigation */
        bool healthy;                   /* SV is healthy */
        unsigned int quality;           /* Signal quality. Quality < 4 --> Poor, Quality >= 4 --> Good */
        const char *gnss_type;          /* GNSS type: GPS/SBAS/QZSS/GLONASS */
    } SV[GNSS_MAX_CHANNELS];            /* Array of satellite vehicles */
} gnss_svs_data_t;

typedef struct gnss_flags_data {
    bool valid_utc_date;                /* UTC valid date flag */
    bool valid_gnss_fix_type;           /* GNSS useful fix type flag */
    bool valid_gnss_fix;                /* GNSS valid fix flag */
    bool diffSoln_flag;                 /* Differential corrections applied flag */
} gnss_flags_data_t;

typedef struct gnss_timing_data {
    unsigned char utc_timestamp[32];        /* ISO8601 UTC timestamp <YYYY-MM-DDTHH:MM:SSZ> */
    unsigned short measRate;                /* Measurement Rate, GPS measurements are taken every measRate milliseconds */
} gnss_timing_data_t;

typedef struct gnss_polling_data {
    gnss_timing_data_t time;                /* Clock data and time */
    gnss_position_data_t position;          /* Data of position */
    gnss_svs_data_t svs_data;               /* Satellite vehicles status */
    gnss_flags_data_t flags;                /* GNSS navigation solution flags */
} gnss_polling_data_t;



/* ========== Definition of Class Gnss ========== */

typedef struct gnss gnss_t;

typedef struct gnss {
    uart_port_t uart_port;                  /* UART port number */
    gnss_polling_data_t data;               /* Data read from receiver */
    gnss_neoxm_t __neoxm_version;           /* Device version (DO NOT MODIFY AFTER INSTANCE INITIALIZATION) */

    /**
     * @brief Initialize Gnss object
     * 
     * @param gnss: Pointer to the Gnss instance
     * @param gnss_params: GNSS parameters needed to initialize a Gnss instance
     * 
     * @retval
     *      - ESP_OK: Success
     *      - ESP_ERR_INVALID_ARG: Invalid argument
     */
    esp_err_t (*init)(gnss_t *gnss, gnss_params_t gnss_params);

    esp_err_t (*measure)(gnss_t *gnss);
} gnss_t;



/* ========== Public functions ========== */

/**
 * @brief Make an instance of Gnss Class
 * 
 * @param gnss: Pointer to the Gnss instance
 * 
 * @retval
 *      - ESP_OK: Gnss instance successfuly made
 *      - ESP_ERR_INVALID_ARG: instance is NULL
 */
esp_err_t Gnss(gnss_t *gnss);

#endif
