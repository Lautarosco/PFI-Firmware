/**
 * @file neo7_ubx_messages.h
 * @brief NEO-7M UBX messages
 */

#ifndef NEO7_UBX_MESSAGES_H
#define NEO7_UBX_MESSAGES_H

#include <hardware_layer/protocols/ubx/classes/ubx_nav/ubx_nav.h>
#include <hardware_layer/protocols/ubx/classes/ubx_cfg/ubx_cfg.h>

/**
 * @file neo7_ubx_messages.h
 * @brief UBX Protocol Messages for NEO-7 GPS Module
 * 
 * https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf, Sec 32
 */



/* ============ ACK messages ============ */

#define UBX_ACK_ACK_ID                  0x01            /* Message Acknowledged */
#define UBX_ACK_NACK_ID                 0x00            /* Message Not-Acknowledged */

typedef enum ubx_ack_ack_len {
    UBX_ACK_ACK_LEN_2 = 2,          /* Type: Output | Desc: ACK message length */
} ubx_ack_ack_len_t;

typedef enum ubx_ack_nack_len {
    UBX_ACK_NACK_LEN_2 = 2,         /* Type: Output | Desc: NACK message length */
} ubx_ack_nack_len_t;



/* ============ CFG messages ============ */

#define UBX_CFG_ANT_ID                  0x13            /* Poll Antenna Control Settings */

typedef enum ubx_cfg_ant_len {
    UBX_CFG_ANT_LEN_0 = 0,          /* Type: Poll request | Desc: Poll Antenna Control Settings */
    UBX_CFG_ANT_LEN_4 = 4,          /* Type: Input/Output | Desc: Antenna Control Settings */
} ubx_cfg_ant_len_t;

#define UBX_CFG_CFG_ID                  0x09            /* Clear, Save and Load configurations */

typedef enum ubx_cfg_cfg_len {
    UBX_CFG_CFG_LEN_12 = 12         /* Type: Command | Desc: Clear, Save and Load configurations */
} ubx_cfg_cfg_len_t;

#define UBX_CFG_DAT_ID                  0x06            /* Poll Datum Setting */

typedef enum ubx_cfg_dat_len {
    UBX_CFG_DAT_LEN_0  = 0,         /* Type: Poll request | Desc: Poll Datum Setting */
    UBX_CFG_DAT_LEN_44 = 44,        /* Type: Input | Desc: Set User-defined Datum */
    UBX_CFG_DAT_LEN_52 = 52         /* Type: Output | Desc: The currently defined Datum */
} ubx_cfg_dat_len_t;

#define UBX_CFG_GNSS_ID                 0x3E            /* Polls the configuration of the GNSS system configuration */

typedef enum ubx_cfg_gnss_len {
    UBX_CFG_GNSS_LEN_0 = 0,         /* Type: Poll request | Desc: Polls the configuration of the GNSS system configuration */
    UBX_CFG_GNSS_LEN_4 = 4          /* Type: Input/Output | Desc: GNSS system configuration */
} ubx_cfg_gnss_len_t;

#define UBX_CFG_INF_ID                  0x02            /* Poll INF message configuration for one protocol */

typedef enum ubx_cfg_inf_len {
    UBX_CFG_INF_LEN_1 = 1,          /* Type: Poll request | Desc: Poll INF message configuration for one protocol */
    UBX_CFG_INF_LEN_0 = 0           /* Type: Input/Output | Desc: Information message configuration */
} ubx_cfg_inf_len_t;

#define UBX_CFG_ITFM_ID                 0x39            /* Polls the Jamming/Interference Monitor configuration */

typedef enum ubx_cfg_itfm_len {
    UBX_CFG_ITFM_LEN_0 = 0,         /* Type: Poll request | Desc: Polls the Jamming/Interference Monitor configuration */
    UBX_CFG_ITFM_LEN_8 = 8          /* Type: Command | Desc: Jamming/Interference Monitor configuration */
} ubx_cfg_itfm_len_t;

#define UBX_CFG_LOGFILTER_ID            0x47            /* Poll Data Logger filter Configuration */

typedef enum ubx_cfg_logfilter_len {
    UBX_CFG_LOGFILTER_LEN_0  = 0,       /* Type: Poll request | Desc: Poll Data Logger filter Configuration */
    UBX_CFG_LOGFILTER_LEN_12 = 12       /* Type: Input/Output | Desc: Data Logger Configuration */
} ubx_cfg_logfilter_len_t;

#define UBX_CFG_MSG_ID                  0x01            /* Poll a message configuration */

typedef enum ubx_cfg_msg_len {
    UBX_CFG_MSG_LEN_2 = 2,          /* Type: Poll request | Desc: Poll a message configuration */
    UBX_CFG_MSG_LEN_8 = 8,          /* Type: Input/Output | Desc: Set Message Rate(s) */
    UBX_CFG_MSG_LEN_3 = 3           /* Type: Input/Output | Desc: Set Message Rate */
} ubx_cfg_msg_len_t;

#define UBX_CFG_NAV5_ID                 0x24            /* Poll Navigation Engine Settings */

typedef enum ubx_cfg_nav5_len {
    UBX_CFG_NAV5_LEN_0  = 0,        /* Type: Poll request | Desc: Poll Navigation Engine Settings */
    UBX_CFG_NAV5_LEN_36 = 36        /* Type: Input/Output | Desc: Navigation Engine Settings */
} ubx_cfg_nav5_len_t;

#define UBX_CFG_NAVX5_ID                0x23            /* Poll Navigation Engine Expert Settings */

typedef enum ubx_cfg_navx5_len {
    UBX_CFG_NAVX5_LEN_0  = 0,       /* Type: Poll request | Desc: Poll Navigation Engine Expert Settings */
    UBX_CFG_NAVX5_LEN_40 = 40       /* Type: Input/Output | Desc: Navigation Engine Expert Settings */
} ubx_cfg_navx5_len_t;

#define UBX_CFG_NMEA_ID                 0x17            /* Poll the NMEA protocol configuration */

typedef enum ubx_cfg_nmea_len {
    UBX_CFG_NMEA_LEN_0  = 0,        /* Type: Poll request | Desc: Poll the NMEA protocol configuration */
    UBX_CFG_NMEA_LEN_4  = 4,        /* Type: Input/Output | Desc: NMEA protocol configuration (deprecated) */
    UBX_CFG_NMEA_LEN_12 = 12,       /* Type: Input/Output | Desc: NMEA protocol configuration */
} ubx_cfg_nmea_len_t;

#define UBX_CFG_PM2_ID                  0x3B            /* Poll extended Power Management configuration */

typedef enum ubx_cfg_pm2_len {
    UBX_CFG_PM2_LEN_0  = 0,         /* Type: Poll request | Desc: Poll extended Power Management configuration */
    UBX_CFG_PM2_LEN_44 = 44         /* Type: Input/Output | Desc: Extended Power Management configuration */
} ubx_cfg_pm2_len_t;

#define UBX_CFG_PRT_ID                  0x00            /* Polls the configuration of the used I/O Port */

typedef enum ubx_cfg_prt_len {
    UBX_CFG_PRT_LEN_0  = 0,         /* Type: Poll request | Desc: Polls the configuration of the used I/O Port */
    UBX_CFG_PRT_LEN_1  = 1,         /* Type: Poll request | Desc: Polls the configuration for one I/O Port */
    UBX_CFG_PRT_LEN_20 = 20         /* Type: Input/Output | Desc: Polls configuration for UART or USB or SPI or DDC (I2C) */
} ubx_cfg_prt_len_t;

#define UBX_CFG_RATE_ID                 0x08            /* Poll Navigation/Measurement Rate Settings */

typedef enum ubx_cfg_rate_len {
    UBX_CFG_RATE_LEN_0  = 0,        /* Type: Poll request | Desc: Poll Navigation/Measurement Rate Settings */
    UBX_CFG_RATE_LEN_6  = 6         /* Type: Input/Output | Desc: Navigation/Measurement Rate Settings */
} ubx_cfg_rate_len_t;

#define UBX_CFG_RINV_ID                 0x34            /* Poll contents of Remote Inventory */

typedef enum ubx_cfg_rinv_len {
    UBX_CFG_RINV_LEN_0  = 0,        /* Type: Poll request | Desc: Poll contents of Remote Inventory */
    UBX_CFG_RINV_LEN_1  = 1         /* Type: Input/Output | Desc: Contents of Remote Inventory */
} ubx_cfg_rinv_len_t;

#define UBX_CFG_RST_ID                  0x04            /* Reset Receiver / Clear Backup Data Structures */

typedef enum ubx_cfg_rst_len {
    UBX_CFG_RST_LEN_4  = 4         /* Type: Command | Desc: Reset Receiver / Clear Backup Data Structures */
} ubx_cfg_rst_len_t;

#define UBX_CFG_RXM_ID                  0x11            /* Poll RXM configuration */

typedef enum ubx_cfg_rxm_len {
    UBX_CFG_RXM_LEN_0  = 0,         /* Type: Poll request | Desc: Poll RXM configuration */
    UBX_CFG_RXM_LEN_2  = 2          /* Type: Input/Output | Desc: RXM configuration */
} ubx_cfg_rxm_len_t;

#define UBX_CFG_SBAS_ID                 0x16            /* Poll contents of SBAS Configuration */

typedef enum ubx_cfg_sbas_len {
    UBX_CFG_SBAS_LEN_0  = 0,        /* Type: Poll request | Desc: Poll contents of SBAS Configuration */
    UBX_CFG_SBAS_LEN_8  = 8         /* Type: Input/Output | Desc: SBAS configuration */
} ubx_cfg_sbas_len_t;

#define UBX_CFG_TP5_ID                  0x31            /* Poll Time Pulse Parameters */

typedef enum ubx_cfg_tp5_len {
    UBX_CFG_TP5_LEN_0  = 0,         /* Type: Poll request | Desc: Poll Time Pulse Parameters */
    UBX_CFG_TP5_LEN_1  = 1,         /* Type: Poll request | Desc: Poll Time Pulse Parameters */
    UBX_CFG_TP5_LEN_32 = 32         /* Type: Input/Output | Desc: Time Pulse Parameters */
} ubx_cfg_tp5_len_t;

#define UBX_CFG_USB_ID                  0x1B            /* Poll a USB configuration */

typedef enum ubx_cfg_usb_len {
    UBX_CFG_USB_LEN_0   = 0,        /* Type: Poll request | Desc: Poll a USB configuration */
    UBX_CFG_USB_LEN_108 = 108       /* Type: Input/Output | Desc: USB configuration */
} ubx_cfg_usb_len_t;


/* ============ MON messages ============ */

#define UBX_MON_HW2_ID                  0x0B            /* Type: Periodic/Polled | Desc: Extended Hardware Status */

typedef enum ubx_mon_hw2_len {
    UBX_MON_HW2_LEN_28 = 28         /* Extended Hardware Status */
} ubx_mon_hw2_len_t;

#define UBX_MON_HW_ID                   0x09            /* Type: Periodic/Polled | Desc: Hardware Status */

typedef enum ubx_mon_hw_len {
    UBX_MON_HW_LEN_60               /* Extended Hardware Status */
} ubx_mon_hw_len_t;

#define UBX_MON_IO_ID                   0x02            /* Type: Periodic/Polled | Desc: I/O Subsystem Status */

typedef enum ubx_mon_io_len {
    UBX_MON_IO_LEN_0                /* I/O Subsystem Status */
} ubx_mon_io_len_t;

#define UBX_MON_MSGPP_ID                0x06            /* Type: Periodic/Polled | Desc: Message Parse and Process Status */

typedef enum ubx_mon_msgpp_len {
    UBX_MON_IO_LEN_120 = 120        /* Message Parse and Process Status */
} ubx_mon_msgpp_len_t;

#define UBX_MON_RXBUF_ID                0x07            /* Type: Periodic/Polled | Desc: Receiver Buffer Status */

typedef enum ubx_mon_rxbuf_len {
    UBX_MON_RXBUF_LEN_24 = 24       /* Receiver Buffer Status */
} ubx_mon_rxbuf_len_t;

#define UBX_MON_RXR_ID                  0x21            /* Type: Output | Desc: Receiver Status Information */

typedef enum ubx_mon_rxr_len {
    UBX_MON_RXBUF_LEN_1 = 1         /* Receiver Status Information */
} ubx_mon_rxr_len_t;

#define UBX_MON_TXBUF_ID                0x08            /* Type: Periodic/Polled | Desc: Transmitter Buffer Status */

typedef enum ubx_mon_txbuf_len {
    UBX_MON_TXBUF_LEN_28 = 28       /* Transmitter Buffer Status */
} ubx_mon_txbuf_len_t;

#define UBX_MON_VER_ID                  0x04            /* Type: Polled Request | Desc: Poll Receiver/Software Version */

typedef enum ubx_mon_ver_len {
    UBX_MON_VER_LEN_0 = 0,          /* Poll Receiver/Software Version */
    UBX_MON_VER_LEN_40 = 40         /* Type: Answer to poll | Desc: Receiver/Software Version */
} ubx_mon_ver_len_t;


/* ============ NAV messages ============ */

#define UBX_NAV_AOPSTATUS_ID            0x60            /* Type: Periodic/Polled | Desc: AssistNow Autonomous Status */

typedef enum ubx_nav_aopstatus_len {
    UBX_NAV_AOPSTATUS_LEN_20 = 20   /* Poll AssistNow Autonomous Status */
} ubx_nav_aopstatus_len_t;

#define UBX_NAV_CLOCK_ID                0x22            /* Type: Periodic/Polled | Desc: Clock Solution */

typedef enum ubx_nav_clock_len {
    UBX_NAV_CLOCK_LEN_20 = 20       /* Clock Solution */
} ubx_nav_clock_len_t;

#define UBX_NAV_DGPS_ID                 0x31            /* Type: Periodic/Polled | Desc: DGPS Data Used for NAV */

typedef enum ubx_nav_dgps_len {
    UBX_NAV_DGPS_LEN_16 = 16        /* DGPS Data Used for NAV */
} ubx_nav_dgps_len_t;

#define UBX_NAV_DOP_ID                  0x04            /* Type: Periodic/Polled | Desc: Dilution of precision */

typedef enum ubx_nav_dop_len {
    UBX_NAV_DOP_LEN_18 = 18         /* Dilution of precision */
} ubx_nav_dop_len_t;

#define UBX_NAV_POSECEF_ID              0x01            /* Type: Periodic/Polled | Desc: Position Solution in ECEF */

typedef enum ubx_nav_posecef_len {
    UBX_NAV_POSECEF_LEN_20 = 20         /* Position Solution in ECEF */
} ubx_nav_posecef_len_t;

#define UBX_NAV_POSLLH_ID               0x02            /* Type: Periodic/Polled | Desc: Geodetic Position Solution */

typedef enum ubx_nav_posllh_len {
    UBX_NAV_POSLLH_LEN_28 = 28         /* Geodetic Position Solution */
} ubx_nav_posllh_len_t;

#define UBX_NAV_PVT_ID                  0x07            /* Type: Periodic/Polled | Desc: Navigation Position Velocity Time Solution */

typedef enum ubx_nav_pvt_len {
    UBX_NAV_PVT_LEN_84 = 84         /* Navigation Position Velocity Time Solution */
} ubx_nav_pvt_len_t;

#define UBX_NAV_SBAS_ID                 0x32            /* Type: Periodic/Polled | Desc: SBAS Status Data */

typedef enum ubx_nav_sbas_len {
    UBX_NAV_SBAS_LEN_12 = 12        /* SBAS Status Data */
} ubx_nav_sbas_len_t;

#define UBX_NAV_SOL_ID                  0x06            /* Type: Periodic/Polled | Desc: Navigation Solution Information */

typedef enum ubx_nav_sol_len {
    UBX_NAV_SOL_LEN_52 = 52         /* Navigation Solution Information */
} ubx_nav_sol_len_t;

#define UBX_NAV_STATUS_ID               0x03            /* Type: Periodic/Polled | Desc: Receiver Navigation Status */

typedef enum ubx_nav_status_len {
    UBX_NAV_STATUS_LEN_16 = 16      /* Receiver Navigation Status */
} ubx_nav_status_len_t;

#define UBX_NAV_SVINFO_ID               0x30            /* Type: Periodic/Polled | Desc: Space Vehicle Information */

typedef enum ubx_nav_svinfo_len {
    UBX_NAV_SVINFO_LEN_8 = 8        /* Space Vehicle Information */
} ubx_nav_svinfo_len_t;

#define UBX_NAV_TIMEGPS_ID              0x20            /* Type: Periodic/Polled | Desc: GPS Time Solution */

typedef enum ubx_nav_timegps_len {
    UBX_NAV_TIMEGPS_LEN_16 = 16     /* Space GPS Time Solution */
} ubx_nav_timegps_len_t;

#define UBX_NAV_TIMEUTC_ID              0x21            /* Type: Periodic/Polled | Desc: UTC Time Solution */

typedef enum ubx_nav_timeutc_len {
    UBX_NAV_TIMEUTC_LEN_20 = 20     /* Space UTC Time Solution */
} ubx_nav_timeutc_len_t;

#define UBX_NAV_VELECEF_ID              0x11            /* Type: Periodic/Polled | Desc: Velocity Solution in ECEF */

typedef enum ubx_nav_velecef_len {
    UBX_NAV_VELECEF_LEN_20 = 20     /* Space Velocity Solution in ECEF */
} ubx_nav_velecef_len_t;

#define UBX_NAV_VELNED_ID               0x12            /* Type: Periodic/Polled | Desc: Velocity Solution in NED */

typedef enum ubx_nav_velned_len {
    UBX_NAV_VELNED_LEN_36 = 36      /* Space Velocity Solution in NED */
} ubx_nav_velned_len_t;

#endif
