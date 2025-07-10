#ifndef NEO7_NMEA_MESSAGES_H
#define NEO7_NMEA_MESSAGES_H

/**
 * @file neo7_nmea_messages.h
 * @brief NMEA Protocol Messages for NEO-7 GPS Module
 * 
 * https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf, Sec 23
 */


/* =========== NMEA Standard Messages =========== */

#define NMEA_CLASS              0xF0        /* NMEA Class for NMEA standard messages */

#define NMEA_DTM_ID             0x0A        /* Datum Reference */
#define NMEA_GBS_ID             0x09        /* GNSS Satellite Fault Detection */
#define NMEA_GGA_ID             0x00        /* Global positioning system fix data */
#define NMEA_GLL_ID             0x01        /* Latitude and longitude, with time of position fix and status */
#define NMEA_GLQ_ID             0x43        /* Poll a standard message (if the current Talker ID is GL) */
#define NMEA_GNQ_ID             0x42        /* Poll a standard message (if the current Talker ID is GN) */
#define NMEA_GNS_ID             0x0D        /* GNSS fix data */
#define NMEA_GPQ_ID             0x40        /* Poll a standard message (if the current Talker ID is GP) */
#define NMEA_GRS_ID             0x06        /* GNSS Range Residuals */
#define NMEA_GSA_ID             0x02        /* GNSS DOP and Active Satellites */
#define NMEA_GST_ID             0x07        /* GNSS Pseudo Range Error Statistics */
#define NMEA_GSV_ID             0x03        /* GNSS Satellites in View */
#define NMEA_RMC_ID             0x04        /* Recommended Minimum data */
#define NMEA_TXT_ID             0x41        /* Text Transmission */
#define NMEA_VTG_ID             0x05        /* Course over ground and Ground speed */
#define NMEA_ZDA_ID             0x08        /* Time and Date */


/* =========== NMEA PUBX Messages (ublox propietary messages) =========== */

#define NMEA_UBX_CLASS          0xF1        /* NMEA Class for UBX messages */

#define NMEA_UBX_CONFIG_ID      0x41        /* Set Protocols and Baudrate */
#define NMEA_UBX_POSITION_ID    0x00        /* Poll a PUBX,00 message | Lat/Long Position Dat */
#define NMEA_UBX_RATE_ID        0x40        /* Set NMEA message output rate */
#define NMEA_UBX_SVSTATUS_ID    0x03        /* Poll a PUBX,03 message | Satellite Status */
#define NMEA_UBX_TIME_ID        0x04        /* Poll a PUBX,04 message | Time of Day and Clock Information */

#endif
