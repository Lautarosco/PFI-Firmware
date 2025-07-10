#ifndef NEO7_UBX_CLASSES_H
#define NEO7_UBX_CLASSES_H

/**
 * @file neo7_ubx_classes.h
 * @brief UBX Protocol Classes for NEO-7 GPS Module
 * 
 * https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf, Sec 31
 */

#define UBX_SYNC1 0xB5      /* First byte of the header of a UBX packet */
#define UBX_SYNC2 0x62      /* Second byte of the header of a UBX packet */

typedef enum ubx_classes {
    UBX_CLASS_NAV = 0x01,           /* Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used */
    UBX_CLASS_RXM = 0x02,           /* Receiver Manager Messages: Satellite Status, RTC Status */
    UBX_CLASS_INF = 0x04,           /* Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice */
    UBX_CLASS_ACK = 0x05,           /* Ack/Nack Messages: as replies to CFG Input Messages */
    UBX_CLASS_CFG = 0x06,           /* Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. */
    UBX_CLASS_MON = 0x0A,           /* Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status */
    UBX_CLASS_AID = 0x0B,           /* AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input */
    UBX_CLASS_TIM = 0x0D,           /* Timing Messages: Time Pulse Output, Timemark Results */
    UBX_CLASS_LOG = 0x21            /* Logging Messages: Log creation, deletion, info and retrieval */
} ubx_classes_t;

#endif
