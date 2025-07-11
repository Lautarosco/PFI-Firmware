#ifndef NEO7_UBX_PACKET_H
#define NEO7_UBX_PACKET_H

typedef enum ubx_packet_index {
    UBX_PACKET_POS_SYNC1 = 0,       /* First byte of the header of a UBX packet */
    UBX_PACKET_POS_SYNC2,           /* Second byte of the header of a UBX packet */
    UBX_PACKET_POS_CLASS,           /* Class of the message */
    UBX_PACKET_POS_ID,              /* ID of the message */
    UBX_PACKET_POS_LENGTH_LSB,      /* Length of the payload (LSB) in little endian */
    UBX_PACKET_POS_LENGTH_MSB,      /* Length of the payload (MSB) in little endian */
    UBX_PACKET_POS_PAYLOAD          /* Payload data */
} ubx_packet_index_t;

#endif
