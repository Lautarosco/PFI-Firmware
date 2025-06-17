#ifndef SERIAL_TYPES_H
#define SERIAL_TYPES_H

typedef enum serial_iface_type {
    /* I2C serial interface */
    IFACE_I2C,

    /* SPI serial interface */
    IFACE_SPI
} serial_iface_type_t;

#endif
