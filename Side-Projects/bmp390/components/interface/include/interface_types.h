#ifndef INTERFACE_TYPES_H
#define INTERFACE_TYPES_H

typedef enum digital_interfaces {
    /* No interface was selected */
    NONE,

    /* I2C interface */
    I2C,

    /* SPI interface */
    SPI
} digital_interfaces_t;

#endif
