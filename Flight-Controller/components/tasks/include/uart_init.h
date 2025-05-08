#ifndef UART_INIT_H
#define UART_INIT_H

#include <driver/uart.h>

/**
 * @brief Initialize UART peripheral on given port
 * @param uart_num: UART port num
 * @retval UART queue
 */
QueueHandle_t uart_init( uart_port_t uart_num );

#endif
