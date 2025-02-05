#include <uart_init.h>
#include <driver/gpio.h>

QueueHandle_t uart_init( uart_port_t uart_num ) {

    /**
     * 
     * Universal Asynchronous Receiver/Transmitter (UART) v5.4
     * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
     * 
     */

    /* 1. Set Communication Parameters */

    /* UART configs */
    static uart_config_t uart_config = {

        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    /* Set UART configs */
    ESP_ERROR_CHECK( uart_param_config( uart_num, &uart_config ) );

    /* 2. Set Communication Pins */

    int tx_io_num = 0;
    int rx_io_num = 0;
    
    if( uart_num == 0 ) {

        tx_io_num = GPIO_NUM_1;
        rx_io_num = GPIO_NUM_3;
    }

    else if( uart_num == 2 ) {

        tx_io_num = GPIO_NUM_17;
        rx_io_num = GPIO_NUM_16;
    }

    ESP_ERROR_CHECK( uart_set_pin( uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );

    /* 3. Install Drivers */

    const int uart_buffer_size = 1024 * 2;

    QueueHandle_t uart_queue;

    ESP_ERROR_CHECK( uart_driver_install( uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0 ) );

    return uart_queue;
}