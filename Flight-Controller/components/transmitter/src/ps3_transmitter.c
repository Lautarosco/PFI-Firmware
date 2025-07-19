#include "transmitter.h"
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

/** @details Private functions implementations */

typedef struct js_btn_wrapper {
    int btn_index;
    bool btn_value;
} js_btn_wrapper_t;

typedef struct tx_btns {
    char * btn_name;
    js_btn_wrapper_t btn_arr[2];
    bool * tx_btn_ptr;
} tx_btns_t;

/**
 * @brief Activate buttons flags
 * @param ps3: ps3_t struct
 * @param event: pse3_event_t struct
 * @retval none
 */
static void controller_event_cb( ps3_t ps3, ps3_event_t event ) {

    /* ========================= START Process incomming data ========================= */

    /* Declared in drone.c source file */
    extern tx_buttons_t * GlobalTxButtons;

    tx_btns_t tx_btns_arr[] = {
        {.btn_name = "cross",    .btn_arr = {{.btn_index = event.button_down.cross,    .btn_value = true}, {.btn_index = event.button_up.cross,    .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->cross)},
        {.btn_name = "triangle", .btn_arr = {{.btn_index = event.button_down.triangle, .btn_value = true}, {.btn_index = event.button_up.triangle, .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->triangle)},
        {.btn_name = "square",   .btn_arr = {{.btn_index = event.button_down.square,   .btn_value = true}, {.btn_index = event.button_up.square,   .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->square)},
        {.btn_name = "circle",   .btn_arr = {{.btn_index = event.button_down.circle,   .btn_value = true}, {.btn_index = event.button_up.circle,   .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->circle)},
        {.btn_name = "up",       .btn_arr = {{.btn_index = event.button_down.up,       .btn_value = true}, {.btn_index = event.button_up.up,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->up)},
        {.btn_name = "down",     .btn_arr = {{.btn_index = event.button_down.down,     .btn_value = true}, {.btn_index = event.button_up.down,     .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->down)},
        {.btn_name = "left",     .btn_arr = {{.btn_index = event.button_down.left,     .btn_value = true}, {.btn_index = event.button_up.left,     .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->left)},
        {.btn_name = "right",    .btn_arr = {{.btn_index = event.button_down.right,    .btn_value = true}, {.btn_index = event.button_up.right,    .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->right)},
        {.btn_name = "l1",       .btn_arr = {{.btn_index = event.button_down.l1,       .btn_value = true}, {.btn_index = event.button_up.l1,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->l1)},
        {.btn_name = "l2",       .btn_arr = {{.btn_index = event.button_down.l2,       .btn_value = true}, {.btn_index = event.button_up.l2,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->l2)},
        {.btn_name = "r1",       .btn_arr = {{.btn_index = event.button_down.r1,       .btn_value = true}, {.btn_index = event.button_up.r1,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->r1)},
        {.btn_name = "r2",       .btn_arr = {{.btn_index = event.button_down.r2,       .btn_value = true}, {.btn_index = event.button_up.r2,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->r2)},
        {.btn_name = "start",    .btn_arr = {{.btn_index = event.button_down.start,    .btn_value = true}, {.btn_index = event.button_up.start,    .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->start)},
        {.btn_name = "reset",    .btn_arr = {{.btn_index = event.button_down.ps,       .btn_value = true}, {.btn_index = event.button_up.ps,       .btn_value = false}}, .tx_btn_ptr = &(GlobalTxButtons->ps)}
    };
    
    // Match received button with buttons array
    for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
    {
        if(tx_btns_arr[i].btn_arr[0].btn_index) {
            (*tx_btns_arr[i].tx_btn_ptr) = tx_btns_arr[i].btn_arr[0].btn_value;
            printf("<%s> button was pressed\n", tx_btns_arr[i].btn_name);
            break;
        } else if(tx_btns_arr[i].btn_arr[1].btn_index) {
            (*tx_btns_arr[i].tx_btn_ptr) = tx_btns_arr[i].btn_arr[1].btn_value;
            printf("<%s> button was released\n", tx_btns_arr[i].btn_name);
            break;
        }
    }

    /* ========================= END Process incomming data ========================= */
}

/**
 * @brief Get mac address of MCU and store it
 * @param mac_addr: Array to store MCU mac address
 * @retval none
 */
static void get_mac_address( uint8_t * mac_addr ) {

    esp_wifi_get_mac( ESP_IF_WIFI_STA, mac_addr );
    ESP_LOGI( TRANSMITTER_TAG, "MCU actual MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac_addr[ 0 ], mac_addr[ 1 ], mac_addr[ GPIO_NUM_2 ], mac_addr[ 3 ], mac_addr[ 4 ], mac_addr[ 5 ] );
}

/**
 * @brief Set MCU a new mac address
 * @param mac_addr: Array containing the new mac address
 * @retval ESP_OK if success
 */
static esp_err_t set_mac_address( const uint8_t * mac_addr ) {

    ESP_ERROR_CHECK( esp_wifi_set_mac( ESP_IF_WIFI_STA, mac_addr ) );
    fflush( stdout );
    ESP_LOGI( TRANSMITTER_TAG, "Setting MCU MAC address to: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac_addr[ 0 ], mac_addr[ 1 ], mac_addr[ GPIO_NUM_2 ], mac_addr[ 3 ], mac_addr[ 4 ], mac_addr[ 5 ] );
    
    return ESP_OK;
}

/**
 * @brief Initialize NVS of MCU
 * @param none
 * @retval none
 */
static void nvs_init( void ) {

    esp_err_t res = nvs_flash_init();
    if ( res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK( res );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

/**
 * @brief Initialize Transmitter
 * @param tx: Pointer to Transmitter object
 * @param mac_p: Mac address of transmitter
 * @retval esp_err_t
 */
esp_err_t ps3_transmitter_init( transmitter_t * tx, const uint8_t mac_p[ MAC_ADDR_SIZE]) {

    ESP_LOGI( TRANSMITTER_TAG, "Initializing Transmitter object..." );

    /* Initialize NVS */
    nvs_init();

    /* Get MCU mac address */
    get_mac_address( tx->bluetooth_connection.mac_addr );

    /* Set transmitter mac address to MCU */
    ESP_ERROR_CHECK( set_mac_address( mac_p ) );

    /* Check if mac address was successfully changed */
    get_mac_address( tx->bluetooth_connection.mac_addr );

    /* Initialize transmitter callback */
    ps3SetEventCallback( controller_event_cb );

    /* Initialize transmitter bluetooth */
    ps3SetBluetoothMacAddress( tx->bluetooth_connection.mac_addr );
    
    /* Initialize transmitter */
    ps3Init();

    /* Initialize GPIO2 */
    ESP_ERROR_CHECK( gpio_reset_pin( GPIO_NUM_2 ) );
    ESP_ERROR_CHECK( gpio_set_direction( GPIO_NUM_2, GPIO_MODE_OUTPUT ) );
    ESP_ERROR_CHECK( gpio_set_level( GPIO_NUM_2, 1 ) );

    ESP_LOGI( TRANSMITTER_TAG, "Waiting for transmitter to connect..." );

    /* Wait until transmitter connects */
    while( !ps3IsConnected() ) {

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    ESP_LOGI( TRANSMITTER_TAG, "Transmitter succesfully connected" );

    ESP_LOGI( TRANSMITTER_TAG, "Transmitter object initialized" );

    return ESP_OK;
}
