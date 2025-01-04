#include <stdio.h>
#include <transmitter.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

const char * TRANSMITTER_TAG = "TRANSMITTER";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Private functions implementations */

/**
 * @brief Activate buttons flags
 * @param ps3: ps3_t struct
 * @param event: pse3_event_t struct
 * @retval none
 */
static void controller_event_cb( ps3_t ps3, ps3_event_t event ) {
    extern tx_buttons_t * GlobalTxButtons;    /* Declared in main.c source file */

    if( event.button_down.cross ) {
        GlobalTxButtons->cross = true;
    }
    else if( event.button_up.cross ) {
        GlobalTxButtons->cross = false;
    }
    else if( event.button_down.square ) {
        GlobalTxButtons->square = true;
    }
    else if( event.button_up.square ) {
        GlobalTxButtons->square = false;
    }
    else if( event.button_down.triangle ) {
        GlobalTxButtons->triangle = true;
    }
    else if( event.button_up.triangle ) {
        GlobalTxButtons->triangle = false;
    }
    else if( event.button_down.circle ) {
        GlobalTxButtons->circle = true;
    }
    else if( event.button_up.circle ) {
        GlobalTxButtons->circle = false;
    }
    else if( event.button_down.up ) {
        GlobalTxButtons->up = true;
    }
    else if( event.button_up.up ) {
        GlobalTxButtons->up = false;
    }
    else if( event.button_down.down ) {
        GlobalTxButtons->down = true;
    }
    else if( event.button_up.down ) {
        GlobalTxButtons->down = false;
    }
    else if( event.button_down.left ) {
        GlobalTxButtons->left = true;
    }
    else if( event.button_up.left ) {
        GlobalTxButtons->left = false;
    }
    else if( event.button_down.right ) {
        GlobalTxButtons->right = true;
    }
    else if( event.button_up.right ) {
        GlobalTxButtons->right = false;
    }
    else if( event.button_down.r1 ) {
        GlobalTxButtons->r1 = true;
    }
    else if( event.button_up.r1 ) {
        GlobalTxButtons->r1 = false;
    }
    else if( event.button_down.l1 ) {
        GlobalTxButtons->l1 = true;
    }
    else if( event.button_up.l1 ) {
        GlobalTxButtons->l1 = false;
    }
    else if( event.button_down.r2 ) {
        GlobalTxButtons->r2 = true;
    }
    else if( event.button_up.r2 ) {
        GlobalTxButtons->r2 = false;
    }
    else if( event.button_down.l2 ) {
        GlobalTxButtons->l2 = true;
    }
    else if( event.button_up.l2 ) {
        GlobalTxButtons->l2 = false;
    }
}

static void get_mac_address( uint8_t * mac_addr ) {
    esp_wifi_get_mac( ESP_IF_WIFI_STA, mac_addr );
    ESP_LOGI( TRANSMITTER_TAG, "MCU actual MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac_addr[ 0 ], mac_addr[ 1 ], mac_addr[ GPIO_NUM_2 ], mac_addr[ 3 ], mac_addr[ 4 ], mac_addr[ 5 ] );
}

static esp_err_t set_mac_address( const uint8_t * mac_addr ) {
    ESP_ERROR_CHECK( esp_wifi_set_mac( ESP_IF_WIFI_STA, mac_addr ) );
    fflush( stdout );
    ESP_LOGI( TRANSMITTER_TAG, "Setting MCU MAC address to: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac_addr[ 0 ], mac_addr[ 1 ], mac_addr[ GPIO_NUM_2 ], mac_addr[ 3 ], mac_addr[ 4 ], mac_addr[ 5 ] );
    
    return ESP_OK;
}

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
 * @brief Initialize Joystick. 
 * @param obj: Pointer to joystick_t struct.
 * @param mac_p: Mac address.
 * @retval esp_err_t
 */
static esp_err_t transmitter_init( transmitter_t * obj, const uint8_t mac_addr_p[ MAC_ADDR_SIZE ] ) {
    ESP_LOGI( TRANSMITTER_TAG, "Initializing Transmitter object..." );

    nvs_init();
    get_mac_address( obj->mac_addr );
    ESP_ERROR_CHECK( set_mac_address( mac_addr_p ) );
    get_mac_address( obj->mac_addr );
    ps3SetEventCallback( controller_event_cb );
    ps3SetBluetoothMacAddress( obj->mac_addr );
    ps3Init();
    ESP_ERROR_CHECK( gpio_reset_pin( GPIO_NUM_2 ) );
    ESP_ERROR_CHECK( gpio_set_direction( GPIO_NUM_2, GPIO_MODE_OUTPUT ) );
    ESP_ERROR_CHECK( gpio_set_level( GPIO_NUM_2, 1 ) );
    ESP_LOGI( TRANSMITTER_TAG, "Waiting for transmitter to connect..." );
    while( !ps3IsConnected() )
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    ESP_LOGI( TRANSMITTER_TAG, "Transmitter succesfully connected" );

    ESP_LOGI( TRANSMITTER_TAG, "Transmitter object initialized" );

    return ESP_OK;
}

transmitter_t * Transmitter( drone_globals_t * global_variables ) {
    ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

    transmitter_t * Tx = malloc( sizeof( transmitter_t ) );

    for (int i = 0; i < MAC_ADDR_SIZE; i++) { Tx->mac_addr[ i ] = 0; }
    Tx->global_variables = global_variables;

    Tx->init = transmitter_init;     /* Pointer assignment to functions */

    ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );

    return Tx;
}
