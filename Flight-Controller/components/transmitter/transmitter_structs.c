#include <transmitter_structs.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#define TRANSMITTER_TAG "TRANSMITTER"


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * Private functions prototypes
 * ----------------------------
 */

/**
 * @brief Copy MCU MAC address into given array
 * @param mac_addr: Array to store MCU MAC Address
 * @retval none
 */
static void get_mac_address( uint8_t * mac_addr );

/**
 * @brief Set MCU new MAC address
 * @param mac_addr: MAC Address to be setted
 * @retval esp_err_t
 */
static esp_err_t set_mac_address( const uint8_t * mac_addr );

/**
 * @brief Initialize NVS of MCU
 * @param none
 * @retval none
 */
static void nvs_init( void );


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


esp_err_t transmitter_init( transmitter_t * self, const uint8_t mac_addr_p[ MAC_ADDR_SIZE ] ) {
    ESP_LOGI( TRANSMITTER_TAG, "Initializing Transmitter object..." );

    nvs_init();
    get_mac_address( self->mac_addr );
    ESP_ERROR_CHECK( set_mac_address( mac_addr_p ) );
    get_mac_address( self->mac_addr );
    ps3SetEventCallback( controller_event_cb );
    ps3SetBluetoothMacAddress( self->mac_addr );
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


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void controller_event_cb( ps3_t ps3, ps3_event_t event ) {
    extern tx_buttons_t * GlobalTxButtons;    /* Declared in drone.c source file */

    /* If cross button is being pressed */
    if( event.button_down.cross ) {    
        
        GlobalTxButtons->cross = true;
    }

    /* If cross button is not being pressed */
    else if( event.button_up.cross ) {
        
        GlobalTxButtons->cross = false;
    }


    /* If square button is being pressed */
    if( event.button_down.square ) {
        
        GlobalTxButtons->square = true;
    }

    /* If square button is not being pressed */
    else if( event.button_up.square ) {
        
        GlobalTxButtons->square = false;
    }


    /* If triangle button is being pressed */
    if( event.button_down.triangle ) {
        
        GlobalTxButtons->triangle = true;
    }

    /* If triangle button is not being pressed */
    else if( event.button_up.triangle ) {
        
        GlobalTxButtons->triangle = false;
    }


    /* If circle button is being pressed */
    if( event.button_down.circle ) {
        
        GlobalTxButtons->circle = true;
    }

    /* If circle button is not being pressed */
    else if( event.button_up.circle ) {
        
        GlobalTxButtons->circle = false;
    }


    /* If up button is being pressed */
    if( event.button_down.up ) {
        
        GlobalTxButtons->up = true;
    }

    /* If up button is not being pressed */
    else if( event.button_up.up ) {
        
        GlobalTxButtons->up = false;
    }


    /* If down button is being pressed */
    if( event.button_down.down ) {
        
        GlobalTxButtons->down = true;
    }

    /* If down button is not being pressed */
    else if( event.button_up.down ) {
        
        GlobalTxButtons->down = false;
    }


    /* If left button is being pressed */
    if( event.button_down.left ) {
        
        GlobalTxButtons->left = true;
    }

    /* If left button is not being pressed */
    else if( event.button_up.left ) {
        
        GlobalTxButtons->left = false;
    }


    /* If right button is being pressed */
    if( event.button_down.right ) {
        
        GlobalTxButtons->right = true;
    }

    /* If right button is not being pressed */
    else if( event.button_up.right ) {
        
        GlobalTxButtons->right = false;
    }


    /* If r1 button is being pressed */
    if( event.button_down.r1 ) {
        
        GlobalTxButtons->r1 = true;
    }

    /* If r1 button is not being pressed */
    else if( event.button_up.r1 ) {
        
        GlobalTxButtons->r1 = false;
    }


    /* If l1 button is being pressed */
    if( event.button_down.l1 ) {
        
        GlobalTxButtons->l1 = true;
    }

    /* If l1 button is not being pressed */
    else if( event.button_up.l1 ) {
        
        GlobalTxButtons->l1 = false;
    }


    /* If r2 button is being pressed */
    if( event.button_down.r2 ) {
        
        GlobalTxButtons->r2 = true;
    }

    /* If r2 button is not being pressed */
    else if( event.button_up.r2 ) {
        
        GlobalTxButtons->r2 = false;
    }


    /* If l2 button is being pressed */
    if( event.button_down.l2 ) {
        
        GlobalTxButtons->l2 = true;
    }

    /* If l2 button is not being pressed */
    else if( event.button_up.l2 ) {
        
        GlobalTxButtons->l2 = false;
    }
}
