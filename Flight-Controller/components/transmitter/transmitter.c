#include <stdio.h>
#include <transmitter.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>



const char * TRANSMITTER_TAG = "TRANSMITTER";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


#if PLAYSTATION_TX

    /** @details Private functions implementations */

    /**
     * @brief Activate buttons flags
     * @param ps3: ps3_t struct
     * @param event: pse3_event_t struct
     * @retval none
     */
    static void controller_event_cb( ps3_t ps3, ps3_event_t event ) {

        /* Declared in drone.c source file */
        extern tx_buttons_t * GlobalTxButtons;

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

        /* If select button is being pressed */
        if( event.button_down.select ) {
            
            GlobalTxButtons->select = true;
        }

        /* If select button is not being pressed */
        else if( event.button_up.select ) {
            
            GlobalTxButtons->select = false;
        }

        /* If start button is being pressed */
        if( event.button_down.start ) {
            
            GlobalTxButtons->start = true;
        }

        /* If start button is not being pressed */
        else if( event.button_up.start ) {
            
            GlobalTxButtons->start = false;
        }

        /* If ps button is being pressed */
        if( event.button_down.ps ) {
            
            GlobalTxButtons->ps = true;
        }

        /* If ps button is not being pressed */
        else if( event.button_up.ps ) {
            
            GlobalTxButtons->ps = false;
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
     * @brief Initialize Transmitter
     * @param obj: Pointer to Transmitter object
     * @param mac_p: Mac address of transmitter
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

        /* Initialize Transmitter mac address in 0 */
        for (int i = 0; i < MAC_ADDR_SIZE; i++) { Tx->mac_addr[ i ] = 0; }

        /* Assign Drone object global variables to Transmitter object global variables  */
        Tx->global_variables = global_variables;

        /* Pointer assignment to functions */
        Tx->init = transmitter_init;

        ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );

        return Tx;
    }

#elif WEBSV_TX

    #include <string.h>
    #include <esp_http_server.h>
    #include <cJSON.h>
    #include <esp_spiffs.h>

    #define WIFI_SSID "ESP32_Transmitter"
    #define WIFI_PASS "12345678"

    static esp_err_t button_handler( httpd_req_t * req ) {

        /* Create a buffer to store events related data */
        char buff[ 100 ];

        /* Read data from HTTP server */
        int ret = httpd_req_recv( req, buff, sizeof( buff ) - 1 );
        if( ret <= 0 ) {

            if( ret == HTTPD_SOCK_ERR_TIMEOUT ) {

                httpd_resp_send_408( req );
            }

            return ESP_FAIL;
        }

        /* Add NULL-terminate to end of buffer */
        buff[ ret ] = '\0';

        /* Parse JSON */
        cJSON * json = cJSON_Parse( buff );

        /* Check if received JSON is valid */
        if( !json ) {

            ESP_LOGE( TRANSMITTER_TAG, "Invalid JSON received" );
            return ESP_ERR_INVALID_RESPONSE;
        }

        /* Store button pressed */
        const char * button = cJSON_GetObjectItem( json, "button" )->valuestring;

        /* Store occurred action */
        const char * action = cJSON_GetObjectItem( json, "action" )->valuestring;

        /* Declared in drone.c source file */
        extern tx_buttons_t * GlobalTxButtons;

        /* If any button was pressed */
        if( !strcmp( action, "pressed" ) ) {

            /* If x button is being pressed */
            if( !strcmp( button, "x" ) ) {

                GlobalTxButtons->cross = true;
            }

            /* If square button is being pressed */
            else if( !strcmp( button, "square" ) ) {

                GlobalTxButtons->square = true;
            }

            /* If triangle button is being pressed */
            else if( !strcmp( button, "triangle" ) ) {

                GlobalTxButtons->triangle = true;
            }

            /* If circle button is being pressed */
            else if( !strcmp( button, "circle" ) ) {

                GlobalTxButtons->circle = true;
            }

            /* If up button is being pressed */
            else if( !strcmp( button, "up" ) ) {

                GlobalTxButtons->up = true;
            }

            /* If down button is being pressed */
            else if( !strcmp( button, "down" ) ) {

                GlobalTxButtons->down = true;
            }

            /* If left button is being pressed */
            else if( !strcmp( button, "left" ) ) {

                GlobalTxButtons->left = true;
            }

            /* If right button is being pressed */
            else if( !strcmp( button, "right" ) ) {

                GlobalTxButtons->right = true;
            }

            /* If r1 button is being pressed */
            else if( !strcmp( button, "r1" ) ) {

                GlobalTxButtons->r1 = true;
            }

            /* If l1 button is being pressed */
            else if( !strcmp( button, "l1" ) ) {

                GlobalTxButtons->l1 = true;
            }

            /* If r2 button is being pressed */
            else if( !strcmp( button, "r2" ) ) {

                GlobalTxButtons->r2 = true;
            }

            /* If l2 button is being pressed */
            else if( !strcmp( button, "l2" ) ) {

                GlobalTxButtons->l2 = true;
            }

            /* If start button is being released */
            else if( !strcmp( button, "start" ) ) {

                GlobalTxButtons->start = true;
            }

            /* If reset button is being released */
            else if( !strcmp( button, "reset" ) ) {

                GlobalTxButtons->ps = true;
            }
        }

        /* If any button was released */
        else if( !strcmp( action, "released" ) ) {

            /* If x button is being released */
            if( !strcmp( button, "x" ) ) {

                GlobalTxButtons->cross = false;
            }

            /* If square button is being released */
            else if( !strcmp( button, "square" ) ) {

                GlobalTxButtons->square = false;
            }

            /* If triangle button is being released */
            else if( !strcmp( button, "triangle" ) ) {
                
                GlobalTxButtons->triangle = false;
            }

            /* If circle button is being released */
            else if( !strcmp( button, "circle" ) ) {

                GlobalTxButtons->circle = false;
            }

            /* If up button is being released */
            else if( !strcmp( button, "up" ) ) {

                GlobalTxButtons->up = false;
            }

            /* If down button is being released */
            else if( !strcmp( button, "down" ) ) {

                GlobalTxButtons->down = false;
            }

            /* If left button is being released */
            else if( !strcmp( button, "left" ) ) {

                GlobalTxButtons->left = false;
            }

            /* If right button is being released */
            else if( !strcmp( button, "right" ) ) {

                GlobalTxButtons->right = false;
            }

            /* If r1 button is being released */
            else if( !strcmp( button, "r1" ) ) {

                GlobalTxButtons->r1 = false;
            }

            /* If l1 button is being released */
            else if( !strcmp( button, "l1" ) ) {

                GlobalTxButtons->l1 = false;
            }

            /* If r2 button is being released */
            else if( !strcmp( button, "r2" ) ) {

                GlobalTxButtons->r2 = false;
            }

            /* If l2 button is being released */
            else if( !strcmp( button, "l2" ) ) {

                GlobalTxButtons->l2 = false;
            }

            /* If start button is being released */
            else if( !strcmp( button, "start" ) ) {

                GlobalTxButtons->start = false;
            }

            /* If reset button is being released */
            else if( !strcmp( button, "reset" ) ) {

                GlobalTxButtons->ps = false;
            }
        }

        cJSON_Delete( json );
        httpd_resp_set_type( req, "application/json" );
        httpd_resp_sendstr( req, "{\"status\":\"ok\"}" );

        return ESP_OK;
    }

    static esp_err_t file_get_handler( httpd_req_t * req ) {

        /* Path to html file stored in flash memory */
        const char * filepath = "/spiffs/index.html";   /* '/base_path/filename.extension' */

        /* Open html in read only mode */
        FILE * file = fopen( filepath, "r" );

        if( !file ) {

            ESP_LOGE( TRANSMITTER_TAG, "Failed to open '%s' file... See function %s in line %d", filepath, __func__, __LINE__ );
            httpd_resp_send_404( req );
            return ESP_FAIL;
        }

        /* Create buffer to store html file related data */
        char buff[ 1024 ];

        /* Read raw data from html file */
        size_t read_bytes;

        /* Send html internal file to HTTP server */
        while( ( read_bytes = fread( buff, 1, sizeof( buff ), file ) ) > 0 ) {

            httpd_resp_send_chunk( req, buff, read_bytes);
        }
        
        /* Close html file */
        fclose( file );

        /* End response */
        httpd_resp_send_chunk( req, NULL, 0 );
        return ESP_OK;
    }

    static void start_webserver( void ) {

        httpd_handle_t server = NULL;
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();

        /* Start HTTP server */
        if( httpd_start( &server, &config ) == ESP_OK ) {

            httpd_uri_t file_get_uri = {

                .uri      = "/",
                .method   = HTTP_GET,
                .handler  = file_get_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler( server, &file_get_uri );

            httpd_uri_t button_uri = {

                .uri      = "/button",
                .method   = HTTP_POST,
                .handler  = button_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler( server, &button_uri );
        }

        else {

            ESP_LOGE( TRANSMITTER_TAG, "Failed to start HTTP server" );
        }
    }

    static void wifi_init_softap( void ) {
        
        /* Initialize WiFi stack config */
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );

        /* Configure WiFi in SoftAP mode */
        wifi_config_t wifi_config = {

            .ap = {

                .ssid = WIFI_SSID,
                .ssid_len = strlen( WIFI_SSID ),
                .password = WIFI_PASS,  /* Must be >= 8 characters, otherwise it'll be an open WiFi */
                .max_connection = 1,    /* Maximum devices connected */
                .authmode = WIFI_AUTH_WPA_PSK,
                .beacon_interval = 200
            }
        };

        /* Check if password is empty */
        if( strlen( ( const char * ) wifi_config.ap.password ) == 0 ) {

            /* Open WiFi */
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }

        /* Set mode */
        ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_AP ) );

        /* Apply configs */
        ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_AP, &wifi_config ) );

        /* Start WiFi */
        ESP_ERROR_CHECK( esp_wifi_start() );

        ESP_LOGI( TRANSMITTER_TAG, "WiFi AP started... SSID: %s, Password: %s", wifi_config.ap.ssid, wifi_config.ap.password );
    }

    /**
     * @brief Initialize Transmitter object
     * @param obj: Pointer to Transmitter object
     * @retval esp_err_t
     */
    static esp_err_t transmitter_init( transmitter_t * obj ) {

        ESP_LOGI( TRANSMITTER_TAG, "Initializing Transmitter object..." );

        /* Initialize No Volatile System partition */
        ESP_ERROR_CHECK( nvs_flash_init() );

        /* Initialize spiffs */
        esp_vfs_spiffs_conf_t config = {

            .base_path              = "/spiffs",    /* See partition table => Spiffs row => Name column */
            .partition_label        = NULL,
            .max_files              = 5,
            .format_if_mount_failed = true
        };

        /* Mount spiffs configs */
        esp_err_t ret = esp_vfs_spiffs_register( &config );

        /* Check if mount was succesfull */
        if( ret != ESP_OK ) {

            if( ret == ESP_ERR_NOT_FOUND ) {

                ESP_LOGE( TRANSMITTER_TAG, "Failed to find spiffs partition" );
            }

            else if( ret == ESP_FAIL ) {

                ESP_LOGE( TRANSMITTER_TAG, "Failed to mount spiffs partition" );
            }

            else {

                ESP_LOGE( TRANSMITTER_TAG, "Failed to initialize spiffs ( %s )", esp_err_to_name( ret ) );
            }
        }

        ESP_LOGI( TRANSMITTER_TAG, "Spiffs mounted succesfully" );

        /* Initialize network interface */
        ESP_ERROR_CHECK( esp_netif_init() );

        /* Initialize event loop to catch HTTP related events */
        ESP_ERROR_CHECK( esp_event_loop_create_default() );

        /* Create default WiFi AP network interface */
        esp_netif_create_default_wifi_ap();

        /* Set up the SoftAP */
        wifi_init_softap();

        /* Start HTTP server */
        start_webserver();

        ESP_LOGI( TRANSMITTER_TAG, "Transmitter object initialized" );

        return ESP_OK;
    }

    transmitter_t * Transmitter( drone_globals_t * global_variables ) {

        ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

        transmitter_t * Tx = malloc( sizeof( transmitter_t ) );

        /* Assign Drone object global variables to Transmitter object global variables  */
        Tx->global_variables = global_variables;

        /* Pointer assignment to functions */
        Tx->init = transmitter_init;

        ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );

        return Tx;
    }

#endif
