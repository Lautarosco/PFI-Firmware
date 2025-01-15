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
     * @param obj: Pointer to Transmitter object
     * @param mac_p: Mac address of transmitter
     * @retval esp_err_t
     */
    static esp_err_t transmitter_init( transmitter_t * obj, const uint8_t mac_addr_p[ MAC_ADDR_SIZE ] ) {

        ESP_LOGI( TRANSMITTER_TAG, "Initializing Transmitter object..." );

        /* Initialize NVS */
        nvs_init();

        /* Get MCU mac address */
        get_mac_address( obj->mac_addr );

        /* Set transmitter mac address to MCU */
        ESP_ERROR_CHECK( set_mac_address( mac_addr_p ) );

        /* Check if mac address was successfully changed */
        get_mac_address( obj->mac_addr );

        /* Initialize transmitter callback */
        ps3SetEventCallback( controller_event_cb );

        /* Initialize transmitter bluetooth */
        ps3SetBluetoothMacAddress( obj->mac_addr );
        
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

    transmitter_t * Transmitter( drone_globals_t * global_variables ) {

        ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

        /* Assign memory for a Transmitter object */
        transmitter_t * Tx = ( transmitter_t * ) malloc( sizeof( transmitter_t ) );

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
    // #include <esp_spiffs.h>

    /* TESTING */

        #include <esp_spp_api.h>
        #include <esp_bt.h>
        #include <esp_bt_main.h>
        #include <esp_gap_bt_api.h>

    /* TESTING */

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

        ESP_LOGE( "DEBUG", "Action: %s | Button: %s", action, button );

        /* Declared in drone.c source file */
        extern tx_buttons_t * GlobalTxButtons;

        /* If any button was pressed */
        if( !strcmp( action, "press" ) ) {

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
        else if( !strcmp( action, "release" ) ) {

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
        const char * filepath = "/spiffs/web_transmitter.html";   /* '/base_path/filename.extension' */

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
     * @brief Manage Bluetooth events
     * @param event: Bluetooth event
     * @param param: details of data received
     * @retval none
     */
    static void spp_event_handler( esp_spp_cb_event_t event, esp_spp_cb_param_t * param ) {

        extern BluetoothData_t * GlobalBluetoothData;

        switch ( event ) {
            
            /* SPP server initialized */
            case ESP_SPP_INIT_EVT:
                if ( param->init.status == ESP_SPP_SUCCESS ) {

                    ESP_LOGI( TRANSMITTER_TAG, "ESP_SPP_INIT_EVT" );
                    esp_spp_start_srv( ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER" );
                }
                break;

            /* SPP server started ( MCU is able to connect via bluetooth ) */
            case ESP_SPP_START_EVT:
                if ( param->start.status == ESP_SPP_SUCCESS ) {

                    ESP_LOGI( TRANSMITTER_TAG, "ESP_SPP_START_EVT" );
                    esp_bt_gap_set_device_name( "esp32" );
                    /* esp_bt_dev_set_device_name( PS3_DEVICE_NAME ); ... Deprecated => Replaced by esp_bt_gap_set_device_name function */
                    esp_bt_gap_set_scan_mode( ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE );
                }
                break;

            /* New device connected to MCU */
            case ESP_SPP_SRV_OPEN_EVT:
                ESP_LOGI( TRANSMITTER_TAG, "New device connected, MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
                    param->srv_open.rem_bda[ 0 ], param->srv_open.rem_bda[ 1 ], param->srv_open.rem_bda[ 2 ],
                    param->srv_open.rem_bda[ 3 ], param->srv_open.rem_bda[ 4 ], param->srv_open.rem_bda[ 5 ]
                );
                break;

            /* Data received */
            case ESP_SPP_DATA_IND_EVT:
                if( GlobalBluetoothData != NULL ) {

                    /* Initialize data length in 0 */
                    GlobalBluetoothData->len = 0;

                    /* Data received, hence state = 1 */
                    GlobalBluetoothData->state = true;
                    for (int i = 0; i < param->data_ind.len; i++) {

                        /* Avoid \r ( ascii = 13 ) and \n ( ascii 10 ) chars */
                        if( ( param->data_ind.data[ i ] != 13 ) && ( param->data_ind.data[ i ] != 10 ) ) {
                            
                            /* Assign character to GlobalBluetoothData data */
                            GlobalBluetoothData->data[ i ] = param->data_ind.data[ i ];

                            /* Increment GlobalBluetoothData data length */
                            GlobalBluetoothData->len++;
                        }
                    }
                }

                else {

                    ESP_LOGE( "BLUETOOTH", "'GlobalBluetoothData' variable is NULL" );
                }
                
                break;

            default:
                break;
        }
    }

    /**
     * @brief Initialize Bluetooth interface
     * @param none
     * @retval none
     */
    static void bluetooth_init() {

        ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_BLE ) );

        esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        ESP_ERROR_CHECK( esp_bt_controller_init( &bt_config ) );

        ESP_ERROR_CHECK( esp_bt_controller_enable( ESP_BT_MODE_CLASSIC_BT ) );

        ESP_ERROR_CHECK( esp_bluedroid_init() );

        ESP_ERROR_CHECK( esp_bluedroid_enable() );

        ESP_ERROR_CHECK( esp_spp_register_callback( spp_event_handler ) );

        esp_spp_cfg_t bt_spp_cfg = {

            .mode = ESP_SPP_MODE_CB,
            .enable_l2cap_ertm = true,
            .tx_buffer_size = 0,
        };

        ESP_ERROR_CHECK( esp_spp_enhanced_init( &bt_spp_cfg ) );
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

        /* Initialize Bluetooth interface */
        bluetooth_init();

        ESP_LOGI( TRANSMITTER_TAG, "Transmitter object initialized" );

        return ESP_OK;
    }

    transmitter_t * Transmitter( drone_globals_t * global_variables ) {

        ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

        transmitter_t * Tx = ( transmitter_t * ) malloc( sizeof( transmitter_t ) );

        /* Assign Drone object global variables to Transmitter object global variables  */
        Tx->global_variables = global_variables;

        /* Pointer assignment to functions */
        Tx->init = transmitter_init;

        ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );

        return Tx;
    }

#endif
