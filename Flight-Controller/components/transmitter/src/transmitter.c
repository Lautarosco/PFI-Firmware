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

    void Transmitter( transmitter_t * Tx, drone_globals_t * global_variables ) {

        ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

        /* Initialize Transmitter mac address in 0 */
        for (int i = 0; i < MAC_ADDR_SIZE; i++) { Tx->mac_addr[ i ] = 0; }

        /* Assign Drone object global variables to Transmitter object global variables  */
        Tx->global_variables = global_variables;

        /* Pointer assignment to functions */
        Tx->init = transmitter_init;

        ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );

    }

#elif WEBSV_TX

    #include <string.h>
    #include <esp_http_server.h>
    #include <cJSON.h>
    #include <ctype.h>

    /* TESTING */

        #include <esp_spp_api.h>
        #include <esp_bt.h>
        #include <esp_bt_main.h>
        #include <esp_gap_bt_api.h>

    /* TESTING */

    #define WIFI_SSID "ESP32_Transmitter"
    #define WIFI_PASS "12345678"

    static esp_err_t update_vars_handler(httpd_req_t *req) {
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

        cJSON * json = cJSON_Parse( buff );

        /* Check if received JSON is valid */
        if( !json ) {

            ESP_LOGE( TRANSMITTER_TAG, "Invalid JSON received" );
            return ESP_ERR_INVALID_RESPONSE;
        }

        /* Store button pressed */
        const char * action = cJSON_GetObjectItem( json, "action" )->valuestring;

        /* Store occurred action */
        const char * html_select = cJSON_GetObjectItem( json, "selected" )->valuestring;

        const char * text = cJSON_GetObjectItem( json, "text" )->valuestring;

        // html entered text is empty
        if (!strcmp(text, "")) {
            printf("Empty input\n");
        } else {
            // Check if html entered text is a valid number
            bool is_digit = true;
            for (int i = 0; i < strlen(text); i++)
            {
                if (!isdigit((unsigned char) text[i])) {
                    is_digit = false;
                    break;
                }
            }

            if (is_digit) {
                float new_value = atof(text);
                ESP_LOGE( "DEBUG", "Action: %s | Opción: %s | Número: %.2f", action, html_select, new_value);
                /* ========================= START Process incomming data ========================= */

                /* Forward declaration to avoid header inclusion */
                typedef struct pid_gain pid_gain_t;

                /* Declared in drone.c source file */
                extern pid_gain_t * GlobalRollGains;
                extern pid_gain_t * GlobalRoll_dGains;
                extern pid_gain_t * GlobalPitchGains;
                extern pid_gain_t * GlobalPitch_dGains;
                extern pid_gain_t * GlobalYawGains;
                extern pid_gain_t * GlobalYaw_dGains;

                /* Declared in controllers.h and defined in controllers.c */
                extern bool set_pid_gain(pid_gain_t *controller_gains, const char label[], float new_value);

                typedef struct gains_label {
                    const char *html_option;
                    const char *name;
                    pid_gain_t *state;
                } gains_label_t;

                gains_label_t arr[] = {
                    // Roll
                    {.html_option = "roll_kp",    .name = "kp", .state = GlobalRollGains},
                    {.html_option = "roll_ki",    .name = "ki", .state = GlobalRollGains},
                    {.html_option = "roll_kd",    .name = "kd", .state = GlobalRollGains},

                    // Roll_d
                    {.html_option = "roll_d_kp",  .name = "kp", .state = GlobalRoll_dGains},
                    {.html_option = "roll_d_ki",  .name = "ki", .state = GlobalRoll_dGains},
                    {.html_option = "roll_d_kd",  .name = "kd", .state = GlobalRoll_dGains},

                    // Pitch
                    {.html_option = "pitch_kp",   .name = "kp", .state = GlobalPitchGains},
                    {.html_option = "pitch_ki",   .name = "ki", .state = GlobalPitchGains},
                    {.html_option = "pitch_kd",   .name = "kd", .state = GlobalPitchGains},

                    // Pitch_d
                    {.html_option = "pitch_d_kp", .name = "kp", .state = GlobalPitch_dGains},
                    {.html_option = "pitch_d_ki", .name = "ki", .state = GlobalPitch_dGains},
                    {.html_option = "pitch_d_kd", .name = "kd", .state = GlobalPitch_dGains},

                    // Yaw
                    {.html_option = "yaw_kp",     .name = "kp", .state = GlobalYawGains},
                    {.html_option = "yaw_ki",     .name = "ki", .state = GlobalYawGains},
                    {.html_option = "yaw_kd",     .name = "kd", .state = GlobalYawGains},

                    // Yaw_d
                    {.html_option = "yaw_d_kp",   .name = "kp", .state = GlobalYaw_dGains},
                    {.html_option = "yaw_d_ki",   .name = "ki", .state = GlobalYaw_dGains},
                    {.html_option = "yaw_d_kd",   .name = "kd", .state = GlobalYaw_dGains},
                };

                for (int i = 0; i < ((sizeof(arr)) / (sizeof(arr[0]))); i++)
                {
                    if (!strcmp(html_select, arr[i].html_option)) {
                        if (!set_pid_gain(arr[i].state, arr[i].name, new_value)) {
                            ESP_LOGE(TRANSMITTER_TAG, "Drone's state <%s> NOT FOUND. See function %s in line %d.", html_select, __func__, __LINE__);
                        }
                    }
                }

                /* ========================= END Process incomming data ========================= */
            } else {
                printf("Wrong input\n");
            }
        }

        httpd_resp_set_type( req, "application/json" );
        httpd_resp_sendstr( req, "{\"status\":\"ok\"}" );

        return ESP_OK;
    }

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

        /* ========================= START Process incomming data ========================= */

        /* Declared in drone.c source file */
        extern tx_buttons_t * GlobalTxButtons;

        typedef struct tx_btns {
            char * btn_name;
            bool * tx_btn_ptr;
        } tx_btns_t;

        // Pair each button with respective drone's attribute
        tx_btns_t tx_btns_arr[] = {
            {.btn_name = "cross",    .tx_btn_ptr = &(GlobalTxButtons->cross)},
            {.btn_name = "triangle", .tx_btn_ptr = &(GlobalTxButtons->triangle)},
            {.btn_name = "square",   .tx_btn_ptr = &(GlobalTxButtons->square)},
            {.btn_name = "circle",   .tx_btn_ptr = &(GlobalTxButtons->circle)},
            {.btn_name = "up",       .tx_btn_ptr = &(GlobalTxButtons->up)},
            {.btn_name = "down",     .tx_btn_ptr = &(GlobalTxButtons->down)},
            {.btn_name = "left",     .tx_btn_ptr = &(GlobalTxButtons->left)},
            {.btn_name = "right",    .tx_btn_ptr = &(GlobalTxButtons->right)},
            {.btn_name = "l1",       .tx_btn_ptr = &(GlobalTxButtons->l1)},
            {.btn_name = "l2",       .tx_btn_ptr = &(GlobalTxButtons->l2)},
            {.btn_name = "r1",       .tx_btn_ptr = &(GlobalTxButtons->r1)},
            {.btn_name = "r2",       .tx_btn_ptr = &(GlobalTxButtons->r2)},
            {.btn_name = "start",    .tx_btn_ptr = &(GlobalTxButtons->start)},
            {.btn_name = "reset",    .tx_btn_ptr = &(GlobalTxButtons->ps)}
        };

        // Check which button was pressed/released
        bool found = false;
        if( !strcmp( action, "press" ) ) {
            // Match received button with buttons array
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = true;
                    printf("<%s> button was pressed\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else if (!strcmp(action, "release")) {
            // Match received button with buttons array
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = false;
                    printf("<%s> button was released\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else {
            // Actions can only be press or release
            ESP_LOGE( TRANSMITTER_TAG, "Action must be press/release. See function %s in line %d", __func__, __LINE__ );
        }

        if (!found) {
            printf("Buttons must be: <");
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++) {
                printf("%s/", tx_btns_arr[i].btn_name);
            }
            printf(">\n");
        }

        /* ========================= END Process incomming data ========================= */

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

            httpd_uri_t update_vars_uri = {
                .uri      = "/test_button",
                .method   = HTTP_POST,
                .handler  = update_vars_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler(server, &update_vars_uri);
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

        extern SerialData_t * GlobalSerialData;

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
                if( GlobalSerialData != NULL ) {

                    /* Initialize data length in 0 */
                    GlobalSerialData->len = 0;

                    /* Data received, hence state = 1 */
                    GlobalSerialData->state = true;
                    for (int i = 0; i < param->data_ind.len; i++) {

                        /* Avoid \r ( ascii = 13 ) and \n ( ascii 10 ) chars */
                        if( ( param->data_ind.data[ i ] != 13 ) && ( param->data_ind.data[ i ] != 10 ) ) {
                            
                            /* Assign character to GlobalSerialData data */
                            GlobalSerialData->data[ i ] = param->data_ind.data[ i ];

                            /* Increment GlobalSerialData data length */
                            GlobalSerialData->len++;
                        }
                    }
                }

                else {

                    ESP_LOGE( "BLUETOOTH", "'GlobalSerialData' variable is NULL" );
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
        ESP_LOGI( TRANSMITTER_TAG, "nvs_flash_init\n" );
        ESP_ERROR_CHECK( nvs_flash_init() );

        /* Initialize network interface */
        ESP_LOGI( TRANSMITTER_TAG, "esp_netif_init\n" );
        ESP_ERROR_CHECK( esp_netif_init() );

        /* Initialize event loop to catch HTTP related events */
        ESP_LOGI( TRANSMITTER_TAG, "esp_event_loop_create_default\n" );
        ESP_ERROR_CHECK( esp_event_loop_create_default() );

        /* Create default WiFi AP network interface */
        ESP_LOGI( TRANSMITTER_TAG, "esp_netif_create_default_wifi_ap\n" );
        esp_netif_create_default_wifi_ap();

        /* Set up the SoftAP */
        ESP_LOGI( TRANSMITTER_TAG, "wifi_init_softap\n" );

        ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
        ESP_LOGI("MEM", "Minimum free heap: %lu", esp_get_minimum_free_heap_size());


        wifi_init_softap();

        /* Start HTTP server */
        ESP_LOGI( TRANSMITTER_TAG, "start_webserver\n" );
        start_webserver();

        /* Initialize Bluetooth interface */
        ESP_LOGI( TRANSMITTER_TAG, "bluetooth_init\n" );
        bluetooth_init();

        ESP_LOGI( TRANSMITTER_TAG, "Transmitter object initialized" );

        return ESP_OK;
    }

    void Transmitter( transmitter_t* Tx, drone_globals_t * global_variables ) {

        ESP_LOGI( "TRANSMITTER", "Making an instance of Transmitter Class..." );

        /* Assign Drone object global variables to Transmitter object global variables  */
        Tx->global_variables = global_variables;

        /* Pointer assignment to functions */
        Tx->init = transmitter_init;

        ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );
    }

#endif
