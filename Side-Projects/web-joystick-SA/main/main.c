#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_spiffs.h>
#include <nvs_flash.h>
#include <cJSON.h>

#define TAG "JOYSTICK"

// Wi-Fi credentials
#define WIFI_SSID "ESP32-Joystick"
#define WIFI_PASS "12345678"

// HTTP handlers
static esp_err_t button_handler(httpd_req_t *req) {
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0'; // Null-terminate the received data
    ESP_LOGI(TAG, "Received JSON: %s", buf);

    // Parse the JSON
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON received");
        return ESP_FAIL;
    }

    const char *button = cJSON_GetObjectItem(json, "button")->valuestring;
    const char *action = cJSON_GetObjectItem(json, "action")->valuestring;

    ESP_LOGI(TAG, "Button: %s, Action: %s", button, action);

    cJSON_Delete(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

static esp_err_t file_get_handler(httpd_req_t *req) {
    const char *filepath = "/spiffs/index.html";
    FILE *file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t file_get_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = file_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &file_get_uri);

        httpd_uri_t button_uri = {
            .uri       = "/button",
            .method    = HTTP_POST,
            .handler   = button_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &button_uri);
    }
}

// Wi-Fi setup
static void wifi_init_softap(void)
{
    // 1. Initialize the Wi-Fi stack/config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 2. Configure the Wi-Fi in SoftAP mode
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,       // Must be >= 8 characters unless open
            .max_connection = 4,         // Limit maximum connections
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .beacon_interval = 200,
        }
    };

    if (strlen((const char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN; // Open Wi-Fi if no password
    }

    // 3. Set mode and apply config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // 4. Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP started: SSID: %s, Password: %s",
             wifi_config.ap.ssid, wifi_config.ap.password);
}

void app_main(void)
{
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "SPIFFS mounted successfully");

    // Initialize the network interface and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Wi-Fi AP netif
    esp_netif_create_default_wifi_ap();

    // Now set up the SoftAP
    wifi_init_softap();

    // Start the web server
    start_webserver();
}
