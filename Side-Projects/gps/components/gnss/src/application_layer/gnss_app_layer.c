#include <application_layer/gnss_app_layer.h>  /* GNSS application layer */

#include <string.h>

static const char *gnss_app_layer_tag = "[GNSS_APP_LAYER]";

static esp_err_t gnss_init(gnss_t *gnss, gnss_params_t gnss_params) {
    if(gnss == NULL) {
        ESP_LOGE(gnss_app_layer_tag, "{Function %s in line %d}: GNSS instance is NULL. Initialize Gnss object --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    gnss->uart_port = gnss_params.uart_port;        /* Set default UART port number */

    switch(gnss_params.gnss_protocol) {
        case GNSS_ENABLE_UBX_ONLY:
            esp_err_t ret = gnss_hwl_disable_nmea(gnss->uart_port);  /* Disable NMEA messages */
            if(ret != ESP_OK) {
                return ESP_FAIL;
            }
            break;
        case GNSS_ENABLE_NMEA_ONLY:
            /* To be implemented */
            break;
        case GNSS_ENABLE_ALL:
            /* Both NMEA and UBX work simmultaneously by default */
            break;
        default:
            ESP_LOGE(gnss_app_layer_tag, "{Function %s in line %d}: Protocol not found", __func__, __LINE__);
            return ESP_ERR_INVALID_ARG;
            break;
    }

    return ESP_OK;
}

void Gnss(gnss_t *gnss) {
    memset((void *) gnss, 0, sizeof(gnss_t));   /* Initialize all attributes of Gnss object to 0 */
    
    gnss->init = gnss_init;
}
