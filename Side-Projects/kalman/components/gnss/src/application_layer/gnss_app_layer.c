/**
 * @file gnss_app_layer.c
 * @brief NEO-xM Applcation Layer
 */


#include <application_layer/gnss_app_layer.h>       /* GNSS application layer */
#include <application_layer/neoxm/neo7m_app_layer.h>

#include <string.h>


/* ========== Private variables ========== */

static const char *gnss_app_layer_tag = "[GNSS_APP_LAYER]";


/* ========== Private functions ========== */

static esp_err_t gnss_neoxm_hal_init(gnss_t *gnss, gnss_params_t gnss_params) {
    if(gnss == NULL) {
        ESP_LOGE(gnss_app_layer_tag, "{Function %s in line %d}: GNSS instance is NULL. Initialize Gnss object --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    switch (gnss_params.neoxm_version) {
        case GNSS_NEO_6M:
            ESP_LOGE(gnss_app_layer_tag, "NEO-6M init function not implemented yet");
            break;

        case GNSS_NEO_7M:
            esp_err_t ret = gnss_neo7m_init(gnss, gnss_params);
            if(ret != ESP_OK) {
                return ESP_FAIL;
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}

static esp_err_t gnss_neoxm_hal_measure(gnss_t *gnss) {
    if(gnss == NULL) {
        ESP_LOGE(gnss_app_layer_tag, "{Function %s in line %d}: GNSS instance is NULL. Initialize Gnss object --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    switch (gnss->__neoxm_version) {
        case GNSS_NEO_6M:
            ESP_LOGE(gnss_app_layer_tag, "NEO-6M measure function not implemented yet");
            break;

        case GNSS_NEO_7M:
            esp_err_t ret = gnss_neo7m_measure(gnss);
            if(ret != ESP_OK) {
                return ESP_FAIL;
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}


/* ========== Public functions ========== */

esp_err_t Gnss(gnss_t *gnss) {
    if(gnss == NULL) {
        ESP_LOGE(gnss_app_layer_tag, "{Function %s in line %d}: GNSS instance is NULL. Initialize Gnss object --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(gnss, 0, sizeof(gnss_t));   /* Initialize all attributes of Gnss object to 0 */
    
    gnss->init    = gnss_neoxm_hal_init;
    gnss->measure = gnss_neoxm_hal_measure;

    ESP_LOGI(gnss_app_layer_tag, "Make an instance of Gnss Class --> OK");

    return ESP_OK;
}
