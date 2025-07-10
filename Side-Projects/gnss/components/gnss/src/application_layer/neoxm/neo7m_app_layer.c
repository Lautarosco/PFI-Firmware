/**
 * @file neo7m_app_layer.c
 * @brief NEO-7M Applcation Layer
 */


#include <application_layer/neoxm/neo7m_app_layer.h>


/* ========== Private variables ========== */

const char *neo7m_app_layer_tag = "[NEO-7M_APP_LAYER]";


/* ========== Public functions ========== */

esp_err_t gnss_neo7m_init(gnss_t *gnss, gnss_params_t gnss_params) {
    if(gnss == NULL) {
        ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: GNSS instance is NULL. Initialize Gnss object --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    if((gnss_params.uart_port < UART_NUM_0) || (gnss_params.uart_port > UART_NUM_2)) {
        ESP_LOGE(neo7m_app_layer_tag, "Invalid UART port: %d", gnss_params.uart_port);
        return ESP_ERR_INVALID_ARG;
    }
    
    gnss->uart_port = gnss_params.uart_port;                /* Set default UART port number */
    gnss->__neoxm_version = gnss_params.neoxm_version;      /* Set device version */

    ESP_LOGI(neo7m_app_layer_tag, "UART port: %d", gnss->uart_port);
    ESP_LOGI(neo7m_app_layer_tag, "Device version: NEO-7M");

    esp_err_t ret;

    gnss_hwl_set_uart_msg_mode(gnss->uart_port, UBX_CLASS_NAV, UBX_NAV_PVT_ID, UBX_MSG_POLLING);
    gnss_hwl_set_uart_msg_mode(gnss->uart_port, UBX_CLASS_NAV, UBX_NAV_DOP_ID, UBX_MSG_POLLING);

    switch(gnss_params.gnss_protocol) {
        case GNSS_ENABLE_UBX_ONLY:
            ret = gnss_hwl_disable_nmea(gnss->uart_port);       /* Disable NMEA messages */
            if(ret != ESP_OK) {
                return ESP_FAIL;
            }
            ESP_LOGI(neo7m_app_layer_tag, "Protocol: UBX");
            break;
        case GNSS_ENABLE_NMEA_ONLY:
            /* To be implemented */
            break;
        case GNSS_ENABLE_ALL:
            /* Both NMEA and UBX work simmultaneously by default */
            break;
        default:
            ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Protocol not found", __func__, __LINE__);
            return ESP_ERR_INVALID_ARG;
            break;
    }

    ret = gnss_hwl_set_meas_rate(gnss->uart_port, gnss_params.measRate);        /* Set receiver GPS measurement rate */
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ubx_cfg_rate_t cfg_rate;
    ret = gnss_hwl_read_cfg_rate(gnss->uart_port, &cfg_rate);       /* Poll receiver GPS measurement rate and update gnss <measRate> attribute */
    if(ret != ESP_OK) {
        ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Read UBX-CFG-RATE --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    gnss->data.time.measRate = cfg_rate.measRate;

    // static ubx_nav_svinfo_t ubx_nav_svinfo;
    // ret = gnss_hwl_read_nav_svinfo(gnss->uart_port, &ubx_nav_svinfo);
    // // ret = gnss_neo7m_update_SVs_data(gnss);       /* Update Satellites vehicles information */
    // if(ret != ESP_OK) {
    //     ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Update SVs info --> FAILED", __func__, __LINE__);
    //     return ESP_FAIL;
    // }

    ret = gnss_hwl_set_dynModel(gnss->uart_port, gnss_params.dynModel);     /* Set dynamic platform model */
    if(ret != ESP_OK) {
        ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Set dynamic portable model --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ret = gnss_hwl_set_static_hold_threshold(gnss->uart_port, gnss_params.static_hold_threshold);       /* Set static hold threshold */
    if(ret != ESP_OK) {
        ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Set static hold threshold --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ubx_cfg_nav5_t ubx_cfg_nav5;
    ret = gnss_hwl_read_cfg_nav5(gnss->uart_port, &ubx_cfg_nav5);
    if(ret != ESP_OK) {
        ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Read UBX-CFG-NAV5 --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    // ubx_nav_dop_t ubx_nav_dop;
    // ret = gnss_hwl_read_nav_dop(gnss->uart_port, &ubx_nav_dop);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Read UBX-NAV-DOP --> FAILED", __func__, __LINE__);
    //     return ESP_FAIL;
    // }
    // gnss->data.position.pDOP = ubx_nav_dop.pDOP / 100.0;        /* Update position DOP */
    // gnss->data.position.vDOP = ubx_nav_dop.vDOP / 100.0;        /* Update vertical DOP */
    // ESP_LOGI(neo7m_app_layer_tag, "{Function %s in line %d}: Initial pDOP: %lf", __func__, __LINE__, gnss->data.position.pDOP);
    // ESP_LOGI(neo7m_app_layer_tag, "{Function %s in line %d}: Initial vDOP: %lf", __func__, __LINE__, gnss->data.position.vDOP);

    gnss_hwl_set_uart_msg_mode(gnss->uart_port, UBX_CLASS_NAV, UBX_NAV_PVT_ID, UBX_MSG_PERIODIC);
    gnss_hwl_set_uart_msg_mode(gnss->uart_port, UBX_CLASS_NAV, UBX_NAV_DOP_ID, UBX_MSG_PERIODIC);

    return ESP_OK;
}

esp_err_t gnss_neo7m_measure(gnss_t *gnss) {
    esp_err_t ret = ESP_OK;

    uint8_t tmp[512];
    int len = uart_read_bytes(gnss->uart_port, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
    if(len <= 0) {
        ESP_LOGW(neo7m_app_layer_tag, "{Function %s in line %d}: UART Rx buffer does not have enough bytes to read from", __func__, __LINE__);
        return ESP_ERR_INVALID_SIZE;
    }

    gnss_payload_t ubx_nav_pvt_pyld[UBX_NAV_PVT_LEN_84];
    gnss_payload_len_t pyld_len = 0;
    ret = gnss_hwl_get_msg_payload(tmp, len, UBX_CLASS_NAV, UBX_NAV_PVT_ID, ubx_nav_pvt_pyld, sizeof(ubx_nav_pvt_pyld), &pyld_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ubx_nav_pvt_t nav_pvt;
    gnss_parse_nav_pvt(ubx_nav_pvt_pyld, pyld_len, &nav_pvt);

    if(!gnss_nav_pvt_fixType(nav_pvt.fixType)) {
        // ESP_LOGW(neo7m_app_layer_tag, "{Function %s in line %d}: Achieve a fix --> FAILED", __func__, __LINE__);
        gnss->data.flags.valid_gnss_fix_type = false;
    } else {
        /* Update flag */
        gnss->data.flags.valid_gnss_fix_type = true;
    }

    if(!gnss_nav_pvt_fix_ok(nav_pvt.flags)) {
        // ESP_LOGW(neo7m_app_layer_tag, "{Function %s in line %d}: Invalid fix", __func__, __LINE__);
        gnss->data.flags.valid_gnss_fix = false;
    } else {
        /* Update flag */
        gnss->data.flags.valid_gnss_fix = true;
    }

    if(!gnss_nav_pvt_diffsoln(nav_pvt.flags)) {
        // ESP_LOGW(neo7m_app_layer_tag, "{Function %s in line %d}: No differential corrections were applied", __func__, __LINE__);
        gnss->data.flags.diffSoln_flag = false;
    } else {
        /* Update flag */
        gnss->data.flags.diffSoln_flag = true;
    }

    if(!gnss_nav_pvt_valid(nav_pvt.valid)) {
        ESP_LOGW(neo7m_app_layer_tag, "{Function %s in line %d}: Invalid UTC date", __func__, __LINE__);
        gnss->data.flags.valid_utc_date = false;
    } else {
        /* Update UTC timestamp */
        gnss->data.flags.valid_utc_date = true;
        snprintf(
            (char *) &(gnss->data.time.utc_timestamp),
            sizeof(gnss->data.time.utc_timestamp),
            "%02hhu-%02hhu-%04huT%02hhu:%02hhu:%02hhuZ",
            nav_pvt.day, nav_pvt.month, nav_pvt.year, nav_pvt.hour, nav_pvt.min, nav_pvt.sec
        );
    }

    /* Check total of satellite vehicles used by the GNSS module to calculate last navigation solution  */
    if(nav_pvt.numSV < GNSS_MIN_SVS) {
        // ESP_LOGW(
        //     neo7m_app_layer_tag,
        //     "{Function %s in line %d}: Receiver linked SVs (%d) is less than minimum value (%d) --> No navigation solution available", __func__, __LINE__, nav_pvt.numSV, GNSS_MIN_SVS
        // );
    } else {
        /* Update position data */
        gnss->data.position.hAcc   = nav_pvt.hAcc / 1000.0;
        gnss->data.position.height = nav_pvt.height / 1000.0;
        gnss->data.position.hMSL   = nav_pvt.hMSL / 1000.0;
        gnss->data.position.lat    = nav_pvt.lat / 1e7;
        gnss->data.position.lon    = nav_pvt.lon / 1e7;
        gnss->data.position.pDOP   = nav_pvt.pDOP / 100.0;
    }
    gnss->data.svs_data.numSV = nav_pvt.numSV;

    gnss_payload_t ubx_nav_dop_pyld[UBX_NAV_DOP_LEN_18];
    ret = gnss_hwl_get_msg_payload(tmp, len, UBX_CLASS_NAV, UBX_NAV_DOP_ID, ubx_nav_dop_pyld, sizeof(ubx_nav_dop_pyld), &pyld_len);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ubx_nav_dop_t nav_dop;
    gnss_parse_nav_dop(ubx_nav_dop_pyld, pyld_len, &nav_dop);

    gnss->data.position.vDOP = nav_dop.vDOP / 100.0;

    return ESP_OK;
}

esp_err_t gnss_neo7m_update_SVs_data(gnss_t *gnss) {
    static ubx_nav_svinfo_t ubx_nav_svinfo;
    
    esp_err_t ret = gnss_hwl_read_nav_svinfo(gnss->uart_port, &ubx_nav_svinfo);
    if(ret != ESP_OK) {
        // ESP_LOGE(neo7m_app_layer_tag, "{Function %s in line %d}: Read UBX-NAV-SVINFO --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    for(int i = 0; i < ubx_nav_svinfo.numCh; i++) {
        gnss->data.svs_data.SV[i].svid = ubx_nav_svinfo.SVs[i].svid;
        gnss->data.svs_data.SV[i].chn = ubx_nav_svinfo.SVs[i].chn;
        gnss->data.svs_data.SV[i].cno = ubx_nav_svinfo.SVs[i].cno;
        gnss->data.svs_data.SV[i].healthy = !gnss_nav_svinfo_unhealthy(ubx_nav_svinfo.SVs[i].flags);
        gnss->data.svs_data.SV[i].svUsed = gnss_nav_svinfo_svUsed(ubx_nav_svinfo.SVs[i].flags);
        gnss->data.svs_data.SV[i].quality = gnss_nav_svinfo_quality(ubx_nav_svinfo.SVs[i].quality);
        gnss->data.svs_data.SV[i].gnss_type = gnss_get_SV_gnss_type(ubx_nav_svinfo.SVs[i].svid);
        ESP_LOGI(
            neo7m_app_layer_tag,
            "Channel %d: SVID: %d, GNSS type: %s, Used for navigation: %d, Healthy: %d, CNO: %d, Quality: %d",
            gnss->data.svs_data.SV[i].chn, gnss->data.svs_data.SV[i].svid, gnss->data.svs_data.SV[i].gnss_type,
            gnss->data.svs_data.SV[i].svUsed, gnss->data.svs_data.SV[i].healthy, gnss->data.svs_data.SV[i].cno,
            gnss->data.svs_data.SV[i].quality
        );
    }

    return ESP_OK;
}

bool gnss_neo7m_vDOP_is_valid(double vDOP) {
    return vDOP < GNSS_MAX_VDOP;
}
