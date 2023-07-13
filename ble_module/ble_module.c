#include "ble_module.h"
#include "esp_log.h"
#include "esp_blufi_api.h"

#include "wifi_credentials.h"


static WiFiCredentials_t wifi_credentials;

static const char *TAG = "BLE_MODULE";



static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param) {
    if (!param) {
        ESP_LOGE(TAG, "Parameter is NULL");
        return;
    }
    switch (event) {
        case ESP_BLUFI_EVENT_INIT_FINISH:
            if (param->init_finish.state != ESP_BLUFI_INIT_OK) {
                ESP_LOGE(TAG, "BLUFI init failed");
                return;
            }
            ESP_LOGI(TAG, "BLUFI init finish");
            esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
            // Remove the esp_blufi_profile_init() call from here
            break;
        case ESP_BLUFI_EVENT_DEINIT_FINISH:
            if (param->deinit_finish.state != ESP_BLUFI_DEINIT_OK) {
                ESP_LOGE(TAG, "BLUFI deinit failed");
                return;
            }
            ESP_LOGI(TAG, "BLUFI deinit finish");
            break;
        case ESP_BLUFI_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "BLUFI ble connect");
            break;
        case ESP_BLUFI_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "BLUFI ble disconnect");
            if (checkWiFiConnection()) {
                ESP_LOGI(TAG, "Wi-Fi connected. Deinitializing BLUFI profile.");
                esp_blufi_profile_deinit();
            } else {
                ESP_LOGI(TAG, "No Wi-Fi connection. Continuing BLE advertising.");
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;  
        case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
            ESP_LOGI(TAG, "BLUFI Set WIFI opmode %d", param->wifi_mode.op_mode);
            break;
        case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
            ESP_LOGI(TAG, "BLUFI request wifi connect to AP");
            break;
        case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
            ESP_LOGI(TAG, "BLUFI request wifi disconnect from AP");
            break;
        case ESP_BLUFI_EVENT_REPORT_ERROR:
            ESP_LOGE(TAG, "BLUFI report error, error code %d", param->report_error.state);
            break;
        case ESP_BLUFI_EVENT_RECV_STA_SSID:
            ESP_LOGI(TAG, "Received SSID: %s, length: %d", param->sta_ssid.ssid, param->sta_ssid.ssid_len);
            bzero(&wifi_credentials, sizeof(WiFiCredentials_t));
            memcpy(wifi_credentials.ssid, param->sta_ssid.ssid, param->sta_ssid.ssid_len);
            wifi_credentials.ssid[param->sta_ssid.ssid_len] = '\0'; 
            break;
        case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
            ESP_LOGI(TAG, "Received password: %s, length: %d", param->sta_passwd.passwd, param->sta_passwd.passwd_len);
           /* for (int i = 0; i < param->sta_passwd.passwd_len; i++) {
                ESP_LOGI(TAG, "%02x ", param->sta_passwd.passwd[i]);
            }
            */
            memcpy(wifi_credentials.password, param->sta_passwd.passwd, param->sta_passwd.passwd_len);
            wifi_credentials.password[param->sta_passwd.passwd_len] = '\0';  // Add null character at the end
            wifi_module_init();
            setWiFiCredentials(&wifi_credentials);

            esp_err_t err = setupWiFi(&wifi_credentials);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to setup WiFi : %s", esp_err_to_name(err));
            }
            break;
        default:
            ESP_LOGI(TAG, "BLUFI cb unknown event %d", event);
            break;
    }
}

void ble_module_init() {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; //set the IO capability to No output No input
    uint8_t key_size = 16; //the key size should be set to max 16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_blufi_callbacks_t example_callbacks = {
        .event_cb = example_event_callback,
    };

    if ((ret = esp_blufi_register_callbacks(&example_callbacks)) != ESP_OK) {
        ESP_LOGE(TAG, "%s register callbacks failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
}

esp_err_t setupBLE() {
    esp_err_t ret;
    // Initialize the BLUFI profile
    // The BLUFI profile is a custom GATT-based profile provided by Espressif.
    // It is used for configuring the Wi-Fi settings over BLE.
    if ((ret = esp_blufi_profile_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize blufi profile failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    // Start advertising
    // This makes the ESP32 discoverable to other BLE devices.
    esp_ble_gap_start_advertising(&adv_params);
    return ESP_OK;
}


esp_err_t disableBLE() {
    esp_err_t ret;

    // Deinitialize the BLUFI profile
    ret = esp_blufi_profile_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s deinitialize blufi profile failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    // Disable the bluedroid stack
    ret = esp_bluedroid_disable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s disable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    // Deinitialize the bluedroid stack
    ret = esp_bluedroid_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s deinitialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    // Disable the BT controller
    ret = esp_bt_controller_disable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s disable BT controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BLE module disabled successfully");
    return ESP_OK;
}