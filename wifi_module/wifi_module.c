#include "wifi_module.h"
#include "freertos/semphr.h"



#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static WiFiCredentials_t wifi_credentials;
static SemaphoreHandle_t wifi_creds_mutex;

static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi_module";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    // Declare the retry count as a static variable
    static int retry_count = 0;

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi station interface started, trying to connect...");
            esp_err_t err = reconnectWiFi();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reconnect WiFi: %s", esp_err_to_name(err));
            }
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGI(TAG, "Disconnected from WiFi network, reason: %d", event->reason);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);

            // If the retry count has reached the maximum, switch to BLE mode
            if (++retry_count >= WIFI_CONNECT_RETRY_MAX) {
                ESP_LOGE(TAG, "Failed to connect to WiFi after %d attempts, switching to BLE mode", retry_count);

                // Deinitialize the WiFi module
                wifi_module_deinit();

                // Switch to BLE mode
                err = setupBLE();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to setup BLE : %s", esp_err_to_name(err));
                }
            } else {
                // If the retry count hasn't reached the maximum, log the error and retry the WiFi setup after a delay
                ESP_LOGE(TAG, "Failed to connect to WiFi, retrying in 1 second...");
                vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 1 second
                err = reconnectWiFi();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to reconnect WiFi: %s", esp_err_to_name(err));
                }
            }
            break;

        case IP_EVENT_STA_GOT_IP:
            ip_event_got_ip_t* event_got_ip = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event_got_ip->ip_info.ip));
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

            // Save the WiFi credentials here
            saveWiFiCredentials(&wifi_credentials);

            // Disable Bluetooth when WiFi is connected
            err = disableBLE();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to disable BLE: %s", esp_err_to_name(err));
            }

            // Reset the retry count
            retry_count = 0;
            break;

        default:
            ESP_LOGI(TAG, "Unhandled event base: %s, event id: %lu", event_base, event_id);
            break;
    }
}

esp_err_t setupWiFi(WiFiCredentials_t* credentials) {
    // Check if the credentials are valid
    // If the credentials pointer is NULL or either the ssid or password is NULL, log an error and return
    if (credentials == NULL || credentials->ssid == NULL || credentials->password == NULL) {
        ESP_LOGE(TAG, "Invalid WiFi credentials");
        return ESP_ERR_INVALID_ARG;
    }

    // Set the WiFi credentials
    // Create a wifi_config_t struct and copy the ssid and password from the credentials
    wifi_config_t wifi_config = {};
    // Take the wifi_creds_mutex to ensure that no other task is accessing the wifi_credentials
    xSemaphoreTake(wifi_creds_mutex, portMAX_DELAY);
    // Copy the ssid and password into the wifi_config struct
    // strncpy is used to ensure that the strings are null-terminated
    strncpy((char *)wifi_config.sta.ssid, credentials->ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, credentials->password, sizeof(wifi_config.sta.password) - 1);
    // Give the wifi_creds_mutex to allow other tasks to access the wifi_credentials
    xSemaphoreGive(wifi_creds_mutex);

    // Set WiFi mode to STA before setting the configuration
    // If the function call fails, log an error and return
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(err));
        return err;
    }

    // Set the WiFi configuration
    // If the function call fails, log an error and return
    err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(err));
        return err;
    }

    // Start the WiFi connection
    // If the function call fails, log an error and return
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
        return err;
    }

    // If all the function calls succeeded, log a success message
    ESP_LOGI(TAG, "WiFi setup completed successfully");
    return ESP_OK;
}


void handleWiFi() {
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", wifi_credentials.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", wifi_credentials.ssid);
        // Switch to BLE mode when WiFi connection fails
        esp_err_t err = setupBLE();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to setup BLE : %s", esp_err_to_name(err));
            }

    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

bool checkWiFiConnection() {
    return xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT;
}

void setWiFiCredentials(const WiFiCredentials_t* new_credentials) {
    // Check if the new credentials are valid
    if (new_credentials->ssid == NULL || new_credentials->password == NULL) {
        ESP_LOGE(TAG, "Invalid WiFi credentials");
        return;
    }

    // Set the WiFi credentials
    xSemaphoreTake(wifi_creds_mutex, portMAX_DELAY);
    memcpy(wifi_credentials.ssid, new_credentials->ssid, sizeof(wifi_credentials.ssid));
    memcpy(wifi_credentials.password, new_credentials->password, sizeof(wifi_credentials.password));
    xSemaphoreGive(wifi_creds_mutex);

    // Check if WiFi is connected before trying to disconnect
    if (checkWiFiConnection()) {
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    }

}


void wifi_module_init() {


    // Create a mutex for WiFi credentials
    wifi_creds_mutex = xSemaphoreCreateMutex();
    if (wifi_creds_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex for WiFi credentials");
        return;
    }

    // Create an event group for WiFi events
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group for WiFi");
        return;
    }

    // Print the size of the free heap before the call
    ESP_LOGI(TAG, "Free heap size before esp_netif_create_default_wifi_sta: %lu bytes", esp_get_free_heap_size());

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the system event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *wifi_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (wifi_sta == NULL) {
        // Create the default WiFi station network interface
        wifi_sta = esp_netif_create_default_wifi_sta();
        if (wifi_sta == NULL) {
            ESP_LOGE(TAG, "Failed to create default WiFi station network interface");
            return;
        }
    }

        // Print the size of the free heap after the call
    ESP_LOGI(TAG, "Free heap size after esp_netif_create_default_wifi_sta: %lu bytes", esp_get_free_heap_size());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
}

void wifi_module_deinit() {

        // Disconnect the WiFi
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(err));
    }

    // Stop the WiFi
    err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(err));
    }

    // Deinitialize the WiFi
    err = esp_wifi_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize WiFi: %s", esp_err_to_name(err));
    }

    // Clean up the mutex and event group
    vSemaphoreDelete(wifi_creds_mutex);
    vEventGroupDelete(s_wifi_event_group);
}

void saveWiFiCredentials(const WiFiCredentials_t* credentials) {
    // Open the NVS partition in read-write mode
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        // If an error occurred while opening the NVS partition, log the error
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        // Save the SSID to NVS
        err = nvs_set_str(my_handle, "ssid", credentials->ssid);
        if (err != ESP_OK) {
            // If an error occurred while saving the SSID, log the error
            ESP_LOGE(TAG, "Failed to save ssid to NVS!");
        }

        // Save the password to NVS
        err = nvs_set_str(my_handle, "password", credentials->password);
        if (err != ESP_OK) {
            // If an error occurred while saving the password, log the error
            ESP_LOGE(TAG, "Failed to save password to NVS!");
        }

        // Commit the changes to NVS
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            // If an error occurred while committing the changes, log the error
            ESP_LOGE(TAG, "Failed to commit changes to NVS!");
        }

        // Close the NVS partition
        nvs_close(my_handle);
    }
}


esp_err_t reconnectWiFi() {
    // Add a delay before retrying the WiFi connection
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Check if the WiFi is already connecting before calling esp_wifi_connect()
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WiFi mode: %s", esp_err_to_name(err));
        return err;
    }

    if (mode != WIFI_MODE_STA) {
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}