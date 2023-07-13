#include "ble_module.h"


#include "wifi_module.h"
#include "wifi_credentials.h"

#include "sensor_module.h"

/* Standard includes. */
#include <string.h>

/* FreeRTOS includes. */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

/* ESP-IDF includes. */
#include <esp_err.h>
#include <esp_log.h>
#include <sdkconfig.h>

/* Network transport include. */
#include "network_transport.h"

/* ESP Secure Certificate Manager include. */
#include "esp_secure_cert_read.h"

/* Demo includes. */
#if CONFIG_GRI_ENABLE_SUB_PUB_UNSUB_DEMO
    #include "sub_pub_unsub_demo.h"
#endif /* CONFIG_GRI_ENABLE_SUB_PUB_UNSUB_DEMO */


/**
 * @brief The AWS RootCA1 passed in from ./certs/root_cert_auth.pem
 */
extern const char root_cert_auth_start[] asm ( "_binary_root_cert_auth_crt_start" );
extern const char root_cert_auth_end[]   asm ( "_binary_root_cert_auth_crt_end" );

/* Global variables ***********************************************************/


/**
 * @brief Logging tag for ESP-IDF logging functions.
 */
static const char * TAG = "main";



void ble_task(void *pvParameters); // Add this line
void wifi_task(void *pvParameters); // Add this line

/**
 * @brief The global network context used to store the credentials
 * and TLS connection.
 */
static NetworkContext_t xNetworkContext;

void sensor_task(void *pvParameters) {
    sht3x_t sht3x_dev;
    sgp40_t sgp40_dev;

    // Setup the sensors
    if (setupSensors(&sht3x_dev, &sgp40_dev) != ESP_OK) {
        ESP_LOGE("sensor_task", "Failed to setup sensors");
        vTaskDelete(NULL);
    }

    while (1) {
        // Read the sensors
        SensorData data = readSensors(&sht3x_dev, &sgp40_dev);

        // Do something with the sensor data...

        // Delay for 1 minute
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

/*
    ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE("app_main", "Failed to initialize I2C bus: %d", ret);
        return;
    }

    // Start the sensor task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    */
   /*
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) erasing NVS!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS erased successfully!");
    }
    */

    vTaskDelay(2000 / portTICK_PERIOD_MS);

   // Open the NVS partition in read-only mode
    nvs_handle_t my_handle;
    ret = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (ret != ESP_OK) {
        // If an error occurred while opening the NVS partition, log the error
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
    } else {
        WiFiCredentials_t credentials;

        // Read the saved SSID from NVS
        size_t ssid_size = sizeof(credentials.ssid);
        ret = nvs_get_str(my_handle, "ssid", credentials.ssid, &ssid_size);
        if (ret != ESP_OK) {
            // If an error occurred while reading the SSID, log the error and switch to BLE mode
            ESP_LOGE(TAG, "Failed to read ssid from NVS!");
            // Create a task for the BLE module
            xTaskCreate(ble_task, "ble_task", 4096, NULL, 5, NULL);
        } else {
            // Read the saved password from NVS
            size_t password_size = sizeof(credentials.password);
            ret = nvs_get_str(my_handle, "password", credentials.password, &password_size);
            if (ret != ESP_OK) {
                // If an error occurred while reading the password, log the error and switch to BLE mode
                ESP_LOGE(TAG, "Failed to read password from NVS!");
                // Create a task for the BLE module
                xTaskCreate(ble_task, "ble_task", 4096, NULL, 5, NULL);
            } else {
                // Create a task for the WiFi module
                ESP_LOGI(TAG, "Credentials readed from NVS!");
                xTaskCreatePinnedToCore(wifi_task, "wifi_task", 8192, &credentials, 5, NULL, 1);
            }
        }

        // Close the NVS partition
        nvs_close(my_handle);
    }
}


void ble_task(void *pvParameters) {

    // Initialize the BLE module
    ble_module_init();

    esp_err_t err = setupBLE();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup BLE : %s", esp_err_to_name(err));
    }

    // Main loop
    while (true) {
        // Delay for a short while
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Delete this task when the loop is done
    vTaskDelete(NULL);
}

void wifi_task(void *pvParameters) {
    // Get the WiFi credentials from the task parameters
    WiFiCredentials_t* credentials = (WiFiCredentials_t*) pvParameters;

    // Initialize the WiFi module
    wifi_module_init();    

    // Initialize the WiFi module with the credentials
    setWiFiCredentials(credentials);

    esp_err_t err = setupWiFi(credentials);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup WiFi : %s", esp_err_to_name(err));
    }

    // Main loop
    while (true) {
        // Handle WiFi events
        handleWiFi();

        // Delay for a short while
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Clean up the WiFi module
    wifi_module_deinit();

    // Delete this task when the loop is done
    vTaskDelete(NULL);
}



static BaseType_t prvInitializeNetworkContext( void )
{
    /* This is returned by this function. */
    BaseType_t xRet = pdPASS;

    /* This is used to store the error return of ESP-IDF functions. */
    esp_err_t xEspErrRet;

    /* Verify that the MQTT endpoint and thing name have been configured by the
     * user. */
    if( strlen( CONFIG_GRI_MQTT_ENDPOINT ) == 0 )
    {
        ESP_LOGE( TAG, "Empty endpoint for MQTT broker. Set endpoint by "
                       "running idf.py menuconfig, then Golden Reference Integration -> "
                       "Endpoint for MQTT Broker to use." );
        xRet = pdFAIL;
    }

    if( strlen( CONFIG_GRI_THING_NAME ) == 0 )
    {
        ESP_LOGE( TAG, "Empty thingname for MQTT broker. Set thing name by "
                       "running idf.py menuconfig, then Golden Reference Integration -> "
                       "Thing name." );
        xRet = pdFAIL;
    }

    /* Initialize network context. */

    xNetworkContext.pcHostname = CONFIG_GRI_MQTT_ENDPOINT;
    xNetworkContext.xPort = CONFIG_GRI_MQTT_PORT;

    /* Get the device certificate from esp_secure_crt_mgr and put into network
     * context. */
    xEspErrRet = esp_secure_cert_get_device_cert( &xNetworkContext.pcClientCert,
                                                  &xNetworkContext.pcClientCertSize );

    if( xEspErrRet == ESP_OK )
    {
        #if CONFIG_GRI_OUTPUT_CERTS_KEYS
            ESP_LOGI( TAG, "\nDevice Cert: \nLength: %"PRIu32"\n%s",
                      xNetworkContext.pcClientCertSize,
                      xNetworkContext.pcClientCert);
        #endif /* CONFIG_GRI_OUTPUT_CERTS_KEYS */
    }
    else
    {
        ESP_LOGE( TAG, "Error in getting device certificate. Error: %s",
                  esp_err_to_name( xEspErrRet ) );

        xRet = pdFAIL;
    }

    /* Putting the Root CA certificate into the network context. */
    xNetworkContext.pcServerRootCA = root_cert_auth_start;
    xNetworkContext.pcServerRootCASize = root_cert_auth_end - root_cert_auth_start;

    if( xEspErrRet == ESP_OK )
    {
        #if CONFIG_GRI_OUTPUT_CERTS_KEYS
            ESP_LOGI( TAG, "\nCA Cert: \nLength: %"PRIu32"\n%s",
                      xNetworkContext.pcServerRootCASize,
                      xNetworkContext.pcServerRootCA );
        #endif /* CONFIG_GRI_OUTPUT_CERTS_KEYS */
    }
    else
    {
        ESP_LOGE( TAG, "Error in getting CA certificate. Error: %s",
                  esp_err_to_name( xEspErrRet ) );

        xRet = pdFAIL;
    }

    #if CONFIG_ESP_SECURE_CERT_DS_PERIPHERAL
        /* If the digital signature peripheral is being used, get the digital
         * signature peripheral context from esp_secure_crt_mgr and put into
         * network context. */

        xNetworkContext.ds_data = esp_secure_cert_get_ds_ctx();

        if( xNetworkContext.ds_data == NULL )
        {
            ESP_LOGE( TAG, "Error in getting digital signature peripheral data." );
            xRet = pdFAIL;
        }
    #else
        xEspErrRet = esp_secure_cert_get_priv_key( &xNetworkContext.pcClientKey,
                                                   &xNetworkContext.pcClientKeySize);

        if( xEspErrRet == ESP_OK )
        {
            #if CONFIG_GRI_OUTPUT_CERTS_KEYS
                ESP_LOGI( TAG, "\nPrivate Key: \nLength: %"PRIu32"\n%s",
                          xNetworkContext.pcClientKeySize,
                          xNetworkContext.pcClientKey );
            #endif /* CONFIG_GRI_OUTPUT_CERTS_KEYS */
        }
        else
        {
            ESP_LOGE( TAG, "Error in getting private key. Error: %s",
                      esp_err_to_name( xEspErrRet ) );

            xRet = pdFAIL;
        }
    #endif /* CONFIG_ESP_SECURE_CERT_DS_PERIPHERAL */

    xNetworkContext.pxTls = NULL;
    xNetworkContext.xTlsContextSemaphore = xSemaphoreCreateMutex();

    if( xNetworkContext.xTlsContextSemaphore == NULL )
    {
        ESP_LOGE( TAG, "Not enough memory to create TLS semaphore for global "
                       "network context." );

        xRet = pdFAIL;
    }

    return xRet;
}