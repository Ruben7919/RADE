#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h" // replace "tcpip_adapter.h" with this
#include <string.h>
#include "ble_module.h"
#include "wifi_credentials.h" // Include the new header file


#define WIFI_CONNECT_RETRY_MAX 3

esp_err_t setupWiFi(WiFiCredentials_t* credentials);

void setWiFiCredentials(const WiFiCredentials_t* new_credentials);

void wifi_module_init(void);

void wifi_module_deinit(void);

void saveWiFiCredentials(const WiFiCredentials_t* credentials);

void handleWiFi();

bool checkWiFiConnection();

esp_err_t reconnectWiFi(void);

#endif