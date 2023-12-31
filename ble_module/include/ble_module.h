#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "wifi_module.h" // Include the WiFi module


#define BLUFI_DEVICE_NAME "RAD"

esp_err_t setupBLE();
esp_err_t disableBLE();

void ble_module_init();

#endif