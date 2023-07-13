#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

typedef struct {
    char ssid[33];
    char password[65];
} WiFiCredentials_t;

extern WiFiCredentials_t wifiCredentials;

#endif // WIFI_CREDENTIALS_H