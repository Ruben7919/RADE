#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

typedef struct {
    char ssid[32];
    char password[64];
} WiFiCredentials_t;

extern WiFiCredentials_t wifiCredentials;

#endif // WIFI_CREDENTIALS_H