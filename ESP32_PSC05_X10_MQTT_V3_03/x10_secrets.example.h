#pragma once
// TEMPLATE: copy this file to x10_secrets.h (same folder) and fill it in.
// ---- Wi-Fi and MQTT SETTINGS (edit before uploading) ----
// Kept in its own tab so the main sketch can be shared without credentials.
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char *MQTT_HOST = "192.168.1.149";  // Broker LAN IP
static const uint16_t MQTT_PORT = 1883;
static const char *MQTT_USER = "";      // Leave blank if no authentication
static const char *MQTT_PASSWORD = "";
