#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
#define WIFI_SSID "YOUR-SSID"
#define WIFI_PASSWORD "YOUR-PASSWORD"

// -- User Location --
#define USER_LAT 54.0 
#define USER_LON -1.0

// -- Radar & Server Settings --
#define DUMP1090_SERVER "192.168.50.100" // IP or hostname of dump1090 server
#define DUMP1090_PORT 8080               // The default web port for dump1090 is often 8080

// I2S pins for MAX98357A audio output
#define I2S_BCLK_PIN  17
#define I2S_LRCLK_PIN 16
#define I2S_DOUT_PIN  27

#endif // CONFIG_H
