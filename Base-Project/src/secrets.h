//school
// #define WIFI_SSID       "IoT"
// #define WIFI_PASSWORD   "KdGIoT13!"
//thuis
#define WIFI_SSID               "Tuganet-2"
#define WIFI_PASSWORD           "Vitor12135434"
#define MQTT_BROKER             "192.168.0.106"
#define MQTT_PORT               1883
#define MQTT_CLIENT_ID          "esp32_base_project"
#define MQTT_USERNAME           "meowqtt"
#define MQTT_PASSWORD           "Ir1neu.-."

#define PUBLISH_TOPIC           "/sensor"
#define SUBSCRIBE_TOPIC         "/sensor/cmd"

#define PUBLISH_INTERVAL_DHT_MS  5000UL
#define PUBLISH_INTERVAL_US_MS   500UL

#define BLE_DEVICE_NAME         "ESP32-Sensor"
#define BLE_SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHAR_UUID           "abcd1234-5678-90ab-cdef-0123456789ab"