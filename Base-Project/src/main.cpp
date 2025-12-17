//		     █████████                  █████                    
//			 ███░░░░░███                ░░███                    
//			░███    ░███  ████████    ███████  ████████   ██████ 
//			░███████████ ░░███░░███  ███░░███ ░░███░░███ ███░░███
//			░███░░░░░███  ░███ ░███ ░███ ░███  ░███ ░░░ ░███████ 
//			░███    ░███  ░███ ░███ ░███ ░███  ░███     ░███░░░  
//			█████   █████ ████ █████░░████████ █████    ░░██████ 
//			░░░░░   ░░░░░ ░░░░ ░░░░░  ░░░░░░░░ ░░░░░      ░░░░░░ 

// Bronnen:
// chatgpt.com (02/10)
// copilot.github.com (02/10)
// https://whadda.com/product/ultrasonic-distance-sensor-wpse306n/ (02/10)
// https://wiki.dfrobot.com/DHT22_Temperature_and_humidity_module_SKU_SEN0137 (02/10)

#include <Arduino.h>
#include <DHT.h>

// // MQTT LIBRARIES
#include <WiFi.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>

// BLE LIBRARIES
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "secrets.h"

unsigned long lastPublishTime = 0;
unsigned long currentInterval;

// remember last sensor values for MQTT
float lastTemp = NAN;
float lastHum  = NAN;
long  lastDistance = -1;

// pins
#define LED_RED_PIN_FAR   D10
#define LED_RED_PIN_NEAR  D11
#define LED_RED_PIN_CLOSE D12

#define LED_GREEN_US_PIN   D5
#define LED_YELLOW_DHT_PIN D3

#define BUTTON_PIN D13

#define DHTTYPE    DHT22
#define DHT_22_PIN D6
DHT dht(DHT_22_PIN, DHTTYPE);

#define ULTRASONIC_TRIG_PIN D7
#define ULTRASONIC_ECHO_PIN D9

// delays
#define DHT_SAMPLE_MS 2000UL
unsigned long lastDhtMs = 0;  

#define BTN_DEBOUNCE_MS 50UL
unsigned long lastBtnEdgeMs = 0;
bool lastBtnRaw = HIGH;     
bool stableBtn  = HIGH;     
bool prevStable = HIGH;  

#define US_SAMPLE_MS 300UL
#define US_PULSE_TIMEOUT_US 25000UL
unsigned long lastUsMs = 0;

// thresholds in cm
#define FAR_THRESH_CM   100
#define NEAR_THRESH_CM  50
#define CLOSE_THRESH_CM 20

WiFiClient network;
MQTTClient mqtt = MQTTClient(256);

BLEServer* bleServer = nullptr;
BLECharacteristic* bleChar = nullptr;
bool bleConnected = false;

/// BASIC FUNCTIONALITY
enum Mode {
    MODE_DHT,
    MODE_ULTRASONIC
};

Mode mode = MODE_DHT;

void showModeLeds() {
    digitalWrite(LED_YELLOW_DHT_PIN,  (mode == MODE_DHT) ? HIGH : LOW);
    digitalWrite(LED_GREEN_US_PIN,    (mode == MODE_ULTRASONIC) ? HIGH : LOW);
}

void allRedOff() {
    digitalWrite(LED_RED_PIN_FAR, LOW);
    digitalWrite(LED_RED_PIN_NEAR, LOW);
    digitalWrite(LED_RED_PIN_CLOSE, LOW);
}

void setRedProgress(long cm) {
    allRedOff();
    if (cm < 0) return;

    if (cm <= FAR_THRESH_CM)   digitalWrite(LED_RED_PIN_FAR,   HIGH);
    if (cm <= NEAR_THRESH_CM)  digitalWrite(LED_RED_PIN_NEAR,  HIGH);
    if (cm <= CLOSE_THRESH_CM) digitalWrite(LED_RED_PIN_CLOSE, HIGH);
}

long readUltrasonicCM() {
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    unsigned long dur = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, US_PULSE_TIMEOUT_US);
    if (dur == 0) return -1; // timeout / out of range

    // cm: speed of sound ~343 m/s => ~29.1 µs per cm round-trip
    return (long)(dur / 29.1 / 2.0);
}

void readDhtSensor(unsigned long currentMillis) {
    if (currentMillis - lastDhtMs < DHT_SAMPLE_MS) return;
    lastDhtMs = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature(); // °C

    if (isnan(h) || isnan(t)) {
        Serial.println(F("DHT read failed"));
        return;
    }

    lastTemp = t;
    lastHum  = h;

    Serial.print(F("DHT22 -> T: "));
    Serial.print(t, 1);
    Serial.print(F(" °C, H: "));
    Serial.print(h, 1);
    Serial.println(F(" %"));

    allRedOff();
}

void readUltrasonicSensor(unsigned long currentMillis) {
    if (currentMillis - lastUsMs < US_SAMPLE_MS) return;
    lastUsMs = currentMillis;

    long cm = readUltrasonicCM();
    lastDistance = cm;

    if (cm >= 0) {
        Serial.print(F("Distance: "));
        Serial.print(cm);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("Distance: out of range / timeout"));
    }

    setRedProgress(cm);
}

void handleSensorReadings(unsigned long currentMillis) {
    if (mode == MODE_DHT) {
        readDhtSensor(currentMillis);
    } else if (mode == MODE_ULTRASONIC) {
        readUltrasonicSensor(currentMillis);
    }
}

void processButtonInput(unsigned long currentMillis) {
    bool raw = digitalRead(BUTTON_PIN);
    if (raw != lastBtnRaw) {
        lastBtnRaw = raw;
        lastBtnEdgeMs = currentMillis;
    }
    if ((currentMillis - lastBtnEdgeMs) > BTN_DEBOUNCE_MS) {
        if (stableBtn != raw) {
            stableBtn = raw;
            // Falling edge: HIGH -> LOW (button pressed)
            if (prevStable == HIGH && stableBtn == LOW) {
                mode = (mode == MODE_DHT) ? MODE_ULTRASONIC : MODE_DHT;
                showModeLeds();
                if (mode == MODE_DHT) allRedOff();
            }
            prevStable = stableBtn;
        }
    }
}
/// BASIC FUNCTIONALITY


/// BLE FUNCTIONALITY
class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("BLE: phone connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("BLE: phone disconnected");
    // keep advertising so phone can reconnect
    BLEDevice::startAdvertising();
  }
};

void bleInit() {
  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleChar = service->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleChar->addDescriptor(new BLE2902());
  bleChar->setValue("Starting...");

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("BLE: advertising started (ESP32-Sensor)");
}

void blePublish() {
    String ble;
    
    if (!bleConnected) return;
    
    if (mode == MODE_DHT) {
        ble = "DHT,t=" + String(lastTemp, 1) + ",h=" + String(lastHum, 1);
    } else {
        ble = "US,d=" + String(lastDistance);
    }

    // std::string notif = ble.c_str();

    // bleChar->setValue(notif);
    bleChar->setValue(ble.c_str());
    bleChar->notify();
    Serial.println("BLE: data notified to phone");
}
/// BLE FUNCTIONALITY

/// MQTT FUNCTIONALITY
// MQTT message handler: switch mode based on payload from Node-RED
void messageHandler(String &topic, String &payload) {
    Serial.println("ESP32 - received from MQTT:");
    Serial.print("- topic: ");
    Serial.println(topic);
    Serial.print("- payload: ");
    Serial.println(payload);

    if (topic == SUBSCRIBE_TOPIC) {
        // expect plain text commands: "DHT", "ULTRASONIC" or "TOGGLE"
        if (payload == "DHT") {
            mode = MODE_DHT;
        } else if (payload == "ULTRASONIC") {
            mode = MODE_ULTRASONIC;
        } else if (payload == "TOGGLE") {
            mode = (mode == MODE_DHT) ? MODE_ULTRASONIC : MODE_DHT;
        }

        showModeLeds();
        if (mode == MODE_DHT) allRedOff();
    }
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT broker at ");
    Serial.print(MQTT_BROKER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    mqtt.begin(MQTT_BROKER, MQTT_PORT, network);
    mqtt.onMessage(messageHandler);

    Serial.println("ESP32 - Connecting to MQTT broker");

    while (!mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.print("Connect failed. lastError=");
        Serial.print(mqtt.lastError());
        Serial.print(" returnCode=");
        Serial.println(mqtt.returnCode());
        delay(1000);  // slow down retries so you can read
    }

    Serial.println("ESP32 - MQTT broker Connected!");

    if (mqtt.subscribe(SUBSCRIBE_TOPIC)) {
        Serial.print("ESP32 - Subscribed to the topic: ");
    } else {
        Serial.print("ESP32 - Failed to subscribe to the topic: ");
    }
    Serial.println(SUBSCRIBE_TOPIC);
}

void sendToMQTT(const char* buffer, size_t n) {
    bool ok = mqtt.publish(PUBLISH_TOPIC, buffer, n);
    Serial.println("ESP32 - sent to MQTT:");
    Serial.print("- topic: ");
    Serial.println(PUBLISH_TOPIC);
    Serial.print("- payload: ");
    Serial.println(buffer);
    Serial.print("- status: ");
    Serial.println(ok ? "OK" : "FAILED");
}
/// MQTT FUNCTIONALITY

/// DATA SENDING FUNCTIONALITY
// send current sensor state to Node-RED as JSON
void sendData(){
    StaticJsonDocument<255> doc;  // ArduinoJson 7 style

    doc["timestamp"] = millis();
    doc["mode"]      = (mode == MODE_DHT) ? "DHT" : "ULTRASONIC";

    if (!isnan(lastTemp))     doc["temperature"] = lastTemp;
    if (!isnan(lastHum))      doc["humidity"]    = lastHum;
    if (lastDistance >= 0)    doc["distance"]    = lastDistance;

    char buffer[256];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));
    buffer[n] = '\0'; // null-terminate for safety
    
    sendToMQTT(buffer, n);
    blePublish();
}
/// DATA SENDING FUNCTIONALITY

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    connectToMQTT();

    bleInit();

    // LEDs
    pinMode(LED_RED_PIN_FAR, OUTPUT);
    pinMode(LED_RED_PIN_NEAR, OUTPUT);
    pinMode(LED_RED_PIN_CLOSE, OUTPUT);

    pinMode(LED_GREEN_US_PIN, OUTPUT);
    pinMode(LED_YELLOW_DHT_PIN, OUTPUT);

    // Button (active LOW)
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Ultrasonic
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    // DHT
    dht.begin();

   


    showModeLeds();
}

void loop() {
    unsigned long currentMillis = millis();

    // keep MQTT connection alive
    mqtt.loop();

    processButtonInput(currentMillis);
    handleSensorReadings(currentMillis);

    // send to MQTT every PUBLISH_INTERVAL
    if (mode == MODE_DHT) {
        currentInterval = DHT_SAMPLE_MS;
    } else {
        currentInterval = US_SAMPLE_MS;
    }
    if (currentMillis - lastPublishTime >= currentInterval) {
        sendData();
        lastPublishTime = currentMillis;
    }
}