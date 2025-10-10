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

#define LED_RED_PIN_FAR D10
#define LED_RED_PIN_NEAR D11
#define LED_RED_PIN_CLOSE D12

#define LED_GREEN_US_PIN D5
#define LED_YELLOW_DHT_PIN D3

#define BUTTON_PIN D13

#define DHTTYPE DHT22
#define DHT_22_PIN D6
DHT dht(DHT_22_PIN, DHTTYPE);

#define ULTRASONIC_TRIG_PIN D7
#define ULTRASONIC_ECHO_PIN D9

//-- delays --
#define DHT_SAMPLE_MS         2000UL
unsigned long lastDhtMs = 0;  

#define BTN_DEBOUNCE_MS       50UL
unsigned long lastBtnEdgeMs = 0;
bool lastBtnRaw = HIGH;     
bool stableBtn  = HIGH;     
bool prevStable = HIGH;  

#define US_SAMPLE_MS          100UL
#define US_PULSE_TIMEOUT_US   25000UL
unsigned long lastUsMs  = 0;

//-- thresholds in cm --
#define FAR_THRESH_CM       100
#define NEAR_THRESH_CM      50
#define CLOSE_THRESH_CM     20

enum Mode {
    MODE_DHT,
    MODE_ULTRASONIC
};

Mode mode = MODE_DHT;
void showModeLeds() {
    digitalWrite(LED_YELLOW_DHT_PIN,  (mode == MODE_DHT) ? HIGH : LOW);
    digitalWrite(LED_GREEN_US_PIN, (mode == MODE_ULTRASONIC) ? HIGH : LOW);
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

//ik heb chatgpt hier gebruikt om te helpen om de functie te maken
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
void handleSensorReadings(unsigned long currentMillis) {
    if (mode == MODE_DHT) {
        readDhtSensor(currentMillis);
    } 
    else if (mode == MODE_ULTRASONIC) {
        readUltrasonicSensor(currentMillis);
    }
}
void readDhtSensor(unsigned long currentMillis) {
    if (currentMillis - lastDhtMs < DHT_SAMPLE_MS) return;  // wait until next sample
    lastDhtMs = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature(); // °C

    if (isnan(h) || isnan(t)) {
        Serial.println(F("DHT read failed"));
        return;
    }

    Serial.print(F("DHT22 -> T: "));
    Serial.print(t, 1);
    Serial.print(F(" °C, H: "));
    Serial.print(h, 1);
    Serial.println(F(" %"));

    // Optional: any DHT-specific LED logic here
    allRedOff();  // keep consistent with your mode behavior
}

void readUltrasonicSensor(unsigned long currentMillis) {
    if (currentMillis - lastUsMs < US_SAMPLE_MS) return;  // wait until next sample
    lastUsMs = currentMillis;

    long cm = readUltrasonicCM();

    if (cm >= 0) {
        Serial.print(F("Distance: "));
        Serial.print(cm);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("Distance: out of range / timeout"));
    }

    setRedProgress(cm);
}

void processButtonInput(unsigned long currentMillis) {
    bool raw = digitalRead(BUTTON_PIN);
    if (raw != lastBtnRaw) {
    lastBtnRaw = raw;
    lastBtnEdgeMs = currentMillis;  // start debounce window
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

void setup() {
    Serial.begin(115200);

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

    // ---- Debounce button (active LOW) ----
   processButtonInput(currentMillis);
   handleSensorReadings(currentMillis);
}