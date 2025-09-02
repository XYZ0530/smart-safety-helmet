// Smart Safety Helmet Firmware
// Description: ESP32-based firmware integrating environmental and motion sensors,
//              multimodal local alerts, and MQTT cloud reporting for real-time
//              safety monitoring on construction sites.

#include <WiFi.h>            // Wi-Fi connectivity
#include <PubSubClient.h>    // MQTT client
#include <Wire.h>            // I2C communication
#include <Adafruit_MPU6050.h>// IMU sensor
#include <Adafruit_Sensor.h> // Sensor abstraction
#include <DHT.h>             // Temperature & humidity sensor

// ——— CONFIGURATION ———
const char* WIFI_SSID     = "Student";            // Wi-Fi network name
const char* WIFI_PASSWORD = "xmustudent";         // Wi-Fi password
const char* MQTT_SERVER   = "broker.hivemq.com"; // MQTT broker address
const int   MQTT_PORT     = 1883;                  // MQTT broker port
const char* MQTT_TOPIC    = "smarthelmet/alert";  // Topic for publishing alerts
// ————————————————————

// ——— PIN ASSIGNMENTS ———
#define PIN_DHT     17    // DHT11 data pin
#define PIN_DTYPE   DHT11 // DHT sensor type
#define PIN_SMOKE   34    // MQ-2 digital output (LOW = smoke detected)
#define PIN_LDR     36    // LDR analog input (ADC1_CH0)
#define PIN_BUZZER  16    // Piezo buzzer output
#define PIN_MOTOR   32    // Vibration motor MOSFET gate
#define PIN_LED_R   25    // RGB LED red channel
#define PIN_LED_G   26    // RGB LED green channel
#define PIN_LED_B   27    // RGB LED blue channel (unused)
#define PIN_LAMP    33    // Headlamp MOSFET gate
#define PIN_BUTTON  4     // Clear-alert button (INPUT_PULLUP)
// —————————————————————

// ——— THRESHOLDS ———
const float ACCEL_G_TH = 2.5;   // Impact threshold in g
const float TEMP_TH_C  = 30.0;  // Temperature threshold in °C
const int   LDR_TH     = 3200;  // ADC reading threshold for low light
// ——————————————————

// Sensor and communication objects
Adafruit_MPU6050 mpu;              // IMU sensor instance
DHT             dht(PIN_DHT, PIN_DTYPE); // Temperature/humidity sensor instance
WiFiClient      wifiClient;        // TCP client for MQTT
PubSubClient    mqtt(wifiClient);  // MQTT client

// State variables
bool    inAlert           = false; // Flag: currently in alert state
bool    lowLightNotified  = false; // Flag: low-light warning sent
uint32_t tLastTemp        = 0;     // Last temperature check timestamp
uint32_t tLastImu         = 0;     // Last IMU check timestamp
uint32_t buttonHoldStart  = 0;     // Timestamp for button long-press detection
// Pattern durations: beep/vibration on1, off1, on2, pause
const uint16_t PAT_DUR[4] = {200, 200, 200, 1000};
uint8_t  patIdx           = 0;     // Current index in alert pattern
uint32_t tPat             = 0;     // Timestamp of last alert pattern step

// ——— HELPER FUNCTIONS ———

// Sets the RGB LED to red/green; blue is unused
void setRedGreen(bool red, bool green) {
  digitalWrite(PIN_LED_R, red ? HIGH : LOW);
  digitalWrite(PIN_LED_G, green ? HIGH : LOW);
  digitalWrite(PIN_LED_B, LOW);
}

// Emits a single short beep for acknowledgments
void singleBeep() {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
}

// Turn headlamp on/off
void lampOn()  { digitalWrite(PIN_LAMP, HIGH); }
void lampOff() { digitalWrite(PIN_LAMP, LOW);  }

// Ensure MQTT connection is active; reconnect if dropped
void ensureMqtt() {
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    mqtt.connect("helmetClient");
  }
}

// Format float value to string with one decimal place
String formatValue(float v) {
  char buf[8];
  dtostrf(v, 4, 1, buf);
  return String(buf);
}

// Publish alert message and start local alarm pattern
void startAlert(const char* type) {
  inAlert = true;              // Enter alert state
  patIdx  = 0;                 // Reset alert pattern
  tPat    = millis();          // Timestamp pattern start
  buttonHoldStart = 0;         // Reset button timer

  // Compose human-readable message based on alert type
  String msg;
  if (strcmp(type, "smoke") == 0) {
    msg = "⚠️ Smoke detected! Possible fire hazard.";
  }
  else if (strcmp(type, "temperature") == 0) {
    float T = dht.readTemperature();
    msg = "🌡️ Heat alert! Temp = " + formatValue(T) + "°C (above safe 30°C).";
  }
  else { // g force / impact
    sensors_event_t a, g, tEvt;
    mpu.getEvent(&a, &g, &tEvt);
    float gmag = sqrt(
      sq(a.acceleration.x / 9.80665F) +
      sq(a.acceleration.y / 9.80665F) +
      sq(a.acceleration.z / 9.80665F)
    );
    msg = "💥 Impact detected! G-force = " + formatValue(gmag) + " g.";
  }

  // Publish via MQTT
  ensureMqtt();
  mqtt.publish(MQTT_TOPIC, msg.c_str());

  // Activate local alarms: beep, vibration, red LED
  Serial.println(msg);
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_MOTOR,  HIGH);
  setRedGreen(true, false);
}

// Clear the alert: stop outputs and reset LEDs
void clearAlert() {
  inAlert = false;
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_MOTOR,  LOW);
  setRedGreen(false, true);
  Serial.println("Alert cleared");
}

// ——— SETUP: runs once on power-up ———
void setup() {
  Serial.begin(9600);             // Initialize serial for debug
  while (!Serial) delay(10);

  // Configure I/O pins
  pinMode(PIN_SMOKE,  INPUT);
  pinMode(PIN_LDR,    INPUT);
  pinMode(PIN_BUZZER, OUTPUT);  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MOTOR,  OUTPUT);  digitalWrite(PIN_MOTOR,  LOW);
  pinMode(PIN_LED_R,  OUTPUT);
  pinMode(PIN_LED_G,  OUTPUT);
  pinMode(PIN_LED_B,  OUTPUT);
  pinMode(PIN_LAMP,   OUTPUT);  digitalWrite(PIN_LAMP,   LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  setRedGreen(false, true);      // Default LED state: green

  // Initialize sensors
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU init failed");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  dht.begin();

  // Connect to Wi-Fi (blocking)
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP=");
  Serial.println(WiFi.localIP());

  // Configure MQTT server
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
}

// ——— MAIN LOOP: runs continuously ———
void loop() {
  uint32_t now = millis();

  // Maintain MQTT connection and process incoming packets
  ensureMqtt();
  mqtt.loop();

  // 1) Low-light detection: one-shot beep & headlamp
  int L = analogRead(PIN_LDR);
  if (L < LDR_TH) {
    if (!lowLightNotified) {
      Serial.print("NOTIFY: low light, LDR="); Serial.println(L);
      singleBeep();
      lampOn();
      lowLightNotified = true;
    }
  } else if (lowLightNotified) {
    lowLightNotified = false;
    lampOff();
    setRedGreen(false, true);
  }

  // 2) Handle active alert: blink/beep/vibrate pattern & button clear
  if (inAlert) {
    if (now - tPat >= PAT_DUR[patIdx]) {
      tPat = now;
      patIdx = (patIdx + 1) & 3;
      bool on = (patIdx == 0 || patIdx == 2);
      digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
      digitalWrite(PIN_MOTOR,  on ? HIGH : LOW);
      setRedGreen(on, false);
    }
    // Clear alert on 3-second long-press
    if (digitalRead(PIN_BUTTON) == LOW) {
      if (buttonHoldStart == 0) buttonHoldStart = now;
      else if (now - buttonHoldStart >= 3000) clearAlert();
    } else {
      buttonHoldStart = 0;
    }
    return; // Skip other checks while alert is active
  }

  // 3) Smoke detection (instant trigger)
  if (digitalRead(PIN_SMOKE) == LOW) {
    startAlert("smoke");
    return;
  }

  // 4) Temperature check every 2s
  static uint32_t tTempCheck = 0;
  if (now - tTempCheck >= 2000) {
    tTempCheck = now;
    float T = dht.readTemperature();
    if (!isnan(T) && T >= TEMP_TH_C) {
      startAlert("temperature");
      return;
    }
  }

  // 5) Impact/fall detection every 100ms
  static uint32_t tImuCheck = 0;
  if (now - tImuCheck >= 100) {
    tImuCheck = now;
    sensors_event_t a, g, tEvt;
    mpu.getEvent(&a, &g, &tEvt);
    float gmag = sqrt(
      sq(a.acceleration.x / 9.80665F) +
      sq(a.acceleration.y / 9.80665F) +
      sq(a.acceleration.z / 9.80665F)
    );
    if (gmag > ACCEL_G_TH) {
      startAlert("g force");
      return;
    }
  }

  // 6) Idle state: ensure outputs off and LED green
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_MOTOR,  LOW);
  setRedGreen(false, true);
}
