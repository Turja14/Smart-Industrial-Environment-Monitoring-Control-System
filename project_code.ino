\begin{lstlisting}[language=C++]
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

/* ===================== LCD DISPLAY ===================== */
// I2C LCD: SDA=GPIO21, SCL=GPIO22 (default ESP32 I2C pins)
// Address 0x27 is most common, try 0x3F if it doesn't work
LiquidCrystal_I2C lcd(0x27, 20, 4);  // 20x4 LCD
unsigned long lastLcdUpdateMillis = 0;
const unsigned long LCD_UPDATE_INTERVAL = 1000;  // Update LCD every 1 second

/* ===================== SERVO MOTOR ===================== */
#define SERVO_PIN 13
Servo scanServo;
int servoAngle = 0;
int servoDirection = 1;  // 1 = increasing, -1 = decreasing
const unsigned long SERVO_MOVE_INTERVAL = 30;  // ms between moves
unsigned long lastServoMoveMillis = 0;

/* ===================== DHT22 ===================== */
#define DHTTYPE DHT22
#define DHTPIN 15
DHT dht(DHTPIN, DHTTYPE);

// Store last valid DHT values
float lastValidTemp = 0.0;
float lastValidHum = 0.0;
bool hasValidDHT = false;

/* ===================== BUZZER PATTERN ===================== */
// Two short beeps (beep-beep), then a pause; repeats while DANGER is active.
#define BEEP1_ON_MS   120
#define BEEP_GAP_MS    80
#define BEEP2_ON_MS   120
#define BEEP_PAUSE_MS 700

/* ===================== MQ Sensors ===================== */
#define MQ2_PIN   34
#define MQ135_PIN 35

/* ===================== LED ===================== */
#define LED_PIN 25
const int MQ2_THRESHOLD = 300;
const int MQ135_THRESHOLD = 3500;

/* ===================== SONAR ===================== */
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DANGER_DISTANCE 20.0   // cm

// How often to re-check distance for buzzer responsiveness
const unsigned long DIST_CHECK_INTERVAL_MS = 80;

/* ===================== BUZZER ===================== */
#define BUZZER_PIN 27  // Buzzer connected to pin D27

/* ===================== RELAY (TEMP AUTOMATION) ===================== */
// Relay module IN pin (change to match your wiring)
// Active-LOW relay: LOW = ON, HIGH = OFF
#define RELAY1_PIN 26
// Active-LOW relay: LOW = ON, HIGH = OFF
const float TEMP_AUTO_ON_THRESHOLD_C = 40.0;
const unsigned long RELAY1_RETRY_AFTER_TRIP_MS = 30000;

bool relay1On = false;          // true = relay ON
bool fanAutoMode = true;        // true = auto mode
unsigned long manualOverrideUntil = 0;
const unsigned long MANUAL_HOLD_MS = 5000;   // next cycle পর্যন্ত 5 sec
unsigned long relay1OvercurrentTripMs = 0; // keep 0 if you don't use overcurrent trips

const char* FAN_CONTROL_TOPIC = "iotfrontier/fan/control";
const char* FAN_STATUS_TOPIC  = "iotfrontier/fan/status";

const char* LED_CONTROL_TOPIC    = "iotfrontier/led/control";
const char* LED_STATUS_TOPIC     = "iotfrontier/led/status";

const char* BUZZER_CONTROL_TOPIC = "iotfrontier/buzzer/control";
const char* BUZZER_STATUS_TOPIC  = "iotfrontier/buzzer/status";

/* ===================== WiFi ===================== */
const char* ssid = "TR Hall -Room-210";
const char* password = "03459133";

/* ===================== MQTT ===================== */
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_client_id = "ESP32_JSON_HISTORY_2MIN";

WiFiClient espClient;
PubSubClient client(espClient);

/* ===================== TIMING ===================== */
const unsigned long SAMPLE_INTERVAL = 5000;   // 5 seconds
unsigned long lastSampleMillis = 0;
unsigned long lastDistCheckMillis = 0;

bool ledManualState = false;
bool buzzerManualState = false;

bool ledManualMode = false;
unsigned long ledManualUntil = 0;
const unsigned long LED_MANUAL_HOLD_MS = 5000;  // same as SAMPLE_INTERVAL

bool buzzerManualMode = false;
unsigned long buzzerManualUntil = 0;
const unsigned long BUZZER_MANUAL_HOLD_MS = 5000;  // same as SAMPLE_INTERVAL

bool dangerDetected = false; // last known ultrasonic status
bool gasDetected = false;    // gas detected from MQ2 or MQ135

enum BuzzerPhase : uint8_t {
  BUZZER_IDLE = 0,
  BUZZER_BEEP1_ON,
  BUZZER_GAP,
  BUZZER_BEEP2_ON,
  BUZZER_PAUSE
};

uint8_t buzzerPhase = BUZZER_IDLE;
unsigned long buzzerPhaseStartMillis = 0;

void updateBuzzer(bool danger) {
  // Simple: Buzzer ON only when object < 20cm, OFF otherwise
  if (danger) {
    digitalWrite(BUZZER_PIN, HIGH);  // ON
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // OFF
  }
}

/* ===================== HISTORY BUFFER ===================== */
/* 2 minutes = 120s → 120 / 5 = 24 samples */
#define HISTORY_SIZE 24

struct Sample {
  float temp;
  float hum;
  int mq2;
  int mq135;
  float dist;
  char status[12];   // SAFE / DANGER / CRITICAL
};

Sample history[HISTORY_SIZE];
int histIndex = 0;
bool historyFull = false;

/* ===================== WiFi ===================== */
void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}
void publishFanState() {
  bool fanOn = !relay1On;   // relay ON => fan OFF, relay OFF => fan ON

  if (fanOn) {
    client.publish(FAN_STATUS_TOPIC, "ON", true);
    Serial.println("Published fan status: ON");
  } else {
    client.publish(FAN_STATUS_TOPIC, "OFF", true);
    Serial.println("Published fan status: OFF");
  }
}
void publishLedState() {
  if (ledManualState) {
    client.publish(LED_STATUS_TOPIC, "ON", true);
  } else {
    client.publish(LED_STATUS_TOPIC, "OFF", true);
  }
}

void publishBuzzerState() {
  if (buzzerManualState) {
    client.publish(BUZZER_STATUS_TOPIC, "ON", true);
  } else {
    client.publish(BUZZER_STATUS_TOPIC, "OFF", true);
  }
}
/* ===================== MQTT ===================== */
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
      client.subscribe(FAN_CONTROL_TOPIC);
      client.subscribe(LED_CONTROL_TOPIC);
      client.subscribe(BUZZER_CONTROL_TOPIC);

      publishFanState();
      publishLedState();
      publishBuzzerState();
    } else {
      Serial.println("failed, retrying...");
      delay(3000);
    }
  }
}

/* ===================== SONAR ===================== */
float readDistanceCM() {
  // Take 3 readings and use the median to filter noise
  float readings[3];

  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) {
      readings[i] = 999.0;  // FAR → SAFE
    } else {
      readings[i] = (duration * 0.0343) / 2.0;
    }
    delayMicroseconds(50);  // Small delay between readings
  }

  // Sort to find median (simple bubble sort for 3 elements)
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (readings[i] > readings[j]) {
        float tempSwap = readings[i];
        readings[i] = readings[j];
        readings[j] = tempSwap;
      }
    }
  }

  // Return median value (middle reading)
  float dist = readings[1];

  // Ignore very short readings (likely noise)
  if (dist < 5.0) return 999.0;

  return dist;
}

/* ===================== STORE HISTORY ===================== */
void storeHistory(float t, float h, int mq2, int mq135, float d, const char* s) {
  history[histIndex].temp = t;
  history[histIndex].hum  = h;
  history[histIndex].mq2  = mq2;
  history[histIndex].mq135 = mq135;
  history[histIndex].dist = d;
  strcpy(history[histIndex].status, s);

  histIndex++;
  if (histIndex >= HISTORY_SIZE) {
    histIndex = 0;
    historyFull = true;
  }
}

/* ===================== PUBLISH HISTORY (JSON) ===================== */
void publishHistory() {
  int count = historyFull ? HISTORY_SIZE : histIndex;

  for (int i = 0; i < count; i++) {
    int idx = historyFull ? (histIndex + i) % HISTORY_SIZE : i;

    char json[256];
    snprintf(json, sizeof(json),
      "{"
      "\"temp\":%.2f,"
      "\"hum\":%.2f,"
      "\"mq2\":%d,"
      "\"mq135\":%d,"
      "\"dist\":%.2f,"
      "\"status\":\"%s\""
      "}",
      history[idx].temp,
      history[idx].hum,
      history[idx].mq2,
      history[idx].mq135,
      history[idx].dist,
      history[idx].status
    );

    client.publish("iotfrontier/history", json);
    delay(30);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  if (String(topic) == FAN_CONTROL_TOPIC) {
    fanAutoMode = false;
    manualOverrideUntil = millis() + MANUAL_HOLD_MS;

    if (msg == "ON") {
      // Fan ON => Relay OFF
      relay1On = false;
      digitalWrite(RELAY1_PIN, HIGH);   // relay OFF
      publishFanState();
    }
    else if (msg == "OFF") {
      // Fan OFF => Relay ON
      relay1On = true;
      digitalWrite(RELAY1_PIN, LOW);    // relay ON
      publishFanState();
    }
  }
  if (String(topic) == LED_CONTROL_TOPIC) {
    ledManualMode = true;
    ledManualUntil = millis() + LED_MANUAL_HOLD_MS;

    if (msg == "ON") {
      ledManualState = true;
      digitalWrite(LED_PIN, HIGH);
      publishLedState();
    }
    else if (msg == "OFF") {
      ledManualState = false;
      digitalWrite(LED_PIN, LOW);
      publishLedState();
    }
  }
  if (String(topic) == BUZZER_CONTROL_TOPIC) {
    buzzerManualMode = true;
    buzzerManualUntil = millis() + BUZZER_MANUAL_HOLD_MS;

    if (msg == "ON") {
      buzzerManualState = true;
      digitalWrite(BUZZER_PIN, HIGH);
      publishBuzzerState();
    }
    else if (msg == "OFF") {
      buzzerManualState = false;
      digitalWrite(BUZZER_PIN, LOW);
      publishBuzzerState();
    }
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

  dht.begin();
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Setup buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Setup relay (active LOW)
  pinMode(RELAY1_PIN, OUTPUT);
  relay1On = true;
  digitalWrite(RELAY1_PIN, LOW); // OFF initially

  // Setup Servo Motor
  ESP32PWM::allocateTimer(0);
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(0);

  // Setup LCD Display
  Wire.begin(21, 22);  // SDA=21, SCL=22
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  IOT FRONTIER  ");
  lcd.setCursor(0, 1);
  lcd.print("   Starting...  ");
  delay(1500);
  lcd.clear();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("\n==============================");
  Serial.println("ESP32 STARTED");
  Serial.println("JSON + 2 MIN HISTORY ENABLED");
  Serial.println("==============================");
}

/* ===================== LOOP ===================== */
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  unsigned long now = millis();
  if (now >= buzzerManualUntil) {
    buzzerManualMode = false;
  }
  if (now >= ledManualUntil) {
   ledManualMode = false;
  }
  // Debounce counter for stable detection
  static int dangerCount = 0;
  const int DANGER_THRESHOLD = 3;  // Need 3 consecutive danger readings
  // Fast ultrasonic checks so buzzer reacts quickly
  if (now - lastDistCheckMillis >= DIST_CHECK_INTERVAL_MS) {
    lastDistCheckMillis = now;
    float distFast = readDistanceCM();

    if (distFast < DANGER_DISTANCE) {
      dangerCount++;
      if (dangerCount >= DANGER_THRESHOLD) {
        dangerDetected = true;
        dangerCount = DANGER_THRESHOLD;  // Cap it
      }
    } else {
      dangerCount = 0;
      dangerDetected = false;
    }
  }

  // Keep distance-based buzzer logic running continuously
  //updateBuzzer(dangerDetected);

  // Gas has higher priority -> continuous buzzer sound
  if (!buzzerManualMode) {
    buzzerManualState = (dangerDetected || gasDetected);
    digitalWrite(BUZZER_PIN, buzzerManualState ? HIGH : LOW);
    publishBuzzerState();
  }

  // Servo scanning - HOLD when object detected (under 20cm)
  if (!dangerDetected && (now - lastServoMoveMillis >= SERVO_MOVE_INTERVAL)) {
    lastServoMoveMillis = now;
    servoAngle += (2 * servoDirection);
    if (servoAngle >= 180) {
      servoAngle = 180;
      servoDirection = -1;
    } else if (servoAngle <= 0) {
      servoAngle = 0;
      servoDirection = 1;
    }
    scanServo.write(servoAngle);
  }

  if (now - lastSampleMillis >= SAMPLE_INTERVAL) {
    lastSampleMillis = now;

    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();
    int mq2 = analogRead(MQ2_PIN);
    int mq135 = analogRead(MQ135_PIN);
    float dist = readDistanceCM();

    // Gas detection for LED + continuous buzzer
    gasDetected = (mq2 >= MQ2_THRESHOLD || mq135 >= MQ135_THRESHOLD);

    if (!ledManualMode) {
      ledManualState = gasDetected;
      digitalWrite(LED_PIN, ledManualState ? HIGH : LOW);
      publishLedState();
    }

    // DHT fallback logic
    if (!isnan(temp) && !isnan(hum)) {
      lastValidTemp = temp;
      lastValidHum  = hum;
      hasValidDHT = true;
    } else if (hasValidDHT) {
      temp = lastValidTemp;
      hum  = lastValidHum;
    }
    if (now >= manualOverrideUntil) {
  fanAutoMode = true;
  }

  if (hasValidDHT && !isnan(temp) && fanAutoMode) {
    if (temp >= TEMP_AUTO_ON_THRESHOLD_C) {
      // temp high => relay OFF => fan ON
      if (relay1On) {
        relay1On = false;
        digitalWrite(RELAY1_PIN, HIGH);
        publishFanState();
      }
    } else {
      // temp low => relay ON => fan OFF
      if (!relay1On) {
        relay1On = true;
        digitalWrite(RELAY1_PIN, LOW);
        publishFanState();
      }
    }
  }
  // Status logic: CRITICAL if temp >= threshold AND object < 20cm
  const char* status;
  if (hasValidDHT && temp >= TEMP_AUTO_ON_THRESHOLD_C && dangerDetected) {
    status = "CRITICAL";
  } else if (dangerDetected) {
    status = "DANGER";
  } else {
    status = "SAFE";
  }

    // Temperature automation for Relay 1 (active LOW) - REVERSED LOGIC
    // ON when temp is BELOW threshold, OFF when temp is AT/ABOVE threshold.
   
    /* -------- LIVE JSON -------- */
    char live[256];
    snprintf(live, sizeof(live),
      "{"
      "\"temp\":%.2f,"
      "\"hum\":%.2f,"
      "\"mq2\":%d,"
      "\"mq135\":%d,"
      "\"dist\":%.2f,"
      "\"status\":\"%s\""
      "}",
      hasValidDHT ? temp : 0.0,
      hasValidDHT ? hum : 0.0,
      mq2,
      mq135,
      dist,
      status
    );

    client.publish("iotfrontier/live", live);

    /* -------- STORE HISTORY -------- */
    storeHistory(hasValidDHT ? temp : 0.0,
                 hasValidDHT ? hum : 0.0,
                 mq2, mq135, dist, status);

    /* -------- SERIAL -------- */
    Serial.println("---------------");
    if (hasValidDHT) {
      Serial.print("Temp   : "); Serial.println(temp);
      Serial.print("Hum    : "); Serial.println(hum);
    } else {
      Serial.println("Temp   : Invalid DHT reading");
      Serial.println("Hum    : Invalid DHT reading");
    }
    Serial.print("MQ2    : "); Serial.println(mq2);
    Serial.print("MQ135  : "); Serial.println(mq135);
    Serial.print("Dist   : "); Serial.print(dist); Serial.println(" cm");
    Serial.print("STATUS : "); Serial.println(status);

    /* -------- LCD DISPLAY -------- */
    static int lcdPage = 0;
    static unsigned long lastPageChange = 0;
    const unsigned long PAGE_CHANGE_INTERVAL = 2000; // 2 sec

    if (millis() - lastPageChange >= PAGE_CHANGE_INTERVAL) {
      lastPageChange = millis();
      lcdPage++;
      if (lcdPage > 2) lcdPage = 0;
    }

    lcd.clear();

    if (lcdPage == 0) {
  // Page 1: Temperature and Humidity (separate rows)

      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      if (hasValidDHT) {
        lcd.print(temp, 1);
        lcd.print((char)223);   // degree symbol
        lcd.print("C");
      } else {
        lcd.print("--.-C");
      }

      lcd.setCursor(0, 1);
      lcd.print("Hum : ");
      if (hasValidDHT) {
        lcd.print(hum, 1);
        lcd.print("%");
      } else {
        lcd.print("--.-%");
      }
    }
    else if (lcdPage == 1) {
      // Page 2: MQ2 and MQ135 (separate rows)

      lcd.setCursor(0, 0);
      lcd.print("MQ2  : ");
      lcd.print(mq2);

      lcd.setCursor(0, 1);
      lcd.print("MQ135: ");
      lcd.print(mq135);
    }
    else if (lcdPage == 2) {
      // Page 3: Distance

      lcd.setCursor(0, 0);
      lcd.print("Distance:");

      lcd.setCursor(0, 1);
      lcd.print(dist, 1);
      lcd.print(" cm");
    }
    else {
      // Page 4: Status

      lcd.setCursor(0, 0);
      lcd.print("System Status:");

      lcd.setCursor(0, 1);
      lcd.print(status);
    }

    /* -------- PUBLISH HISTORY EVERY 2 MIN -------- */
    static int sampleCount = 0;
    sampleCount++;
    if (sampleCount >= HISTORY_SIZE) {
      publishHistory();
      sampleCount = 0;
    }
  }
}
\end{lstlisting}