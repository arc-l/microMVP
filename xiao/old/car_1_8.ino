#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ====== Set per-car ID (1~8) ======
#define CAR_ID 1   // <<< 改成 1~8

// =====================
// DRV8835 pins (MODE=1 PHASE/ENABLE)
// =====================
const int APHASE = D3;
const int AENBL  = D2;   // PWM
const int BPHASE = D1;
const int BENBL  = D0;   // PWM

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_A = 0;
const int CH_B = 1;

float DEAD_BAND = 0.05;
bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

// =====================
// WiFi STA config
// =====================
const char* AP_SSID = "microMVP_AP";
const char* AP_PASS = "12345678";

const uint16_t UDP_PORT = 9001;
WiFiUDP udp;

unsigned long lastCmdMs = 0;
const uint32_t COMMAND_TIMEOUT_MS = 500;

// --------------------- Motor helpers ---------------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void motorBrake() {
  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

void setMotorA(float cmd) {
  if (INVERT_LEFT) cmd = -cmd;
  cmd = clampf(cmd, -1.0f, 1.0f);
  if (fabs(cmd) < DEAD_BAND) { ledcWrite(AENBL, 0); return; }
  digitalWrite(APHASE, (cmd < 0) ? HIGH : LOW);
  int duty = (int)(fabs(cmd) * 255.0f);
  ledcWrite(AENBL, duty);
}

void setMotorB(float cmd) {
  if (INVERT_RIGHT) cmd = -cmd;
  cmd = clampf(cmd, -1.0f, 1.0f);
  if (fabs(cmd) < DEAD_BAND) { ledcWrite(BENBL, 0); return; }
  digitalWrite(BPHASE, (cmd < 0) ? HIGH : LOW);
  int duty = (int)(fabs(cmd) * 255.0f);
  ledcWrite(BENBL, duty);
}

void driveWheels(float vl, float vr) {
  setMotorA(vl);
  setMotorB(vr);
}

void setupMotor() {
  pinMode(APHASE, OUTPUT);
  pinMode(BPHASE, OUTPUT);

  // 把 PWM 绑定到对应引脚，并设定频率/分辨率
  // 返回值是 bool，失败会返回 false
  if (!ledcAttach(AENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach AENBL failed");
  }
  if (!ledcAttach(BENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach BENBL failed");
  }

  digitalWrite(APHASE, LOW);
  digitalWrite(BPHASE, LOW);

  // 初始刹车
  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

void connectToAP() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(AP_SSID, AP_PASS);

  Serial.print("[CAR] Connecting to ");
  Serial.println(AP_SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\n[CAR] WiFi timeout, retry...");
      WiFi.disconnect(true);
      delay(200);
      WiFi.begin(AP_SSID, AP_PASS);
      t0 = millis();
    }
  }
  Serial.println("\n[CAR] Connected!");
  Serial.print("[CAR] IP: "); Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.printf("\n=== CAR %d STA + DRV8835 ===\n", CAR_ID);

  setupMotor();
  connectToAP();

  udp.begin(UDP_PORT);
  Serial.print("[CAR] UDP listen port "); Serial.println(UDP_PORT);

  lastCmdMs = millis();
}

void loop() {
  // Reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[CAR] WiFi dropped, reconnect...");
    connectToAP();
    udp.begin(UDP_PORT);
  }

  // Receive UDP
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    static char buf[1024];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';

      StaticJsonDocument<1024> doc;
      DeserializationError err = deserializeJson(doc, buf);
      if (!err) {
        JsonObject actions = doc["actions"].as<JsonObject>();
        if (!actions.isNull()) {
          char key[8];
          snprintf(key, sizeof(key), "%d", CAR_ID);
          if (actions.containsKey(key) && actions[key].is<JsonArray>()) {
            JsonArray arr = actions[key].as<JsonArray>();
            if (arr.size() >= 2) {
              float vl = arr[0].as<float>();
              float vr = arr[1].as<float>();
              driveWheels(vl, vr);
              lastCmdMs = millis();
            }
          }
        }
      } else {
        Serial.print("[CAR] JSON parse error: ");
        Serial.println(err.c_str());
      }
    }
  }

  // Watchdog
  if (millis() - lastCmdMs > COMMAND_TIMEOUT_MS) {
    motorBrake();
    lastCmdMs = millis();
  }

  delay(1);
}
