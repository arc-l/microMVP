// xiao_car_espnow_ledcAttach.ino
// Xiao car: receive ESP-NOW broadcast (33 bytes), parse slot for CAR_ID,
// drive motors via PHASE/ENABLE using ledcAttach.
//
// ESP-NOW in (broadcast):
//   [level(1)] + payload(32)  => 33 bytes
//
// Board: Seeed XIAO ESP32-C6
// IMPORTANT: Keep ESPNOW_CHANNEL same as gateway (e.g., 1)

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// -------- ESP-NOW channel --------
static const uint8_t ESPNOW_CHANNEL = 1;   // MUST match gateway

// -------- Car ID --------
static const int CAR_ID = 6;   // <<< make sure this matches your car
static const int CARS_PER_LEVEL = 10;
static const int CAR_LEVEL = (CAR_ID - 1) / CARS_PER_LEVEL;
static const int CAR_INDEX = CAR_ID - CAR_LEVEL * CARS_PER_LEVEL; // 1..10

// -------- Motor pins (KEEP SAME) --------
const int APHASE = D3;
const int AENBL  = D2;   // PWM
const int BPHASE = D1;
const int BENBL  = D0;   // PWM

// -------- PWM config (KEEP SAME) --------
static const uint32_t PWM_FREQ = 20000;
static const uint8_t  PWM_RES  = 8;       // bits, duty 0..255

// -------- Calibration (KEEP SAME) --------
static const int LEFT_MAX  = 255;  // scale 0..127 -> 0..LEFT_MAX
static const int RIGHT_MAX = 255;

// -------- Safety --------
static const uint32_t CMD_TIMEOUT_MS = 200;
static uint32_t lastCmdMs = 0;

// -------- Debug stats --------
static uint32_t dbg_any = 0;
static uint32_t dbg_len33 = 0;
static uint32_t dbg_level_ok = 0;
static uint32_t dbg_slot_ok = 0;
static uint32_t dbg_nonzero = 0;
static uint32_t dbg_timeout = 0;
static uint32_t dbg_lastPrintMs = 0;
static int dbg_lastL = 0;
static int dbg_lastR = 0;

// ---------- helpers ----------
static inline int decodeThrust(uint8_t b) {
  int mag = (int)(b & 0x7F); // 0111 1111，get absolute value
  int sgn = (b & 0x80) ? -1 : 1; // 1000 0000，get sign
  return mag * sgn; // -127..127
}

static inline int dutyFromThrust(int thrustAbs, int maxDuty) {
  int d = (thrustAbs * maxDuty) / 127;
  if (d < 0) d = 0;
  if (d > 255) d = 255;
  return d;
}

// -------- motor control (KEEP SAME) --------
void setupMotor() {
  pinMode(APHASE, OUTPUT);
  pinMode(BPHASE, OUTPUT);

  if (!ledcAttach(AENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach AENBL failed");
  }
  if (!ledcAttach(BENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach BENBL failed");
  }

  digitalWrite(APHASE, LOW);
  digitalWrite(BPHASE, LOW);

  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

void runMotors(int rightThrust, int leftThrust) {
  digitalWrite(BPHASE, (rightThrust >= 0) ? LOW : HIGH);
  digitalWrite(APHASE, (leftThrust  >= 0) ? LOW : HIGH);

  int rabs = abs(rightThrust);
  int labs = abs(leftThrust);

  int rduty = dutyFromThrust(rabs, RIGHT_MAX);
  int lduty = dutyFromThrust(labs, LEFT_MAX);

  ledcWrite(BENBL, rduty);
  ledcWrite(AENBL, lduty);
}

void stopMotors() {
  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

// -------- ESP-NOW setup --------
static void setupWiFiForEspNow() {
  // ESP-NOW works in STA mode (no AP connect needed)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Fix channel to match gateway
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("[CAR] STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[CAR] ESPNOW channel=%u\n", ESPNOW_CHANNEL);
}

static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  dbg_any++;

  if (len != 33 || data == nullptr) return;
  dbg_len33++;

  uint8_t level = data[0];
  if ((int)level != CAR_LEVEL) return;
  dbg_level_ok++;

  const uint8_t* payload = &data[1];

  int rightPos = 3 * CAR_INDEX;
  int leftPos  = 3 * CAR_INDEX - 1;

  if (!(rightPos >= 0 && rightPos < 32 && leftPos >= 0 && leftPos < 32)) return;
  dbg_slot_ok++;

  uint8_t rc = payload[rightPos];
  uint8_t lc = payload[leftPos];

  int rightThrust = decodeThrust(rc);
  int leftThrust  = decodeThrust(lc);

  dbg_lastR = rightThrust;
  dbg_lastL = leftThrust;
  if (rightThrust != 0 || leftThrust != 0) dbg_nonzero++;

  runMotors(rightThrust, leftThrust);
  lastCmdMs = millis();

  // Low-rate RX print (optional)
  uint32_t now = millis();
  if (now - dbg_lastPrintMs > 200) { // 5Hz
    dbg_lastPrintMs = now;
    Serial.printf("[RX] level=%u rc=%u lc=%u rTh=%d lTh=%d\n",
                  level, rc, lc, rightThrust, leftThrust);
  }
}

static void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[CAR][ERR] esp_now_init failed");
    while (true) delay(1000);
  }
  Serial.println("[CAR] esp_now_init OK");

  // Register receive callback
  esp_now_register_recv_cb(onEspNowRecv);

  // No need to add peers for receiving broadcast
}

// -------- main --------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== Xiao Car ESP-NOW Receiver ===");
  Serial.printf("[CAR] CAR_ID=%d CAR_LEVEL=%d CAR_INDEX=%d\n", CAR_ID, CAR_LEVEL, CAR_INDEX);

  setupMotor();

  setupWiFiForEspNow();
  setupEspNow();

  lastCmdMs = millis();
  dbg_lastPrintMs = millis();
}

void loop() {
  // Safety stop if command stream lost
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
    dbg_timeout++;
    // Avoid printing too often
    static uint32_t lastT = 0;
    uint32_t now = millis();
    if (now - lastT > 1000) {
      lastT = now;
      Serial.println("[SAFE] timeout -> stop");
    }
  }

  // 1Hz stats
  static uint32_t lastStat = 0;
  uint32_t now = millis();
  if (now - lastStat > 1000) {
    lastStat = now;
    Serial.printf("[STAT] any=%lu len33=%lu levelOK=%lu slotOK=%lu nonzero=%lu timeout=%lu last(L,R)=(%d,%d)\n",
                  dbg_any, dbg_len33, dbg_level_ok, dbg_slot_ok, dbg_nonzero, dbg_timeout,
                  dbg_lastL, dbg_lastR);
    dbg_any = dbg_len33 = dbg_level_ok = dbg_slot_ok = dbg_nonzero = dbg_timeout = 0;
  }

  delay(1);
}
