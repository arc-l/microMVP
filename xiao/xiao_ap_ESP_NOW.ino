// xiao_ap_espnow_bridge.ino
// Xiao_AP (Gateway): USB Serial -> ESP-NOW broadcast
//
// Serial in from PC (Framed):
//   0xAA 0x55 | level(1) | len(1=32) | payload(32) | checksum(1)
// checksum = XOR(level, len, payload...)
//
// ESP-NOW out (broadcast):
//   [level(1)] + payload(32)  => 33 bytes
//
// Board: Seeed XIAO ESP32-C6
// Baud: 115200
// Channel: set ESPNOW_CHANNEL below (must match ALL cars)

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// --------------------
// Serial settings
// --------------------
static const uint32_t SERIAL_BAUD = 115200;

// --------------------
// ESP-NOW settings
// --------------------
static const uint8_t ESPNOW_CHANNEL = 1;   // MUST match car receivers
static const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// --------------------
// Frame format
// --------------------
static const uint8_t HDR0 = 0xAA;
static const uint8_t HDR1 = 0x55;
static const uint8_t PAYLOAD_LEN_EXPECTED = 32;

// Parser state machine
enum RxState { WAIT_HDR0, WAIT_HDR1, WAIT_LEVEL, WAIT_LEN, WAIT_PAYLOAD, WAIT_CKSUM };
static RxState state = WAIT_HDR0;

static uint8_t rx_level = 0;
static uint8_t rx_len = 0;
static uint8_t rx_payload[PAYLOAD_LEN_EXPECTED];
static uint8_t rx_idx = 0;
static uint8_t rx_cksum = 0;

// Debug stats (1Hz)
static uint32_t stat_ok = 0;
static uint32_t stat_bad_ck = 0;
static uint32_t stat_bad_len = 0;
static uint32_t stat_send_ok = 0;
static uint32_t stat_send_fail = 0;
static uint32_t stat_last_ms = 0;

static inline uint8_t crc8_xor_update(uint8_t c, uint8_t b) { return (uint8_t)(c ^ b); }

// ---- C6 new signature ----
static void onEspNowSend(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) stat_send_ok++;
  else stat_send_fail++;
}

static void setupWiFiForEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Fix channel (critical)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("[GW] STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[GW] ESPNOW channel=%u\n", ESPNOW_CHANNEL);
}

static void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[GW][ERR] esp_now_init failed");
    while (true) delay(1000);
  }
  Serial.println("[GW] esp_now_init OK");

  // Add broadcast peer (recommended on IDF5)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  // peer.ifidx = WIFI_IF_STA; // some cores expose this; safe to omit

  esp_err_t perr = esp_now_add_peer(&peer);
  if (perr == ESP_OK) Serial.println("[GW] broadcast peer added");
  else Serial.printf("[GW] esp_now_add_peer(bcast) => %d (may be OK if already added)\n", (int)perr);

  // Register send callback (C6 signature)
  esp_now_register_send_cb(onEspNowSend);
}

static void espNowBroadcastLevelPayload(uint8_t level, const uint8_t* payload32) {
  uint8_t out[1 + PAYLOAD_LEN_EXPECTED];
  out[0] = level;
  memcpy(out + 1, payload32, PAYLOAD_LEN_EXPECTED);

  esp_err_t err = esp_now_send(BCAST_MAC, out, sizeof(out));
  if (err != ESP_OK) {
    stat_send_fail++;
    // Serial.printf("[GW][ERR] esp_now_send => %d\n", (int)err);
  }
}

// Feed one byte into the serial frame parser.
// Returns true if a complete valid frame was assembled and broadcast.
static bool feedByte(uint8_t b) {
  switch (state) {
    case WAIT_HDR0:
      if (b == HDR0) state = WAIT_HDR1;
      break;

    case WAIT_HDR1:
      state = (b == HDR1) ? WAIT_LEVEL : WAIT_HDR0;
      break;

    case WAIT_LEVEL:
      rx_level = b;
      rx_cksum = 0;
      rx_cksum = crc8_xor_update(rx_cksum, rx_level);
      state = WAIT_LEN;
      break;

    case WAIT_LEN:
      rx_len = b;
      rx_cksum = crc8_xor_update(rx_cksum, rx_len);
      if (rx_len != PAYLOAD_LEN_EXPECTED) {
        stat_bad_len++;
        state = WAIT_HDR0;
        break;
      }
      rx_idx = 0;
      state = WAIT_PAYLOAD;
      break;

    case WAIT_PAYLOAD:
      rx_payload[rx_idx++] = b;
      rx_cksum = crc8_xor_update(rx_cksum, b);
      if (rx_idx >= rx_len) state = WAIT_CKSUM;
      break;

    case WAIT_CKSUM:
      if (b == rx_cksum) {
        stat_ok++;
        espNowBroadcastLevelPayload(rx_level, rx_payload);
        state = WAIT_HDR0;
        return true;
      } else {
        stat_bad_ck++;
        state = WAIT_HDR0;
      }
      break;
  }
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("=== Xiao Gateway USB->ESP-NOW Bridge (C6) ===");
  Serial.println("[INFO] Serial in: AA 55 level len(32) payload(32) cksum(xor)");
  Serial.println("[INFO] ESPNOW out: [level(1)] + payload(32) broadcast");

  setupWiFiForEspNow();
  setupEspNow();

  stat_last_ms = millis();
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    feedByte(b);
  }

  // 1Hz stats
  uint32_t now = millis();
  if (now - stat_last_ms >= 1000) {
    stat_last_ms = now;
    Serial.printf("[STAT] frame_ok=%lu bad_ck=%lu bad_len=%lu send_ok=%lu send_fail=%lu\n",
                  stat_ok, stat_bad_ck, stat_bad_len, stat_send_ok, stat_send_fail);
    stat_ok = stat_bad_ck = stat_bad_len = 0;
    stat_send_ok = stat_send_fail = 0;
  }

  delay(1);
}
