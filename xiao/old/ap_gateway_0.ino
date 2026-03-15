#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// =====================
// Network (AP gateway only)
// =====================
static const int CAR_ID = 1;
const char* AP_SSID = "microMVP_AP";
const char* AP_PASS = "12345678";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);
IPAddress UDP_BROADCAST(192, 168, 4, 255);

const uint16_t TCP_PORT = 9000;   // PC -> AP (TCP)
const uint16_t UDP_PORT = 9001;   // AP -> Cars (UDP)

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
WiFiUDP udp;

// TCP line buffer
static const size_t LINE_BUF_MAX = 1024;
char lineBuf[LINE_BUF_MAX];
size_t lineLen = 0;

void resetLineBuf() {
  lineLen = 0;
  lineBuf[0] = '\0';
}

void setupSoftAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("[AP0] softAP failed!");
    while (true) delay(1000);
  }
  Serial.print("[AP0] SSID: "); Serial.println(AP_SSID);
  Serial.print("[AP0] IP: "); Serial.println(WiFi.softAPIP());
}

// Parse JSON line, DO NOT execute any motor action, only forward by UDP
void handleJsonLine(const char* line) {
  // (optional) parse just for validation/log (can remove if you want max speed)
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("[AP0] JSON parse error: ");
    Serial.println(err.c_str());
    // 即使 parse 失败，你也可以选择仍然转发 raw line（看你需求）
    // return;
  }

  // Safety: If someone still sends actions["0"], AP0 ignores it.
  // We do nothing here on purpose.

  // Forward raw line via UDP broadcast
  udp.beginPacket(UDP_BROADCAST, UDP_PORT);
  udp.write((const uint8_t*)line, strlen(line));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== AP0 Gateway (AP + TCP->UDP Forwarder ONLY) ===");

  setupSoftAP();

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  // UDP sender (we can bind local port or not; begin is ok)
  udp.begin(UDP_PORT);

  Serial.print("[AP0] TCP_PORT="); Serial.println(TCP_PORT);
  Serial.print("[AP0] UDP_PORT="); Serial.println(UDP_PORT);
  Serial.println("[AP0] Ready: PC connect WiFi -> TCP 192.168.4.1:9000");

  resetLineBuf();
}

void loop() {
  // Accept TCP client
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      tcpClient = newClient;
      tcpClient.setNoDelay(true);
      Serial.print("[AP0] TCP client connected: ");
      Serial.println(tcpClient.remoteIP());
      tcpClient.println("{\"type\":\"hello\",\"from\":\"ap0\"}");
      resetLineBuf();
    }
  }

  // Read TCP stream -> lines
  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available() > 0) {
      int c = tcpClient.read();
      if (c < 0) break;
      if (c == '\r') continue;

      if (c == '\n') {
        if (lineLen > 0) {
          // trim tail spaces
          while (lineLen > 0 && (lineBuf[lineLen-1]==' ' || lineBuf[lineLen-1]=='\t')) lineLen--;
          lineBuf[lineLen] = '\0';

          Serial.print("[AP0] TCP IN: ");
          Serial.println(lineBuf);

          handleJsonLine(lineBuf);

          // optional ack
          tcpClient.println("{\"type\":\"ack\"}");

          resetLineBuf();
        } else {
          resetLineBuf();
        }
      } else {
        if (lineLen < LINE_BUF_MAX - 1) {
          lineBuf[lineLen++] = (char)c;
          lineBuf[lineLen] = '\0';
        } else {
          Serial.println("[AP0] Line too long, drop.");
          resetLineBuf();
        }
      }
    }
  }

  delay(1);
}
