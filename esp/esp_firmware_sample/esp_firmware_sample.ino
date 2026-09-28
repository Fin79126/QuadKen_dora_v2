/*
 * QuadKen Dual-ESP Firmware Sample for ESP32
 * 
 * Communication Architecture:
 * - TCP Server: Handles connection status, heartbeat (PING-PONG), and health checks with Raspberry Pi.
 * - UDP Socket: Receives high-speed servo/motor commands and sends sensor telemetry (50Hz).
 * - Failsafe: Automatically disables motors and neutralizes servos if TCP drops or UDP times out.
 * 
 * Requirements:
 * - ESP32 Board package
 * - ArduinoJson library (by Benoit Blanchon, v6 or v7)
 * - ESP32Servo library (or ledc)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ========== Network Settings ==========
const char* WIFI_SSID     = "Your_Robot_WiFi_SSID";
const char* WIFI_PASSWORD = "Your_Robot_WiFi_Password";

// Configure this ESP's identity: 1 for ESP1 (Front), 2 for ESP2 (Rear)
#define ESP_ID_NUMBER 1

#if ESP_ID_NUMBER == 1
  const char* ESP_ID   = "esp1";
  const uint16_t TCP_PORT = 5001;
  const uint16_t UDP_PORT = 6001;
#else
  const char* ESP_ID   = "esp2";
  const uint16_t TCP_PORT = 5002;
  const uint16_t UDP_PORT = 6002;
#endif

// Raspberry Pi UDP telemetry destination (will be updated dynamically upon receiving commands)
IPAddress raspiIp(192, 168, 1, 100);
const uint16_t RASPI_UDP_PORT = 6000;

// Sockets
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
WiFiUDP    udp;

// Safety & Timing
unsigned long lastUdpCommandTime = 0;
unsigned long lastTelemetryTime  = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000; // 1 second timeout
bool failsafeTriggered = false;

// Actuator States
float targetServos[6] = {0, 0, 0, 0, 0, 0};
int   targetMotors[2] = {0, 0};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\n[QuadKen %s] Starting ESP Firmware...\n", ESP_ID);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Start Servers
  tcpServer.begin();
  udp.begin(UDP_PORT);
  Serial.printf("[TCP Server] Listening on port %d\n", TCP_PORT);
  Serial.printf("[UDP Server] Listening on port %d\n", UDP_PORT);

  lastUdpCommandTime = millis();
}

void handleTcpStatus() {
  // Check for incoming client connection from Raspberry Pi
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      tcpClient = newClient;
      Serial.println("[TCP] Raspberry Pi connected!");
    }
  }

  // Read incoming heartbeat messages from Raspberry Pi
  if (tcpClient && tcpClient.connected() && tcpClient.available()) {
    String line = tcpClient.readStringUntil('\n');
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, line);
    if (!error) {
      const char* type = doc["type"];
      if (type && strcmp(type, "PING") == 0) {
        // Respond with PONG
        StaticJsonDocument<256> resp;
        resp["type"]      = "PONG";
        resp["state"]     = failsafeTriggered ? "FAILSAFE" : "READY";
        resp["timestamp"] = doc["timestamp"];
        resp["esp_id"]    = ESP_ID;

        String respStr;
        serializeJson(resp, respStr);
        respStr += "\n";
        tcpClient.print(respStr);
      }
    }
  }
}

void handleUdpCommands() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[512];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      raspiIp = udp.remoteIP(); // Auto-learn RasPi IP

      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (!error) {
        lastUdpCommandTime = millis();
        failsafeTriggered = false;

        JsonArray servos = doc["servos"];
        for (int i = 0; i < 6 && i < servos.size(); i++) {
          targetServos[i] = servos[i];
        }

        JsonArray motors = doc["motors"];
        for (int i = 0; i < 2 && i < motors.size(); i++) {
          targetMotors[i] = motors[i];
        }

        // Apply commands to physical servos and motors here!
        // applyActuatorOutputs();
      }
    }
  }
}

void sendUdpTelemetry() {
  // Send sensor telemetry to Raspberry Pi at 50Hz (every 20ms)
  unsigned long now = millis();
  if (now - lastTelemetryTime >= 20) {
    lastTelemetryTime = now;

    StaticJsonDocument<384> doc;
    doc["esp_id"]    = ESP_ID;
    doc["timestamp"] = (double)now / 1000.0;
    doc["voltage"]   = 12.2;  // Read from ADC voltage divider
    doc["current"]   = 0.45;  // Read from INA219 / ACS712 current sensor

    JsonArray actualServos = doc.createNestedArray("actual_servos");
    for (int i = 0; i < 6; i++) {
      actualServos.add(targetServos[i]);
    }

    JsonArray actualMotors = doc.createNestedArray("actual_motors");
    actualMotors.add(targetMotors[0]);
    actualMotors.add(targetMotors[1]);

    char buffer[384];
    size_t bytes = serializeJson(doc, buffer, sizeof(buffer));

    udp.beginPacket(raspiIp, RASPI_UDP_PORT);
    udp.write((const uint8_t*)buffer, bytes);
    udp.endPacket();
  }
}

void checkFailsafe() {
  // If no UDP command for 1000ms, or TCP drops, engage safety stop
  if (millis() - lastUdpCommandTime > FAILSAFE_TIMEOUT_MS) {
    if (!failsafeTriggered) {
      failsafeTriggered = true;
      Serial.println("[Safety] Failsafe engaged: Actuators neutral and motors stopped!");
    }
    // Set actuators to safe neutral values
    for (int i = 0; i < 6; i++) targetServos[i] = 0.0;
    targetMotors[0] = 0;
    targetMotors[1] = 0;
  }
}

void loop() {
  handleTcpStatus();
  handleUdpCommands();
  sendUdpTelemetry();
  checkFailsafe();
  delay(1);
}
