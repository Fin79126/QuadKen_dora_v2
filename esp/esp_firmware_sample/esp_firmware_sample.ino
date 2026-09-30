/*
 * QuadKen Dual-ESP Firmware Sample for ESP32 (Underwater AUV)
 * 
 * Hardware Roles:
 * - ESP #1 (ID: 1): Thrust & Drag-Steering Unit
 *     * 2x BLDC Motors (Forward thrust, PWM)
 *     * 4x Servos (Membrane leg deployment: Top, Right, Bottom, Left)
 * - ESP #2 (ID: 2): Ballast & Buoyancy Unit
 *     * 4x Servos (Head water-intake / ballast adjustment)
 * 
 * Communication Architecture:
 * - TCP Server: Handles connection status, heartbeat (PING-PONG), and health checks with Raspberry Pi.
 * - UDP Socket: Receives high-speed servo/motor commands and sends sensor telemetry (50Hz).
 * - Failsafe: Automatically stops BLDC thrusters and closes/purges ballast if connection drops.
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

// Configure this ESP's identity: 1 for ESP1 (Thrust/Steer), 2 for ESP2 (Ballast)
#define ESP_ID_NUMBER 1

#if ESP_ID_NUMBER == 1
  const char* ESP_ID      = "esp1";
  const uint16_t TCP_PORT = 5001;
  const uint16_t UDP_PORT = 6001;
  const int NUM_SERVOS    = 4;  // 4 Membrane leg deployment servos
  const int NUM_MOTORS    = 2;  // 2 BLDC Forward thrusters
#else
  const char* ESP_ID      = "esp2";
  const uint16_t TCP_PORT = 5002;
  const uint16_t UDP_PORT = 6002;
  const int NUM_SERVOS    = 4;  // 4 Head water-intake ballast servos
  const int NUM_MOTORS    = 0;  // No thruster motors on ESP2
#endif

// Raspberry Pi UDP telemetry destination (auto-learned on command reception)
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
float targetServos[4] = {0, 0, 0, 0};
int   targetMotors[2] = {0, 0};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\n[QuadKen AUV %s] Starting ESP Firmware...\n", ESP_ID);

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
  Serial.printf("[TCP Server] Listening on port %d (Status)\n", TCP_PORT);
  Serial.printf("[UDP Server] Listening on port %d (Actuators)\n", UDP_PORT);

  lastUdpCommandTime = millis();
}

void handleTcpStatus() {
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      tcpClient = newClient;
      Serial.println("[TCP] Raspberry Pi connected for status/heartbeat.");
    }
  }

  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available()) {
      String line = tcpClient.readStringUntil('\n');
      if (line.length() == 0) continue;

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, line);
      if (!error && doc["type"] == "PING") {
        StaticJsonDocument<256> resp;
        resp["type"]      = "PONG";
        resp["state"]     = failsafeTriggered ? "FAILSAFE" : "READY";
        resp["timestamp"] = doc["timestamp"];
        resp["battery"]   = 12.4; // Read from ADC

        String out;
        serializeJson(resp, out);
        out += "\n";
        tcpClient.print(out);
      }
    }
  }
}

void handleUdpCommands() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
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
        for (int i = 0; i < NUM_SERVOS && i < servos.size(); i++) {
          targetServos[i] = servos[i];
        }

        JsonArray motors = doc["motors"];
        for (int i = 0; i < NUM_MOTORS && i < motors.size(); i++) {
          targetMotors[i] = motors[i];
        }

        // Apply commands to physical hardware here!
        // ESP1: Set PWM on BLDC ESCs, Set Servo angles on membrane legs
        // ESP2: Set Servo angles on ballast intake valves
      }
    }
  }
}

void sendUdpTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryTime >= 20) {
    lastTelemetryTime = now;

    StaticJsonDocument<384> doc;
    doc["esp_id"]    = ESP_ID;
    doc["timestamp"] = (double)now / 1000.0;
    doc["voltage"]   = 12.4;  // Read from ADC voltage divider
    doc["current"]   = 0.45;  // Read from INA219 current sensor

    JsonArray actualServos = doc.createNestedArray("actual_servos");
    for (int i = 0; i < NUM_SERVOS; i++) {
      actualServos.add(targetServos[i]);
    }

    if (NUM_MOTORS > 0) {
      JsonArray actualMotors = doc.createNestedArray("actual_motors");
      for (int i = 0; i < NUM_MOTORS; i++) {
        actualMotors.add(targetMotors[i]);
      }
    }

    char buffer[384];
    size_t bytes = serializeJson(doc, buffer, sizeof(buffer));

    udp.beginPacket(raspiIp, RASPI_UDP_PORT);
    udp.write((const uint8_t*)buffer, bytes);
    udp.endPacket();
  }
}

void checkFailsafe() {
  if (millis() - lastUdpCommandTime > FAILSAFE_TIMEOUT_MS) {
    if (!failsafeTriggered) {
      failsafeTriggered = true;
      Serial.println("[Safety] Failsafe engaged: Stopping thrusters, purging ballast!");
    }
    // Safe values:
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetServos[i] = 0.0;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
      targetMotors[i] = 0;
    }
  }
}

void loop() {
  handleTcpStatus();
  handleUdpCommands();
  sendUdpTelemetry();
  checkFailsafe();
  delay(1);
}
