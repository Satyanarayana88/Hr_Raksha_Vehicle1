#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <math.h>

// --- VEHICLE-SPECIFIC CONFIGURATION ---
// Set to "VEH_01" for Vehicle 1, "VEH_02" for Vehicle 2
#define VEHICLE_ID "VEH_02" 

// Target MAC Address (Set to Vehicle 2 MAC for Vehicle 1, and vice versa)
// Target MAC Address: Universal ESP-NOW Broadcast
uint8_t targetPeerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* firebaseBaseUrl = "https://hr-raksha-default-rtdb.firebaseio.com/crash_reports/vehicle_2.json";

Adafruit_MPU6050 mpu;

#define BUFFER_SIZE 20
struct SensorSample { float ay, gz; };
SensorSample ringBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

typedef struct struct_v2v_payload {
  char vehicle_id[10];
  float impact_g;
  float rot_rate;
  float pre_impact_ay;
  float pre_impact_gz;
  float temperature;
  bool crash_event; // Flags whether packet is a routine update or active crash
} struct_v2v_payload;

struct_v2v_payload v2vPayload;
struct_v2v_payload incomingPeerData;
bool peerDataReceived = false;

unsigned long lastBroadcastTime = 0;
const unsigned long broadcastInterval = 200; // Heartbeat interval in ms

void pushSample(float ay, float gz) {
  ringBuffer[bufferIndex] = {ay, gz};
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferIndex == 0) bufferFull = true;
}

float getMaxPreImpactAy() {
  float maxAy = 0.0;
  int count = bufferFull ? BUFFER_SIZE : bufferIndex;
  for (int i = 0; i < count; i++) {
    if (fabs(ringBuffer[i].ay) > fabs(maxAy)) maxAy = ringBuffer[i].ay;
  }
  return maxAy;
}

float getMaxPreImpactGz() {
  float maxGz = 0.0;
  int count = bufferFull ? BUFFER_SIZE : bufferIndex;
  for (int i = 0; i < count; i++) {
    if (fabs(ringBuffer[i].gz) > fabs(maxGz)) maxGz = ringBuffer[i].gz;
  }
  return maxGz;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Silent background transmission status logging
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  struct_v2v_payload tempPayload;
  memcpy(&tempPayload, incomingData, sizeof(tempPayload));
  
  if (strcmp(tempPayload.vehicle_id, VEHICLE_ID) != 0) {
    incomingPeerData = tempPayload;
    peerDataReceived = true;
    
    if (tempPayload.crash_event) {
      Serial.print("\n[CRASH SIGNAL RECEIVED FROM OPPONENT]: ");
      Serial.println(incomingPeerData.vehicle_id);
    }
  }
}

void sendNormalizedFirebaseReport(float hostG, float hostRotRate, float preAy, float preGz, float tempC) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  http.begin(client, firebaseBaseUrl);
  http.addHeader("Content-Type", "application/json");

  String jsonPayload = "{";
  jsonPayload += "\"timestamp\":" + String(millis()) + ",";
  
  jsonPayload += "\"host\":{";
  jsonPayload += "\"id\":\"" + String(VEHICLE_ID) + "\",";
  jsonPayload += "\"impact_g\":" + String(hostG, 2) + ",";
  jsonPayload += "\"rot_rate\":" + String(hostRotRate, 2) + ",";
  jsonPayload += "\"pre_ay\":" + String(preAy, 2) + ",";
  jsonPayload += "\"pre_gz\":" + String(preGz, 2) + ",";
  jsonPayload += "\"temp\":" + String(tempC, 2);
  jsonPayload += "},";

  jsonPayload += "\"opponent\":{";
  if (peerDataReceived) {
    jsonPayload += "\"id\":\"" + String(incomingPeerData.vehicle_id) + "\",";
    jsonPayload += "\"impact_g\":" + String(incomingPeerData.impact_g, 2) + ",";
    jsonPayload += "\"rot_rate\":" + String(incomingPeerData.rot_rate, 2) + ",";
    jsonPayload += "\"pre_ay\":" + String(incomingPeerData.pre_impact_ay, 2) + ",";
    jsonPayload += "\"pre_gz\":" + String(incomingPeerData.pre_impact_gz, 2) + ",";
    jsonPayload += "\"temp\":" + String(incomingPeerData.temperature, 2);
  } else {
    jsonPayload += "\"id\":\"UNREGISTERED_PEER\",";
    jsonPayload += "\"impact_g\":0.0,\"rot_rate\":0.0,\"pre_ay\":0.0,\"pre_gz\":0.0,\"temp\":0.0";
  }
  jsonPayload += "}}";

  int httpCode = http.POST(jsonPayload);
  if (httpCode > 0) Serial.println("Firebase Mutual Report Pushed Successfully!");
  http.end();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!mpu.begin()) {
    Serial.println("MPU6050 Init Failed!");
    while (1) delay(10);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  uint8_t currentChannel = WiFi.channel();

  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetPeerAddress, 6);
  peerInfo.channel = currentChannel;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("Hr Raksha Mutual V2V Telemetry Engine Active!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float axG = a.acceleration.x / 9.81;
  float ayG = a.acceleration.y / 9.81;
  float azG = a.acceleration.z / 9.81;

  float gxDeg = g.gyro.x * 57.2958;
  float gyDeg = g.gyro.y * 57.2958;
  float gzDeg = g.gyro.z * 57.2958;

  float impactG = sqrt(axG * axG + ayG * ayG + azG * azG);
  float rotRate = sqrt(gxDeg * gxDeg + gyDeg * gyDeg + gzDeg * gzDeg);

  float preAy = getMaxPreImpactAy();
  float preGz = getMaxPreImpactGz();

  pushSample(ayG, gzDeg);

  // 1. Continuous V2V Heartbeat (Broadcasting live metrics every 200 ms)
  if (millis() - lastBroadcastTime > broadcastInterval) {
    lastBroadcastTime = millis();
    
    strncpy(v2vPayload.vehicle_id, VEHICLE_ID, sizeof(v2vPayload.vehicle_id));
    v2vPayload.impact_g = impactG;
    v2vPayload.rot_rate = rotRate;
    v2vPayload.pre_impact_ay = preAy;
    v2vPayload.pre_impact_gz = preGz;
    v2vPayload.temperature = temp.temperature;
    v2vPayload.crash_event = false;

    esp_now_send(targetPeerAddress, (uint8_t *)&v2vPayload, sizeof(v2vPayload));
  }

  // 2. High-Priority Crash Detection Trigger
  if (impactG > 2.5) {
    Serial.println("\n*** MUTUAL IMPACT DETECTED ***");

    // Immediately alert peer of crash event
    v2vPayload.crash_event = true;
    v2vPayload.impact_g = impactG;
    esp_now_send(targetPeerAddress, (uint8_t *)&v2vPayload, sizeof(v2vPayload));

    // Allow 100 ms for mutual packet processing
    delay(100);

    // Push complete normalized report (Host + Opponent) to Firebase
    sendNormalizedFirebaseReport(impactG, rotRate, preAy, preGz, temp.temperature);
    delay(3000); // Debounce
  }

  delay(50);
}