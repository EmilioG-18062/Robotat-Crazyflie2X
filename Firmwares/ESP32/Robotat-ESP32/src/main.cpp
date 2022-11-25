/**
 *    __  ___    ________
 *   / / / / |  / / ____/
 *  / / / /| | / / / __
 * / /_/ / | |/ / /_/ /
 * \____/  |___/\____/
 *
 * Robotat ESP32 bridge firmware
 */

// ============================================================================
// LIBRERIAS
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================================================
// VARIABLES
// ============================================================================

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Define variables to store incoming readings

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
struct struct_message_pose {
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

// Create a struct_message to hold incoming sensor readings
struct_message_pose incomingReadings;
struct_message_pose RobotatReadings;

esp_now_peer_info_t peerInfo;

// ============================================================================
// PROTOTIPO DE FUNCIONES
// ============================================================================
void showMac(void);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void initESPNow(void);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  initESPNow();
  showMac();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &RobotatReadings, sizeof(RobotatReadings));
}

// ============================================================================
// FUNCIONES
// ============================================================================
void showMac(void){
  Serial.println(WiFi.macAddress());
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
}

void initESPNow(void){
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}