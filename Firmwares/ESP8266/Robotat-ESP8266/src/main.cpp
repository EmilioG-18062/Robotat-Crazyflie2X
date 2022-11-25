/**
 *    __  ___    ________
 *   / / / / |  / / ____/
 *  / / / /| | / / / __
 * / /_/ / | |/ / /_/ /
 * \____/  |___/\____/
 *
 * Robotat ESP8266 bridge firmware
 */

// ============================================================================
// LIBRERIAS
// ============================================================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

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

// ============================================================================
// PROTOTIPO DE FUNCIONES
// ============================================================================
void showMac(void);
void OnDataSent(uint8_t *mac_addr, uint8_t status);
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
void initESPNow(void);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  initESPNow();
  showMac();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &RobotatReadings, sizeof(RobotatReadings));
}

// ============================================================================
// FUNCIONES
// ============================================================================
void showMac(void) {
  Serial.println(WiFi.macAddress());
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
}

// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
}

void initESPNow(void) {
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);

  // Add peer        
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}