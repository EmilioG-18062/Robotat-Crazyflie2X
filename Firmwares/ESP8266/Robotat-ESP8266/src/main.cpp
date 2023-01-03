/**
 *    __  ___    ________
 *   / / / / |  / / ____/
 *  / / / /| | / / / __
 * / /_/ / | |/ / /_/ /
 * \____/  |___/\____/
 *
 * Robotat ESP8266 bridge firmware
 */

/** Handles incoming JSON messages and transform
 *  them to CPX format then transmits via UART 
 *  to a Crazyflie
 */

// ============================================================================
// LIBRERIAS
// ============================================================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <structures.h>

// ==============================================
// VARIABLES
// ==============================================

const char* ssid     =  "Robotat";
const char* password =  "iemtbmcit116";
const char* host     =  "192.168.50.200";
const uint16_t port  =  1883;

WiFiClient client;
cpxPacket packet;

byte canSend      = true;
byte incomingByte = 1;
byte lastByte     = 0x00;

StaticJsonDocument<256> doc;

bool needJson = true;

// ==============================================
// PROTOTIPO DE FUNCIONES
// ==============================================
void startWifi(const char* ssid, const char* password);
void connectTCP(const char*host, const uint16_t port);
void sendToCF(cpxPacket packet);
void HandlerPosePackage(void);

// ==============================================
// SETUP
// ==============================================
void setup() {
  Serial.begin(115200);

  packet.source = ESP32;
  packet.destination = STM32;

  startWifi(ssid, password);
  connectTCP(host, port);
  client.print("1013");
}

// ==============================================
// LOOP
// ==============================================
void loop() {
  if(canSend == true){
    if (client.available() > 0){
      //read back one line from the server
      String line = client.readStringUntil('}');
      line += "}";

      DeserializationError err = deserializeJson(doc, line);
      if(err.code() == DeserializationError::Ok){
        HandlerPosePackage();
        sendToCF(packet);
        canSend = false;
      }
    }
  }

  else{
    incomingByte = Serial.read();
    if(incomingByte == 0x00 && lastByte == START){
      canSend = true;
      client.print("1013");
    }
    lastByte = incomingByte;
  }
}

// ==============================================
// FUNCIONES
// ==============================================
void sendToCF(cpxPacket packet){
  Serial.write(START);
  Serial.write(packet.length+2);
  Serial.write(RSV | LP | packet.source | packet.destination);
  Serial.write(packet.function);
  Serial.write(packet.data, packet.length);
}

void HandlerPosePackage(void){
  double pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pose[0] = (double)doc["data"][0];
  pose[1] = (double)doc["data"][1];
  pose[2] = (double)doc["data"][2];
  pose[3] = (double)doc["data"][3];
  pose[4] = (double)doc["data"][4];
  pose[5] = (double)doc["data"][5];
  pose[6] = (double)doc["data"][6];

  //Pruebas
  Serial.println();
  Serial.print("x = ");
  Serial.println(pose[0]);
  Serial.print("y = ");
  Serial.println(pose[1]);
  Serial.print("z = ");
  Serial.println(pose[2]); 

  memcpy(packet.data, (uint8_t*)pose, 7*sizeof(double));
  packet.length = sizeof(pose);
  packet.function = POSE;
}

void startWifi(const char* ssid, const char* password){
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void connectTCP(const char*host, const uint16_t port){
  while (!client.connect(host, port)) {
    Serial.println("Connection failed.");
    Serial.println("Waiting 5 seconds before retrying...");
    delay(5000);
  }
}
