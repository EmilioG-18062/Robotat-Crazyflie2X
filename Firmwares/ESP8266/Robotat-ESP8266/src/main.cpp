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

// ==============================================
// VARIABLES
// ==============================================
#define RSV     B00000000
#define START   0xFF
#define STM32   B00000001
#define ESP32   B00010000

#define POSE B00010000
#define SETP B00010001

uint8 LP = B01000000;

const char* ssid = "Robotat";
const char* password =  "iemtbmcit116";
const char* host = "192.168.50.200";
const uint16_t port = 1883;
WiFiClient client;

char msj[64];

struct cpxPacket {
  int length;
  int source;
  int destination;
  int function;
  uint8_t data[100];
};

cpxPacket packet;

byte canSend = true;
byte incomingByte = 1;
byte lastByte = 0x00;
byte packetLenght = 0x00;

double pose[7] = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0};

StaticJsonDocument<256> doc;

unsigned long lastTimeSend = 0;
bool needJson = true;

// ==============================================
// PROTOTIPO DE FUNCIONES
// ==============================================
void startWifi(const char* ssid, const char* password);
void connectTCP(const char*host, const uint16_t port);

// ==============================================
// SETUP
// ==============================================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  packet.source = ESP32;
  packet.destination = STM32;

  startWifi(ssid, password);
  connectTCP(host, port);
}

// ==============================================
// LOOP
// ==============================================
void loop() {
  if(canSend == true){

    if(needJson == true){
      client.print("102");
      needJson = false;
    } 

    if (client.available() > 0){
      //read back one line from the server
      String line = client.readStringUntil('}');
      needJson = true;
      line = line + "}";

      DeserializationError err = deserializeJson(doc, line);
      if(err.code() == DeserializationError::Ok){
        pose[0] = (double)doc["data"][0];
        pose[1] = (double)doc["data"][1];
        pose[2] = (double)doc["data"][2];
        pose[3] = (double)doc["data"][3];
        pose[4] = (double)doc["data"][4];
        pose[5] = (double)doc["data"][5];
        pose[6] = (double)doc["data"][6];

        memcpy(packet.data, (uint8_t*)pose, 7*sizeof(double));
        packet.length = sizeof(pose);
        packet.function = POSE;
        sendToCF(packet);
        lastTimeSend = millis();
        canSend = false;
      }
    }
    //Pruebas
    //memcpy(packet.data, (uint8_t*)pose, 7*sizeof(double));
    //packet.length = sizeof(pose);
    //packet.function = POSE;
    //sendToCF(packet);
    //canSend = false;
  }

  else{
    incomingByte = Serial.read();
    if(incomingByte == 0x00 && lastByte == START){
      canSend = true;
      Serial.println();
      Serial.print("x = ");
      Serial.println(pose[0]);
      Serial.print("y = ");
      Serial.println(pose[1]);
      Serial.print("z = ");
      Serial.println(pose[2]);
    }
    lastByte = incomingByte;
  }

  //if((millis() - lastTimeSend) > 100){
  //  lastByte = 0;
  //  incomingByte = 1;
  //  lastTimeSend =  millis();
  //  canSend = true;
  //}
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
