#include <stdint.h>

#define RSV     B00000000
#define START   0xFF
#define STM32   B00000001
#define ESP32   B00010000

#define POSE    B00010000
#define SETP    B00010001

#define LP      B01000000

struct cpxPacket {
  int length;
  int source;
  int destination;
  int function;
  uint8_t data[100];
};