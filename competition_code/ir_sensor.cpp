#include "ir_sensor.h"
#include <Arduino.h>
#include <inttypes.h>

float read_distance (unsigned char pin){
  return 0.0;
}

uint16_t read_raw (unsigned char pin){
  return analogRead (pin);
}
