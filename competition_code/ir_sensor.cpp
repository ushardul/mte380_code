#include "ir_sensor.h"
#include <Arduino.h>
#include <inttypes.h>

void init_sensor (DSensor * sens, uint8_t pin){
  sens->v[0] = 0;
  sens->v[1] = 0;
  sens->v[2] = 0;
  sens->read_pin = pin;
}

uint16_t read_raw (DSensor * sens){
  return analogRead (sens->read_pin);
}

float read_filtered (DSensor * sens){
  
  sens->v[0] = sens->v[1];
  sens->v[1] = sens->v[2];
  sens->v[2] = (6.745527388907e-2 * analogRead (sens->read_pin)) + ( -0.4128015981 * sens->v[0]) + (  1.1429805025 * sens->v[1]);
  return (sens->v[0] + sens->v[2]) +2 * sens->v[1];
}

float read_distance (DSensor * sens){
  int a_value = read_filtered (sens);
  return SCALE/(a_value + SHIFT_X) + SHIFT_Y;
}

