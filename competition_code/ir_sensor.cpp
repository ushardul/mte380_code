#include "ir_sensor.h"
#include <Arduino.h>
#include <inttypes.h>

void init_sensor (DSensor * sens, uint8_t pin, uint8_t sensor_type){
  sens->v[0] = 0;
  sens->v[1] = 0;
  sens->v[2] = 0;
  sens->read_pin = pin;
  
  if (sensor_type == IR_SENSOR_150){
    sens->SCALE = SCALE_150;
    sens->SHIFT_X = SHIFT_X_150;
    sens->SHIFT_Y = SHIFT_Y_150;
  } else if (sensor_type == IR_SENSOR_80){
    sens->SCALE = SCALE_80;
    sens->SHIFT_X = SHIFT_X_80;
    sens->SHIFT_Y = SHIFT_Y_80;
  } else if (sensor_type == IR_SENSOR_30){
    sens->SCALE = SCALE_30;
    sens->SHIFT_X = SHIFT_X_30;
    sens->SHIFT_Y = SHIFT_Y_30;
  }
}

uint16_t read_raw (DSensor * sens){
  return analogRead (sens->read_pin);
}

// See: http://www.schwietering.com/jayduino/filtuino/
float read_filtered (DSensor * sens){
  
  sens->v[0] = sens->v[1];
  sens->v[1] = sens->v[2];
  sens->v[2] = (6.745527388907e-2 * analogRead (sens->read_pin)) + ( -0.4128015981 * sens->v[0]) + (  1.1429805025 * sens->v[1]);
  return (sens->v[0] + sens->v[2]) +2 * sens->v[1];
}

float read_distance (DSensor * sens){
  int a_value = read_raw (sens);
  return (sens->SCALE)/(a_value + sens->SHIFT_X) + (sens->SHIFT_Y);
}

