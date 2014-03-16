#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <Arduino.h>
#include <inttypes.h>

const uint16_t SCALE = 14220;
const uint8_t SHIFT_X = 6.682;
const uint8_t SHIFT_Y = -1.473;

struct dist_sensor {
  uint8_t read_pin;
  float v[3];
};
typedef dist_sensor DSensor;

void init_sensor (DSensor *, uint8_t pin);
uint16_t read_raw (DSensor *);
float read_filtered(DSensor *);
float read_distance (DSensor *);

#endif
