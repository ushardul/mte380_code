#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <Arduino.h>
#include <inttypes.h>

// constants for 150 cm IR sensor
const float SCALE = 14220;
const float SHIFT_X = 6.682;
const float SHIFT_Y = -1.473;

// constants for 30 cm IR sensor
//const float SCALE = 2696;
//const float SHIFT_X = 1.168;
//const float SHIFT_Y = -0.6625;

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
