#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <Arduino.h>
#include <inttypes.h>

#define IR_SENSOR_150 0
#define IR_SENSOR_80 1
#define IR_SENSOR_30 2

// constants for 150 cm IR sensor
const float SCALE_150 = 14220;
const float SHIFT_X_150 = 6.682;
const float SHIFT_Y_150 = -1.473;

// constants for 80 cm IR sensors
const float SCALE_80 = 6495;
const float SHIFT_X_80 = -3.948;
const float SHIFT_Y_80 = -4.271;

// constants for 30 cm IR sensor
const float SCALE_30 = 2696;
const float SHIFT_X_30 = 1.168;
const float SHIFT_Y_30 = -0.6625;

struct dist_sensor {
  uint8_t read_pin;
  float v[3];
  float SCALE;
  float SHIFT_X;
  float SHIFT_Y;
};
typedef dist_sensor DSensor;

void init_sensor (DSensor *, uint8_t, uint8_t);
uint16_t read_raw (DSensor *);
float read_filtered(DSensor *);
float read_distance (DSensor *);

#endif
