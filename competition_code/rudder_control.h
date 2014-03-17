#ifndef RUDDER_CONTROL_H
#define RUDDER_CONTROL_H
#include <Servo.h>
#include <Arduino.h>
#include <inttypes.h>

struct servo_control{
  Servo * servo;
  uint8_t pin;
  int16_t zero_angle;
  int16_t max_lim;
  int16_t min_lim;
};

typedef servo_control Rudder;

void init_rudder (Rudder *, Servo *, uint8_t pin, int16_t zero_angle, int16_t min_lim, int16_t max_lim);

void set_angle (Rudder *, int16_t);

#endif
