#ifndef RUDDER_CONTROL_H
#define RUDDER_CONTROL_H
#include <Servo.h>
#include <Arduino.h>
#include <inttypes.h>

struct servo_control{
  Servo * servo;
  uint8_t pin;
  uint8_t zero_angle;
  uint8_t max_lim;
  uint8_t min_lim;
};

typedef servo_control Rudder;

void init_rudder (Rudder *, Servo *, uint8_t pin, uint8_t zero_angle, int8_t min_lim, int8_t max_lim);

void set_angle (Rudder *, int8_t);

#endif
