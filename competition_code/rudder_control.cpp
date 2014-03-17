#include "rudder_control.h"
#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>

void init_rudder (Rudder * rudder, Servo * servo, uint8_t pin, uint8_t zero_angle, int8_t min_lim, int8_t max_lim){
  rudder ->servo = servo;
  rudder->pin = pin;
  rudder->zero_angle = zero_angle;
  rudder->min_lim = min_lim + zero_angle;
  rudder->max_lim = max_lim + zero_angle;
  
  (rudder->servo)->attach (rudder->pin);
  (rudder->servo)->write (zero_angle);
}

void set_angle (Rudder * rudder, int8_t angle){
  int16_t new_angle = angle + rudder->zero_angle;
  if (new_angle > rudder->min_lim && new_angle < rudder->max_lim){
    (rudder->servo)->write (new_angle);
  }
}
