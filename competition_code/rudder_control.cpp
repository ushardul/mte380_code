#include "rudder_control.h"
#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>

void init_rudder (Rudder * rudder, Servo * servo, uint8_t pin, int16_t zero_angle, int16_t min_lim, int16_t max_lim){
  rudder ->servo = servo;
  rudder->pin = pin;
  rudder->zero_angle = zero_angle;
  rudder->min_lim = min_lim ;
  rudder->max_lim = max_lim ;
  
  (rudder->servo)->attach (rudder->pin);
  (rudder->servo)->write (zero_angle);
}

void set_angle (Rudder * rudder, int16_t angle){
  if (angle < rudder->min_lim){
    angle = rudder->min_lim;
  } else if (angle > rudder->max_lim){
    angle = rudder->max_lim;
  }
  (rudder->servo)->write (angle + rudder->zero_angle);
}
