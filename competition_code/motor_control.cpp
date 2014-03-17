#include "motor_control.h"
#include <Arduino.h>
#include <inttypes.h>


void arm_motor(Servo * brushless, int pin){
  brushless->attach(pin);
  brushless->write(100);
  delay(10000);
  
}
void motor_speed(Servo * brushless, int power){
  int throttle = 0;
  if(power<0 || power>100)
  return;
  {
      throttle = map(power, 0, 100, 100, 179);
      brushless->write(throttle);
  }
}
void stop_motor(Servo * brushless){
  int stop_val = map(56, 0, 100, 0, 179);
  brushless->write(stop_val);
}

