#include "motor_control.h"
#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>

Servo brushless;

void arm_motor(){
  brushless.attach(9);
  brushless.write(180);
  delay(10000);
  brushless.write(0);
  
}
int motorspeed(int power){
int throttle = 0;
  arm_motor();
if(power<0 || power>100)
Serial.println("% power not in range");
else
  {
      throttle = map(power, 0, 100, 0, 179);
      brushless.write(throttle);
}
void stopmotor(){
  brushless.write(0);
}

