#include "motor_control.h"
#include <Arduino.h>
#include <inttypes.h>


void arm_motor(Servo brushless){
  brushless.attach(10);
  brushless.write(100);
  delay(10000);
  //brushless.write();
  
}
void motor_speed(Servo brushless, int power){
  int throttle = 0;
  //arm_motor();
  if(power<0 || power>100)
  Serial.println("% power not in range");
  else
  {
      throttle = map(power, 0, 100, 0, 179);
      brushless.write(throttle);
      Serial.println(throttle);
}
}
void stop_motor(Servo brushless){
  brushless.write(100);
}

