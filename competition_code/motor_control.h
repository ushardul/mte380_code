#ifndef motor_control
#define motor_control
#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>

void arm_motor(Servo);
int motor_speed(Servo,int);
void stop_motor(Servo);

#endif
