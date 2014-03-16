#ifndef motor_control
#define motor_control
#include <Arduino.h>
#include <inttypes.h>

void arm_motor();
int motor_speed(int power);
void stop_motor();

#endif
