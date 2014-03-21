#include "DCMotor.h"
#include <Arduino.h>
#include "Constant.h"

void set_speed_init(int pot_speed) //Take in Pot value 0-1023 and map to motor
{
  int motorspeed = map(pot_speed,0,1023,0,255);
  analogWrite(PIN_LEFT_MOTOR, motorspeed);
  analogWrite(PIN_RIGHT_MOTOR, motorspeed);
}

void set_speed_left(int set_speed_left) // Take in speed and set motor to speed
{
  analogWrite(PIN_LEFT_MOTOR, set_speed_left);
}

void set_speed_right(int set_speed_right) // Take in speed and set motor to speed
{
  analogWrite(PIN_RIGHT_MOTOR, set_speed_right);
}

void ramp_speed(int speed1, int speed2, int steps) //Take in initial and ramp up or down to new speed
{
  analogWrite(PIN_LEFT_MOTOR, speed1);
  analogWrite(PIN_RIGHT_MOTOR, speed1);
  for(int i; i<steps; i++)
  {
    
  }
}
    
void stop_motor();
{
  analogWrite(PIN_LEFT_MOTOR, 0);
  analogWrite(PIN_RIGHT_MOTOR, 0);
}
  



