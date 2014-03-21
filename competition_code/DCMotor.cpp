#include "DCMotor.h"
#include <Arduino.h>
#include "Constant.h"

void set_speed_left(int set_speed_left) // Take in speed and set motor to speed
{
  #ifndef MOTOROFF
  analogWrite(PIN_LEFT_MOTOR, set_speed_left);
  #endif
}

void set_speed_both(int speed_left, int speed_right) {
  #ifndef MOTOROFF
  analogWrite(PIN_LEFT_MOTOR, speed_left);
  analogWrite(PIN_RIGHT_MOTOR, speed_right);
  #endif
}

void set_speed_right(int set_speed_right) // Take in speed and set motor to speed
{
  #ifndef MOTOROFF
  analogWrite(PIN_RIGHT_MOTOR, set_speed_right);
  #endif
}

void ramp_speed(int speed1, int speed2, int steps) //Take in initial and ramp up or down to new speed
{
  int step_value = abs((speed2 - speed1) / steps);
  int newspeed = speed1;

  for (int i; i < steps; i++)
  {
    if (speed2 > speed1)
    {
      analogWrite(PIN_LEFT_MOTOR, newspeed);
      analogWrite(PIN_RIGHT_MOTOR, newspeed);
      step_value += newspeed;
    }
    else if (speed2 < speed1)
    {
      analogWrite(PIN_LEFT_MOTOR, newspeed);
      analogWrite(PIN_RIGHT_MOTOR, newspeed);
      step_value -= newspeed;
    }
    else
      return;
    steps++;
  }
}

void stop_motor()
{
  analogWrite(PIN_LEFT_MOTOR, 0);
  analogWrite(PIN_RIGHT_MOTOR, 0);
}




