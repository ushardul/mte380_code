#include "ir_sensor.h"
#include "motor_control.h"
#define PIN_SIDE_SENSOR A0
#define ANGLED_SIDE_SENSOR A1
#include <Servo.h>

DSensor side;
DSensor angled;

Servo brushless;

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, ANGLED_SIDE_SENSOR);
  arm_motor(brushless);
  motor_speed(brushless,60);
  delay(15000);
  stop_motor(brushless);
}

void loop (){
  int value = read_distance (&side);
  Serial.println (value);
  delay (40); 
}
