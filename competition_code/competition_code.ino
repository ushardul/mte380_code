#include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11

DSensor side;
DSensor angled;

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, PIN_ANGLED_SENSOR);
}

void loop (){
  int value = read_distance (&side);
  int rudder_pos = analogRead (A2);
  Serial.println (rudder_pos);
  delay (40); 
}
