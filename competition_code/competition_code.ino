#include "ir_sensor.h"
#define PIN_SIDE_SENSOR A0
#define ANGLED_SIDE_SENSOR A1

DSensor side;
DSensor angled;

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, ANGLED_SIDE_SENSOR);
}

void loop (){
  int value = read_distance (&side);
  Serial.println (value);
  delay (40); 
}
