#include "ir_sensor.h"

filter f;

void setup (){
  Serial.begin (9600);
}

void loop (){
  int value = read_raw (A0);
  Serial.println (f.step (value));
  delay (40); 
}
