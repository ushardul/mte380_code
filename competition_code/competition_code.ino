 #include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11
#include <Servo.h>

DSensor side;
DSensor angled;

Servo brushless;
Servo rudder;

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, PIN_ANGLED_SENSOR);
  arm_motor(brushless);
  motor_speed(brushless,60);
  delay(10000);
  rudder.attach (PIN_RUDDER_CONTROL);
}

void loop (){
  int value = read_distance (&side);
  int rudder_pos = analogRead (A2);
  rudder_pos =  map (rudder_pos, 0, 1023, 0, 179);
  rudder.write (rudder_pos);
  Serial.println (rudder_pos);
  
  delay (40);
}
