#include "ir_sensor.h"
#include "motor_control.h"
<<<<<<< HEAD
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11
=======
#define PIN_SIDE_SENSOR A0
#define ANGLED_SIDE_SENSOR A1
#include <Servo.h>
>>>>>>> 41e3b27d1105fe7ad2a2a6a7e3f53375867a7660

DSensor side;
DSensor angled;

Servo brushless;

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
<<<<<<< HEAD
  init_sensor (&angled, PIN_ANGLED_SENSOR);
=======
  init_sensor (&angled, ANGLED_SIDE_SENSOR);
  arm_motor(brushless);
  motor_speed(brushless,64);
  delay(10000);
>>>>>>> 41e3b27d1105fe7ad2a2a6a7e3f53375867a7660
}

void loop (){
  int value = read_distance (&side);
  int rudder_pos = analogRead (A2);
  Serial.println (rudder_pos);
  delay (40); 
}
