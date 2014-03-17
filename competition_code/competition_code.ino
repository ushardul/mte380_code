#include <Servo.h>
#include "PID_v1.h"
#include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11
#define PIN_MAIN_SWITCH 2 
#define PIN_SPEED_POT A2

static DSensor side;
static DSensor angled;

static Servo brushless;

static Rudder rudder;
static Servo rudder_servo;

int state, reading, previous=LOW, pot_power;
double reference, input, output;
long time=0, debounce = 200;

PID rudder_control (&input, &output, &reference,0.5, 1, 2, DIRECT);

void setup (){
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  Serial.begin (9600);
  pot_power = analogRead(PIN_SPEED_POT);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, PIN_ANGLED_SENSOR);
  arm_motor(&brushless, PIN_MOTOR_CONTROL);
  input = read_distance (&side);
  reference = 20;
  init_rudder (&rudder, &rudder_servo, PIN_RUDDER_CONTROL, 90, -60, 30);
  rudder_control.SetMode (AUTOMATIC);
}

void loop (){
  reading = digitalRead(PIN_MAIN_SWITCH);
  if (reading==HIGH && previous==LOW && millis()-time>debounce)
  {
    motor_speed(&brushless, pot_power);
    time=millis();
  }
  else
  stop_motor(&brushless);    
  float side_sensor = read_distance (&side);
  float angle_sensor = read_distance (&angled);
  Serial.print ("{");
  Serial.print (side_sensor);
  Serial.print (",");
  Serial.print (angle_sensor);
  Serial.print ("}");
  Serial.print ("->");
  
  input = side_sensor;
  rudder_control.Compute ();  
  set_angle (&rudder, output);
  
  Serial.println (output);
  delay (40);
}
