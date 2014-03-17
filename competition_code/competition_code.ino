#include <Servo.h>
#include "PID_v1.h"
#include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11

static DSensor side;
static DSensor angled;

static Servo brushless;

static Rudder rudder;
static Servo rudder_servo;


double reference, input, output;
PID rudder_control (&input, &output, &reference,100, 0, 0, DIRECT);

void setup (){
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, PIN_ANGLED_SENSOR);
  //arm_motor(brushless);
  motor_speed(brushless,60);
  //delay(10000);
  input = read_distance (&side);
  reference = 20;
  init_rudder (&rudder, &rudder_servo, PIN_RUDDER_CONTROL, 90, -50, 50);
  rudder_control.SetMode (AUTOMATIC);
}

void loop (){
  float side_sensor = read_distance (&side);
  float angle_sensor = read_distance (&angled);
  Serial.print ("{");
  Serial.print (side_sensor);
  Serial.print (",");
  Serial.print (angle_sensor);
  Serial.print ("}");
  Serial.print ("->");
  
  input = side_sensor;
  output = rudder_control.Compute ();  
  set_angle (&rudder, output);
  
  Serial.println (output);
  delay (40);
}
