#include <Servo.h>
#include "PID_v1.h"
#include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_SIDE_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11
#define PIN_MAIN_SWITCH 9 
#define PIN_SPEED_POT A2

#define DEBOUNCE 200

static DSensor side;
static DSensor angled;

static Servo brushless;

static Rudder rudder;
static Servo rudder_servo;

static volatile uint8_t e_stop_flag = 0;
int state=LOW, reading, previous=LOW, pot_power, pot_power_range;

double reference, input, output;
long time=0;

PID rudder_control (&input, &output, &reference,0.5, 1, 2, DIRECT);

void setup (){
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  attachInterrupt (0, e_stop, RISING);
  Serial.begin (9600);
  init_sensor (&side, PIN_SIDE_SENSOR);
  init_sensor (&angled, PIN_ANGLED_SENSOR);
  arm_motor(&brushless, PIN_MOTOR_CONTROL);
  init_rudder (&rudder, &rudder_servo, PIN_RUDDER_CONTROL, 90, -60, 30);
  
  input = read_distance (&side);
  reference = 20;
  rudder_control.SetSampleTime (20);
  rudder_control.SetOutputLimits (-30, 30);
  rudder_control.SetMode (AUTOMATIC);
}

void loop (){
  if (e_stop_flag == 1){
    while (true);
  }
  reading = digitalRead(PIN_MAIN_SWITCH);
  if (reading==HIGH && previous==LOW && millis()-time>DEBOUNCE)
  {
    if (state == HIGH)
    {
      state = LOW;
      stop_motor(&brushless);
    }
    else
    {
      state = HIGH;
      pot_power = analogRead(PIN_SPEED_POT);
      pot_power_range = map(pot_power,0,1023,0,100);
      motor_speed(&brushless, pot_power_range);
    }
    time=millis();
  }
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

void e_stop (){
  stop_motor (&brushless);
  rudder_control.SetMode (MANUAL);
  e_stop_flag = 1; 
}
