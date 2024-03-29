#include <Servo.h>
#include "PID_v1.h"
#include "ir_sensor.h"
#include "motor_control.h"
#include "rudder_control.h"

#define PIN_FRONT_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_SPEED_POT A2
#define PIN_KP_POT A3
#define PIN_KD_POT A4
#define PIN_KI_POT A5
#define PIN_MOTOR_CONTROL 10
#define PIN_RUDDER_CONTROL 11
#define PIN_MAIN_SWITCH 9 

#define DEBOUNCE 400

static DSensor front;
static DSensor angled;

static Servo brushless;

static Rudder rudder;
static Servo rudder_servo;

static volatile uint8_t e_stop_flag = 0;
int state=LOW, reading, previous=LOW, pot_power, pot_power_range;

double reference, input, output;
long time=0;

PID rudder_control (&input, &output, &reference,1, 5, 2, DIRECT);

void setup (){
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  attachInterrupt (0, e_stop, RISING);
  Serial.begin (9600);
  init_sensor (&front, PIN_FRONT_SENSOR, IR_SENSOR_150);
  init_sensor (&angled, PIN_ANGLED_SENSOR, IR_SENSOR_80);
  init_rudder (&rudder, &rudder_servo, PIN_RUDDER_CONTROL, 100, -40, 40);
  arm_motor(&brushless, PIN_MOTOR_CONTROL);
  input = read_distance (&front);
  reference = 20;
  rudder_control.SetSampleTime (20);
  rudder_control.SetOutputLimits (-30, 30);
  rudder_control.SetMode (MANUAL);
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
      rudder_control.SetMode (MANUAL);
      set_angle (&rudder, 0);
    }
    else
    {
      state = HIGH;
      pot_power = analogRead(PIN_SPEED_POT);
      pot_power_range = map(pot_power,0,1023,0,100);
      motor_speed(&brushless, pot_power_range);
      
      int kp = analogRead (PIN_KP_POT);
      int kd = analogRead (PIN_KD_POT);
      int ki = analogRead (PIN_KI_POT);
      
      kp = map (kp, 0, 1023, 0, 10);
      kd = map (kd, 0, 1023, 0, 10);
      ki = map (ki, 0, 1023, 0, 10);
      rudder_control.SetTunings (kp, kd, ki);
      rudder_control.SetMode (AUTOMATIC);
      
      Serial.print ("{");
      Serial.print (kp);
      Serial.print (",");
      Serial.print (kd);
      Serial.print (",");
      Serial.print (ki);
      Serial.println ("}");
    }
    time=millis();
  }
  float front_sensor = read_distance (&front);
  float angle_sensor = read_distance (&angled);
  Serial.print ("{");
  Serial.print (front_sensor);
  Serial.print (",");
  Serial.print (angle_sensor);
  Serial.print ("}");
  Serial.print ("->");
  
  input = angle_sensor*0.707;
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
