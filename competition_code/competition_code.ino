#include <Servo.h>
#include "PID_v1.h"
#include "ir_sensor.h"
#include "DCMotor.h"
#include "rudder_control.h"
#include "AFMotor.h"

#define PIN_FRONT_SENSOR A0
#define PIN_ANGLED_SENSOR A1
#define PIN_SPEED_POT A2
#define PIN_KP_POT A3
#define PIN_KD_POT A4
#define PIN_KI_POT A5
#define PIN_RUDDER_CONTROL 13
#define PIN_MAIN_SWITCH 2
#define PIN_LEFT_MOTOR 1
#define PIN_RIGHT_MOTOR 2

#define DEBOUNCE 400

static DSensor front;
static DSensor angled;

static Rudder rudder;
static Servo rudder_servo, rudder_control;

AF_DCMotor left_motor(PIN_LEFT_MOTOR, MOTOR12_64KHZ); 
AF_DCMotor right_motor(PIN_RIGHT_MOTOR, MOTOR12_64KHZ);

static volatile uint8_t e_stop_flag = 0;
int state=LOW, reading, previous=LOW, pot_power, pot_power_range;
uint8_t speedLeft, speedRight

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
      set_Speed_both(motor* left_motor, motor* right_motor, 0, 0)
    }
    else
    {
      state = HIGH;
      pot_power = analogRead(PIN_SPEED_POT);
      pot_power_range = map(pot_power,0,1023,0,255);
      set_Speed_both(motor* left_motor, motor* right_motor,pot_power_range,pot_power_range)
      
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
