#include "PID_v1.h"s
#include "ir_sensor.h"
#include "DCMotor.h"
#include "Constant.h"

static DSensor front;
static DSensor angled;

static volatile uint8_t e_stop_flag = 0;
int state=LOW, reading, previous=LOW, pot_power;
double speedLeft, speedRight;

double sideRef, sideDist, deltaSpeed, frontDist;

double reference, input, output;
long time=0;

const double AGGKP = 0.45;
const double CONSKP = 0.1;
const double AGGKD= 0.2;
const double CONSKD = 0;
const double AGGKI = 0;
const double CONSKI = 0;

PID speedControl(&sideDist,&deltaSpeed,&sideRef, AGGKP,AGGKI,AGGKD,REVERSE);

const int MAX_MOTOR_SPEED = 60;
const int MIN_MOTOR_SPEED = 0;
const double LEFT_LIMIT = -1*MAX_MOTOR_SPEED; // limit the PID deltaSpeed change to maximum motor speed
const double RIGHT_LIMIT = MAX_MOTOR_SPEED;

const double MIN_SIDE_DIST = 20.0; // 20 cm minimum distance from wall
const double MAX_SIDE_DIST = 30.0; // 40 cm max distance away from wall

const double MIN_FRONT_DIST = 50.0; // 30 cm minimum forward distance from wall
const double SIDE_DIST_DESIRED = 25.0; //25 cm desired distance away from wall
const double OUTER_MARGIN = 8.0; // 2 cm margin threshold
const double INNER_MARGIN = 8.0; 
int thresholdRegion = 0; //

void setup (){
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  pinMode(PIN_LEFT_MOTOR, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR, OUTPUT);
  attachInterrupt (0, e_stop, RISING);
  Serial.begin (9600);
  init_sensor (&front, PIN_FRONT_SENSOR, IR_SENSOR_150);
  init_sensor (&angled, PIN_ANGLED_SENSOR, IR_SENSOR_80);

  sideRef = SIDE_DIST_DESIRED;
  sideDist = read_distance (&angled);

  deltaSpeed = 0;
  speedControl.SetOutputLimits(LEFT_LIMIT,RIGHT_LIMIT);
  speedControl.SetSampleTime(20);
  speedControl.SetMode(AUTOMATIC);  
  speedLeft = MAX_MOTOR_SPEED;
  speedRight = MAX_MOTOR_SPEED;
 
  analogWrite(PIN_LEFT_MOTOR,speedLeft);
  analogWrite(PIN_RIGHT_MOTOR,speedRight);
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
      //set_Speed_both(&left_motor, &right_motor, 0, 0);
    }
    else
    {
      state = HIGH;
      pot_power = analogRead(PIN_SPEED_POT);
      pot_power = map(pot_power,0,1023,0,255);
      
      int kp = analogRead (PIN_KP_POT);
      int kd = analogRead (PIN_KD_POT);
      int ki = analogRead (PIN_KI_POT);
      
      kp = map (kp, 0, 1023, 0, 10);
      kd = map (kd, 0, 1023, 0, 10);
      ki = map (ki, 0, 1023, 0, 10);
      
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
  
  frontDist =  read_distance (&front);
  sideDist =  read_distance (&angled);

  if (sideDist > SIDE_DIST_DESIRED - INNER_MARGIN && sideDist < SIDE_DIST_DESIRED + OUTER_MARGIN) {
    speedControl.SetTunings(CONSKP, CONSKI, CONSKD);
    thresholdRegion = 1;
  }
  else {
    speedControl.SetTunings(AGGKP, AGGKI, AGGKD);
    thresholdRegion = 0;
  }

  speedControl.Compute();
  if (frontDist < MIN_FRONT_DIST) { 
    speedControl.SetMode(MANUAL);
    speedLeft = MIN_MOTOR_SPEED;
    speedRight = MAX_MOTOR_SPEED;
    //set_Motor_Speed(PIN_LEFT_MOTOR,PIN_RIGHT_MOTOR,speedLeft,speedRight);
    //set_Speed_both(&left_motor, &right_motor,speedLeft,speedRight);
    analogWrite(PIN_LEFT_MOTOR,speedLeft);
    analogWrite(PIN_RIGHT_MOTOR,speedRight);
    speedControl.SetMode(AUTOMATIC);
  } else {
    if (thresholdRegion) {
    speedControl.SetMode(MANUAL);
    speedLeft = MAX_MOTOR_SPEED;
    speedRight = MAX_MOTOR_SPEED;
    //set_Motor_Speed(PIN_LEFT_MOTOR,PIN_RIGHT_MOTOR,speedLeft,speedRight);
    //set_Speed_both(&left_motor, &right_motor,speedLeft,speedRight);
    analogWrite(PIN_LEFT_MOTOR,speedLeft);
    analogWrite(PIN_RIGHT_MOTOR,speedRight);
    speedControl.SetMode(AUTOMATIC);
    }else {
       speedLeft += deltaSpeed;
       speedRight -= deltaSpeed;
       if (speedLeft < MIN_MOTOR_SPEED)
         speedLeft = MIN_MOTOR_SPEED;
       if (speedLeft > MAX_MOTOR_SPEED)
         speedLeft = MAX_MOTOR_SPEED;
       if (speedRight < MIN_MOTOR_SPEED)
         speedRight = MIN_MOTOR_SPEED;
       if (speedRight > MAX_MOTOR_SPEED)
         speedRight = MAX_MOTOR_SPEED;
         
       analogWrite(PIN_LEFT_MOTOR,speedLeft);
       analogWrite(PIN_RIGHT_MOTOR,speedRight);
    }
  }
  Serial.print("Left Motor Speed: ");
  Serial.print(speedLeft);
  Serial.print("\t Right Motor Speed: ");
  Serial.print(speedRight);
  Serial.print("\t deltaSpeed: ");
  Serial.print(deltaSpeed);
  Serial.print("\t Front Dist: ");
  Serial.print(frontDist);
  Serial.print("\t Side Dist: ");
  Serial.println(sideDist);
  delay (10);
}

void e_stop (){
  
  analogWrite(PIN_LEFT_MOTOR,0);
  analogWrite(PIN_RIGHT_MOTOR,0);
  e_stop_flag = 1; 
}
