
#include "PID_v1.h"
#include "ir_sensor.h"
#include "DCMotor.h"
#include "Constant.h"
#include "NewPing.h"

//define to enable serial debugging
#define DEBUG

#define PROGRAM_STATE_WAITING           0
#define PROGRAM_STATE_RUNNING           1

#define MAX_MOTOR_SPEED                 100
#define MAX_MOTOR_SPEED_RIGHT		(MAX_MOTOR_SPEED)
#define MAX_MOTOR_SPEED_LEFT		(MAX_MOTOR_SPEED+1)
#define SPEED_CHANGE                    30
#define MIN_MOTOR_SPEED			0
#define LEFT_LIMIT			(-1 * (MAX_MOTOR_SPEED)) // limit the PID deltaSpeed change to maximum motor speed
#define RIGHT_LIMIT			(MAX_MOTOR_SPEED)

#define PID_KP                          1
#define PID_KI                          1
#define PID_KD                          1

#define MIN_SIDE_DIST			20.0		// 20 cm minimum distance from wall
#define MAX_SIDE_DIST			30		// 40 cm max distance away from wall

#define MIN_FRONT_DIST			65		// 30 cm minimum forward distance from wall
#define SIDE_DIST_DESIRED		20		//25 cm desired distance away from wall
#define OUTER_MARGIN			2				// 2 cm margin threshold
#define INNER_MARGIN			2

#define TURN_SAMPLES_REQ                0

//static DSensor front;
static DSensor angled;

NewPing sonar(PIN_USONIC_TRIGGER, PIN_USONIC_ECHO, USONIC_MAX_DIST);

static volatile uint8_t e_stop_flag = 0;

int programState = PROGRAM_STATE_WAITING; // 0 - waiting for button, 1 - running

//button stuff
int buttonReading;
int previousButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 100;

//motor speeds
double speedLeft, speedRight;

//PID params and sensor values
double sideRef = SIDE_DIST_DESIRED, deltaSpeed = 0;
double sideDist, frontDist;

PID speedControl(&sideDist, &deltaSpeed, &sideRef, 1, 0, 0, REVERSE);

void processMainButtonPush();

uint8_t turn_samples = 0;

void setup() {
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  pinMode(PIN_LEFT_MOTOR, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR, OUTPUT);
  attachInterrupt (0, e_stop, RISING);
#ifdef DEBUG
  Serial.begin (9600);
#endif
  //init_sensor (&front, PIN_FRONT_SENSOR, IR_SENSOR_150);
  init_sensor (&angled, PIN_ANGLED_SENSOR, IR_SENSOR_80);

  speedControl.SetOutputLimits(LEFT_LIMIT, RIGHT_LIMIT);
  speedControl.SetSampleTime(10);
  speedControl.SetMode(AUTOMATIC);
  
  stop_motor();
  
  int n = 0;
  while (n < 10){
    frontDist = sonar.ping_cm();
    if (frontDist > 10 && frontDist < 15)
      n++;
    Serial.println(frontDist);
    delay (50);
  }
  Serial.println("STARTING");
  programState = PROGRAM_STATE_RUNNING;
}

void loop() {
  bool withinThreshold;
  
  if (e_stop_flag == 1) {
    while (true);
  }

  processMainButtonPush();

  if (programState == PROGRAM_STATE_RUNNING) {
    
    //set_speed_both(MAX_MOTOR_SPEED_LEFT,MAX_MOTOR_SPEED_RIGHT);}
    frontDist = sonar.ping_cm();//read_distance(&front);
    sideDist = read_distance(&angled);
    
    speedControl.Compute();

    //determine if side distance within margins
    if (sideDist > (SIDE_DIST_DESIRED - INNER_MARGIN) && sideDist < (SIDE_DIST_DESIRED + OUTER_MARGIN))
      withinThreshold = true;
    else
      withinThreshold = false;

    //hard code sharp turn
    if (frontDist < MIN_FRONT_DIST && frontDist > 20) {
      turn_samples ++;
      if (turn_samples > TURN_SAMPLES_REQ){
        speedControl.SetMode(MANUAL);
        speedLeft = 5;
        speedRight = MAX_MOTOR_SPEED_RIGHT + SPEED_CHANGE; // + 20 so turns faster
        set_speed_both(speedLeft, speedRight);
        Serial.println ("Turning");
        speedControl.SetMode(AUTOMATIC);
        delay (1000);
        turn_samples = 0;
      }
    }
    else {
      //if within margins go straight
      if (withinThreshold) {
        speedControl.SetMode(MANUAL);
        speedLeft = MAX_MOTOR_SPEED_LEFT;
        speedRight = MAX_MOTOR_SPEED_RIGHT;
        set_speed_both(speedLeft, speedRight);
        speedControl.SetMode(AUTOMATIC);
      }
      //if outside margin
      else {
        speedLeft += deltaSpeed;
        speedRight -= deltaSpeed;

        if (speedLeft < MIN_MOTOR_SPEED)
          speedLeft = MIN_MOTOR_SPEED;
        if (speedLeft > MAX_MOTOR_SPEED_LEFT)
          speedLeft = MAX_MOTOR_SPEED_LEFT;

        if (speedRight < MIN_MOTOR_SPEED)
          speedRight = MIN_MOTOR_SPEED;
        if (speedRight > MAX_MOTOR_SPEED_RIGHT)
          speedRight = MAX_MOTOR_SPEED_RIGHT;

        set_speed_both(speedLeft, speedRight);
      }
    }

#ifdef DEBUG
    //Serial.print("Left Motor Speed: ");
    //Serial.print(speedLeft);
    //Serial.print("\t Right Motor Speed: ");
    //Serial.print(speedRight);
    //Serial.print("\t deltaSpeed: ");
    //Serial.print(deltaSpeed);
    //Serial.print("\t Front Dist: ");
    //Serial.print(frontDist);
    Serial.print("\t Side Dist: ");
    Serial.print(sideDist);
    //Serial.println("\t");
    Serial.println ("");
#endif
  }
  /*else {
    Serial.println("WAITING");
  }*/
  delay (20);
}

void processMainButtonPush() {
  buttonReading = digitalRead(PIN_MAIN_SWITCH);
  if (buttonReading==HIGH && (millis()-lastDebounceTime)>debounceDelay)
  {
    if (programState == PROGRAM_STATE_RUNNING)
    {
      programState = PROGRAM_STATE_WAITING;
      stop_motor();
    }
    else
    { 
      programState = PROGRAM_STATE_RUNNING;
      //pot_power = analogRead(PIN_SPEED_POT);
      //pot_power = map(pot_power,0,1023,0,255);
      
      //double kp = analogRead(PIN_KP_POT);
      //double kd = 0;//analogRead (PIN_KD_POT)/1023.0*10;
      //double ki = analogRead(PIN_KI_POT);
      
      //kp = map(kp, 0, 1023, 1, 100)*0.01;
      //ki = map(ki, 0, 1023, 0, 5);
      
      double kp = PID_KP;
      double ki = PID_KI;
      double kd = PID_KD;
      
      //ramp_speed (0, pot_power);
      
      speedControl.SetMode (AUTOMATIC);
      speedControl.SetTunings (kp, ki, kd);
      
      //Serial.print ("{");
      //Serial.print (kp);
      //Serial.print (",");
      //Serial.print (kd);
      //Serial.print (",");
      //Serial.print (ki);
      //mSerial.println ("}");
    }
    lastDebounceTime = millis();
  }
}

void e_stop () {
  stop_motor();
  e_stop_flag = 1;
}
