#include "PID_v1.h"s
#include "ir_sensor.h"
#include "DCMotor.h"
#include "Constant.h"

//define to enable serial debugging
#define DEBUG

#define PROGRAM_STATE_WAITING           0
#define PROGRAM_STATE_RUNNING           1

#define MAX_MOTOR_SPEED			60
#define MIN_MOTOR_SPEED			0
#define LEFT_LIMIT			(-1 * (MAX_MOTOR_SPEED)) // limit the PID deltaSpeed change to maximum motor speed
#define RIGHT_LIMIT			(MAX_MOTOR_SPEED)

#define MIN_SIDE_DIST			20.0		// 20 cm minimum distance from wall
#define MAX_SIDE_DIST			30.0		// 40 cm max distance away from wall

#define MIN_FRONT_DIST			50.0		// 30 cm minimum forward distance from wall
#define SIDE_DIST_DESIRED		25.0		//25 cm desired distance away from wall
#define OUTER_MARGIN			5				// 2 cm margin threshold
#define INNER_MARGIN			5

static DSensor front;
static DSensor angled;

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
double sideRef, deltaSpeed;
double sideDist, frontDist;
int kp = 1;
int ki = 0;
int kd = 0;

PID speedControl(&sideDist, &deltaSpeed, &sideRef, kp, ki, kd, REVERSE);

void processMainButtonPush();

void setup() {
  pinMode(PIN_MAIN_SWITCH, INPUT);
  pinMode(PIN_SPEED_POT, INPUT);
  pinMode(PIN_LEFT_MOTOR, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR, OUTPUT);
  attachInterrupt (0, e_stop, RISING);
#ifdef DEBUG
  Serial.begin (9600);
#endif
  init_sensor (&front, PIN_FRONT_SENSOR, IR_SENSOR_150);
  init_sensor (&angled, PIN_ANGLED_SENSOR, IR_SENSOR_80);

  sideRef = SIDE_DIST_DESIRED;
  sideDist = read_distance (&angled);

  deltaSpeed = 0;
  speedControl.SetOutputLimits(LEFT_LIMIT, RIGHT_LIMIT);
  speedControl.SetSampleTime(20);
  speedControl.SetMode(AUTOMATIC);
  speedLeft = MAX_MOTOR_SPEED;
  speedRight = MAX_MOTOR_SPEED;

  set_speed_both(speedLeft, speedRight);
}

void loop() {
  bool withinThreshold;

  if (e_stop_flag == 1) {
    while (true);
  }

  processMainButtonPush();

  if (programState == PROGRAM_STATE_RUNNING) {

    frontDist = read_distance(&front);
    sideDist = read_distance(&angled) * 0.707;

    speedControl.Compute();

    //determine if side distance within margins
    if (sideDist > (SIDE_DIST_DESIRED - INNER_MARGIN) && sideDist < (SIDE_DIST_DESIRED + OUTER_MARGIN))
      withinThreshold = true;
    else
      withinThreshold = false;

    //hard code sharp turn
    if (frontDist < MIN_FRONT_DIST) {
      speedControl.SetMode(MANUAL);
      speedLeft = MIN_MOTOR_SPEED;
      speedRight = MAX_MOTOR_SPEED + 20; // + 20 so turns faster
      set_speed_both(speedLeft, speedRight);
      speedControl.SetMode(AUTOMATIC);
    }
    else {
      //if within margins go straight
      if (withinThreshold) {
        speedControl.SetMode(MANUAL);
        speedLeft = MAX_MOTOR_SPEED;
        speedRight = MAX_MOTOR_SPEED;
        set_speed_both(speedLeft, speedRight);
        speedControl.SetMode(AUTOMATIC);
      }
      //if outside margin
      else {
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

        set_speed_both(speedLeft, speedRight);
      }
    }

#ifdef DEBUG
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
#endif
  }
  else
    Serial.println("WAITING");

  delay (10);
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
      
      int kp = analogRead (PIN_KP_POT)/1023.0*10;
      int kd = 0;//analogRead (PIN_KD_POT)/1023.0*10;
      int ki = analogRead (PIN_KI_POT)/1023.0*10;
      
      //ramp_speed (0, pot_power);
      
      speedControl.SetMode (AUTOMATIC);
      speedControl.SetTunings (kp, kd, ki);
      
      Serial.print ("{");
      Serial.print (kp);
      Serial.print (",");
      Serial.print (kd);
      Serial.print (",");
      Serial.print (ki);
      Serial.println ("}");
    }
    lastDebounceTime = millis();
  }
}

void e_stop () {
  stop_motor();
  e_stop_flag = 1;
}
