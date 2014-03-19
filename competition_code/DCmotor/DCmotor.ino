#include <AFMotor.h>

#define PIN_LEFT_MOTOR 1
#define PIN_RIGHT_MOTOR 2

AF_DCMotor left_motor(PIN_LEFT_MOTOR, MOTOR12_64KHZ); 
AF_DCMotor right_motor(PIN_RIGHT_MOTOR, MOTOR12_64KHZ);
 
void setup() {
  Serial.begin(9600)
  motor.setSpeed(255);     // set the speed to 200/255
}
 
void loop() {
  Serial.print("tick");
  
  motor.run(FORWARD);      // turn it on going forward
  delay(1000);
 
  Serial.print("tock");
  motor.run(BACKWARD);     // the other way
  delay(1000);
  
  Serial.print("tack");
  motor.run(RELEASE);      // stopped
  delay(1000);
}
