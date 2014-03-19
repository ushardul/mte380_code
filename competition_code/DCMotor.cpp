#include <AFMotor.h>

void set_Speed_both(motor* left, motor* right, uint8_t speedLeft, uint8_t speedRight)
{
  left->setSpeed(speedLeft);
  right->setSpeed(speedRight);
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
