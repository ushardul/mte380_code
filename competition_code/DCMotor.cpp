#include <AFMotor.h>
#include "DCMotor.h"

void set_Speed_both(AF_DCMotor* left, AF_DCMotor* right, uint8_t speedLeft, uint8_t speedRight)
{
  left->setSpeed(speedLeft);
  left->run(FORWARD);
  right->setSpeed(speedRight);
  right->run(FORWARD);
}
 
 

//  motor.run(FORWARD);      // turn it on going forward
//  delay(1000);
// 
//  Serial.print("tock");
//  motor.run(BACKWARD);     // the other way
//  delay(1000);
//  
//  Serial.print("tack");
//  motor.run(RELEASE);      // stopped
//  delay(1000);

