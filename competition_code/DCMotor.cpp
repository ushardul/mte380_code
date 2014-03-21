#include "DCMotor.h"

void set_speed_init(int speedLeft, int speedRight){
}
  

/*void set_Speed_both(AF_DCMotor* left, AF_DCMotor* right, double speedLeft, double speedRight)
{
  left->setSpeed(speedLeft);
  if (speedLeft > 0)
    left->run(FORWARD);
  else
    left->run(RELEASE);  
  
  right->setSpeed(speedRight);
  if (speedRight > 0)
    right->run(FORWARD);
  else
    right->run(RELEASE); 
}*/
/* 
void set_Motor_Speed(int PIN_LEFT_MOTOR, int PIN_RIGHT_MOTOR, double speedLeft, double speedRight)
{
  analogWrite(PIN_LEFT_MOTOR,speedLeft);
  analogWrite(PIN_RIGHT_MOTOR,speedRight);
} 
*/
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

