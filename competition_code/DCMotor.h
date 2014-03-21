#ifndef DCMotor_H
#define DCMotor_H

void set_speed_init(int pot_speed);  //Take in Pot value 0-1023 and map to motor 
void set_speed_left(int set_speed_left); // Take in speed and set motor to speed
void set_speed_right(int set_speed_right); // Take in speed and set motor to speed
void ramp_speed(int speed1, int speed2, int steps); //Take in initial
void stop_motor();

#endif
