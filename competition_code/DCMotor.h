#ifndef DCMotor_H
#define DCMotor_H

void set_speed_init(int); //Take in Pot value 0-1023 and map to motor 
void set_speed_left(int); // Take in speed and set motor to speed
void ramp_speed(int, int); //Take in initial
void stop_motor();

#endif
