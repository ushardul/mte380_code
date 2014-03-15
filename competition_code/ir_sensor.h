#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <Arduino.h>
#include <inttypes.h>

float read_distance (unsigned char);
uint16_t read_raw (unsigned char);

//Low pass butterworth filter order=2 alpha1=0.1 
class filter
{
	public:
		filter()
		{
			v[0]=0.0;
			v[1]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (6.745527388907e-2 * x)
				 + ( -0.4128015981 * v[0])
				 + (  1.1429805025 * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};

#endif
