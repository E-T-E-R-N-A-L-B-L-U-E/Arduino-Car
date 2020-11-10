#include "Arduino.h"
#include <Wire.h>

class MotorController {
private:
	int pin_vcc, pin_gnd;
	int vcc_value, gnd_value;
	int speed;
	int direction;
public:
	const int FORWARD = 1, BACKWARD = 2, RELEASE = 4;
	MotorController();
	MotorController( int vcc, int gnd );
	void run( int direction );
	void setSpeed( int s );
};
