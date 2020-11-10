#include "motorcontroller.h"
#include "Arduino.h"
#include <Wire.h>

MotorController::MotorController() {
}

MotorController::MotorController( int vcc, int gnd ){
	pin_vcc = vcc, pin_gnd = gnd;
	pinMode( pin_vcc, OUTPUT );
	pinMode( pin_gnd, OUTPUT );
	vcc_value = gnd_value = 0;
	speed = 0;
	direction = MotorController::RELEASE;
}

void MotorController::run( int d ){
	direction = d;
	if ( direction = MotorController::FORWARD ) {
		digitalWrite( pin_vcc, HIGH );
		digitalWrite( pin_gnd, LOW );
	} else {
		if( direction = MotorController::BACKWARD ) {
			digitalWrite( pin_vcc, LOW );
			digitalWrite( pin_gnd, HIGH );
		} else {
			digitalWrite( pin_vcc, LOW );
			digitalWrite( pin_gnd, LOW );
		}
	}
}

void MotorController::setSpeed( int s ) {
	//TO-DO
}


