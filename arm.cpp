#include "arm.h"
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

Arm::Arm(){
//	reset();
}

Arm::Arm( Adafruit_Servo *arm, Adafruit_Servo *breaker ) {
	_arm_servo = arm;
	_breaker_servo = breaker;
//  Serial.print( _arm_servo == NULL ); Serial.print( " " ); Serial.println( _breaker_servo == NULL );
//	reset();
}


void Arm::setServo( Adafruit_Servo *arm, Adafruit_Servo *breaker ){
  _arm_servo = arm;
  _breaker_servo = breaker;
}

void Arm::reset(){
	(_arm_servo)->writeServo( _mods[ 0 ] );
	(_breaker_servo)->writeServo( _breaker_wait_degree );
	_current_mode = 0;
}

void Arm::toMode( int mode ) {
	if ( mode > _mods_cnt || mode < 0 )
	       return;	
	if ( mode == _current_mode ) {
		return ;
	} 
	(_arm_servo)->writeServo( _mods[ mode ] );
	_current_mode = mode;
}

void Arm::breakIt(){
  _breaker_servo->writeServo( _breaker_start_degree );
  delay( 800 );
	(_breaker_servo)->writeServo( _breaker_end_degree );
	delay( 800 );
	(_breaker_servo)->writeServo( _breaker_wait_degree );
}

int Arm::getMode(){
	return _current_mode;
}
