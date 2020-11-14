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
  _elevator_mode = 0;
  _is_breaking = false;
}

void Arm::toMode( int mode ) {
//  Serial.println( "enter: toMode" );
//  Serial.print( "to: " ); Serial.println( mode );
	if ( mode > _mods_cnt || mode < 0 )
	       return;	
//	if ( mode == _current_mode && _current_mode == 0 ) {
//		return ;
//	} 
  if( mode == 0 ) {
    _arm_servo->writeServo( _mods[ mode ] );
  }else {
    _arm_servo->writeServo( _elevator_mode_degree[ _elevator_mode ] );
//    _arm_servo->writeServo( 90 );
  }
//	(_arm_servo)->writeServo( _mods[ mode ] );
	_current_mode = mode;
// Serial.println( "exit: toMode" );
}

void Arm::setElevatorMode( int mode ) {
//  Serial.println( "enter: setElevatorMode" );
  _elevator_mode = mode;
//  Serial.print( "current:" ); Serial.println( _elevator_mode );
  toMode( _current_mode );
//  Serial.println( "exit: setElevatorMode" );
}

void Arm::breakIt(){
  if ( _is_breaking )
    return;
//  Serial.println( "enter breakIt ");
  _is_breaking = true;
  _breaker_servo->writeServo( _breaker_start_degree );
  delay( 800 );
	(_breaker_servo)->writeServo( _breaker_end_degree );
	delay( 800 );
	(_breaker_servo)->writeServo( _breaker_wait_degree );
  delay( 500 );
  _is_breaking = false;
//  Serial.println( "exit breakIt ");
}

int Arm::getMode(){
	return _current_mode;
}
