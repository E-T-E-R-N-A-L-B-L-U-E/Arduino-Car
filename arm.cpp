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

Arm::~Arm(){
  
}


void Arm::setServo( Adafruit_Servo *arm, Adafruit_Servo *breaker ){
  _arm_servo = arm;
  _breaker_servo = breaker;
}

void Arm::reset(){
  _break_mode = 0;
	(_arm_servo)->writeServo( _mods[ 0 ] );
	(_breaker_servo)->writeServo( _breaker_wait_degree[ _break_mode ] );
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
//  while ( _breaker_servo->readDegrees() != _breaker_start_degree );
  delay( 800 );
	(_breaker_servo)->writeServo( _breaker_end_degree );
	delay( 800 );
//  while ( _breaker_servo->readDegrees() != _breaker_end_degree );
	(_breaker_servo)->writeServo( _breaker_wait_degree[ _break_mode ] );
  delay( 500 );
//  while ( _breaker_servo->readDegrees() != _breaker_wait_degree );
  _is_breaking = false;
//  Serial.println( "exit breakIt ");
}

void Arm::changeBreakMode(){
  _break_mode = ( _break_mode + 1 ) % 3;
  (_breaker_servo)->writeServo( _breaker_wait_degree[ _break_mode ] );
  delay( 300 );
}

int Arm::getMode(){
	return _current_mode;
}
