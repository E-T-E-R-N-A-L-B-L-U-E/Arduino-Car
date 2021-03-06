#include "elevatorcontroller.h"
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"

ElevatorController::ElevatorController(){
	current_position = 0;
}

ElevatorController::ElevatorController( Adafruit_DCMotor *left, Adafruit_DCMotor *right, Arm *arm ){
	elevator_left = left, elevator_right = right;
  _arm = arm;
	current_position = 0;
	onMoving = false;
}

ElevatorController::~ElevatorController(){
  
}

void ElevatorController::toStandardPosition( int pos_id) {
//  Serial.println("enter: toStandardPosition");
//	if ( onMoving )
//		return;
//   Serial.println("go");
	onMoving = true;
	if ( pos_id < 0 ) 
		pos_id = 0;
	if ( pos_id >= position_cnt )
		pos_id = position_cnt - 1;
	onMoving = false;
//  Serial.print( "to position: " );Serial.println( pos_id );
	moveTo( standard_position[ pos_id ] );
//  Serial.println("exit: toStandardPosition");
}

void ElevatorController::moveTo( int pos ){
//	if ( onMoving )
//		return;
//  Serial.println( "enter: moveTo" );
	onMoving = true;
  if ( pos > position_max ) pos = position_max;
	if ( pos < position_min ) pos = position_min;
	if ( pos > current_position ) {
//		elevator_left.run( FORWARD );
//		elevator_right.run( FORWARD );
//		elevator_left.setSpeed( elevator_speed_normal );
//		elevator_right.setSpeed( elevator_speed_normal );
//		delay( pos - current_position );
		runUpward( pos - current_position );
	} else {
		if( pos < current_position ) {
//			elevator_left.run( BACKWARD );
//			elevator_right.run( BACKWARD );
//			elevator_left.setSpeed( elevator_speed_normal );
//			elevator_right.setSpeed( elevator_speed_normal );
//			delay( round( to_down_rate * ( current_position - pos ) ) );
			runDownward( current_position - pos );
		} else {
			onMoving = false;
			return ;
		}
	}
//	elevator_left.setSpeed( 0 );
//	elevator_right.setSpeed( 0 );
//	elevator_left.run( RELEASE );
//	elevator_right.run( RELEASE );
  current_position = pos;
//  Serial.print( "at Position" ); Serial.println(  getStandardPosition()  );
	onMoving = false;
//  Serial.println( "exit: moveTo" );
}

void ElevatorController::runUpward( const int &runtime ){
//  Serial.println( "enter: runUpward" );
//	if ( onMoving ) 
//		return;
	if ( runtime <= 0 ) {
		return;
	}
	onMoving = true;
	int goal = current_position + runtime;
	if ( goal > position_max ) 
		goal = position_max;
//	elevator_left.run( FORWARD );
//	elevator_right.run( FORWARD );
//	elevator_left.setSpeed( elevator_speed_normal );
//	elevator_right.setSpeed( elevator_speed_normal );
//	delay( goal - current_position );
//	elevator_left.setSpeed( 0 );
//	elevator_right.setSpeed( 0 );
//	elevator_left.run( RELEASE );
//	elevator_right.run( RELEASE );
	run( goal - current_position, 1, 1 );
	current_position = goal;
//  Serial.print( "at Position" ); Serial.println( getStandardPosition() );
//  Serial.print( "current Position:" ); Serial.println( current_position );
	onMoving = false;
  _arm->setElevatorMode( getStandardPosition() );
//  Serial.println( "exit: runUpward" );
}

void ElevatorController::runDownward( const int &runtime ) {
  int real_runtime = round( to_down_rate * runtime );
//  Serial.println( "enter runDownward" );
//	if ( onMoving )
//		return 0;
	if ( real_runtime <= 0 ) {
		return;
	}
	onMoving = true;
	int goal = current_position - round( to_down_rate * real_runtime );
	if ( goal < position_min )
		goal = position_min;
//	elevator_left.run( BACKWARD );
//	elevator_right.run( BACKWARD );
//	elevator_left.setSpeed( elevator_speed_normal );
//	elevator_right.setSpeed( elevator_speed_normal );
//	delay( current_position - goal );
//	elevator_left.setSpeed( 0 );
//	elevator_right.setSpeed( 0 );
//	elevator_left.run( RELEASE );
//	elevator_right.run( RELEASE );
	run( current_position - goal, -1, 1 );
	current_position = current_position - runtime;
//  Serial.print( "at Position" ); Serial.println( getStandardPosition() );
//  Serial.print( "current Position:" ); Serial.println( current_position );
	onMoving = false;
  _arm->setElevatorMode( getStandardPosition() );
//  Serial.println( "exit runDownward");
}

// this function should only be used privately
void ElevatorController::run( const int &runtime, const int &direct, const int stop_flag = 0 ) {
	if ( runtime <= 0 ) {
		return;
	}
	if ( direct == 1 ) {
		elevator_left->run( FORWARD );
		elevator_right->run( FORWARD );
		elevator_left->setSpeed( elevator_speed_normal );
		elevator_right->setSpeed( elevator_speed_normal );
		delay( runtime );
    if ( stop_flag ) {
      elevator_left->run( BACKWARD );
      elevator_right->run( BACKWARD );
      delay( stop_speed );
    }
	} else {
		if ( direct == -1 ) {
			elevator_left->run( BACKWARD );
			elevator_right->run( BACKWARD );
			elevator_left->setSpeed( elevator_speed_normal );
			elevator_right->setSpeed( elevator_speed_normal );
			delay( runtime );
      if ( stop_flag ) {
        elevator_left->run( FORWARD );
        elevator_right->run( FORWARD );
        delay( stop_speed );
      }
		}
	}
//	elevator_left->setSpeed( 0 );
	elevator_right->setSpeed(  0 );
	elevator_left->run( BRAKE );
	elevator_right->run( BRAKE );
}

bool ElevatorController::atStandardPosition() {
	for ( int i = 0; i < position_cnt; i ++ )
		if ( standard_position[ i ] == current_position ) 
			return true;
	return false;
}

int ElevatorController::getStandardPosition() {
	if ( !atStandardPosition() )
		return -1;
	for ( int i = 0; i < position_cnt; i ++ )
		if ( standard_position[ i ] == current_position )
			return i;
	return 0;
}

int ElevatorController::getPosition() {
	return current_position;
} 

void ElevatorController::reset() {
  run( -position_max , -1 );
}
