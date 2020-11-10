#include "elevatorcontroller.h"
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"

ElevatorController::ElevatorController(){
	current_position = 0;
}

ElevatorController::ElevatorController( Adafruit_DCMotor *left, Adafruit_DCMotor *right ){
	elevator_left = left, elevator_right = right;
	current_position = 0;
	onMoving = false;
}

void ElevatorController::toStandardPosition( int pos_id) {
//	if ( onMoving )
//		return;
//   Serial.println("go");
	onMoving = true;
	if ( pos_id < 0 ) 
		pos_id = 0;
	if ( pos_id >= position_cnt )
		pos_id = position_cnt - 1;
	onMoving = false;
	moveTo( standard_position[ pos_id ] );
}

void ElevatorController::moveTo( int pos ){
//	if ( onMoving )
//		return;
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
			runDownward( round( to_down_rate * ( current_position - pos ) ) );
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
	onMoving = false;
}

void ElevatorController::runUpward( const int &runtime ){
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
	onMoving = false;
}

void ElevatorController::runDownward( const int &runtime ) {
//	if ( onMoving )
//		return 0;
	if ( runtime <= 0 ) {
		return;
	}
	onMoving = true;
	int goal = current_position - round( to_down_rate * runtime );
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
	current_position = goal;
	onMoving = false;
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
	elevator_left->setSpeed( 0 );
	elevator_right->setSpeed(  0 );
	elevator_left->run( RELEASE );
	elevator_right->run( RELEASE );
}

bool ElevatorController::atStandardPosition() {
	for ( int i = 0; i < 3; i ++ )
		if ( standard_position[ i ] == current_position ) 
			return true;
	return false;
}

int ElevatorController::getStandardPosition() {
	if ( !atStandardPosition() )
		return -1;
	for ( int i = 0; i < 3; i ++ )
		if ( standard_position[ i ] == current_position )
			return i;
	return 0;
}

int ElevatorController::getPosition() {
	return current_position;
} 
