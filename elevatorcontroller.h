#ifndef __ELEVATORCONTROLLER_H__
#define __ELEVATORCONTROLLER_H__

#include "Arduino.h"
#include "arm.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"

class ElevatorController {
	const int upward_speed = 430;    // time needed to run upward 5 centimeter
//	const int downward_speed = 144;    // time needed to run downward 5 centimeter
	const int elevator_speed_normal = 100;
	const double to_down_rate = 0.78;
  const double to_stop_rate = 0.1;
  const int downward_speed = round( to_down_rate * upward_speed );    // time needed to run downward 5 centimeter
  const int stop_speed = round( to_stop_rate * upward_speed );

	const int position_min = 0;
	const int position_cnt = 4;
  const int position_max = ( position_cnt - 1 ) * upward_speed + 100;
	const int standard_position[ 4 ] = {
		0,
		1 * upward_speed,
		2 * upward_speed,
    2 * upward_speed + 1
	};
	int current_position;
	bool onMoving;
	Adafruit_DCMotor *elevator_left, *elevator_right;
  Arm *_arm;

	void run( const int &runtime, const int &direct, const int stop_flag );


public:
	ElevatorController( );
	ElevatorController( Adafruit_DCMotor *left, Adafruit_DCMotor *right, Arm *arm );


	void toStandardPosition( int pos_id );

	/*
	 * move to a standard position
	 * const int &pos : the standard position that need to move to
	 */
	void moveTo( int pos );

	void runUpward( const int &runtime );

	void runDownward( const int &runtime );
// in the functions above, the para 'runtime' is a length item
//
//  in the function below, the para 'runtime' is time
//  so the uppper bound and the lower bound will not work

	
	bool atStandardPosition();

	int getStandardPosition();

	int getPosition();

  void reset();
	
};



#endif
