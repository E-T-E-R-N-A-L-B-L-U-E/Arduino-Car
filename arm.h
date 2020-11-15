#ifndef __ARM_H__
#define __ARM_H__

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Arm{
//  Adafruit_MotorShield AFMS;
	Adafruit_Servo *_arm_servo;
	Adafruit_Servo *_breaker_servo;
	int _arm_degree, _breaker_degree;
	int _current_mode;
	int _mods_cnt = 2;
	int _mods[ 2 ] = { 50, 140 };
	int _breaker_start_degree = 150;
	int _breaker_end_degree = 30;
  int _breaker_wait_degree[ 3 ] = { 90, 60, 150 };
  int _elevator_mode;
  int _elevator_mode_degree[ 4 ] = { 150, 140, 140, 100 };
  int _break_mode = 0;

  bool _is_breaking;
public:
	Arm();
	Arm( Adafruit_Servo *arm, Adafruit_Servo *breaker );
  ~Arm();

  void setServo( Adafruit_Servo *arm, Adafruit_Servo *breaker );
	void reset();
	void toMode( int mod );
	void breakIt();
  void setElevatorMode( int mod );
  void changeBreakMode();
	int getMode();
};

#endif
