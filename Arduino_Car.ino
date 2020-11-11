#include <Wire.h>
#include "PS2X_lib.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "elevatorcontroller.h"
#include "arm.h"

#define __DEBUG_ARM__

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_left = AFMS.getMotor( 4 );
Adafruit_DCMotor *motor_right = AFMS.getMotor( 3 );
Adafruit_DCMotor *elevator_left = AFMS.getMotor( 2 );
Adafruit_DCMotor *elevator_right = AFMS.getMotor( 1 );
//extern Adafruit_Servo *servo1;
//extern Adafruit_Servo *servo2;
//Adafruit_Servo *servo1;
//Adafruit_Servo *servo2;
PS2X joystick;
ElevatorController elevator_controller( elevator_left, elevator_right );
//Arm &arm;

#ifndef __DEBUG_ARM__
Adafruit_Servo *servo1 = AFMS.getServo( 7 );
Adafruit_Servo *servo2 = AFMS.getServo( 6 );
Arm arm = Arm( servo1, servo2 );
#else
Adafruit_Servo *_arm_servo = AFMS.getServo( 7 );
Adafruit_Servo *_breaker_servo = AFMS.getServo( 6 );
#endif

const int c_speed_backward_normal = 255;
// the normal speed moving backward, it should be lower than the forward speed
const int c_speed_forward_normal = 255;
const int c_elevator_speed_normal = 150;
// the normal speed moving forward
const int c_speed_turn = 200;
// the speed when the car turn left of turn right
const double c_speed_slow_rate = 0.5;
const double c_speed_slow_turn_rate = 0.6;
const int c_elevator_runtime_high = 700;
const int c_elevator_runtime_mid = 300;
const int c_elevator_runtime_low = 0;
// the rate when the low speed command is given


int speed_left, speed_right;
int elevator_direction, elevator_runtime, elevator_speed;
bool turn_mode;

/*
 * used to initialize the joystick
 * which connect the robot with the joystick
 */
void setupJoystick( PS2X &joystick );
/*
 * used to initialize the motor
 * which release all motor
 */
void setupMotor( Adafruit_DCMotor *m1, int s1, Adafruit_DCMotor *m2, int s2 ); 
/*
 * used to control the speed and the direction of the car
 * This function mainly contains two parts
 * Fisrt, if the y direction value of the left PSS is negative,
 * the car will move forward at speed_forward_normal( 255 )
 * If the y direction value of the left PSS is positive,
 * the car will move backward at speed_backward_normal( 200 ).
 * What should be emphasised is that if the absolute value of the y direction is lower than 50,
 * the speed of both motors will be set to 0, which means nothing will happens.
 * 
 * Second, if the x direction value of the left PSS is negative,
 * the speed of the left motor will be lowered ( left speed = left speed * rate A )
 * Oppositely, if the x direction value of the right PSS is positive,
 * the speed of the right motor will be lowered similarly.
 * The rate A is calculate by a speed fuction f(x) = 1 - (e^(x/32))/(e^4) 
 * because x <= 128, the value of f(x) should between 0 ~ 1
 * the meaning of this function is that larger the absolute value of x direction is,
 * the smaller the speed of one motor will be.
 * Additionally, the speed of one motor will reduce faster when the absolute value of x direction become larger.
 */
void execuateRunCommand( PS2X &joystick, int &speed_left, int &speed_right );
/*
 * used to control the direction of the car
 * If the x direction value of the right PSS is negative,
 * the car will turn left,
 * which means the left motor will rotate reversely and the right motor will rotate normally.
 * the speed of both motor is c_speed_turn ( 50 ).
 */
void execuateTurnCommand( PS2X &joystick, int &speed_left, int &speed_right, bool &turn_mode );
/*
 * used to control the speed of the car
 * if the L2 BSP button is pressed,
 * the speed of the car will be slower ( speed = speed * rate B )
 * rate B = c_speed_slow_rate ( 0.3 )
 */
void execuateSlowMode( PS2X &joystick, int &speed_left, int &speed_right, bool turn_mode );
void execuateElevatorMove( PS2X &joystick, int &direct, int &elevator_speed, int &runtime );
/*
 * used of change the speed of the car physically
 */
void setCarSpeed( Adafruit_DCMotor *motor_left, Adafruit_DCMotor *motor_right, const int &speed_left, const int &speed_right );
void runElevator( Adafruit_DCMotor *elevator_left, Adafruit_DCMotor *elevator_right, const int &direct, const int &elevator_speed, const int &runtime );
void execuateChangeArmMode( PS2X &joystick, Arm &arm );
void execuateBreak( PS2X &joystick, Arm &arm );

#ifdef __DEBUG_ARM__
  int _arm_degree, _breaker_degree;
  int _current_mode;
  int _mods_cnt = 2;
  int _mods[ 2 ] = { 60, 90 };
  int _breaker_start_degree = 0;
  int _breaker_end_degree = 120;

  void _arm_reset();
  void _arm_toMode( int mod );
  void _arm_breakIt();
  int _arm_getMode();
#endif





void setup() {
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  AFMS.begin(50);
  Serial.println("in");
//  Adafruit_Servo *servo1 = AFMS.getServo( 7 );
//  Adafruit_Servo *servo2 = AFMS.getServo( 6 );
//  Arm myarm( servo1, servo2 );
//  servo1 = AFMS.getServo( 7 );
//  servo2 = AFMS.getServo( 6 );
//  servo1->writeServo(20);
//  arm = Arm( servo1, servo2 );
//  arm.setServo( &servo1, &servo2 );
//  myarm.reset();
  setupJoystick( joystick );
  speed_left = speed_right = 0;
  turn_mode = false;
  elevator_direction = 0;
  elevator_runtime = 0;
  elevator_speed = 0;
  setupMotor( motor_left, speed_left, motor_right, speed_right, elevator_left, 0, elevator_right, 0 );

}

void loop() {
  // put your main code here, to run repeatedly:
  joystick.read_gamepad( false, 0 );
//  Serial.println( servo1->readDegrees());
  speed_left = speed_right = 0;
  elevator_runtime = 0;
  elevator_speed = 0;
  turn_mode = false;

  execuateRunCommand( joystick, speed_left, speed_right );
  execuateTurnCommand( joystick, speed_left, speed_right, turn_mode );
  execuateSlowMode( joystick, speed_left, speed_right, turn_mode );
  execuateElevatorMove( joystick, elevator_direction, elevator_speed, elevator_runtime );
#ifndef __DEBUG_ARM__
  execuateChangeArmMode( joystick, arm );
  execuateBreak( joystick, arm );

#else
  execuateChangeArmMode( joystick );
  execuateBreak( joystick );
#endif

  setCarSpeed( motor_left, motor_right, speed_left, speed_right );
  delay( 20 );
}









void setupJoystick( PS2X &joystick ){
  int error;
  do { 
    error = joystick.config_gamepad( 13, 11, 10, 12, true, true ); 
    if ( error == 0 ) { 
      break; 
    } else { 
      delay(100); 
    } 
  } while ( true ); 
  Serial.println( "Connect joystick succesfully!" );
}

void setupMotor( Adafruit_DCMotor *m1, int s1, Adafruit_DCMotor *m2, int s2, Adafruit_DCMotor *m3, int s3, Adafruit_DCMotor *m4, int s4 ) {
  m1->setSpeed( s1 );
  m2->setSpeed( s2 );
  m1->run(RELEASE);
  m2->run(RELEASE);
  m3->setSpeed( s3 );
  m4->setSpeed( s4 );
  m3->run(RELEASE);
  m4->run(RELEASE);
}

void execuateRunCommand( PS2X &joystick, int &speed_left, int &speed_right ){
#define speedFunction( value ) ( 1.0 - pow( 2.718, 1.0 * value / 32 ) / max_func )
  const int Y_TRESHOLD = 20;
  const int X_TRESHOLD = 5;
  const double max_func = pow( 2.718, 4.0 );
  int y_value = -( joystick.Analog( PSS_LY ) - 127 );
  int x_value = joystick.Analog( PSS_LX );
//  Serial.print( y_value ); Serial.print( " " ); Serial.println( 127 - x_value );
  if ( abs( y_value ) + abs( x_value - 127 ) < Y_TRESHOLD )
    return;
  if ( y_value < 0 ){
    speed_left = speed_right = -c_speed_backward_normal;
  } else 
    if ( y_value > 0 ){
      speed_left = speed_right = c_speed_forward_normal;
    }
// calculate y value
  x_value = ( x_value <= 127 ) ? x_value - 128 : x_value - 127;
  if ( abs( x_value ) < X_TRESHOLD )
    return;
  if ( x_value < 0 ) {
    speed_left = round( speedFunction( (-x_value) ) * speed_left );
  } else {
    speed_right = round( speedFunction( x_value ) * speed_right );
  }
#undef speedFunction
}

void execuateTurnCommand( PS2X &joystick, int &speed_left, int &speed_right, bool &turn_mode ){
  const int TRESHOLD = 50;
  int x_value = ( joystick.Analog( PSS_RX ) - 127 );
  if ( abs( x_value ) < TRESHOLD )
    return ;
  turn_mode = true;
  if ( x_value < 0 ) {
    speed_left = -c_speed_turn;
    speed_right = c_speed_turn;
  } else 
    if ( x_value > 0 ) {
      speed_left = c_speed_turn;
      speed_right = -c_speed_turn;
    }
}

void execuateSlowMode( PS2X &joystick, int &speed_left, int &speed_right, bool turn_mode ) {
    double rate = turn_mode ? c_speed_slow_turn_rate : c_speed_slow_rate;
    bool flag = joystick.Button( PSB_L2 );
    if ( flag ) {
      speed_left = round( static_cast< double >( speed_left ) * rate );
      speed_right = round( static_cast< double >( speed_right ) * rate );
    }
}

void execuateElevatorMove( PS2X &joystick, int &direct, int &elevator_speed, int &runtime ){
  if ( joystick.Button( PSB_BLUE ) && joystick.NewButtonState( PSB_BLUE ) ){
    direct = 1;
    elevator_controller.toStandardPosition( elevator_controller.getStandardPosition() + 1 );
  } else {
    if (joystick.Button( PSB_GREEN ) && joystick.NewButtonState( PSB_GREEN ) ){
      direct = -1;
      elevator_controller.toStandardPosition( elevator_controller.getStandardPosition() - 1 );
    }
  }
}

#ifndef __DEBUG_ARM__
void execuateChangeArmMode( PS2X &joystick, Arm &arm ) {
  if ( joystick.Button( PSB_PINK ) && joystick.NewButtonState( PSB_PINK ) ){
//    Serial.println("in1");
    arm.toMode( !arm.getMode() );
//static int i = 80;
//    servo1->writeServo( i + 90 );
//    i=(i+80)%160;
//    Serial.println("in2");
  }
}

void execuateBreak( PS2X &joystick, Arm &arm ) {
  if ( joystick.Button( PSB_RED ) && joystick.NewButtonState( PSB_RED ) ){
    arm.breakIt();
  }
}

#else
void execuateChangeArmMode( PS2X &joystick ) {
  if ( joystick.Button( PSB_PINK ) && joystick.NewButtonState( PSB_PINK ) ){
//    Serial.println("in1");
//    arm.toMode( !arm.getMode() );
//static int i = 80;
//    servo1->writeServo( i + 90 );
//    i=(i+80)%160;
//    Serial.println("in2");
    _arm_toMode( !_arm_getMode() );
  }
}

void execuateBreak( PS2X &joystick ) {
  if ( joystick.Button( PSB_RED ) && joystick.NewButtonState( PSB_RED ) ){
    _arm_breakIt();
  }
}
#endif

void setCarSpeed( Adafruit_DCMotor *motor_left, Adafruit_DCMotor *motor_right, const int &speed_left, const int &speed_right ) {
  if( speed_left >= 0 ) {
    motor_left->run( FORWARD );
    motor_left->setSpeed( speed_left );
  } else {
    motor_left->run( BACKWARD );
    motor_left->setSpeed( -speed_left );
  }

  if( speed_right >= 0 ) {
    motor_right->run( FORWARD );
    motor_right->setSpeed( speed_right );
  } else {
    motor_right->run( BACKWARD );
    motor_right->setSpeed( -speed_right );
  }
//  if( speed_left == 0 && speed_right == 0 ) {
//    motor_right->run( BRAKE );
//    motor_left->run( BRAKE );
//  }
}

#ifdef __DEBUG_ARM__
  void _arm_reset(){
    Serial.println( "0" );
    _arm_servo->writeServo( _mods[ 0 ] );
    _breaker_servo->writeServo( _breaker_start_degree );
    _current_mode = 0;
  }
  void _arm_toMode( int mode ) {
    if ( mode > _mods_cnt || mode < 0 )
      return;
    if (mode == _current_mode ) 
      return;
    _arm_servo->writeServo( _mods[ mode ] );
    _current_mode = mode; 
    Serial.print( "1: " ); Serial.println( _mods[ mode ] );
  }
  void _arm_breakIt(){
    Serial.println( "2" );
    _breaker_servo->writeServo( _breaker_end_degree );
    delay( 500 );
    _breaker_servo->writeServo( _breaker_start_degree );
  }
  int _arm_getMode(){
    return _current_mode;
  }
#endif
