#include <Wire.h>
#include "PS2X_lib.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "elevatorcontroller.h"
#include "arm.h"


Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motor_left;
Adafruit_DCMotor *motor_right;
Adafruit_DCMotor *elevator_left;
Adafruit_DCMotor *elevator_right;
PS2X joystick;

Adafruit_Servo *servo1;
Adafruit_Servo *servo2;
Arm arm;
ElevatorController elevator_controller;

const int c_speed_backward_normal = 255;
// the normal speed moving backward, it should be lower than the forward speed
const int c_speed_forward_normal = 255;
const int c_elevator_speed_up = 50;
const int c_elevator_speed_down = 30;
// the normal speed moving forward
const int c_speed_turn = 200;
// the speed when the car turn left of turn right
const double c_speed_slow_rate = 0.5;
const double c_speed_slow_turn_rate = 0.3;
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
void execuateElevatorMove( PS2X &joystick, int &direct );
/*
 * used of change the speed of the car physically
 */
void setCarSpeed( Adafruit_DCMotor *motor_left, Adafruit_DCMotor *motor_right, const int &speed_left, const int &speed_right );
void runElevator( Adafruit_DCMotor *elevator_left, Adafruit_DCMotor *elevator_right, const int &direct, const int &elevator_speed, const int &runtime );
void execuateChangeArmMode( PS2X &joystick, Arm &arm );
void execuateBreak( PS2X &joystick, Arm &arm );
void execuateReset( PS2X &joystick );
void execuateAdjustElevator( PS2X &joystick, int &elevator_speed );
void setElevatorSpeed( Adafruit_DCMotor *elevator_left, Adafruit_DCMotor *elevator_right, const int &elevator_speed );




void setup() {
  // put your setup code here, to run once:
  AFMS = Adafruit_MotorShield();
  Serial.begin( 9600 );
  AFMS.begin(50);
  motor_left = AFMS.getMotor( 4 );
  motor_right = AFMS.getMotor( 3 );
  elevator_left = AFMS.getMotor( 2 );
  elevator_right = AFMS.getMotor( 1 );
  servo1 = AFMS.getServo( 7 );
  servo2 = AFMS.getServo( 6 );
  arm = Arm( servo1, servo2 );
  elevator_controller = ElevatorController( elevator_left, elevator_right, &arm );
  arm.reset();
  setupJoystick( joystick );
  speed_left = speed_right = 0;
  turn_mode = false;
  elevator_direction = 0;
  elevator_runtime = 0;
  elevator_speed = 0;
  setupMotor( motor_left, speed_left, motor_right, speed_right, elevator_left, 0, elevator_right, 0 );

}

void loop() {
//  Serial.println( "enter loop" );
  joystick.read_gamepad( false, 0 );
  speed_left = speed_right = 0;
  elevator_runtime = 0;
  elevator_speed = 0;
  turn_mode = false;

  execuateReset( joystick );

  execuateRunCommand( joystick, speed_left, speed_right );
  execuateTurnCommand( joystick, speed_left, speed_right, turn_mode );
  execuateSlowMode( joystick, speed_left, speed_right, turn_mode );
  execuateElevatorMove( joystick, elevator_direction );
  execuateChangeArmMode( joystick, arm );
  execuateBreak( joystick, arm );
  execuateAdjustElevator( joystick, elevator_speed );

  setCarSpeed( motor_left, motor_right, speed_left, speed_right );
  setElevatorSpeed( elevator_left, elevator_right, elevator_speed );
//  Serial.println(arm.getMode());
  delay( 20 );
//  Serial.println( "exit loop" );
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
  if( joystick.Button( PSB_R2 ) ){
     return;
  }
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

void execuateElevatorMove( PS2X &joystick, int &direct ){
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

void execuateChangeArmMode( PS2X &joystick, Arm &arm ) {
  if ( joystick.Button( PSB_PINK ) && joystick.NewButtonState( PSB_PINK ) ){
    arm.toMode( !arm.getMode() );
  }
}

void execuateBreak( PS2X &joystick, Arm &arm ) {
  if ( joystick.Button( PSB_RED ) && joystick.NewButtonState( PSB_RED ) ){
    arm.breakIt();
  }
}

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
}
void execuateReset( PS2X &joystick ){
  if( joystick.Button( PSB_L1 ) && joystick.Button( PSB_L2 ) && joystick.Button( PSB_R1 ) && joystick.Button( PSB_R2 ) ) {
    setup();
  }
}

void execuateAdjustElevator( PS2X &joystick, int &elevator_speed ) {
  if( !joystick.Button( PSB_R2 ) )
    return;
//  Serial.println( "enter adjust elevator" );
  const int TRESHOLD = 50;
  int y_value = ( joystick.Analog( PSS_RY ) - 127 );
  if ( abs( y_value ) < TRESHOLD )
    return ;
  if ( y_value < 0 ) {
    elevator_speed = c_elevator_speed_up;
  } else 
    if ( y_value > 0 ) {
      elevator_speed = -c_elevator_speed_down;
    }
//  Serial.println( "exit adjust elevator" );
}

void setElevatorSpeed( Adafruit_DCMotor *elevator_left, Adafruit_DCMotor *elevator_right, const int &elevator_speed ) {
//  if (elevator_speed == 0 )
//    return;
//  Serial.println( "enter setElevatorSpeed" );
  if ( elevator_speed > 0 ) {
    elevator_left->run( FORWARD );
    elevator_right->run( FORWARD );
    elevator_left->setSpeed( elevator_speed );
    elevator_right->setSpeed( elevator_speed );
  } else {
      elevator_left->run( BACKWARD );
      elevator_right->run( BACKWARD );
      elevator_left->setSpeed( -elevator_speed );
      elevator_right->setSpeed( -elevator_speed );
  }
//  delay( 10 );
//  elevator_left->run( RELEASE );
//  elevator_right->run( RELEASE );
//  elevator_left->setSpeed( 0 );
//  elevator_right->setSpeed( 0 );
//  Serial.println( "exit setElevatorSpeed" );
}
