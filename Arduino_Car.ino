#include <Wire.h>
#include "PS2X_lib.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "elevatorcontroller.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_left = AFMS.getMotor( 4 );
Adafruit_DCMotor *motor_right = AFMS.getMotor( 3 );
Adafruit_DCMotor *elevator_left = AFMS.getMotor( 2 );
Adafruit_DCMotor *elevator_right = AFMS.getMotor( 1 );
PS2X joystick;
ElevatorController elevator_controller( elevator_left, elevator_right );

const int c_speed_backward_normal = 255;
// the normal speed moving backward, it should be lower than the forward speed
const int c_speed_forward_normal = 255;
const int c_elevator_speed_normal = 150;
// the normal speed moving forward
const int c_speed_turn = 150;
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






void setup() {
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  AFMS.begin();
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

  speed_left = speed_right = 0;
  elevator_runtime = 0;
  elevator_speed = 0;
  turn_mode = false;
//  Serial.println(joystick.Analog(PSS_LX));

  execuateRunCommand( joystick, speed_left, speed_right );
  execuateTurnCommand( joystick, speed_left, speed_right, turn_mode );
  execuateSlowMode( joystick, speed_left, speed_right, turn_mode );
  execuateElevatorMove( joystick, elevator_direction, elevator_speed, elevator_runtime );

  setCarSpeed( motor_left, motor_right, speed_left, speed_right );
//  runElevator( elevator_left, elevator_right, elevator_direction, elevator_speed, elevator_runtime );
//  Serial.print( speed_left );
//  Serial.print(" " );
//  Serial.println( speed_right );

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
  const int X_TRESHOLD = 20;
  const double max_func = pow( 2.718, 4.0 );
  int y_value = -( joystick.Analog( PSS_LY ) - 127 );
  if ( abs( y_value ) < Y_TRESHOLD )
    return;
  if ( y_value < 0 ){
    speed_left = speed_right = -c_speed_backward_normal;
  } else 
    if ( y_value > 0 ){
      speed_left = speed_right = c_speed_forward_normal;
    }
// calculate y value
  int x_value = joystick.Analog( PSS_LX );
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
//    runtime = c_elevator_runtime_high;
//    elevator_speed = c_elevator_speed_normal;
    elevator_controller.toStandardPosition( elevator_controller.getStandardPosition() + 1 );
  } else {
    if (joystick.Button( PSB_GREEN ) && joystick.NewButtonState( PSB_GREEN ) ){
      direct = -1;
//      runtime = c_elevator_runtime_high;
//      elevator_speed = c_elevator_speed_normal;
      elevator_controller.toStandardPosition( elevator_controller.getStandardPosition() - 1 );
    }
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

//void runElevator( Adafruit_DCMotor *elevator_left, Adafruit_DCMotor *elevator_right, const int &direct, const int &elevator_speed, const int &runtime ){
//  if ( elevator_speed == 0 )
//    return;
//  if ( direct > 0 ) {
//    elevator_left->run( FORWARD );
//    elevator_left->setSpeed( elevator_speed );
//    elevator_right->run( FORWARD );
//    elevator_right->setSpeed( elevator_speed );
//  } else {
//    if( direct < 0 ) {
//      elevator_left->run( BACKWARD );
//      elevator_left->setSpeed( elevator_speed );
//      elevator_right->run( BACKWARD );
//      elevator_right->setSpeed( elevator_speed );
//    } else {
//      return;
//    }
//  }
//  delay( runtime );
//  elevator_left->run( RELEASE );
////  elevator_left->setSpeed( 0 );
//  elevator_right->run( RELEASE );
////  elevator_right->setSpeed( 0 );
//}