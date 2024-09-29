/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       smoot                                                     */
/*    Created:      9/15/2024, 9:35:11 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;

brain  Brain;
controller Controller1 = controller(primary);

//for 4610K robot
motor leftFront = motor(PORT11, ratio18_1, true);
motor rightFront = motor(PORT5, ratio18_1, false);
motor leftMiddle = motor(PORT19, ratio18_1, true);
motor rightMiddle = motor(PORT18, ratio18_1, false);
motor leftBack = motor(PORT15, ratio18_1, true);
motor rightBack = motor(PORT12, ratio18_1, false);
digital_out pnu1 = digital_out(Brain.ThreeWirePort.A);
digital_out pnu2 = digital_out(Brain.ThreeWirePort.B);
motor intake = motor(PORT17, ratio18_1, true);


inertial aniNertial = inertial(PORT20);

extern brain Brain;
extern controller Controller1;
extern motor leftFront;
extern motor rightFront;
extern motor leftBack;
extern motor rightBack;
extern motor rightMiddle;
extern motor leftMiddle;
extern digital_out pnu1;
extern digital_out pnu2;
extern inertial aniNertial;
extern motor intake;

//aniGle is the global angle! (named after anika obv)
int aniGle = 0;

void pre_auton(void) {
  aniNertial.calibrate();

}

void turn(int angle) { 
  leftFront.resetPosition();
  rightFront.resetPosition();
  rightMiddle.resetPosition();
  leftMiddle.resetPosition();
  rightBack.resetPosition();
  leftBack.resetPosition();

  aniGle = angle; 

  double derivative;
  double error = angle - (leftFront.position(degrees)+rightFront.position(degrees))/2;
  double kp = 0.5;
  double kpt = 1.3;
  double kd = .4;
  double tError = angle - aniNertial.rotation(degrees);

  while(fabs(error)>3){
        
    double previousError=error;
    error = angle - (leftFront.position(degrees)+rightFront.position(degrees))/2;
    derivative = error-previousError;
    tError = angle - aniNertial.rotation(degrees);

    double power = error*kp+derivative*kd;
    if(power>100){
      power=100;
    }
    leftFront.spin(forward,power+tError*kpt,pct);
    rightFront.spin(forward,power-tError*kpt,pct);
    leftMiddle.spin(forward,power+tError*kpt,pct);
    rightMiddle.spin(forward,power-tError*kpt,pct);
    leftBack.spin(forward,power+tError*kpt,pct);
    rightBack.spin(forward,power-tError*kpt,pct);
    wait(10,msec);
  }
  rightFront.stop();
  rightMiddle.stop();
  rightBack.stop();
  leftFront.stop();
  leftBack.stop();
  leftMiddle.stop();
}

//tune kp for robot (same for right turns!)
//change minSpeed
void turnL(int target){
  aniGle = target;
  double error;

  while(aniNertial.rotation(degrees) > aniGle){
    error = abs(target - aniNertial.rotation(degrees));
    double kp = 0.5;
    double minSpeed = 5;
    double speed = error * kp + minSpeed;

    leftFront.spin(reverse, speed, pct);
    rightFront.spin(fwd, speed, pct);
    rightMiddle.spin(fwd, speed, pct);
    leftBack.spin(reverse, speed, pct);
    rightBack.spin(fwd, speed, pct);
    leftMiddle.spin(reverse, speed, pct);
    wait(10,msec);
  }

  leftFront.setStopping(brake);
  rightFront.setStopping(brake);
  leftBack.setStopping(brake);
  rightBack.setStopping(brake);
  leftMiddle.setStopping(brake);
  rightMiddle.setStopping(brake);

  leftFront.stop();
  leftMiddle.stop();
  rightMiddle.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();
}

void turnR(int target){
  aniGle = target;
  double error;

  while(aniNertial.rotation(degrees) < aniGle){
    error = target - aniNertial.rotation(degrees);
    double kp = 0.5;
    double minSpeed = 5;
    double speed = error * kp + minSpeed;
    
    leftFront.spin(fwd, speed, pct);
    leftBack.spin(fwd, speed, pct);
    leftMiddle.spin(fwd, speed, pct);
    rightFront.spin(reverse, speed, pct);
    rightBack.spin(reverse, speed, pct);
    rightMiddle.spin(reverse, speed, pct);
    wait(10,msec);
  }

  leftFront.setStopping(brake);
  leftBack.setStopping(brake);
  rightBack.setStopping(brake);
  rightMiddle.setStopping(brake);
  leftMiddle.setStopping(brake);
  rightFront.setStopping(brake);

  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  leftMiddle.stop();
  rightMiddle.stop();
  rightBack.stop();
}

//minimum and maximum paramaters for different scenarios
void moveF(double target, int min, int max) { 
  leftFront.resetPosition();

  while(leftFront.position(degrees) < target) {
    double error = target - leftFront.position(degrees); 
    double kp = .4;
    double speed = error * kp + min; 
    if(speed > max){
      speed = max;
    }
    leftFront.spin(fwd, speed, pct);
    rightFront.spin(fwd, speed, pct);
  }

  leftFront.setStopping(brake);
  rightFront.setStopping(brake);
  leftFront.stop();
  rightFront.stop();

  wait(25, msec);

  leftFront.setStopping(coast);
  rightFront.setStopping(coast);
}

void moveR(double target, int min, int max){
  leftFront.resetPosition();

  while(leftFront.position(degrees) > -target) {
    double error = target - leftFront.position(degrees); 
    double kp = .6;
    double speed = error * kp + min; 
    if(speed > max){
      speed = max;
    }
    leftFront.spin(fwd, -speed, pct);
    rightFront.spin(fwd, -speed, pct);
  }

  leftFront.setStopping(brake);
  rightFront.setStopping(brake);
  leftFront.stop();
  rightFront.stop();

  wait(25, msec);

  leftFront.setStopping(coast);
  rightFront.setStopping(coast);
}

/*At least three (3) Scored Rings of the Alliance's color
A minimum of two (2) Stakes on the Alliance's side of the Autonomous Line with at least (1) Ring of the Alliance's color Scored
Neither Robot contacting / breaking the plane of the Starting Line
At least One (1) Robot contacting the Ladder */

void autonomous(void) {
}

void drive(){
  while(true) {
    //drive
    leftFront.spin(fwd, Controller1.Axis3.position() + (Controller1.Axis1.position()/3), pct);
    leftBack.spin(fwd, Controller1.Axis3.position() + (Controller1.Axis1.position()/3), pct);
    rightFront.spin(fwd, Controller1.Axis3.position() - (Controller1.Axis1.position()/3), pct);
    rightBack.spin(fwd, Controller1.Axis3.position() - (Controller1.Axis1.position()/3), pct);   
    leftMiddle.spin(fwd, Controller1.Axis3.position() + (Controller1.Axis1.position()/3), pct);
    rightMiddle.spin(fwd, Controller1.Axis3.position() - (Controller1.Axis1.position()/3), pct);
  }
}

void codingChallenge(){
    //24 fwd, right 45, 12 rev, left 90
    moveF(1050, 10, 40);
    turnL(-45);
    moveR(700, 10, 40);
    turnR(45);
}

void twofivezeroninea(){
    moveF(1500, 10, 40);
    turnR(45);
    moveF(525, 10, 40);
    turnR(90);
    moveR(1500, 10, 40);
    turnR(180);
}

void usercontrol(void) {
  while (1) {
    aniNertial.calibrate();
    wait(3, sec);
     
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
