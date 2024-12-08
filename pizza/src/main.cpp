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
motor rightBack = motor(PORT18, ratio18_1, false);
motor leftBack = motor(PORT14, ratio18_1, true);
motor rightMiddle = motor(PORT5, ratio18_1, false);
motor leftMiddle = motor(PORT11, ratio18_1, true);
motor rightFront = motor(PORT2, ratio18_1, false);
motor leftFront = motor(PORT15, ratio18_1, true);
digital_out clamp = digital_out(Brain.ThreeWirePort.H);
motor intake = motor(PORT8, ratio18_1, true);
inertial aniNertial = inertial(PORT21);

bumper beep = bumper(Brain.ThreeWirePort.A); //find the correct port for this

extern bumper beep;

extern brain Brain;
extern controller Controller1;
extern motor leftFront;
extern motor rightFront;
extern motor leftBack;
extern motor rightBack;
extern motor rightMiddle;
extern motor leftMiddle;
extern digital_out clamp;
extern inertial aniNertial;
extern motor intake;

//aniGle is the global angle! (named after anika obv)
int aniGle = 0;
int beepy = 0;

//tune kp for robot (same for right turns!)
//change minSpeed
void turnL(int target){
  aniGle = target;
  double error;
  double deriviative;
  double kd = .2;

  while(aniNertial.rotation(degrees) > aniGle + .5){
    double prevErr = error;
    error = abs(target - aniNertial.rotation(degrees));
    deriviative = error - prevErr;
    double kp = .3;
    double minSpeed = 5;
    double speed = error * kp + deriviative * kd + minSpeed;

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
  wait(200, msec);
}
void turnR(int target){
  aniGle = target;
  double error;
  double deriviative;
  double kd = .2;

  while(aniNertial.rotation(degrees) < aniGle - .5){
    double prevErr = error;
    error = target - aniNertial.rotation(degrees);
    deriviative = error - prevErr;
    double kp = .3;
    double minSpeed = 5;
    double speed = error * kp + deriviative * kd + minSpeed;
    
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
  wait(200,msec);
}
void moveDonuts(int ms, int speed) { 
  leftFront.spin(reverse, speed, pct);
  rightFront.spin(reverse, speed, pct);  
  leftMiddle.spin(reverse, speed, pct);
  rightMiddle.spin(reverse, speed, pct);
  leftBack.spin(reverse, speed, pct);
  rightBack.spin(reverse, speed, pct);
  intake.spin(reverse, 100, pct);

  wait(ms, msec);

  leftFront.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();
  leftMiddle.stop();
  rightMiddle.stop();
  intake.stop();

}
//minimum and maximum paramaters for different scenarios
void moveF(double target, int min, bool donut) { 
  leftFront.resetPosition();
  rightFront.resetPosition();
  double deriviative;
  double error = target - (leftFront.position(degrees)+rightFront.position(degrees))/2; 
  double kd = .09;

  while((leftFront.position(degrees)+rightFront.position(degrees))/2 < target) {
    double prevErr = error;
    error = target - (leftFront.position(degrees)+rightFront.position(degrees))/2; 
    double kp = .04;
    deriviative = error - prevErr;
    double speed = error * kp + deriviative * kd + min; 
    int max = 80;
    if(speed > max){
      speed = max;
    }
    leftFront.spin(fwd, speed, pct);
    rightFront.spin(fwd, speed, pct);
    leftBack.spin(fwd, speed, pct);
    rightBack.spin(fwd, speed, pct);
    leftMiddle.spin(fwd, speed, pct);
    rightMiddle.spin(fwd, speed, pct);
    if(donut){
      intake.spin(reverse, 100, pct);
    }
    wait(10, msec);
  }

  intake.stop();
  leftFront.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();
  leftMiddle.stop();
  rightMiddle.stop();

  wait(200, msec);
}

void moveR(double target, int min, int max, bool donut){
  leftFront.resetPosition();
  rightFront.resetPosition();
  double deriviative;
  double error = target - (leftFront.position(degrees)+rightFront.position(degrees))/2;
  double kd = .09;

  while((leftFront.position(degrees)+rightFront.position(degrees))/2 > -target) {
    double prevErr = error;
    error = target - (leftFront.position(degrees)+rightFront.position(degrees))/2; 
    double kp = .04;
    deriviative = error - prevErr;
    double speed = error * kp + deriviative * kd + min; 
    if(speed > max){
      speed = max;
    }
    if(donut){
      intake.spin(reverse, 100, pct);
    }
    leftFront.spin(fwd, -speed, pct);
    rightFront.spin(fwd, -speed, pct);
    leftBack.spin(fwd, -speed, pct);
    rightBack.spin(fwd, -speed, pct);
    leftMiddle.spin(fwd, -speed, pct);
    rightMiddle.spin(fwd, -speed, pct);
    wait(10,msec);
  }

  leftFront.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();
  leftMiddle.stop();
  rightMiddle.stop();
  intake.stop();

  wait(200, msec);
}

void donut(int ms){
  intake.spin(reverse, 100, pct);
  wait(ms,msec);
  intake.stop();
}

/*At least three (3) Scored Rings of the Alliance's color
A minimum of two (2) Stakes on the Alliance's side of the Autonomous Line with at least (1) Ring of the Alliance's color Scored
Neither Robot contacting / breaking the plane of the Starting Line
At least One (1) Robot contacting the Ladder */
//do not pass the Autonomous Line


//FOR CLAMP: TRUE IS UP, FALSE IS DOWN  
//dont do this! it is good!
/*
void v1Rightrb(){ //starting parallel to the mogo on the right side, move forward and grab the mogo
  //knock over piles and get the blue rings
  //touch the ladder
  //if the other team gets at least one donut on mogo
  //start with preload
  //moveF 2 squares
  clamp.set(true);
  moveF(500, 10);
  //clamp set true
  wait(400, msec);
  clamp.set(false);
  wait(400, msec);
  //left -90 degrees
  turnL(-90);
  //moveR 1 square
  //intake start
  moveR(300, 10, 80);
  donut(900);
  turnR(-40);
  moveR(800, 5, 30);

}*/

void blueRightAutonomous(){
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnL(-90);
  //moveR 1 square
  //intake start
  moveR(400, 10, 80, true);
  donut(1300);
  turnL(-180);
  moveR(140, 10, 100, true);
  donut(2500);
  moveF(200, 5, true);
  turnR(-75);
  moveF(1000, 10, false);
}

//test this!
void redRightAutonomous(){
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnL(-90);
  //moveR 1 square
  //intake start
  moveR(350, 10, 80, false);
  donut(2000);
  turnR(-75);
  moveF(550, 5, false);
}

void redLeftAutonomous(){
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnR(90);
  //moveR 1 square
  //intake start
  moveR(400, 10, 80, true);
  donut(1300);
  turnR(180);
  moveR(140, 10, 100, true);
  donut(2500);
  moveF(200, 5, true);
  turnL(75);
  moveF(1000, 10, false);
}

void blueLeftAutonomous(){
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnR(90);
  //moveR 1 square
  //intake start
  moveR(350, 10, 80, false);
  donut(2000);
  turnL(75);
  moveF(550, 5, false);
  /*
  turnL(45);
  moveR(575, 15, 100);
  turnL(-15);
  turnR(20);
  donut(2700);
  */
}

void autoSkills(){

}
/*
void v1Leftrb(){ //half awp
  //starting parallel to the mogo on the right side, move forward and grab the mogo
  //knock over piles and get the blue rings
  //touch the ladder
  //if the other team gets at least one donut on mogo
  //start with preload
  //move 2 squares
  moveF(900, 10);
  //right 90
  turnR(90);
  //clamp set true
  clamp.set(true);
  //reverse 1 square
  moveR(950, 10, 80);
  //intake start
  //turn left 45
  turnL(50);
  //moveF 1 square
  moveF(550, 10);
}

//redo this! this is bad!
void v2LeftB(){
  moveF(2000, 10);
  clamp.set(true);
  clamp.set(false);
  turnR(90);
  moveF(2000, 10);
  clamp.set(true);
  turnR(-90);
  moveF(400, 10);
  turnL(180);
  moveF(350, 10);
  turnL(-90);
  moveF(500, 10);
}

//redo this! it is bad!
void v2RightR(){
  moveF(2000, 10);
  clamp.set(true);
  clamp.set(false);
  turnL(-90);
  moveF(2000, 10);
  clamp.set(true);
  turnL(90);
  moveF(400, 10);
  turnR(180);
  moveF(350, 10);
  turnR(90);
  moveF(500, 10);
}

void codingChallenge(){
  //24 fwd, right 45, 12 rev, left 90
  moveF(1050, 10);
  turnL(-45);
  moveR(700, 10, 80);
  turnR(45);
}*/
/*
void twofivezeroninea(){
  //from the finals at highlander summit
  //blue right
  moveF(1500, 10);
  turnR(45);
  moveF(525, 10);
  turnR(90);
  moveR(1500, 10, 80);
  turnR(180);
}

void foursixtenc(){
  //blue left
  //back up 2 squares
  moveR(2100, 10);
  //turn right 45
  turnR(45);
  //reverse 1/2 square
  moveR(500, 10);
  //forward 2 squares
  moveF(2100, 10);
  //reverse a little
  moveR(100, 10);
  //180 left
  turnL(-90);
  //forward 2 squares
  moveF(2100, 10);
  //right 75
  turnR(-15);
  //forward to corner
  moveF(1700, 10);
  //reverse a little
  moveR(200, 10);
  //right 180
  turnR(165);
  //forward across the field
  moveF(5250, 10);
}

void oneonesixeighta(){
  //move forward a little bit
  moveF(200, 10);
  //right 45 
  turnR(45);
  //move back a little bit
  moveR(200, 10);
  //forward a little bit
  moveF(200, 10);
  // left back to 0
  turnL(0);
  //forward 2 squares
  moveF(2000, 10);
  //turn to -90
  turnL(-90);
  //reverse 2 squares
  moveR(2000, 10);
  //turns left to -45
  turnL(-45);
  //forward to the bar
  moveF(2000, 10);

}

void onesixtyninetyninec(){
  //right 15
  turnR(15);
  //reverse 1 square
  moveR(1000, 10);
  //left -15
  turnL(-15);
  //reverse 1 square
  moveR(1000, 10);
  //turn right 90
  turnR(90);
  //forward 2 squares
  moveF(2000, 10);
  //right 180
  turnR(180);
  //forward little
  moveF(200, 10);
  //left 125
  turnL(125);
  //reverse a little
  moveR(200, 10);
  //back to 180
  turnR(180);
  //forward little
  moveF(200, 10);
  //left 125
  turnL(125);
  //reverse all the way back
  moveR(5000, 10);
  //a little forward
  moveF(200, 10);
  //left 15
  turnL(15);
}
*/
//slot 1 - redleft
//slot 2 - redright
//slot 3 - blueleft
//slot 4 - blueright

void pre_auton(void) {
  aniNertial.calibrate();
  aniNertial.resetRotation();
  while(true){
    if(beep.pressing()){
      beepy++;
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      if(beepy == 1){
        Brain.Screen.print("red left auton");
      } else if(beepy == 2){
        Brain.Screen.print("red right auton");
      } else if(beepy == 3){
        Brain.Screen.print("blue left auton");
      } else if(beepy == 4){
        Brain.Screen.print("blue right auton");
      } else if(beepy == 5){
        Brain.Screen.print("auto skills?");
      } else {
        beepy = 0;
        Brain.Screen.print("no auton - click again to go back to red left");
      }
    }
    wait(20, msec);
  }
}
void autonomous(void) {
  switch(beepy){
    case 1:
      redLeftAutonomous();
      break;
    case 2:
      redRightAutonomous();
      break;
    case 3:
      blueLeftAutonomous();
      break;
    case 4:
      blueRightAutonomous();
      break;
    case 5:
      autoSkills();
      break;
  }
}

void drive(){
  while(true) {
    //test this, i reversed the movement for driver controlled only ._.
    leftFront.spin(fwd, -(Controller1.Axis3.position() - (Controller1.Axis1.position()/5)), pct);
    leftBack.spin(fwd, -(Controller1.Axis3.position() - (Controller1.Axis1.position()/5)), pct);
    rightFront.spin(fwd, -(Controller1.Axis3.position() + (Controller1.Axis1.position()/5)), pct);
    rightBack.spin(fwd, -(Controller1.Axis3.position() + (Controller1.Axis1.position()/5)), pct);   
    leftMiddle.spin(fwd, -(Controller1.Axis3.position() - (Controller1.Axis1.position()/5)), pct);
    rightMiddle.spin(fwd, -(Controller1.Axis3.position() + (Controller1.Axis1.position()/5)), pct);
  
    if(Controller1.ButtonL1.pressing()){
      intake.spin(reverse, 100, pct);
    } else if(Controller1.ButtonL2.pressing()){
      intake.spin(forward, 100, pct);
    } else {
      intake.stop();
    }
  
    if(Controller1.ButtonR1.pressing()){
      clamp.set(true);
    } else if(Controller1.ButtonR2.pressing()){
      clamp.set(false);
    }

    wait(50, msec);
  }
}

void usercontrol(void) {
  while (1) {
    
    drive();
    /*
    aniNertial.calibrate();
    wait(3, sec);
    aniNertial.resetRotation();
    turnR(90);
    turnL(0);*/
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
