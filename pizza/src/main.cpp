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
digital_out deviDoinker = digital_out(Brain.ThreeWirePort.D);
bumper beep = bumper(Brain.ThreeWirePort.A); 

extern bumper beep;
extern digital_out deviDoinker;
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

int aniGle = 0;
int beepy = 0;
int intakeToggle = 0;
int tessaToggle = 0;

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
//https://www.vexrobotics.com/high-stakes-manual?srsltid=AfmBOorzsToCl6EP84RN2FEUXlbi8F7sifry7AFLoduSPw0JQdBVvpiy

void blueRightAutonomous(){
  /*moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnL(-90);
  //moveR 1 square
  //intake start
  moveR(400, 10, 80, true);
  donut(1500);
  turnL(-180);
  moveR(140, 10, 100, true);
  donut(2500);
  moveF(200, 5, true);
  turnR(-75);
  moveF(1000, 10, false);*/
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnL(-90);
  //moveR 1 square
  //intake start
  moveR(360, 10, 80, true);
  donut(2300);
  turnR(-80);
  moveF(550, 5, true);
}
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
  moveR(360, 10, 80, true);
  donut(2300);
  turnR(-80);
  moveF(550, 5, true);
}
void redLeftAutonomous(){
  /*
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
  donut(1500);
  turnR(180);
  moveR(140, 10, 100, true);
  donut(2500);
  moveF(200, 5, true);
  turnL(75);
  moveF(1000, 10, false);*/
  moveF(500, 10, false);
  //clamp set true
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //left -90 degrees
  turnR(90);
  //moveR 1 square
  //intake start
  moveR(360, 10, 80, true);
  donut(2300);
  turnL(80);
  moveF(550, 5, true);
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
  moveR(360, 10, 80, true);
  donut(2300);
  turnL(80);
  moveF(550, 5, true);
  /*
  turnL(45);
  moveR(575, 15, 100);
  turnL(-15);
  turnR(20);
  donut(2700);
  */
}
void autoSkills(){
  moveF(80, 10, false);
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  //got first mogo

  turnL(-100);
  moveR(270, 10, 90, false);
  donut(900);
  moveR(180, 10, 90, true);
  donut(1200);
  //got preload and 2 rings

  moveF(210, 10, false);
  turnR(-15);
  moveR(140, 10, 90, true);
  donut(1000);
  //got 3rd ring

  moveF(120, 10,false);
  turnL(-155);
  moveR(240, 10, 90, true);
  donut(1800);
  //got 4th ring

  turnL(90);
  moveR(250, 10, 80, true);
  donut(1800);

  turnR(135);
  moveR(80, 10, 80, true);
  donut(800);
  moveF(1000, 10, true);
  clamp.set(false);

  turnR(90);
  moveR(1200, 10, 80, false);
  turnL(0);
  moveF(80, 10, true);
  wait(200, msec);
  clamp.set(true);
  wait(200, msec);
  turnR(90);

  moveR(250, 10, 80, false);
  donut(1000);
  moveR(250, 10, 80, true);
  donut(1000);

  moveF(250, 10, true);
  turnL(0);
  moveR(120, 10, 80, true);
  donut(1000);

  turnL(-135);
  moveR(300, 10, 80, false);
}

void pre_auton(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("calibrating inertial...");
  aniNertial.calibrate();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("resetting rotation...");
  aniNertial.resetRotation();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("hold the bumper to select auton!");

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
        Brain.Screen.print("auto skills");
      } else {
        beepy = 0;
        Brain.Screen.print("no auton - keep holding the bumper to go back to red left!");
      }
    }
    wait(500, msec);
  }
}
void autonomous(void) {
  autoSkills();
  /*
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
  */
}
void drive(){
  while(true) {
    double leftSpeed = -(Controller1.Axis3.position() - (Controller1.Axis1.position() * .75));
    double rightSpeed = -(Controller1.Axis3.position() + (Controller1.Axis1.position() * .75));

    //exponents that work: 2.12, 2.2, 2.28, 2.296, 2.36, 2.6, 2.76, 2.92, 3
    leftFront.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    leftBack.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    leftMiddle.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    rightFront.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
    rightBack.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
    rightMiddle.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
  
    if(Controller1.ButtonL2.pressing()){
      intake.spin(forward, 100, pct);
    } else if(Controller1.ButtonL1.pressing()){
      intakeToggle++;
      if(intakeToggle == 1){
        intake.spin(reverse, 100, pct);
      } else {
        intake.stop();
        intakeToggle = 0;
      }
    }
  
    if(Controller1.ButtonR1.pressing()){
      clamp.set(true);
    } else if(Controller1.ButtonR2.pressing()){
      clamp.set(false);
    }

    if(Controller1.ButtonA.pressing()){
      if(tessaToggle == 0){
        deviDoinker.set(false);
        tessaToggle++;
      } else if(tessaToggle == 1){
        deviDoinker.set(true);
        tessaToggle--;
      }
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
