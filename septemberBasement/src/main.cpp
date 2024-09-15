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

// A global instance of competition
competition Competition;
brain  Brain;
controller Controller1 = controller(primary);
motor leftFront = motor(PORT1, ratio18_1, true);
motor rightFront = motor(PORT10, ratio18_1, false);
motor leftMiddle = motor(PORT11, ratio18_1, true);
motor rightMiddle = motor(PORT18, ratio18_1, false);
motor leftBack = motor(PORT15, ratio18_1, true);
motor rightBack = motor(PORT20, ratio18_1, false);
extern brain Brain;
extern controller Controller1;
extern motor leftFront;
extern motor rightFront;
extern motor leftBack;
extern motor rightBack;
extern motor rightMiddle;
extern motor leftMiddle;

void drive(){
  while(true) {
    //drive
    leftFront.spin(forward, Controller1.Axis3.position() + (Controller1.Axis1.position()/2), pct);
    leftBack.spin(forward, Controller1.Axis3.position() + (Controller1.Axis1.position()/2), pct);
    rightFront.spin(forward, Controller1.Axis3.position() - (Controller1.Axis1.position()/2), pct);
    rightBack.spin(forward, Controller1.Axis3.position() - (Controller1.Axis1.position()/2), pct);   
    leftMiddle.spin(forward, Controller1.Axis3.position() + (Controller1.Axis1.position()/2), pct);
    rightMiddle.spin(forward, Controller1.Axis3.position() - (Controller1.Axis1.position()/2), pct);
  }
}
    


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    drive();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
