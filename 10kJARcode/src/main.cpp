#include "vex.h"

using namespace vex;
competition Competition;
int aniGle = 0;
int beepy = 0;
int intakeToggle = 0;
int tessaToggle = 0;
color Blue = color(66, 187, 252);
color Red = color(250, 37, 52);
color allianceColor = Red;
color notAllianceColor = Blue;
/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftFront, leftMiddle, leftBack),

//Right Motors:
motor_group(rightFront, rightMiddle, rightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT21,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.8,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
5.5,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  aniNertial.calibrate();
  aniNertial.resetRotation();
  aniNertial.resetHeading();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    aniNertial.resetRotation();
    if(beep.pressing()){
      beepy++;
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      if(beepy == 1){
        Brain.Screen.printAt(5, 140, "red left auton");
      } else if(beepy == 2){
        Brain.Screen.printAt(5, 140, "red right auton");
      } else if(beepy == 3){
        Brain.Screen.printAt(5, 140, "blue left auton");
      } else if(beepy == 4){
        Brain.Screen.printAt(5, 140, "blue right auton");
      } else if(beepy == 5){
        Brain.Screen.printAt(5, 140, "auto skills");
      } else {
        beepy = 0;
        Brain.Screen.printAt(5, 140, "no auton - keep holding the bumper to go back to red left!");
      }
    }
    wait(600, msec);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  drive_test();
  /*
  switch(beepy){ 
    case 0: //nothing
      drive_test();
      break;
    case 1: //red left       
      drive_test();
      break;
    case 2: //red right
      turn_test();
      break;
    case 3: //blue left
      swing_test();
      break;
    case 4: //blue right
      full_test();
      break;
    case 5: //skills
      odom_test();
      break;
  }*/

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

int reDirect(){
  while(true){
    if(Controller1.ButtonB.pressing()){
      while(opt.hue() > 40 && allianceColor == Red){
        if(Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()){
          break;
        }
        intake.spin(forward, 100, pct);
        intakeToggle = 2;
      }
      intake.resetPosition();
      wait(10, msec);
      while(intake.position(degrees) > -1300){
        intake.spin(reverse, 100, pct);
      }
      intake.stop();
      intakeToggle = 0;
      while((!(opt.hue() > 170 && opt.hue() < 250)) && allianceColor == Blue){
        if(Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()){
          break;
        }
        intake.spin(forward, 100, pct);
        intakeToggle = 2;
      }
      intake.resetPosition();
      wait(10, msec);
      while(intake.position(degrees) > -1300){
        intake.spin(reverse, 100, pct);
      }
      intake.stop();
      intakeToggle = 0;
    
    }
    wait(25, msec);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    thread r (reDirect);
    //.7 is too fast, .6 is too slow
    
    double leftSpeed = -(Controller1.Axis3.position() - (Controller1.Axis1.position() * .62));
    double rightSpeed = -(Controller1.Axis3.position() + (Controller1.Axis1.position() * .62));
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(opt.hue());
    katieRerouter.setStopping(coast);

    //exponents that *should* work: 2.12, 2.2, 2.28, 2.296, 2.36, 2.6, 2.76, 2.92, 3
    leftFront.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    leftBack.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    leftMiddle.spin(fwd, pow(leftSpeed * .01, 3) * 100, pct);
    rightFront.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
    rightBack.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
    rightMiddle.spin(fwd, pow(rightSpeed * .01, 3) * 100, pct);
  
    if(Controller1.ButtonL2.pressing()){
      intake.spin(reverse, 100, pct);
    } else if(Controller1.ButtonL1.pressing()){
      intakeToggle++;
      if(intakeToggle == 1){
        intake.spin(forward, 100, pct);
      } else {
        intake.stop();
        intakeToggle = 0;
      }
    }
  
    if(Controller1.ButtonR1.pressing()){
      clamp1.set(true);
    } else if(Controller1.ButtonR2.pressing()){
      clamp1.set(false);
    }

    if(Controller1.ButtonA.pressing()){
      if(tessaToggle == 0){
        deviDoinker.set(false);
        tessaToggle++;
        wait(100, msec);
      } else if(tessaToggle == 1){
        deviDoinker.set(true);
        tessaToggle--;
        wait(100, msec);
      }
    } 

    if(Controller1.ButtonX.pressing()){
      katieRerouter.spin(forward, 100, pct);
    } else if(Controller1.ButtonY.pressing()){
      katieRerouter.spin(reverse, 100, pct);
    }
    
    wait(150, msec);

    //chassis.control_arcade();
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
