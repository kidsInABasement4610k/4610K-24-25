#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
motor rightBack = motor(PORT18, ratio18_1, false);
motor leftBack = motor(PORT14, ratio18_1, true);
motor rightMiddle = motor(PORT5, ratio18_1, false);
motor leftMiddle = motor(PORT11, ratio18_1, true);
motor rightFront = motor(PORT2, ratio18_1, false);
motor leftFront = motor(PORT15, ratio18_1, true);
motor katieRerouter = motor(PORT1, ratio18_1, true);
digital_out clamp1 = digital_out(Brain.ThreeWirePort.H);
motor intake = motor(PORT8, ratio18_1, false);
inertial aniNertial = inertial(PORT21);
digital_out deviDoinker = digital_out(Brain.ThreeWirePort.D);
bumper beep = bumper(Brain.ThreeWirePort.A); 
optical opt = optical(PORT7);
controller Controller1 = controller(primary);

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}