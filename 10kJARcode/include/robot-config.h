using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:

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
extern digital_out clamp1;
extern inertial aniNertial;
extern motor intake;
extern motor katieRerouter;
extern optical opt;


void  vexcodeInit( void );