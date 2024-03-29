/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Pure_Persuit.h"
#include "User.h"

using namespace vex;

competition Competition;



void pre_auton(void) {
  
  vexcodeInit();
  
}



void autonomous(void) {
  Left.resetPosition();
  Right.resetPosition();
  Odometry O;
  std::vector<Vector> v1{{0,0},{-10,10},{-15,15}};// path
  std::vector<Vector> v2{{15,15},{10,10},{0,0}};// path
  std::vector<Vector> v3{{0,0},{0,10},{10,15},{0,25},{0,35}};// path
  movement(v3,&O);
}



void usercontrol(void) {
  // User control code here, inside the loop
  Odometry O;
  Left.resetPosition();
  Right.resetPosition();
  while (1) {
    user_drive_standard(O);
    wait(20, msec);
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
