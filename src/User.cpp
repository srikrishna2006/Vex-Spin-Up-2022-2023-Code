#include <vex.h>
#include <User.h>

void user_drive_standard(Odometry &O)
{
  // Assigns  joystick axis values to variables 
  int axis3val = Controller1.Axis3.value();
  int axis4val = Controller1.Axis4.value();
  int  LV = 0;
  int  RV = 0;
  // Turning movement velocity declaration 
  int turnvel = 0;
  // Forward/backwards movement velocity declaration 
  int latvel = 0;
  // Threshold to provide for driver joystick error (sensitivity)
  int threshold = 10;
	

  //States that if the value of the joystick is greater than the threshold, 
  //set the velocity equal to the latvel variable
  if(abs(axis4val) > threshold)

    latvel = axis4val;
  // If the joystick val is less than 10, then don't take the value 
  else

    latvel = 0;
  // Same logic as above 
  if(abs(axis3val) > threshold)

    turnvel = axis3val;

  else

    turnvel = 0;
  // Allows for movement using axis vals 

  // If right side movement is needed, LV is greater and vice versa

  // If straight movement is needed the turnvel will be 0 so both velocities are the 
  // same 
  LV = turnvel + latvel;
  RV = turnvel - latvel;

  O.update();

  printf("%f,%f,%f,%f\n",O.posX,O.posY,O.currentTheta(),O.currentThetaRadians());

  // Gives move command to motors based on velocities from the axis vals on joystick
  
  LB.spin(vex::forward,LV,percent);
  RB.spin(vex::forward,RV,percent);
  LF.spin(vex::forward,LV,percent);
  RF.spin(vex::forward,RV,percent); 
}

void tankdrive()
{
  int LV = Controller1.Axis3.value();
  int RV = Controller1.Axis2.value();

  LB.spin(vex::forward,LV,percent);
  RB.spin(vex::forward,RV,percent);
  LF.spin(vex::forward,LV,percent);
  RF.spin(vex::forward,RV,percent); 

}