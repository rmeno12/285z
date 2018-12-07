#include "robot-config.h"

vex::competition    Competition;


void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
    Gyro.startCalibration(); 
    Gyro.changed(printShitAlso);
}

void autonomous( void ) {
    resetMotors();  
    task distanceMonitor(printShit);
}

void usercontrol( void ) {
    leftDriveFront.setStopping(brakeType::coast);
    leftDriveBack.setStopping(brakeType::coast);
    rightDriveFront.setStopping(brakeType::coast);
    rightDriveBack.setStopping(brakeType::coast);
    
    arm.setStopping(brakeType::hold);
    
    flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);//-107,-299//187
    
    Controller.ButtonR1.pressed(puIndexer); 
    Controller.ButtonR1.released(pdIndexer);
    
    Controller.ButtonR2.pressed(toggleIntake);

    Controller.ButtonL1.pressed(raiseArm);
    Controller.ButtonL1.released(stopArm);
    
    Controller.ButtonL2.pressed(lowerArm);
    Controller.ButtonL2.released(stopArm);
    
    Controller.Axis2.changed(driveRight);
    Controller.Axis3.changed(driveLeft);
    
    Controller.ButtonA.pressed(toggleBrake);
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
    
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}
