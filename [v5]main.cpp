#include "robot-config.h"
#include <iostream>
#include <vector>
using namespace std;
    
    
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
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
double rev_left {0};
double rev_right {0};

double rpm() 
{
    double rotations {flywheel.velocity(vex::velocityUnits::rpm)*25};
 
    return rotations;
}

void driveLeft(double power)
{
    drive_l1.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
    drive_l2.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
}

double getLeft()
{
    rev_left = (drive_l1.rotation(vex::rotationUnits::rev) + drive_l2.rotation(vex::rotationUnits::rev)) / 2;

    double revolutions {rev_left};
    
    return revolutions;
}

void driveRight(double power)
{
    drive_r2.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
    drive_r2.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
}

double getRight()
{
    rev_right = (drive_r1.rotation(vex::rotationUnits::rev) + drive_r2.rotation(vex::rotationUnits::rev)) / 2;
    
    double revolutions {rev_right};
    
    return revolutions;
}

void ballIntake(double power)
{
    ball_intake.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
}

void ballIndexer(double power)
{
    ball_indexer.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
}

void flywheelPower(double power)
{
    flywheel.spin(vex::directionType::fwd, power, vex::velocityUnits::rpm);
}

void get_rpm_then_shoot(int target, int duration){
	int curr {0};

	while(curr < duration)
    {
		curr++;
		flywheelPower(target/25);
        
		if(rpm() >= target) 
        {
            ballIntake(100);
            ballIndexer(100);
        }
	}
}


/////////////////////////////////
//             Red             //
/////////////////////////////////
void red()
{
    int error {10000};
    int target {1234};          // Go Forward //
    rev_left = rev_right = 0;
        
    get_rpm_then_shoot(900, 300);
    ballIntake(100);
    ballIndexer(100);
    
    while(error > 0)
    {
        error = target - (-getLeft() + getRight()) / 2;
        
        if(error > 150)
        {
            driveLeft(100);
            driveRight(100);
        }
        else
        {
            driveLeft(50);
            driveRight(50);
        }
    }
    driveLeft(-10);
    driveRight(-10);
    vex::task::sleep(300);
    
    error = 10000;
    target = -1700;          // Go Backward //
    rev_left = rev_right = 0;
            
    while(error > 0)
    {
        error = fabs(target - (-getLeft() + getRight()) / 2);
        
        if(error > 150)
        {
            driveLeft(-100);
            driveRight(-100);
        }
        else
        {
            driveLeft(-50);
            driveRight(-50);
        }
    }
    driveLeft(10);
    driveRight(10);
    vex::task::sleep(300);
    
    driveLeft(-100);        // Turn To Face Platforms //
    driveRight(100);
    vex::task::sleep(420);
    driveLeft(10);
    driveRight(-10);
    
    error = 10000;
    target = -1200;         // Go On Platforms //
    rev_left = rev_right = 0;
        
    while(error > 0)
    {
        error = fabs(target - (-getLeft() + getRight()) / 2);
        driveLeft(-100);
        driveRight(-100);
    }
    driveLeft(10);
    driveRight(10);
    vex::task::sleep(300);
}


//////////////////////////////////
//             Blue             //
//////////////////////////////////
void blue()
{
    int error {10000};
    int target {1234};          // Go Forward //
    rev_left = rev_right = 0;
    
    get_rpm_then_shoot(900, 300);
    ballIntake(100);
    ballIndexer(100);
    
    while(error > 0)
    {
        error = target - (-getLeft() + getRight()) / 2;
        
        if(error > 150)
        {
            driveLeft(100);
            driveRight(100);
        }
        else
        {
            driveLeft(50);
            driveRight(50);
        }
    }
    driveLeft(-10);
    driveRight(-10);
    vex::task::sleep(300);
    
    error = 10000;
    target = -1700;          // Go Backward //
    rev_left = rev_right = 0;
            
    while(error > 0)
    {
        error = fabs(target - (-getLeft() + getRight()) / 2);
        
        if(error > 150)
        {
            driveLeft(-100);
            driveRight(-100);
        }
        else
        {
            driveLeft(-50);
            driveRight(-50);
        }
    }
    driveLeft(10);
    driveRight(10);
    vex::task::sleep(300);
    
    driveLeft(100);         // Turn To Face Platforms //
    driveRight(-100);
    vex::task::sleep(420);
    driveLeft(10);
    driveRight(-10);
    
    error = 10000;
    target = -1200;         // Go On Platforms //
    rev_left = rev_right = 0;
    
    while(error > 0)
    {
        error = fabs(target - (-getLeft() + getRight()) / 2);
        driveLeft(-100);
        driveRight(-100);
    }
    driveLeft(10);
    driveRight(10);
    vex::task::sleep(300);
}

void autonomous( void ) 
{
    bool is_red = false;
	bool is_row = false;

	if(!is_red && !is_row) 		{blue();}
	else if(is_red && !is_row) 	{red();}
	//else if(!is_red && is_row)	{row_blue();}
	//else if(is_red && is_row)	{row_red();}
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

void usercontrol( void ) 
{
    int target {900};
    
    bool toggle_ballIntake = false;
    rev_left = rev_right = 0;
            
    while (true) 
    {
//++++++++++++ Tank Drive Control Setup ++++++++++++//
        
// Left Drive Setup //
        drive_l1.spin(vex::directionType::fwd, controller.Axis3.value(), vex::velocityUnits::pct);
        drive_l2.spin(vex::directionType::fwd, controller.Axis3.value(), vex::velocityUnits::pct);
        
// Right Drive Setup //
        drive_r1.spin(vex::directionType::fwd, controller.Axis2.value(), vex::velocityUnits::pct);
        drive_r2.spin(vex::directionType::fwd, controller.Axis2.value(), vex::velocityUnits::pct);

        
//++++++++++++ Flywheel Control Setup ++++++++++++//      
        if(rpm() != target)
        {
            flywheelPower(target/25);
        }
        
// High RPM //
        if(controller.ButtonY.pressing())
        {
            flywheel.spin(vex::directionType::fwd, 900, vex::velocityUnits::rpm);
        }
        
// Low RPM //
        else if(controller.ButtonB.pressing())
        {
            flywheel.spin(vex::directionType::fwd, 450, vex::velocityUnits::rpm);
        }

        
//++++++++++++ Ball Intake & Indexer Control Setup ++++++++++++//
        
// Ball Intake Setup //
        // Toggle Setup //
        if(!controller.ButtonR2.pressing() && !ball_intake.isSpinning())
        {
            ball_intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        else if(controller.ButtonR2.pressing() && !ball_intake.isSpinning())
        {
            ball_intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(!controller.ButtonR2.pressing() && ball_intake.isSpinning())
        {
            ball_intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(controller.ButtonR2.pressing() && ball_intake.isSpinning())
        {
            ball_intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        // Reverse Setup //
        if(!controller.ButtonDown.pressing() && !ball_intake.isSpinning())
        {
            // This Should Do Nothing //
        }
        else if(controller.ButtonDown.pressing() && !ball_intake.isSpinning())
        {
            // This Should Do Nothing //
        }
        else if(!controller.ButtonDown.pressing() && ball_intake.isSpinning())
        {
            // This Should Do Nothing //
        }
        else if(controller.ButtonDown.pressing() && ball_intake.isSpinning())
        {
            ball_intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        
// Ball Indexer Setup //
        // Forward Setup //
        if(controller.ButtonR1.pressing())
        {
            ball_indexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else
        {
            ball_indexer.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        // Reverse Setup //
        if(controller.ButtonRight.pressing() && controller.ButtonR1.pressing())
        {
            ball_indexer.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        else if(controller.ButtonRight.pressing() && !controller.ButtonR1.pressing())
        {
            ball_indexer.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        else if(!controller.ButtonRight.pressing())
        {
            // This Should Do Nothing //
        }

//++++++++++++ The Might 'L' Setup ++++++++++++//

        vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

/*/!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!/*/
/*/                   DO NOT TOUCH                   /*/
/*/                   DO NOT TOUCH                   /*/
/*/                   DO NOT TOUCH                   /*/
/*/!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!/*/
int main() 
{    
    //Run the pre-autonomous function. 
    pre_auton();
    
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    while(true) 
    {
        //vex::task::sleep(100);
    }
}
