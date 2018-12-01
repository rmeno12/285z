#include "robot-config.h"
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

int printShit(){
    static int leftAvg {0};
    static int rightAvg {0};
    static int rot {0};
    
    while(1){
        leftAvg = (leftDriveFront.rotation(rotationUnits::deg) + leftDriveBack.rotation(rotationUnits::deg)) / 2;
        rightAvg = (rightDriveFront.rotation(rotationUnits::deg) + rightDriveBack.rotation(rotationUnits::deg)) / 2;
        rot = Gyro.value(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 30, "%d", leftAvg);
        Brain.Screen.printAt(10, 60, "%d", rightAvg);
        Brain.Screen.printAt(10, 90, "%d", rot);
    }
    
    return 0;
}
void printStuff(){
    static int leftAvg {0};
    static int rightAvg {0};
    static int rot {0};
    
    while(1){
        leftAvg = (leftDriveFront.rotation(rotationUnits::deg) + leftDriveBack.rotation(rotationUnits::deg)) / 2;
        rightAvg = (rightDriveFront.rotation(rotationUnits::deg) + rightDriveBack.rotation(rotationUnits::deg)) / 2;
        rot = Gyro.value(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 30, "%d", leftAvg);
        Brain.Screen.printAt(10, 60, "%d", rightAvg);
        Brain.Screen.printAt(10, 90, "%d", rot);
    }
}

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
    Gyro.startCalibration();
    Gyro.changed(printStuff);
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
void driveLeft(){
    int scaledPower {Controller.Axis3.value()};
    
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^2/100) * scaledPower ) / 100;
    
    if (Controller.Axis3.value() < 0)
        scaledPower = scaledPower * -0.8;
    else
        scaledPower = scaledPower *  0.8;
    
    leftDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct);
    leftDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct);
}

void driveRight(){
    int scaledPower {Controller.Axis2.value()};
    
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^2/100) * scaledPower ) / 100;
    
    if (Controller.Axis2.value() < 0)
        scaledPower = scaledPower * -0.8;
    else
        scaledPower = scaledPower *  0.8;
    
    rightDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct);
    rightDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct);
}

void moveLeft(int distance, int power){
    leftDriveFront.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct);
    leftDriveBack.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct);
}

void moveRight(int distance, int power){
    rightDriveFront.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct);
    rightDriveBack.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct);
}

void resetMotors(){
    rightDriveFront.setRotation(0, rotationUnits::deg);
    rightDriveBack.setRotation(0, rotationUnits::deg);
    leftDriveFront.setRotation(0, rotationUnits::deg);
    leftDriveBack.setRotation(0, rotationUnits::deg);
}

static bool reverse {false};
static bool toggle {false};

void toggleIntake()
{
    toggle = !toggle;

    if (toggle)
        ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
    else
        ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
}

void puIntake()
    {ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);}
void pdIntake()
{
    ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
    reverse = false;
}
void rIntake()
{
    ballIntake.spin(directionType::fwd, -100, velocityUnits::pct);
    reverse = true;
}

void puIndexer()
{
    indexer.spin(directionType::fwd, 100, velocityUnits::pct);
    ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
    reverse = true;
}
void pdIndexer()
{
    indexer.spin(directionType::fwd, 0, velocityUnits::pct);
    ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
    reverse = false;
}
void rIndexer()
    {indexer.spin(directionType::fwd, -100, velocityUnits::pct);}

void autonomous( void ) {
    resetMotors();    
    
    int step1 {1200}; //1200
    ballIntake.spin(directionType::fwd, 50, velocityUnits::pct);
    
    moveLeft(step1, 50);
    moveRight(step1, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) < (step1 - 50)){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200);
    resetMotors();                     
    
    moveLeft(-step1, 50);
    moveRight(-step1, 50);
    
    flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);
    
    while(leftDriveFront.rotation(rotationUnits::deg) > (-step1 + 50)){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200);
    resetMotors();                     
                
    int step2 {210}; //210
    moveRight(step2, 50);
    moveLeft(-step2, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) < step2-5){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200); 
    resetMotors();
    
    int counter {0};
    while(flywheel.velocity(velocityUnits::rpm) == 170 && counter < 500){
        counter++;
        indexer.spin(directionType::fwd, 100, velocityUnits::pct);
        ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    
    while(counter < 500){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200); 
    resetMotors();
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

void usercontrol( void ) {
    leftDriveFront.setStopping(brakeType::coast);
    leftDriveBack.setStopping(brakeType::coast);
    rightDriveFront.setStopping(brakeType::coast);
    rightDriveBack.setStopping(brakeType::coast);
    
    Controller.ButtonR1.pressed(puIndexer);   //> indexer 
    Controller.ButtonR1.released(pdIndexer);
    
    Controller.ButtonR2.pressed(toggleIntake);      //> intake toggle

    Controller.ButtonL2.pressed(rIntake);     //> reverse
    Controller.ButtonL2.pressed(rIndexer);

    Controller.ButtonL2.released(pdIntake);
    Controller.ButtonL2.released(pdIndexer);
    
    Controller.Axis2.changed(driveRight);
    Controller.Axis3.changed(driveLeft);
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
