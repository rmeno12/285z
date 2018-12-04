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

bool brakeToggle = false;

void toggleBrake(){
    brakeToggle = !brakeToggle;
    if(brakeToggle){
        leftDriveFront.setStopping(brakeType::brake);
        leftDriveBack.setStopping(brakeType::brake);
        rightDriveFront.setStopping(brakeType::brake);
        rightDriveBack.setStopping(brakeType::brake);
    }
    else{
        leftDriveFront.setStopping(brakeType::coast);
        leftDriveBack.setStopping(brakeType::coast);
        rightDriveFront.setStopping(brakeType::coast);
        rightDriveBack.setStopping(brakeType::coast);
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
        scaledPower = scaledPower * -0.6;
    else
        scaledPower = scaledPower *  0.6;
    
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
        scaledPower = scaledPower * -0.6;
    else
        scaledPower = scaledPower *  0.6;
    
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
{
    ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
}
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

void lowFlagPark()
{
    task distanceMonitor(printShit);
    resetMotors();
    
    int step1 {1250};
    moveRight(step1, 50);
    moveLeft(step1, 50);
    
    	
    
    this_thread::sleep_for(200);
    resetMotors();                     
            
    int step2 {-step1 -720};
    moveRight(step2, 50);
    moveLeft(step2, 50);
  
    while(leftDriveFront.rotation(rotationUnits::deg) > (step2 + 50)){
        this_thread::sleep_for(10);
    }
    
    this_thread::sleep_for(200);
    resetMotors();                     
                         
    moveRight(-205, 50);
    moveLeft(205, 50);
                         
    while(leftDriveFront.rotation(rotationUnits::deg) < 200){
        this_thread::sleep_for(10);
    }
    
    this_thread::sleep_for(200); 
    resetMotors();
    
    int step3 {1725};
    moveRight(step3, 80);
    moveLeft(step3, 80);
}

void getBall()
{
    
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
    Brain.Screen.setPenWidth(5);
    flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);
    
    while(leftDriveFront.rotation(rotationUnits::deg) > (-step1 + 60)){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200);
    resetMotors();
}
    
void blueTurn()
{
    int step2 {220}; //210
    moveRight(-step2, 50);
    moveLeft(step2, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) < step2 -5){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200); 
    resetMotors();
}

void redTurn()
{
    int step2 {220}; //210
    moveRight(step2, 50);
    moveLeft(-step2, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) > -step2 +5){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(200); 
    resetMotors();
}

void getRowRed()
{

    redTurn();
        
    puIndexer();
    this_thread::sleep_for(1000); 

//    while(){
//        this_thread::sleep_for(10);
//    }
    this_thread::sleep_for(200); 
    resetMotors();
    pdIndexer();
    
    int step3 {1255};
    moveRight(step3, 50);
    moveLeft(step3, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) < (step3 - 50)){
        this_thread::sleep_for(10);
    }	
    
    this_thread::sleep_for(200);
    resetMotors();  

    int step4 {-step3 -700};
    moveRight(step4, 50);
    moveLeft(step4, 50);
  
    while(leftDriveFront.rotation(rotationUnits::deg) > (step4 + 50)){
        this_thread::sleep_for(10);
    }
    
    this_thread::sleep_for(200);
    resetMotors();                     
    
    blueTurn();
    
    this_thread::sleep_for(200); 
    resetMotors();
    
    int step5 {1725};
    moveRight(step5, 80);
    moveLeft(step5, 80);

}

void getRowBlue()
{
    //redTurn();
    blueTurn();
    
    puIndexer();
    this_thread::sleep_for(1000); 

//    while(){
//        this_thread::sleep_for(10);
//    }
    this_thread::sleep_for(200); 
    resetMotors();
    pdIndexer();
    
    int step3 {1255};
    moveRight(step3, 50);
    moveLeft(step3, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) < (step3 - 50)){
        this_thread::sleep_for(10);
    }	
    
    this_thread::sleep_for(200);
    resetMotors();  

    int step4 {-step3 -700};
    moveRight(step4, 50);
    moveLeft(step4, 50);
  
    while(leftDriveFront.rotation(rotationUnits::deg) > (step4 + 50)){
        this_thread::sleep_for(10);
    }
    
    this_thread::sleep_for(200);
    resetMotors();                     
    
    //blueTurn();
    redTurn();
    
    this_thread::sleep_for(200); 
    resetMotors();
    
    int step5 {1725};
    moveRight(step5, 80);
    moveLeft(step5, 80);
}

void finalTonRed()
{
    flywheel.spin(directionType::fwd, 153, velocityUnits::rpm);//-107,-299//187
    this_thread::sleep_for(5500);
    puIndexer();
    this_thread::sleep_for(2000);
    pdIndexer();
    puIntake();
    flywheel.spin(directionType::fwd, 191, velocityUnits::rpm);
    
    blueTurn();
    getBall();
    redTurn();
    pdIntake();
    
    moveLeft(-107, 50);
    moveRight(-300, 50);
    
    while(leftDriveFront.rotation(rotationUnits::deg) > -107 +5){
        this_thread::sleep_for(10);
    }
    this_thread::sleep_for(1000); 
    resetMotors();
    
    puIndexer();
}

void autonomous( void ) {
    resetMotors();  
    task distanceMonitor(printShit);                     
   /*            
    getBall();
    
    bool isRed {1};
    if(isRed)
        getRowRed();     
    else
        getRowBlue();
            
    //goPark();
    
    //lowFlagPark();
    */
    finalTonRed();
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

void fluxFly()
{
    int lineValue = (aLine.value(vex::percentUnits::pct) + bLine.value(vex::percentUnits::pct)) / 2;
    if(lineValue <69)
        flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);
    else
        flywheel.spin(directionType::fwd, 50, velocityUnits::rpm);
}

void usercontrol( void ) {
    flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);//-107,-299//187
    //Controller.ButtonX.pressed(finalTonRed);
    //aLine.changed(fluxFly);
    //bLine.changed(fluxFly);
    
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
