using namespace vex;
    
brain Brain;
controller Controller = controller();

motor leftDriveFront(PORT1, false);
motor leftDriveBack(PORT2, false);
motor rightDriveFront(PORT3, true);
motor rightDriveBack(PORT4, true);

motor flywheel(PORT5, false);
motor ballIntake(PORT6, true);
motor indexer(PORT7, true);

line aLine = line(Brain.ThreeWirePort.A);
line bLine = line(Brain.ThreeWirePort.B);

gyro Gyro = gyro(Brain.ThreeWirePort.C);
