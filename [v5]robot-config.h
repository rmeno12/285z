vex::brain Brain;


//++++++++++++ Motor Initialization ++++++++++++//

// Drive Motors //
vex::motor drive_l1 = vex::motor(vex::PORT1, false);
vex::motor drive_l2 = vex::motor(vex::PORT2, false);
vex::motor drive_r1 = vex::motor(vex::PORT3, true);
vex::motor drive_r2 = vex::motor(vex::PORT4, true);

// Ball Related Stuff //
vex::motor ball_intake = vex::motor(vex::PORT5);
vex::motor ball_indexer = vex::motor(vex::PORT6);
vex::motor flywheel = vex::motor(vex::PORT7);

// Cap Related Stuff //
vex::motor theL = vex::motor(vex::PORT8);
