#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    pot_arm,        sensorNone)
#pragma config(Sensor, dgtl1,  enc_fw,         sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  enc_drive_right, sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  enc_drive_left, sensorQuadEncoder)
#pragma config(Motor,  port1,           intake_ball,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           drive_r1,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           drive_r2,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           drive_r3,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           drive_l1,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           drive_l2,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           drive_l3,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           flywheel,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           lift,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          indexer,       tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorValue[enc_fw] = 0;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;
	SensorValue[pot_arm] = 0;
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

float rpm(){
	SensorValue[enc_fw] = 0;
	wait1Msec(10);

	float curr = SensorValue[enc_fw];
	float rpm = (curr / 0.1666667) * 5;

	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDNumber(0, 5, rpm);

	return rpm;
}

task aut_bang(){
	int high = 120;
	int low = 70;
	int target = 930;
	while(true){
		if(rpm() < target) {motor[flywheel] = high;}
		else if(rpm() > target) {motor[flywheel] = low;}
	}
}

	void get_rpm_then_shoot(int target, int high, int low, int duration){
		int curr = 0;

		while(curr < duration){
			curr++;

			if(rpm() < target) {motor[flywheel] = high;}
			else if(rpm() > target) {motor[flywheel] = low;}

			if(rpm() >= target) {motor[intake_ball] = motor[indexer] = 127;}
		}
	}

	void red(){
		int counter = 0;
		int target = 0;
		int high = 0;
		int low = 0;
		int error = 10000;
		int enc_target = 700;

		SensorValue[enc_drive_left] = 0;
		SensorValue[enc_drive_right] = 0;

		/*/FRONT/*/	target = 930; high = 100; low = 50;
		/*/BACK/*/	//target = 1025; high = 80; low = 30;
		
		while(counter < 300)
		{
		counter++;

		if(rpm() < target) {motor[flywheel] = high;}
		else if(rpm() > target) {motor[flywheel] = low;}

		if(rpm() >= target) {motor[intake_ball] = motor[indexer] = 127;}		/*///SHOOT THE BALL/*/
	}

	get_rpm_then_shoot(930, 120, 70, 300);
	motor[flywheel] = motor[intake_ball] = motor[indexer] = 0;			/*/STOP FLYWHEEL AND BALL INTAKE/*/

	while(error > 0){
		error = enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2;
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150) {motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 120;}
		else {motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 60;}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;

	wait1Msec(300);
	error = 10000;
	enc_target = -910;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150) {motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -120;}
		else {motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -60;}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;

	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -100;	/*/TURN TO FACE PLATFORMS/*/
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 100;
	wait1Msec(390);
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 10;
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;

	error = 10000;							/*/BEGIN DRIVING ON PLATFORMS/*/
	enc_target = -700;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -120;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -60;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;
}

void blue(){
	int counter = 0;
	int target = 0;
	int high = 0;
	int low = 0;
	int error = 10000;
	int enc_target = 700;

	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	/*/FRONT/*/	target = 930; high = 100; low = 50;
	/*/BACK/*/	//target = 1025; high = 80; low = 30;

	while(counter < 300)
	{
		counter++;

		if(rpm() < target) {motor[flywheel] = high;}
		else if(rpm() > target) {motor[flywheel] = low;}

		if(rpm() >= target) {motor[intake_ball] = motor[indexer] = 127;}		/*/SHOOT THE BALL/*/
	}
	motor[flywheel] = motor[intake_ball] = motor[indexer] = 0;

	while(error > 0){
		error = enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2;
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 80;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;

	wait1Msec(300);
	error = 10000;
	enc_target = -910;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;

	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 100;	/*/TURN TO FACE THE PLATFORMS/*/
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -100;
	wait1Msec(460);
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -10;
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;

	error = 10000;							/*/BEGIN DRIVING ON PLATFORMS/*/
	enc_target = -700;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;
}

void get_ball()
{

	int error = 10000;
	int enc_target = -550;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;	
}

void return_to_init_pos()
{
	int error = 10000;
	int enc_target = 570;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2;
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 40;
		}

		if(error < 400){
			motor[intake_ball] = 0;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;	
}

void get_high_flag_pos()
{
	int error = 10000;
	int enc_target = -120;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -40;
		motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -45;
	}

	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;
}

void get_mid_flag_pos()
{
	int error = 10000;
	int enc_target = 470;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2;
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 80;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 45;
		}

		if(error < 300){
			motor[intake_ball] = 0;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;	
}

void get_low_flag_pos()
{
	int error = 10000;
	int enc_target = 210;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2;
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 80;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 45;
		}

		if(error < 300){
			motor[intake_ball] = 0;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;	
}

void get_parking_pos()
{
	int error = 10000;
	int enc_target = -910;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;
}

void get_platform_pos()
{
	int error = 10000;							/*/BEGIN DRIVING ON PLATFORMS/*/
	int enc_target = -700;
	SensorValue[enc_drive_left] = 0;
	SensorValue[enc_drive_right] = 0;

	while(error > 0){
		error = abs(enc_target - (-SensorValue[enc_drive_left] + SensorValue[enc_drive_right]) / 2);
		displayLCDNumber(1, 5, error);
		writeDebugStreamLine("Error: %d", error);

		if(error > 150){
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -120;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -70;
		}
		else{
			motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;
			motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -40;
		}
	}
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;
}

void row_blue()
{
	startTask(aut_bang);
	motor[intake_ball] = 127;

	get_ball();
	wait1Msec(300);

	return_to_init_pos();
	wait1Msec(200);

	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -60;	/*/TURN TO FACE PLATFORMS/*/
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 60;
	wait1Msec(570);
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 10;
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;
	wait1Msec(500);

	get_high_flag_pos();
	motor[indexer] = 127;
	wait1Msec(300);
	motor[indexer] = 0;
	wait1Msec(200);

	get_mid_flag_pos();
	motor[indexer] = motor[intake_ball] = 127;
	wait1Msec(600);
	motor[indexer] = motor[intake_ball] = 0;

	get_low_flag_pos();
	wait1Msec(300);

	get_parking_pos();
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 100;	/*/TURN TO FACE THE PLATFORMS/*/
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -100;
	wait1Msec(460);
	motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -10;
	motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 10;

	get_platform_pos();
}

task autonomous()
{
	row_blue();
}

//THE FOLLOWING IS IN MEMORY OF OUR OLD SUPER JANK AND CHOPPY AUTON THAT WAS BASED ON TIME...RIP AUTOJANK//
/****************************************************************************************************************************************/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 127;		//RED	*/
/*//motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 65;		//BLUE	*/
/*wait1Msec(1500);															*/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = 100; motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 127;		//RED	*/
/*//motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -27; motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 127;	//BLUE	*/
/*wait1Msec(500);															*/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -10;										*/
/*motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -10;										*/
/*																	*/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -127;			*/
/*wait1Msec(1500);															*/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = -127;	//THIS SHOULD BE NEGATIVE WHEN RED					*/
/*motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = 127;	//THIS SHOULD BE NEGATIVE WHEN BLUE					*/
/*wait1Msec(1000);															*/
/*motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = -127;			*/
/*wait1Msec(3000);															*/
/****************************************************************************************************************************************/



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

const unsigned int TrueSpeed[128] =
{
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
	25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
	28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
	33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
	37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
	41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
	46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
	52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
	71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
	80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
	88, 89, 89, 90, 90, 127, 127, 127
};

task usercontrol()
{
	motor[flywheel] = 40;
	int target = 0;

	int speed_drive_L = 0;
	int speed_drive_R = 0;

	bool toggle_ball = false;

	while (true)
	{
		if(vexRT[Ch2] <= 0)
		{
			speed_drive_R = -TrueSpeed[abs(vexRT[Ch2])];
		}
		else
		{
			speed_drive_R = TrueSpeed[vexRT[Ch2]];
		}

		if(vexRT[Ch3] <= 0)
		{
			speed_drive_L = -TrueSpeed[abs(vexRT[Ch3])];
		}
		else
		{
			speed_drive_L = TrueSpeed[vexRT[Ch3]];
		}

		//tank drive
		motor[drive_r1] = motor[drive_r2] = motor[drive_r3] = speed_drive_R;
		motor[drive_l1] = motor[drive_l2] = motor[drive_l3] = speed_drive_L;


		//bang bang control for flywheel
		if(rpm() < target)
		{motor[flywheel] = 70;}
		else if(rpm() > target)
		{motor[flywheel] = 30;}

		//Flywheel Target//
		if(vexRT[Btn8L]){
			target = 920;
		}
		else if(vexRT[Btn8D]){
			target = 720;
			motor[flywheel] = 0;
		}


		//Other Moter Controls//
		if(vexRT[Btn5U]){
			motor[lift] = 127;
		}
		else if(vexRT[Btn5D]){
			motor[lift] = -127;
		}
		else{
			motor[lift] = 0;
		}

		if(vexRT[Btn6U]){
			motor[indexer] = 100;
		}
		else if(vexRT[Btn7R]){
			motor[indexer] = -100;
		}
		else{
			motor[indexer] = 0;
		}

		//if button && !toggle_ball//
		if(vexRT[Btn6D]){
			while(vexRT[Btn6D]){}
			toggle_ball = !toggle_ball;
		}
		if(toggle_ball){
			motor[intake_ball] = 127;
		}
		else if(vexRT[Btn7D] && !toggle_ball){
			motor[intake_ball] = -127;
		}
		else{
			motor[intake_ball] = 0;
		}
	}
}
