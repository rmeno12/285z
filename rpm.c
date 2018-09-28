#pragma config(Sensor, dgtl1,  enc,            sensorQuadEncoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	SensorValue[enc] = 0;

	//Sense RPM
	//float lastPos = 0;

	while()
	{
		SensorValue[enc] = 0;
		motor[port2] = 127;
		motor[port3] = -127;

		wait1Msec(100);

		float curr = SensorValue[enc];
		float vel = (curr / 1.66667) * 5;
		//float vel = (curr) / (0.000166667);
		//float lastPos = SensorValue[enc];
		writeDebugStreamLine("%f", vel);
	}
}
