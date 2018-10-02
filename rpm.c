#pragma config(Sensor, dgtl1,  enc,            sensorQuadEncoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	SensorValue[enc] = 0;
	
	while()
	{
		SensorValue[enc] = 0;

		wait1Msec(100);

		float curr = SensorValue[enc];
		float vel = (curr / 1.66667) * 5;
		writeDebugStreamLine("%f", vel);
	}
}
