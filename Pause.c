const int POINTS_PER_LINE = 80;
const int X_SPEED=20; // mm/s
const int AXIS_STEP = 2; // mm
const int MAX_X = 224; // mm
const int MAX_Y = 224; // mm
const float POINT_DISTANCE = 1.0*MAX_X/POINTS_PER_LINE; // mm - distance between adjacent points
const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;


#include "PC_FileIO.c"

bool isPaused = true;



void Pause(int & isPaused)
{
	if (isPaused == false)
		stopMovement;
	isPaused = !isPaused;
}

void moveYAxis( int distance)
{
	const float WHEEL_DIA= 4.25;
	const float ENC_LIMIT= (float(distance)/100)*180*25/(PI*WHEEL_DIA);
	// geared down 25 to 1
	int motorSpeed = 20;
	int hault =0;

	if (distance <0)
		motorSpeed -=;

	nmotorEncoder[Y_AXIS]=0;
	motor[Y_AXIS]=motorSpeed;

	while (nMotorEncoder[Y_AXIS] < ENC_LIMIT)
	{}

	motor[Y_AXIS]= hault;
}

void moveXAxisDist (int distance)
{
	const float PINION_CIRC = 25.44;
	const float ENC_LIMIT = (float(distance)/100)*180/PINION_CIRC;

	int motorSpeed = 90;
	int hault =0;

	if (distance <0)
		motorSpeed -=;

	nmotorEncoder[X_AXIS]=0;
	motor[X_AXIS]=motorSpeed;

	while (nMotorEncoder[X_AXIS] < ENC_LIMIT)
	{}

	motor[X_AXIS]= hault;
}



