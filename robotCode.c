const int POINTS_PER_LINE = 80;
const float X_SPEED=20; // mm/s
const float AXIS_STEP = 2; // mm
const float MAX_X = 224; // mm
const float MAX_Y = 224; // mm
const float POINT_DISTANCE = 1.0*MAX_X/POINTS_PER_LINE; // mm - distance between adjacent points
const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;

#include "PC_FileIO.c"

float calculateRPM();

int getDistanceToNextPoint(bool* points, int currentPoint) {
	int nextPoint = 0;
	// traverse the array to find the next point we plot - by default we say this is the next index in the array
	for (nextPoint = currentPoint+1; nextPoint < POINTS_PER_LINE && points[nextPoint] == 0; nextPoint++);

	// check that we actually found a point and return it, if not, return -1
	// this check is necessary for the case where our current point is the last point
	if (nextPoint < POINTS_PER_LINE)
		return nextPoint-currentPoint;
	return -1;
}

void zeroAxis(int axis){ //0 = x , 1 = y , 2 = z
	if(axis == 0){
		motor[X_AXIS] = -25;
		while (!SensorValue[X_LIMIT_SWITCH]);
		motor[X_AXIS] = 0;
		nMotorEncoder[X_AXIS] = 0;
	} else if (axis == 1){
		motor[Y_AXIS] = -100;
		while (!SensorValue[Y_LIMIT_SWITCH]);
		motor[Y_AXIS] = 0;
		nMotorEncoder[Y_AXIS] = 0;
	} else {
		motor[Z_AXIS] = 50;
		while (!SensorValue[Z_LIMIT_SWITCH]);
		motor[Z_AXIS] = 0;
		nMotorEncoder[Z_AXIS] = 0;
	}
}

void zeroAllAxis(){
	zeroAxis(2); //z axis lifts pen
	zeroAxis(0); //x axis zeroes
	zeroAxis(1); //y axis zeroes
}

int calcZAxisMotorPower(float rpm) {
	// based on a linear trendline to convert rpm to motor power
	// trendline uses 11 data points (motor power from 0 to 100 in increments of 10 and associated rpm)
	return 0.3838*rpm-0.9373;
}

void adjustPenSpeed(bool* points, int* speeds) {
	// distanceToNextPoint is in mm
	// speed is in degrees per second and then converted using above conversion factor
	int distanceToNextPoint = 0, speed = 0;

	for (int currentPoint = 0; currentPoint < POINTS_PER_LINE; currentPoint += distanceToNextPoint) {
		distanceToNextPoint = getDistanceToNextPoint(points, currentPoint);

		// if we still have another point to plot
		if (distanceToNextPoint != -1)
		{
			// time to next point is minutes
			float timeToNextPoint = distanceToNextPoint*POINT_DISTANCE / X_SPEED / 60.0;
			// convert rpm to motor power using linear model
			// we only want to travel 1/4 of a revolution so that the marker strike is quick and the in between movement is slow
			speed = calcZAxisMotorPower(0.25 / timeToNextPoint);
		}
		// if we dont, fill the rest of the array with 0
		else
		{
			speed = 0;
			distanceToNextPoint = POINTS_PER_LINE-currentPoint;
		}

		// fill the speed area until the next point with our desired speed
		for (int pointToSet = currentPoint; pointToSet < currentPoint+distanceToNextPoint; pointToSet++)
			speeds[pointToSet] = speed;
	}
}

int calculatePID();

void readNextLine(TFileHandle fin, bool* points){
	int integerIn;
	for (int index = 0; index < POINTS_PER_LINE; index++){
		readIntPC(fin, integerIn);
		points[index] = (integerIn == 1);
	}
}

float fmod(float dividend, float divisor) {
	return dividend - ((int)(dividend/divisor))*divisor;
}

void stopMovement()
{
	const float TOL = 0.000001;

	// wait until we reach the place in the array
	while(fmod(time1[T2], (1.0*X_SPEED)) >= TOL);
	motor[X_AXIS] = motor[Y_AXIS] = motor[Z_AXIS] = 0;
}

void moveYAxis();

void pause();

void scan();

task main()
{
	TFileHandle fin;
	bool points[POINTS_PER_LINE];
	int speeds[POINTS_PER_LINE];

	if (!openReadPC(fin, "pointFile.txt" )){
		displayString(2, "Could not open point file.");
	}
	zeroAllAxis();
	readNextLine(fin, points);


	TFileHandle fout;
	openWritePC(fout, "rpmMeasurements.txt");

	for (int i = 100; i <= 100; i += 10)
	{
		nMotorEncoder[Z_AXIS] = 0;
		time1[T1] = 0;
		motor[Z_AXIS] = i;
		displayString(1, "Set Motor");
		wait1Msec(30000);
		float time = time1[T1];
		float encoder = nMotorEncoder[Z_AXIS];
		displayString(2, "Recorded Data");
		motor[Z_AXIS] = 0;
		encoder /= 360;
		time /= 1000;
		time /= 60;
		float rpm = encoder/time;
		writeFloatPC(fout, rpm);
		writeCharPC(fout, ' ');
		displayString(3, "Calculated rpm");
		wait1Msec(1000);
		eraseDisplay();
	}

}
