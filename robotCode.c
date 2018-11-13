
const int POINTS_PER_LINE = 80;
const int X_SPEED=20; //mm/s
const int AXIS_STEP = 2; //mm
const int MAX_X = 224; //mm
const int MAX_Y = 224; //mm
const int POINT_DISTANCE = MAX_X/POINTS_PER_LINE; // mm - distance between adjacent points
const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;

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

void zeroAllAxis();

void zeroAxis();

void adjustPenSpeed(bool* points, int* speeds) {
	// distanceToNextPoint is in mm
	// speed is in degrees per second
	int distanceToNextPoint = 0, speed = 0;

	for (int currentPoint = 0; currentPoint < POINTS_PER_LINE; currentPoint += distanceToNextPoint) {
		distanceToNextPoint = getDistanceToNextPoint(points, currentPoint);

		if (distanceToNextPoint != -1)
		{
			int timeToNextPoint = distanceToNextPoint*POINT_DISTANCE / X_SPEED;
			speed = 360 / timeToNextPoint;
		}
		else
		{
			speed = 0;
			distanceToNextPoint = POINTS_PER_LINE-currentPoint;
		}

		for (int pointToSet = currentPoint; pointToSet < currentPoint+distanceToNextPoint; pointToSet++)
			speeds[pointToSet] = speed;
	}
}

int calculatePID();

void readNextLine();

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



}
