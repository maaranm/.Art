
const int POINTS_PER_LINE = 80;
const int X_SPEED=20; //mm/s
const int AXIS_STEP = 2; //mm
const int MAX_X = 224; //mm
const int MAX_Y = 224; //mm
const int POINT_DISTANCE = MAX_X/POINTS_PER_LINE;
const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;

float calculateRPM();

int getDistanceToNextPoint(bool* points, int currentPoint) {
	int nextPoint = currentPoint+1;
	for (nextPoint = currentPoint; points[nextPoint] == 0; nextPoint++);
	return nextPoint-currentPoint;
}

void zeroAllAxis();

void zeroAxis();

void adjustPenSpeed(bool* points) {
	int distanceToNextPoint = 0;

	for (int currentPoint = 0; currentPoint < POINTS_PER_LINE; currentPoint += distanceToNextPoint) {
		distanceToNextPoint = getDistanceToNextPoint(points, currentPoint);
		int timeToNextPoint = distanceToNextPoint*POINT_DISTANCE / X_SPEED;
		// speed is in degrees per second
		int speed = 360 / timeToNextPoint;

		for (int pointToSet = currentPoint; pointToSet < currentPoint+distanceToNextPoint; pointToSet++)
			points[pointToSet] = speed;
	}
}

int calculatePID();

void readNextLine();

void stopMovement()
{
	while(SensorValue[Z_LIMIT_SWITCH] == 0)
	{}
	motor[X_AXIS] = motor[Y_AXIS] = motor[Z_AXIS] = 0;
}

void moveYAxis();

void pause();

void scan();

task main()
{



}
