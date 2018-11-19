const int POINTS_PER_LINE = 81;
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
const int SCAN_NXN = 3;
const int SCAN_STEP = SCAN_NXN * POINT_DISTANCE;
const int SCAN_MATRIX = POINTS_PER_LINE/SCAN_NXN;
bool isPaused = true;


float calculateRPM();

int timeToNextPoint();

void zeroAllAxis();

void zeroAxis();

void adjustPenSpeed();

int calculatePID();

void readNextLine();

void stopMovement();

void moveYAxis(int distance)
{
	const float WHEEL_DIA= 4.25;
	const float ENC_LIMIT= (distance/100.0)*180*25/(PI*WHEEL_DIA);
	// geared down 25 to 1
	int motorSpeed = 20;
	int hault =0;

	if (distance <0)
		motorSpeed *= -1;

	nMotorEncoder[Y_AXIS]=0;
	motor[Y_AXIS]=motorSpeed;

	while (nMotorEncoder[Y_AXIS] < ENC_LIMIT)
	{}

	motor[Y_AXIS]= hault;
}

void moveXAxis (int distance)
{
	const float PINION_CIRC = 25.44;
	const float ENC_LIMIT = (distance/100.0)*180/PINION_CIRC;

	int motorSpeed = 90;
	int hault =0;

	if (distance <0)
		motorSpeed *= -1;

	nMotorEncoder[X_AXIS]=0;
	motor[X_AXIS]=motorSpeed;

	while (nMotorEncoder[X_AXIS] < ENC_LIMIT)
	{}

	motor[X_AXIS]= hault;
}

void pause()
{
	if (isPaused == false)
		stopMovement;
	isPaused = !isPaused;
}



int scanArray[SCAN_MATRIX][SCAN_MATRIX];

void scan(int*scanArray);
{
	for (int initialize= 0; initialize < SCAN_MATRIX*SCAN_MATRIX; initialize++)
	{
		scanArray[initialize] = 0 ;
	}

	SensorType[SCANNER_SENSOR] = sensorEV3_Color;
	SensorMode[SCANNER_SENSOR] = modeEV3Color_Ambient;
	zeroAllAxis();
	moveXAxis(POINT_DISTANCE); // change these values
	moveYAxis(POINT_DISTANCE);
	int direction = 1;

	int arrayIndex = 0;
	for (int scanY = 0 ; scanY < SCAN_MATRIX; scanY++)
	{
		for (int scanX = 0; scanX < SCAN_MATRIX; scanX++)
		{
			scanArray[arrayIndex] = SensorValue[SCANNER_SENSOR];
			moveXAxis(direction * SCAN_STEP);
			arrayIndex++;
		}

		moveYAxis(SCAN_STEP)
		direction *= -1 ;
	}
	zeroAllAxis();
}







task main()
{



}
