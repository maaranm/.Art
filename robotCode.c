const int X_SPEED=20; //mm/s
const int AXIS_STEP = 2; //mm
const int MAX_X = 224; //mm
const int MAX_Y = 224; //mm
const tMotor X_AXIS = motorD;
const int tMotor Y_AXIS = motorA;
const int tMotor Z_AXIS = motorC;
const int tSensors X_LIMIT_SWITCH = S2;
const int tSensors Y_LIMIT_SWITCH = S1;
const int tSensors Z_LIMIT_SWITCH = S3;
const int tSensors SCANNER_SENSOR = S4;
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

void moveXAxisDist (int distance)
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

void scan();

task main()
{



}
