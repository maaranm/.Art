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

float calculateRPM();

int timeToNextPoint();

void zeroAllAxis();

void zeroAxis();

void adjustPenSpeed();

int calculatePID();

void readNextLine();

void stopMovement();

void moveYAxis();

void pause()
{
	
}

void scan();

task main()
{



}
