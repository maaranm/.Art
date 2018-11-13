const int X_SPEED=20; //mm/s
const int AXIS_STEP = 2; //mm
const int MAX_X = 224; //mm
const int MAX_Y = 224; //mm
const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;

float calculateRPM();

int timeToNextPoint();

void zeroAllAxis(){
	zeroAxis
}

void zeroAxis(int axis){ //0 = x , 1 = y , 2 = z
	if(axis == 0){
		motor[X_AXIS] = -25;
		while (!SensorValue[X_LIMIT_SWITCH]);
		motor[X_AXIS] = 0;
		nMotorEncoder[X_AXIS] = 0;
	} else if (axis == 1){
		motor[Y_AXIS] = 100;
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

void adjustPenSpeed();

int calculatePID();

void readNextLine(){
	
}

void stopMovement();

void moveYAxis();

void pause();

void scan();

task main()
{



}
