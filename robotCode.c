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

float lastError = 0; //used by PID function
float lastEncVal = 0, lastTimeVal = 0; //used by RPM calculation

float calculateRPM(tMotor motorInterest){
	int currentEncVal = nMotorEncoder[motorInterest];
	int deltaEnc = currentEncVal - lastEncVal;
	int deltaTime = time1[T1] - lastTimeVal;
	float seconds = deltaTime / 1000.0;
	float rotations = deltaEnc / 360.0;
	float rpm = rotations / (seconds/60);
	return rpm;
}

int timeToNextPoint();

void zeroAllAxis();

void zeroAxis();

void adjustPenSpeed();

int calculatePID(float target, float curVal, float kp, float kd){
	float error = target-curVal;
	int outputPow = error*kp + lastError*kd;
	lastError = error;
	return outputPow;
}

void readNextLine();

void stopMovement();

void moveYAxis();

void pause();

void scan();

task main()
{



}
