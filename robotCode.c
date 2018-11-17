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

float lastError = 0, target = 0, kpF = 2, kdF = 0.05, kpR = 1, kdR = 0; //used by PID function
float lastEncVal = 0, lastTimeVal = 0; //used by RPM calculation

int pidOutput = 0;

float calculateRPM(tMotor motorInterest){
	int currentEncVal = nMotorEncoder[motorInterest];
	int deltaEnc = currentEncVal - lastEncVal;
	int deltaTime = time1[T1] - lastTimeVal;
	float rotations = deltaEnc / 360.0;
	float rpm = (rotations/deltaTime)*60000;
	displayString(2, "RPM = %f", rpm);
	lastEncVal = currentEncVal;
	lastTimeVal = time1[T1];
	return rpm;
}

void setXRPM(float rpm){
	int offset = 3.9229;
	if(rpm>=0)
		offset = - 3.284;
	int power = (rpm*0.7061 + offset;
	motor[X_AXIS] = power;
}


//int timeToNextPoint();

//void zeroAllAxis();

//void zeroAxis();

//void adjustPenSpeed();

task calculatePID(){
	while(true){
		float curVal = calculateRPM(X_AXIS);
		float error = target-curVal;
		int outputPow = 0;
		outputPow = error*kpF + lastError*kdF;
		displayString(3, "Power = %d", outputPow);
		displayString(4, "Error = %f", error);
		lastError = error;
		pidOutput = outputPow;
		setXRPM(outputPow);
		wait1Msec(10);
	}
}

//void readNextLine();

//void stopMovement();

//void moveYAxis();

//void pause();

//void scan();

task main()
{
	eraseDisplay();
	nMotorEncoder[X_AXIS] = 0;
	while(true){
		target = 60;
		wait1Msec(100);
		startTask(calculatePID);
		while(nMotorEncoder[X_AXIS]<1000){
		}
		stopTask(calculatePID);
		lastEncVal = 0;
		lastTimeVal = 0;
		eraseDisplay();
		target = -60;
		wait1Msec(100);
		startTask(calculatePID);
		while(nMotorEncoder[X_AXIS] > 0){
		}
		stopTask(calculatePID);
	}

}
