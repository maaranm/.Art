const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;
const int PAUSE_BUTTON = (int)buttonEnter;

const int POINTS_PER_LINE = 60;
const float X_SPEED = 20; // mm/s
const float AXIS_STEP = 2; // mm
const float MAX_X = 224; // mm
const float MAX_Y = 224; // mm
const float POINT_DISTANCE = 1.0*MAX_X/POINTS_PER_LINE; // mm - distance between adjacent points
const float TIME_BETWEEN_POINTS = POINT_DISTANCE / X_SPEED; // s - time between adjacent points

enum Axes {X, Y, Z};

float lastError = 0, target = 0, kpF = 2, kdF = 0.05, kpR = 1, kdR = 0; //used by PID function
float lastEncVal = 0, lastTimeVal = 0; //used by RPM calculation
int pidOutput = 0;

#include "PC_FileIO.c"

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
	if(axis == (int)X){
		motor[X_AXIS] = -25;
		while (!SensorValue[X_LIMIT_SWITCH]);
		motor[X_AXIS] = 0;
		nMotorEncoder[X_AXIS] = 0;
	} else if (axis == (int)Y){
		motor[Y_AXIS] = -100;
		while (!SensorValue[Y_LIMIT_SWITCH]);
		motor[Y_AXIS] = 0;
		nMotorEncoder[Y_AXIS] = 0;
	} else {
		// check which direction to zero the z axis in (so we don't plot a point accidentally)
		if(nMotorEncoder[Z_AXIS]%360 < 180)
			motor[Z_AXIS] = -50;
		else
			motor[Z_AXIS] = 50;
		while (!SensorValue[Z_LIMIT_SWITCH]);
		motor[Z_AXIS] = 0;
		nMotorEncoder[Z_AXIS] = 0;
	}
}

void zeroAllAxis(){
	zeroAxis(Z); //z axis lifts pen
	zeroAxis(X); //x axis zeroes
	zeroAxis(Y); //y axis zeroes
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
			float timeToNextPoint = distanceToNextPoint*TIME_BETWEEN_POINTS / 60.0;
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
	motor[X_AXIS] = motor[Y_AXIS] = motor[Z_AXIS] = 0;
}

void moveYAxis(){}

void pause(){}

void scan(){}

task main()
{
	// display message telling them to select image on PC then transfer file
	displayString(1, "Please select image on PC");
	displayString(2, "then transfer file to EV3");
	displayString(12, "Press enter to continue");
	while(!getButtonPress(buttonEnter));
	while(getButtonPress(buttonEnter));
	eraseDisplay();

	// confirmation screen that they have uploaded the image they want
	displayString(1, "If the correct imge is uploaded");
	displayString(2, "press enter to start the plot");
	while(!getButtonPress(buttonEnter));
	while(getButtonPress(buttonEnter));
	eraseDisplay();

	// open file now that user has confirmed it is correct file
	TFileHandle fin;
	openReadPC(fin, "outputBro.txt" )
	int rowsToPlot = 0;
	// check that file exists and get the number of rows we are plotting from the file
	if (!readIntPC(fin, rowsToPlot)){
		displayString(2, "Could not open point file.");
		wait10Msec(500);
	}
	else
	{
		rowsToPlot = 60;
		// declare two parallel arrays to store whether or not we plot a respective point and to store the speed of the z axis at each point
		bool points[POINTS_PER_LINE];
		int speeds[POINTS_PER_LINE];

		// start plotting the points
		zeroAllAxis();
		time1[T1] = 0;
		for(int rowNumber = 0; rowNumber < 60; rowNumber++) {
			displayString(6,"rowNum = %d", rowNumber);
			// read next line of image to plot
			readNextLine(fin, points);
			// calculates the speeds of the z axis for the given row
			adjustPenSpeed(points, speeds);
			// zero encoder on z axis so we can zero pen properly
			nMotorEncoder[Z_AXIS] = 0;
			// zero timer so we know when to plot each point
			time1[T2] = 0;
			if(rowNumber%2 == 0){
				target = 46.875;
			}
			else{
				target = -46.875;
			}
			startTask(calculatePID);
			// set x axis to constant speed -- changes directions based on odd or even row

			for (int pointNumber = 0; pointNumber < POINTS_PER_LINE; pointNumber++) {
				displayString(5,"pointNum = %d", pointNumber);
				if(getButtonPress(PAUSE_BUTTON))
					pause();

				// zero time to do the actual pen stroke
				time1[T3] = 0;

				// plot the point if we should
				if (points[pointNumber])
				{
					motor[Z_AXIS] = 100;
					while(SensorValue[Z_LIMIT_SWITCH] == 0);
				}
				// set the z axis to its desired speed between points
				motor[Z_AXIS] = speeds[pointNumber];
				// wait until we reach the next "point"
				time1[T4] = 0;
				while(time1{T4] < TIME_BETWEEN_POINTS*1000-time1[T3]);
			}

			stopTask(calculatePID);
			// zero z axis
			zeroAxis((int)Z);

			// move y axis to next row
			// do not do this if we are on the last line
			if (rowNumber < POINTS_PER_LINE-1)
				moveYAxis();
		}
		scan();

		displayString(1, "Accuracy: %f", 0.82);
		displayString(12, "Press enter to continue");
		while(!getButtonPress(buttonEnter));
		while(getButtonPress(buttonEnter));
		eraseDisplay();
	}
}
