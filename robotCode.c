const tMotor X_AXIS = motorD;
const tMotor Y_AXIS = motorA;
const tMotor Z_AXIS = motorC;
const tSensors X_LIMIT_SWITCH = S2;
const tSensors Y_LIMIT_SWITCH = S1;
const tSensors Z_LIMIT_SWITCH = S3;
const tSensors SCANNER_SENSOR = S4;
const int PAUSE_BUTTON = (int)buttonEnter;

const int POINTS_PER_LINE = 120; //80
const float X_SPEED = 5.5; // mm/s
const int X_FAST_SPEED_MULT = 20;
const float X_DISTANCE_PER_ROTATION = 25.44; //mm per rotation
const float Z_80_DEG_PER_SEC = 688.4181119;
const int MAX_X = 160; // mm 160
const float POINT_DISTANCE = 1.0*MAX_X/POINTS_PER_LINE; // mm - distance between adjacent points
const int MAX_X_ENC = ((MAX_X+2*POINT_DISTANCE)/X_DISTANCE_PER_ROTATION)*360.0;
const int SLOW_TICKS = 180;

float encLimit=0; //For the Y axis

const int DEBOUNCE = 200; // touch sensor debounce

// tested the colour sensor to determine size of the scannable matrix -- divide by point distance
const int SCAN_NXN = 24;
const int SCAN_PER_LINE = 1.0*MAX_X/SCAN_NXN +0.5;
const int SCAN_Y_MATRIX = 1.5*SCAN_PER_LINE +0.5;

float lastError = 0, target = X_SPEED*60/25.4, kpF = 2, kdF = 0.05, kpR = 1, kdR = 0; //used by PID function
float lastEncVal = 0, lastTimeVal = 0; //used by RPM calculation
int pidOutput = 0;

short int scanArray[SCAN_PER_LINE*SCAN_Y_MATRIX];

#include "PC_FileIO.c"

float calculateRPM(tMotor motorInterest)
{
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

void setXRPM(float rpm)
{
	int offset = 3.9229;
	if(rpm>=0)
		offset = - 3.284;
	int power = (rpm*0.7061 + offset);
	motor[X_AXIS] = power;
}

int getNextPlottedPointIndex(bool* points, int currentPoint)
{
	int nextPoint = 0;
	// traverse the array to find the next point we plot - by default we say this is the next index in the array
	for (nextPoint = currentPoint+1; nextPoint < POINTS_PER_LINE && points[nextPoint] == 0; nextPoint++)
	{}
	// check that we actually found a point and return its index, if not, return -1
	// this check is necessary for the case where our current point is the last point
	if (nextPoint < POINTS_PER_LINE)
		return nextPoint;
	return -1;
}

void zeroAllAxis()
{
	const int BACK_OFF_ENCODER_VAL = 150;
	bool firstXHit = false, backedOff = false;
	motor[X_AXIS] = -100;
	motor[Z_AXIS] = 80;
	motor[Y_AXIS] = -100;
	while(!SensorValue[Y_LIMIT_SWITCH] || !backedOff || (backedOff && !SensorValue[X_LIMIT_SWITCH]) || !SensorValue[Z_LIMIT_SWITCH])
	{
		if (motor[Z_AXIS] && SensorValue[Z_LIMIT_SWITCH])
			motor[Z_AXIS] = 0;
		if (!firstXHit)
		{
			if (motor[X_AXIS] && SensorValue[X_LIMIT_SWITCH])
			{
				nMotorEncoder[X_AXIS] = 0;
				motor[X_AXIS] = 40;
				firstXHit = true;
			}
		}
		else
		{
			if (!backedOff)
			{
				if(abs(nMotorEncoder[X_AXIS]) >= BACK_OFF_ENCODER_VAL)
				{
					backedOff = true;
					setXRPM(-target);
				}
			}
			else
			{
				if (motor[X_AXIS] && SensorValue[X_LIMIT_SWITCH])
					motor[X_AXIS] = 0;
			}
		}
		if (motor[Y_AXIS] && SensorValue[Y_LIMIT_SWITCH])
			motor[Y_AXIS] = 0;
	}
	motor[X_AXIS] = 0;
	motor[Z_AXIS] = 0;
	motor[Y_AXIS] = 0;
	nMotorEncoder[X_AXIS] = nMotorEncoder[Y_AXIS] = nMotorEncoder[Z_AXIS] = 0;
}

void getEncoderValuesAtPoints(bool* points, int* encoderValues)
{
	int nextPlottedPoint = -1, plottedPointNum = 0;

	while((nextPlottedPoint = getNextPlottedPointIndex(points, nextPlottedPoint)) != -1)
	{
		float encoderValAtNextPoint = ((nextPlottedPoint+2)*POINT_DISTANCE-180.0/Z_80_DEG_PER_SEC*X_SPEED*0.637)/X_DISTANCE_PER_ROTATION*360.0;
		encoderValues[plottedPointNum] = encoderValAtNextPoint;
		plottedPointNum++;
	}

	for (int fillEmpty = plottedPointNum; fillEmpty < POINTS_PER_LINE; fillEmpty++)
			encoderValues[fillEmpty] = -1;
}

task calculatePID()
{
	while(true)
	{
		float curVal = calculateRPM(X_AXIS);
		float error = target-curVal;
		int outputPow = 0;
		outputPow = error*kpF + (error-lastError)*kdF;
		displayString(3, "Power = %d", outputPow);
		displayString(4, "Error = %f", error);
		lastError = error;
		pidOutput = outputPow;
		setXRPM(outputPow);
		wait1Msec(10);
	}
}

bool readNextLine(TFileHandle fin, bool* points)
{
	bool hasPoint = false;
	int integerIn;
	for (int index = 0; index < POINTS_PER_LINE; index++)
	{
		readIntPC(fin, integerIn);
		points[index] = (integerIn == 1);
		hasPoint = (hasPoint || points[index]);
	}
	return hasPoint;
}

void moveYAxis(int distance)
{
	const float WHEEL_DIA= 42.5; //mm
	const float GEAR_RATIO = 25.0/1.0; 	// geared down 25 to 1
	encLimit += abs(distance)/(PI*WHEEL_DIA)*360*GEAR_RATIO;
	int motorSpeed = 100;
	int hault = 0;

	if (distance <0)
		motorSpeed *= -1;

	//nMotorEncoder[Y_AXIS]=0; //don't need this because encoder target/limit is a added onto
	motor[Y_AXIS]= motorSpeed;

	while (abs(nMotorEncoder[Y_AXIS]) <= encLimit)
	{}
	motor[Y_AXIS] = motorSpeed/5;
	wait1Msec(500);

	motor[Y_AXIS]= hault;
}

void moveXAxis (int distance)
{
	const float PINION_CIRC = 25.44; //mm
	const float ENC_LIMIT = abs(distance)*360/PINION_CIRC; //mm*(deg*rot-1)*(mm-1*rot)

	int motorSpeed = 80;

	if (distance <0)
		motorSpeed *= -1;

	nMotorEncoder[X_AXIS] = 0;
	motor[X_AXIS] = motorSpeed;

	while (abs(nMotorEncoder[X_AXIS]) < ENC_LIMIT)
	{}

	motor[X_AXIS]= 0;
}

void pause(float xRPM, bool&fast)
{
	// turn off all motors
	motor[X_AXIS] = motor[Y_AXIS] = motor[Z_AXIS] = 0;
	//give time for button to be released
	while(getButtonPress(PAUSE_BUTTON))
	{}
	eraseDisplay();
	displayString(2, "Press enter to continue");
	// wait for button to be pressed again
	while(!getButtonPress(PAUSE_BUTTON))
	{}
	while(getButtonPress(PAUSE_BUTTON))
	{}
	eraseDisplay();
	setXRPM(xRPM);
	fast = false;
}

void checkPaper()
{
	SensorType[SCANNER_SENSOR] = sensorLightActive;
	wait1Msec(50);
	moveXAxis(12*POINT_DISTANCE+50);
	moveYAxis(6*POINT_DISTANCE+80);
	wait1Msec(100);
	eraseDisplay();
	while(SensorValue[SCANNER_SENSOR] < 50)
	{
		displayString(4,"Place Paper");
		displayString(5, "COlor = %d", SensorValue[SCANNER_SENSOR]);
	}
	eraseDisplay();
	displayString(4, "Paper check complete");
	wait1Msec(2000);
	zeroAllAxis();
	eraseDisplay();
}
void scan(int scanLines)
{
	for (int initialize= 0; initialize < SCAN_PER_LINE*SCAN_Y_MATRIX; initialize++)
	{
		scanArray[initialize] = 0;
	}

	SensorType[SCANNER_SENSOR] = sensorLightActive;
	// SensorMode[SCANNER_SENSOR] = modeEV3Color_Ambient;
	zeroAllAxis();
	moveXAxis(6*POINT_DISTANCE+8); // change these values
	moveYAxis(6*POINT_DISTANCE + 55);
	int direction = 1;

	int arrayIndex = 0;
	for (int scanY = 0 ; scanY < scanLines; scanY++)
	{
		displayString(2, "%d", scanY);
		wait1Msec(200);
		scanArray[arrayIndex] = (short int)((SensorValue[SCANNER_SENSOR]-10)*255.0/55.0);
		arrayIndex++;
		for (int scanX = 0; scanX < SCAN_PER_LINE-1; scanX++)
		{
			moveXAxis(direction * SCAN_NXN);
			wait1Msec(200);
			scanArray[arrayIndex] = (short int)((SensorValue[SCANNER_SENSOR]-10)*255.0/55.0);
			arrayIndex++;
		}
		if(scanY < scanLines-1)
			moveYAxis(SCAN_NXN);
		direction *= -1 ;
	}
	zeroAllAxis();
}

void displayTime(int rowNumber, long time)
{
	// convert timer value into hours, minutes, and seconds
	long hours =  time /3600000, minutes = (time-hours*3600000) / 60000, seconds = ((time-hours*3600000) - minutes*60000) / 1000;
	// print the time to the display
	displayString(rowNumber, "Time: %02d:%02d:%02d", hours, minutes, seconds);
}


float scanEval( short int*scanArray, TFileHandle & fin, int scanLines)
{
	const int TOL = 50;
	int accurate = 0;
	for ( int count = 0 ; count < SCAN_PER_LINE*scanLines; count ++ )
	{
		int origImage =0;
		readIntPC(fin, origImage);
		if (abs(scanArray[count] - origImage) < TOL)
			accurate ++;
	}

	return accurate*100.0/ (SCAN_PER_LINE*scanLines);
}











task main()
{
	// display message telling them to select image on PC then transfer file
	displayString(1, "Please select image on PC");
	displayString(2, "then transfer file to EV3");
	displayString(12, "Press enter to continue");
	while(!getButtonPress(buttonEnter))
	{}
	while(getButtonPress(buttonEnter))
	{}
	eraseDisplay();

	// confirmation screen that they have uploaded the image they want
	displayString(1, "Verify that image is");
	displayString(2, "uploaded to EV3");
	displayString(12, "Press enter to continue");
	while(!getButtonPress(buttonEnter))
	{}
	while(getButtonPress(buttonEnter))
	{}
	eraseDisplay();

	// open file now that user has confirmed it is correct file
	TFileHandle fin;
	openReadPC(fin, "image.txt");
	int rowsToPlot = 0;
	// check that file exists and get the number of rows we are plotting from the file
	if (!readIntPC(fin, rowsToPlot)){
		displayString(1, "Could not open image file");
		displayString(12, "Press enter to end program");
		while(!getButtonPress(buttonEnter))
		{}
		while(getButtonPress(buttonEnter))
		{}
	}

	else
	{
		// declare two parallel arrays to store whether or not we plot a respective point and to store the speed of the z axis at each point
		bool points[POINTS_PER_LINE];
		int encoderValues[POINTS_PER_LINE];
		// initialize both arrays (encoder values is -1 since we make -1 equal to no points left to plot in line)

		// zero the axes
		zeroAllAxis();
		// confirmation screen that they have placed the paper correctly
		//checkPaper();
		displayString(12, "Press enter to start plot");
		while(!getButtonPress(buttonAny))
		{}
		while(getButtonPress(buttonAny))
		{}
		eraseDisplay();
		encLimit = 0;

		//*************************************** START PLOTTING POINTS ***************************************//
		// zero timer to track running time of plot

		time1[T1] = 0;
		for(int rowNumber = 0, rowPlotted = 0; rowNumber < rowsToPlot; rowNumber++)
		{
			// read next line of image to plot
			// if it has points to plot, then plot them, else skip the plotting
			if(readNextLine(fin, points))
			{
				// calculates the encoder values of the x axis for the given row at plotted points
				getEncoderValuesAtPoints(points, encoderValues);
				// zero encoder of x axis so we can determine when to plot points


				// FIX THIS LATER --> NO PID RIGHT NOW
				bool fast = false;
				int xPow = target;
				if(rowPlotted%2 == 0){
					nMotorEncoder[X_AXIS] = 0;
					xPow = target;
					setXRPM(xPow);
				}
				else{
					xPow = target*-1;
					setXRPM(xPow);
					while (nMotorEncoder[X_AXIS] > MAX_X_ENC){}
					nMotorEncoder[X_AXIS] = 0;
				}

				//startTask(calculatePID);
				//setXRPM(xPow);
				// set x axis to constant speed -- changes directions based on odd or even row

				int pointNumber = 0;
				while((nMotorEncoder[X_AXIS] < MAX_X_ENC && rowPlotted%2==0) || (rowPlotted%2==1 && !SensorValue[X_LIMIT_SWITCH]))
				{
					// display running time of plot
					displayTime(1, time1[T1]);
					// display plot information (row and column)
					displayString(4,"Row Number: %d", rowNumber+1);
					displayString(5,"Point Number: %d", pointNumber+1);

					if(getButtonPress(PAUSE_BUTTON))
						pause(xPow, fast);

					// plot the point if we should
					if (encoderValues[pointNumber] != -1 && abs(nMotorEncoder[X_AXIS]) >= encoderValues[pointNumber])
					{
						motor[Z_AXIS] = 80;
						while(SensorValue[Z_LIMIT_SWITCH] == 1){}
						wait1Msec(DEBOUNCE);
						while(SensorValue[Z_LIMIT_SWITCH] == 0){}
						motor[Z_AXIS] = 0;
						if(pointNumber == POINTS_PER_LINE-1)
							encoderValues[pointNumber] = -1;
						else
							pointNumber++;
					}
					// checks if there are no more points or if the points are far awaw
					else if ((encoderValues[pointNumber] == -1 && abs(nMotorEncoder[X_AXIS]) < (MAX_X_ENC - SLOW_TICKS)) || abs(nMotorEncoder[X_AXIS]) < (encoderValues[pointNumber] - SLOW_TICKS))
					{
						//only sets motor power if it is not already fast
						if (!fast)
						{
							setXRPM(xPow*X_FAST_SPEED_MULT);
						//setXRPM(xPow*X_FAST_SPEED_MULT);
							fast = true;
						}
					}
					//sets speed back to slow when the points are close or at the end of the line
					else if (fast)
					{
						setXRPM(xPow);
						fast = false;
					}
				}
				motor[X_AXIS] = 0;
				rowPlotted++;
			}

			// move y axis to next row
			// do not do this if we are on the last line
			if (rowNumber < rowsToPlot-1)
				moveYAxis(POINT_DISTANCE); //1.2375 is because image was becoming squished //not needed
			float estTime = 1.0*time1[T1]/(rowNumber+1)*rowsToPlot;
			displayTime(2, estTime);
		}

		eraseDisplay();
		long plotTime = time1[T1];
		displayTime(1, plotTime);
		zeroAllAxis();
		encLimit = 0;
		int scanLines = (int)(1.0*rowsToPlot*POINT_DISTANCE/SCAN_NXN +0.5);
		eraseDisplay();
		displayString(10, "%d,%d", SCAN_PER_LINE, scanLines);
		scan(scanLines);

		displayString(3, "Accuracy: %f", scanEval(scanArray, fin, scanLines));
		displayString(12, "Press enter to finish plot");

		// wait for buttton press to end program
		while(!getButtonPress(buttonEnter))
		{}
		while(getButtonPress(buttonEnter))
		{}
		eraseDisplay();
	}
}
