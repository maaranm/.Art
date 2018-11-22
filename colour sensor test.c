
task main()
{

	SensorType[S4] = sensorEV3_Color;
	SensorMode[S4] = modeEV3Color_Color;

	for (int count = 0; count < 30; count ++)
	{
		while (!getButtonPress(buttonAny))
		{}
		while (getButtonPress(buttonAny))
		{}
		int r = 0, g = 0, b =0 ;

		getColorRGB(S4, r, g, b);

		int rgb = (r+g+b)/3;

		displayString( 3, "Value= %d", rgb);

	}


}
