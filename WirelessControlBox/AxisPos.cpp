// 
// 
// 
#include <due_can.h>
#include "variant.h"
#include "AxisPos.h"

//
void AxisPos::sendRequest(CANBus can1)
{
	uint16_t channel1[2] = { ARM1ID, ARM1RXID };
	uint16_t channel2[2] = { ARM2ID, ARM2RXID };
	uint8_t requestAngles[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	requestAngles[1] = LOWER;
	can1.sendFrame(ARM1ID, requestAngles);
	can1.sendFrame(ARM2ID, requestAngles);
	requestAngles[1] = UPPER;
	can1.sendFrame(ARM1ID, requestAngles);
	can1.sendFrame(ARM2ID, requestAngles);
}

//
void AxisPos::updateAxisPos(CANBus can1, uint8_t channel)
{
	// Request CAN frame addressed to paremeter ID
	uint8_t* temp = can1.getFrame();

	// If correct message is returned 
	if (temp[1] == LOWER)
	{

		// Determine which channel to write values too
		if (channel == ARM1RXID)
		{
			// Two bytes per axis to reach max of 360 degrees
			a1c1 = (temp[2] * 255) + temp[3];
			a2c1 = (temp[4] * 255) + temp[5];
			a3c1 = (temp[6] * 255) + temp[7];
			//Serial.println("Arm 1 Lower");
		}
		else if (channel == ARM2RXID)
		{
			a1c2 = (temp[2] * 255) + temp[3];
			a2c2 = (temp[4] * 255) + temp[5];
			a3c2 = (temp[6] * 255) + temp[7];
			//Serial.println("Arm 2 Lower");
		}
	}
	if (temp[1] == UPPER)
	{
		if (channel == ARM1RXID)
		{
			a4c1 = (temp[2] * 255) + temp[3];
			a5c1 = (temp[4] * 255) + temp[5];
			a6c1 = (temp[6] * 255) + temp[7];
			//Serial.println("Arm 1 Upper");
		}
		else if (channel == ARM2RXID)
		{
			a4c2 = (temp[2] * 255) + temp[3];
			a5c2 = (temp[4] * 255) + temp[5];
			a6c2 = (temp[6] * 255) + temp[7];
			//Serial.println("Arm 2 Upper");
		}
	}
}

// Update and draw the Axis positions on the view page
void AxisPos::drawAxisPos(UTFT LCD)
{
	//LCD.setColor(0xB5B5B5);
	//LCD.fillRoundRect(340, 35 + 0, 360, 75 + 0);
	//LCD.fillRoundRect(340, 35 + 45, 360, 75 + 45);
	//LCD.fillRoundRect(340, 35 + 90, 360, 75 + 90);
	//LCD.fillRoundRect(340, 35 + 135, 360, 75 + 135);
	//LCD.fillRoundRect(340, 35 + 180, 360, 75 + 180);
	//LCD.fillRoundRect(340, 35 + 225, 360, 75 + 225);

	// Text color
	LCD.setColor(0xFFFF);

	// Text background color
	LCD.setBackColor(0xB5B5B5);

	// Draw angles 
	LCD.printNumI(a1c1, 205, 48);
	LCD.printNumI(a2c1, 205, 93);
	LCD.printNumI(a3c1, 205, 138);
	LCD.printNumI(a4c1, 205, 183);
	LCD.printNumI(a5c1, 205, 228);
	LCD.printNumI(a6c1, 205, 273);
	LCD.printNumI(a1c2, 315, 48);
	LCD.printNumI(a2c2, 315, 93);
	LCD.printNumI(a3c2, 315, 138);
	LCD.printNumI(a4c2, 315, 183);
	LCD.printNumI(a5c2, 315, 228);
	LCD.printNumI(a6c2, 315, 273);
}

// Get angle for programming
int AxisPos::getA1C1()
{
	return a1c1;
}
int AxisPos::getA2C1()
{
	return a2c1;
}
int AxisPos::getA3C1()
{
	return a3c1;
}
int AxisPos::getA4C1()
{
	return a4c1;
}
int AxisPos::getA5C1()
{
	return a5c1;
}
int AxisPos::getA6C1()
{
	return a6c1;
}
int AxisPos::getA1C2()
{
	return a1c2;
}
int AxisPos::getA2C2()
{
	return a2c2;
}
int AxisPos::getA3C2()
{
	return a3c2;
}
int AxisPos::getA4C2()
{
	return a4c2;
}
int AxisPos::getA5C2()
{
	return a5c2;
}
int AxisPos::getA6C2()
{
	return a6c2;
}
