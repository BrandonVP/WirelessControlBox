// 
// 
// 
#include <due_can.h>
#include "variant.h"
#include "AxisPos.h"

// Default Constructor
AxisPos::AxisPos()
{
	// Currently unused
}


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

/*
// Request current axis angle from either desired channel
// Channel contains both TX and RX CAN address
void AxisPos::armSearch(CANBus can1, uint16_t * channel)
{
	// Used to end while loop when task is complete
	bool isDone = true;

	// Outgoing message frame
	uint8_t requestAngles[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// Request bank 2, axis 1-3
	requestAngles[1] = LOWER;

	// Send frame requesting axis 1-3 positions 
	can1.sendFrame(channel[0], requestAngles);

	// Delay needed to prevent missed messages
	delay(MSGDELAY);

	// Get run time
	unsigned long timer = millis();

	// While loop until reply is received or timeout occurs
	while (isDone && (millis() - timer < 40))
	{
		// Request CAN frame addressed to paremeter ID
		uint8_t* temp = can1.getFrame(channel[1]);

		// If correct message is returned 
		if (temp[0] == LOWER)
		{
			// Check if reply was recieved
			isDone = can1.hasMSGr();

			// Determine which channel to write values too
			if (channel[1] == ARM1RXID)
			{
				// Two bytes per axis to reach max of 360 degrees
				a1c1 = (temp[2] * 255) + temp[3];
				a2c1 = (temp[4] * 255) + temp[5];
				a3c1 = (temp[6] * 255) + temp[7];
				isResponseCh1 = true;
			}
			else if (channel[1] == ARM2RXID)
			{
				a1c2 = (temp[2] * 255) + temp[3];
				a2c2 = (temp[4] * 255) + temp[5];
				a3c2 = (temp[6] * 255) + temp[7];
				isResponseCh2 = true;
			}
		}
	}
	
	// Reduce delay by skipping if arm is disconnected
	if (isResponseCh1 == true || isResponseCh2 == true)
	{
		// Always = true, hasMSGr resets to true for the CANBus class
		isDone = can1.hasMSGr();

		// Clear out old angle values
		// This is a bandaid for an undiscovered bug where axis 4-6 angles show up on the opposite channel when adding node to program
		can1.resetMSGFrame();

		// Request bank 2, axis 4-6
		requestAngles[1] = UPPER;

		// Send frame requesting axis 4-6 positions 
		can1.sendFrame(channel[0], requestAngles);

		// Delay needed to prevent missed messages
		delay(MSGDELAY);

		// Reset timer value to current
		timer = millis();

		// Repeat previous loop for axis 4-6
		while (isDone && (millis() - timer < 40))
		{
			uint8_t* temp = can1.getFrame(channel[1]);
			if (temp[0] == UPPER)
			{
				isDone = can1.hasMSGr();
				if (channel[1] == ARM1RXID)
				{
					a4c1 = (temp[2] * 255) + temp[3];
					a5c1 = (temp[4] * 255) + temp[5];
					a6c1 = (temp[6] * 255) + temp[7];
				}
				else if (channel[1] == ARM2RXID)
				{
					a4c2 = (temp[2] * 255) + temp[3];
					a5c2 = (temp[4] * 255) + temp[5];
					a6c2 = (temp[6] * 255) + temp[7];
				}
			}
		}
	}
}
*/

// Update and draw the Axis positions on the view page
void AxisPos::drawAxisPos(UTFT LCD)
{
	// Text color
	LCD.setColor(0xFFFF);

	// Text background color
	LCD.setBackColor(0xB5B5B5);

	String test = String(printf("%02d", a3c1));
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

/*
// Updates axis position without drawing
void AxisPos::updateAxisPos(CANBus can1)
{
	uint16_t channel1[2] = { ARM1ID, ARM1RXID };
	uint16_t channel2[2] = { ARM2ID, ARM2RXID };
	armSearch(can1, channel1);
	armSearch(can1, channel2);
}
*/

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
