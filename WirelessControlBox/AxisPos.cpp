/*
 Name:    AxisPos.cpp
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/

#include <due_can.h>
#include "variant.h"
#include "AxisPos.h"
#include "extern.h"

CAN_Message AxisBuffer;

// Manually request axis angle from it's controller
// Not used anymore because the controller now broadcasts it
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

//#define DEBUG_updateAxisPos
// Update axis positions from incoming CAN messages
void AxisPos::updateAxisPos()
{
	uint16_t result = 0;

	if (AxisPostionFrame.stack_size() > 0)
	{
		AxisPostionFrame.pop(&AxisBuffer);
	}
	else
	{
		return;
	}
	
	// Process lower axis positions
	if (AxisBuffer.data[1] == LOWER)
	{
		// Determine which channel to write values too
		if (AxisBuffer.id == ARM1RXID)
		{
#if defined DEBUG_updateAxisPos
			Serial.println("Arm 1 Lower");
#endif
			// Two bytes per axis to reach max of 360 degrees
			result = (AxisBuffer.data[2] * 255) + AxisBuffer.data[3];
			if (a1c1 != result)
			{
				a1c1 = result;
				draw_a1c1 = true;
			}
			result = (AxisBuffer.data[4] * 255) + AxisBuffer.data[5];
			if (a2c1 != result)
			{
				a2c1 = result;
				draw_a2c1 = true;
			}
			result = (AxisBuffer.data[6] * 255) + AxisBuffer.data[7];
			if (a3c1 != result)
			{
				a3c1 = result;
				draw_a3c1 = true;
			}
		}
		else if (AxisBuffer.id == ARM2RXID)
		{
#if defined DEBUG_updateAxisPos
			Serial.println("Arm 2 Lower");
#endif
			result = (AxisBuffer.data[2] * 255) + AxisBuffer.data[3];
			if (a1c2 != result)
			{
				a1c2 = result;
				draw_a1c2 = true;
			}
			result = (AxisBuffer.data[4] * 255) + AxisBuffer.data[5];
			if (a2c2 != result)
			{
				a2c2 = result;
				draw_a2c2 = true;
			}
			result = (AxisBuffer.data[6] * 255) + AxisBuffer.data[7];
			if (a3c2 != result)
			{
				a3c2 = result;
				draw_a3c2 = true;
			}
		}
	}
	if (AxisBuffer.data[1] == UPPER)
	{
		if (AxisBuffer.id == ARM1RXID)
		{
			result = (AxisBuffer.data[2] * 255) + AxisBuffer.data[3];
			if (a4c1 != result)
			{
				a4c1 = result;
				draw_a4c1 = true;
			}
			result = (AxisBuffer.data[4] * 255) + AxisBuffer.data[5];
			if (a5c1 != result)
			{
				a5c1 = result;
				draw_a5c1 = true;
			}
			result = (AxisBuffer.data[6] * 255) + AxisBuffer.data[7];
			if (a6c1 != result)
			{
				a6c1 = result;
				draw_a6c1 = true;
			}
#if defined DEBUG_updateAxisPos
			Serial.println("Arm 1 Upper");
#endif
		}
		else if (AxisBuffer.id == ARM2RXID)
		{
			result = (AxisBuffer.data[2] * 255) + AxisBuffer.data[3];
			if (a4c2 != result)
			{
				a4c2 = result;
				draw_a4c2 = true;
			}
			result = (AxisBuffer.data[4] * 255) + AxisBuffer.data[5];
			if (a5c2 != result)
			{
				a5c2 = result;
				draw_a5c2 = true;
			}
			result = (AxisBuffer.data[6] * 255) + AxisBuffer.data[7];
			if (a6c2 != result)
			{
				a6c2 = result;
				draw_a6c2 = true;
			}
#if defined DEBUG_updateAxisPos
			Serial.println("Arm 2 Upper");
#endif
		}
	}
}

// Update and draw the Axis positions on the view page
void AxisPos::drawAxisPosUpdate()
{
	// Text color
	myGLCD.setColor(0xFFFF);

	// Text background color
	myGLCD.setBackColor(0xB5B5B5);

	// Draw angles 
	if (draw_a1c1 && a1c1 < 361)
	{
		uint16_t temp = a1c1;
		myGLCD.printNumI(temp, 205, 48, 3, '0');
		draw_a1c1 = false;
	}
	if (draw_a2c1 && a2c1 < 361)
	{
		myGLCD.printNumI(a2c1, 205, 93, 3, '0');
		draw_a2c1 = false;
	}
	if (draw_a3c1 && a3c1 < 361)
	{
		myGLCD.printNumI(a3c1, 205, 138, 3, '0');
		draw_a3c1 = false;
	}
	if (draw_a4c1 && a4c1 < 361)
	{
		myGLCD.printNumI(a4c1, 205, 183, 3, '0');
		draw_a4c1 = false;
	}
	if (draw_a5c1 && a5c1 < 361)
	{
		myGLCD.printNumI(a5c1, 205, 228, 3, '0');
		draw_a5c1 = false;
	}
	if (draw_a6c1 && a6c1 < 361)
	{
		myGLCD.printNumI(a6c1, 205, 273, 3, '0');
		draw_a6c1 = false;
	}
	if (draw_a1c2 && a1c2 < 361)
	{
		uint16_t temp = a1c2;
		myGLCD.printNumI(temp, 315, 48, 3, '0');
		draw_a1c2 = false;
	}
	if (draw_a2c2 && a2c2 < 361)
	{
		myGLCD.printNumI(a2c2, 315, 93, 3, '0');
		draw_a2c2 = false;
	}
	if (draw_a3c2 && a3c2 < 361)
	{
		myGLCD.printNumI(a3c2, 315, 138, 3, '0');
		draw_a3c2 = false;
	}
	if (draw_a4c2 && a4c2 < 361)
	{
		myGLCD.printNumI(a4c2, 315, 183, 3, '0');
		draw_a4c2 = false;
	}
	if (draw_a5c2 && a5c2 < 361)
	{
		myGLCD.printNumI(a5c2, 315, 228, 3, '0');
		draw_a5c2 = false;
	}
	if (draw_a6c2 && a6c2 < 361)
	{
		myGLCD.printNumI(a6c2, 315, 273, 3, '0');
		draw_a6c2 = false;
	}
}

// Update and draw the Axis positions on the view page
void AxisPos::drawAxisPos()
{
	// Text color
	myGLCD.setColor(0xFFFF);

	// Text background color
	myGLCD.setBackColor(0xB5B5B5);

	myGLCD.printNumI(a1c1, 205, 48, 3, '0');
	myGLCD.printNumI(a2c1, 205, 93, 3, '0');
	myGLCD.printNumI(a3c1, 205, 138, 3, '0');
	myGLCD.printNumI(a4c1, 205, 183, 3, '0');
	myGLCD.printNumI(a5c1, 205, 228, 3, '0');
	myGLCD.printNumI(a6c1, 205, 273, 3, '0');
	myGLCD.printNumI(a1c2, 315, 48, 3, '0');
	myGLCD.printNumI(a2c2, 315, 93, 3, '0');
	myGLCD.printNumI(a3c2, 315, 138, 3, '0');
	myGLCD.printNumI(a4c2, 315, 183, 3, '0');
	myGLCD.printNumI(a5c2, 315, 228, 3, '0');
	myGLCD.printNumI(a6c2, 315, 273, 3, '0');
}

// Returns the saved position of axis 1 channel 1 
uint16_t AxisPos::getA1C1()
{
	return a1c1;
}

// Returns the saved position of axis 2 channel 1 
uint16_t AxisPos::getA2C1()
{
	return a2c1;
}

// Returns the saved position of axis 3 channel 1 
uint16_t AxisPos::getA3C1()
{
	return a3c1;
}

// Returns the saved position of axis 4 channel 1 
uint16_t AxisPos::getA4C1()
{
	return a4c1;
}

// Returns the saved position of axis 5 channel 1 
uint16_t AxisPos::getA5C1()
{
	return a5c1;
}

// Returns the saved position of axis 6 channel 1 
uint16_t AxisPos::getA6C1()
{
	return a6c1;
}

// Returns the saved position of axis 1 channel 2 
uint16_t AxisPos::getA1C2()
{
	return a1c2;
}

// Returns the saved position of axis 2 channel 2 
uint16_t AxisPos::getA2C2()
{
	return a2c2;
}

// Returns the saved position of axis 3 channel 2 
uint16_t AxisPos::getA3C2()
{
	return a3c2;
}

// Returns the saved position of axis 4 channel 2 
uint16_t AxisPos::getA4C2()
{
	return a4c2;
}

// Returns the saved position of axis 5 channel 2 
uint16_t AxisPos::getA5C2()
{
	return a5c2;
}

// Returns the saved position of axis 6 channel 2 
uint16_t AxisPos::getA6C2()
{
	return a6c2;
}
