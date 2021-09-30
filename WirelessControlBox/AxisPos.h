/*
 Name:    AxisPos.h
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/

#include "CANBus.h"
#include <UTFT.h>
#include "can_buffer.h"

#ifndef _AXISPOS_H
#define _AXISPOS_H

#define LOWER 0x01
#define ARM1ID 0xA0
#define ARM1RXID 0xC1
#define UPPER 0x02
#define ARM2ID 0xB0
#define ARM2RXID 0xC2
#define MSGDELAY 0x0A

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class AxisPos
{
 private:
	 // Angle values for channel 1
	 uint16_t a1c1 = 0;
	 uint16_t a2c1 = 0;
	 uint16_t a3c1 = 0;
	 uint16_t a4c1 = 0;
	 uint16_t a5c1 = 0;
	 uint16_t a6c1 = 0;
	 // Angle values for channel 2
	 uint16_t a1c2 = 0;
	 uint16_t a2c2 = 0;
	 uint16_t a3c2 = 0;
	 uint16_t a4c2 = 0;
	 uint16_t a5c2 = 0;
	 uint16_t a6c2 = 0;

	 // Draw angle values for channel 1
	 bool draw_a1c1 = false;
	 bool draw_a2c1 = false;
	 bool draw_a3c1 = false;
	 bool draw_a4c1 = false;
	 bool draw_a5c1 = false;
	 bool draw_a6c1 = false;
	 // Draw angle values for channel 2
	 bool draw_a1c2 = false;
	 bool draw_a2c2 = false;
	 bool draw_a3c2 = false;
	 bool draw_a4c2 = false;
	 bool draw_a5c2 = false;
	 bool draw_a6c2 = false;

 public:
	void drawAxisPosUpdate();
	void drawAxisPos();
	void updateAxisPos();
	void sendRequest(CANBus);
	uint16_t getA1C1();
	uint16_t getA2C1();
	uint16_t getA3C1();
	uint16_t getA4C1();
	uint16_t getA5C1();
	uint16_t getA6C1();
	uint16_t getA1C2();
	uint16_t getA2C2();
	uint16_t getA3C2();
	uint16_t getA4C2();
	uint16_t getA5C2();
	uint16_t getA6C2();
};
#endif // _AXISPOS_H
