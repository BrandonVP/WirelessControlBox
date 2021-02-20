// AxisPos.h
#include "CANBus.h"
#include <UTFT.h>
#ifndef _AxisPos_h
#define _AxisPos_h
#define LOWER 0x01
#define ARM1ID 0xA0
#define ARM1RXID 0xC1
#define UPPER 0x02
#define ARM2ID 0x0B0
#define ARM2RXID 0xC2
#define MSGDELAY 0x0A


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class CANBus;

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

	 // Objects to request CAN messages and write to LCD
	 CANBus can1;
	 UTFT LCD;

	 // Used to enable writing angle values to LCD if the controller responded with the requested angles
	 bool isResponseCh1 = false;
	 bool isResponseCh2 = false;

 public:
	AxisPos();
	void drawAxisPos(UTFT, bool);
	void updateAxisPos();
	void armSearch(uint16_t*);
	int getA1C1();
	int getA2C1();
	int getA3C1();
	int getA4C1();
	int getA5C1();
	int getA6C1();
	int getA1C2();
	int getA2C2();
	int getA3C2();
	int getA4C2();
	int getA5C2();
	int getA6C2();
};
#endif
