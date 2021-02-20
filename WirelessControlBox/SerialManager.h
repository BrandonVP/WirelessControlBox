// SerialManager.h

#ifndef _SerialManager_h
#define _SerialManager_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

typedef struct CAN_FRAME1 {
	uint16_t id;
	uint8_t byte[8];
} CAN_FRAME1;

class SerialManager
{
private:


public:
	uint8_t available();
	void startSerial();
	void read(CAN_FRAME1&);
	void sendFrame(CAN_FRAME1);
};

#endif

