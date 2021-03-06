// SerialManager.h

#ifndef _SerialManager_h
#define _SerialManager_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define FLOW_CONTROL_VALUE 0xFF
#define SIZE_OF_FRAME 10
#define ARRAY_SIZE 8

typedef struct CAN_FRAME1 {
	uint16_t id;
	uint8_t byte[8];
} CAN_FRAME1;

class SerialManager
{
private:

public:
	bool byteInbox();
	void startSerial();
	void readFrame(CAN_FRAME1&);
	void sendFrame(CAN_FRAME1);
};
#endif
