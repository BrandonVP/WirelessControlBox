// SerialManager.h
#pragma once
#ifndef _SerialManager_h
#define _SerialManager_h

//#include "common.h"
#include "can_buffer.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define ARRAY_SIZE 8

// States
#define START_BYTE              (0)
#define PACKET_LENGTH           (1)
#define CAN_BUS_ID1              (2)
#define CAN_BUS_ID2              (3)
#define CAN_BUS_DATA            (4)
#define END_BYTE                (5)
// 
#define STARTING_BYTE           (0xFE)
#define ENDING_BYTE             (0xFD)
#define PACKET_SIZE             (0x09)

class SerialManager
{
private:
	uint8_t state = 0;
	uint8_t packetIndex = 0;
	CAN_Message CAN_FRAME1;
public:
	bool readFrame(CAN_Message&);
	void sendFrame(CAN_Message);
};
#endif
