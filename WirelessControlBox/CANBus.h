/*
 Name:    CANBus.h
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/

#include "SerialManager.h"
#include "SDCard.h"
#include "can_buffer.h"
#ifndef _CANBus_h
#define _CANBus_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class Program;
class SerialManager;

class CANBus
{
 private:
	 CAN_Message incoming;
	 CAN_Message outgoing;
	 uint8_t MSGFrame[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	 typedef byte frame[8];
	 SerialManager Can0;
 public:
	void sendFrame(uint16_t, byte*);
	void sendFrame(CAN_Message);
	uint8_t* getFrame();
	void resetMSGFrame();
	uint8_t processFrame();
	//void start();
};
#endif
