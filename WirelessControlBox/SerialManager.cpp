// 
// 
// 

#include "SerialManager.h"

void SerialManager::startSerial()
{
	//Serial3.begin(19200); // Tx ok, Rx ok
}


uint8_t SerialManager::available()
{
	Serial.println(Serial3.available());
	if (Serial3.available() > 9)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

void SerialManager::readFrame(CAN_FRAME1 &rxCAN)
{
	if ((Serial3.available() > SIZE_OF_FRAME) && (Serial3.read() == FLOW_CONTROL_VALUE))
	{
		rxCAN.id = Serial3.read();
		for (uint8_t i = 0; i < ARRAY_SIZE; i++)
		{
			rxCAN.byte[i] = Serial3.read();
		}
	}
}

void SerialManager::sendFrame(CAN_FRAME1 txCAN)
{
	Serial3.write(0xFF);
	Serial3.write(txCAN.id);
	for (uint8_t i = 0; i < ARRAY_SIZE; i++)
	{
		Serial3.write(txCAN.byte[i]);
	}
}