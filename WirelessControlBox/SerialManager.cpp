// 
// 
// Serial out debugging will make program unstable / lock up

#include "SerialManager.h"

void SerialManager::startSerial()
{
	//Serial3.begin(19200); 
}


bool SerialManager::byteInbox()
{
	if (Serial3.available() > 0)
	{
		Serial.print("Bytes in Que: ");
		Serial.println(Serial3.available());
	}
	if (Serial3.available() >= 10)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SerialManager::readFrame(CAN_FRAME1 &rxCAN)
{
	//if ((Serial3.available() >= 9) && (Serial3.read() == FLOW_CONTROL_VALUE))
	if ((Serial3.available() >= 10))
	{
		while (Serial3.read() != FLOW_CONTROL_VALUE) 
		{
			// Read until flow control value is hit
			// This auto aligns the messages
		}
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