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

void SerialManager::read(CAN_FRAME1 & rxCAN)
{
	if (Serial3.available() > 9)
	{
		uint8_t test = Serial3.read();
		if ( test == 0xFF)
		{
			rxCAN.id = Serial3.read();
			rxCAN.byte[0] = Serial3.read();
			rxCAN.byte[1] = Serial3.read();
			rxCAN.byte[2] = Serial3.read();
			rxCAN.byte[3] = Serial3.read();
			rxCAN.byte[4] = Serial3.read();
			rxCAN.byte[5] = Serial3.read();
			rxCAN.byte[6] = Serial3.read();
			rxCAN.byte[7] = Serial3.read();
		}
	}
}

void SerialManager::sendFrame(CAN_FRAME1 txCAN)
{
	Serial3.write(0xFF);
	Serial3.write(txCAN.id);
	Serial3.write(txCAN.byte[0]);
	Serial3.write(txCAN.byte[1]);
	Serial3.write(txCAN.byte[2]);
	Serial3.write(txCAN.byte[3]);
	Serial3.write(txCAN.byte[4]);
	Serial3.write(txCAN.byte[5]);
	Serial3.write(txCAN.byte[6]);
	Serial3.write(txCAN.byte[7]);
}