#include "SerialManager.h"

#define DEBUG_READFRAME
//#define DEBUG_SENDFRAME

// Returns true when full packet is recieved
bool SerialManager::readFrame(CAN_Message &rxCAN)
{
    if (Serial3.available() > 0)
    {
        uint8_t recByte = Serial3.read();
        switch (state)
        {
        case START_BYTE:
#if defined DEBUG_READFRAME
            Serial.print("STARTING_BYTE: ");
            Serial.println(recByte, 16);
#endif
            if (recByte == STARTING_BYTE)
            {
                state = PACKET_LENGTH;
                return false;
            }
            break;
        case PACKET_LENGTH:
#if defined DEBUG_READFRAME
            Serial.print("PACKET_LENGTH: ");
            Serial.println(recByte, 16);
#endif
            state = CAN_BUS_ID1;
            if (recByte == PACKET_SIZE)
            {
                packetIndex = 0;
                return false;
            }
            else
            {
                // Bad packet
                state = START_BYTE;
            }
            break;
        case CAN_BUS_ID1:
#if defined DEBUG_READFRAME
            Serial.print("CAN_BUS_I1D: ");
            Serial.println(recByte, 16);
#endif
            rxCAN.id = recByte;
            state = CAN_BUS_ID2;
            break;
        case CAN_BUS_ID2:
#if defined DEBUG_READFRAME
            Serial.print("CAN_BUS_ID2: ");
            Serial.println(recByte, 16);
#endif
            rxCAN.id += recByte;
            state = CAN_BUS_DATA;
            break;
        case CAN_BUS_DATA:
#if defined DEBUG_READFRAME
            Serial.print("CAN_BUS_DATA: ");
            Serial.println(recByte, 16);
#endif
            rxCAN.data[packetIndex] = recByte;
            packetIndex++;
            if (packetIndex == PACKET_SIZE - 1)
            {
                state = END_BYTE;
            }
            break;
        case END_BYTE:
#if defined DEBUG_READFRAME
            Serial.print("END_BYTE: ");
            Serial.println(recByte, 16);
#endif
            if (recByte == ENDING_BYTE)
            {
                state = START_BYTE;
                
#if defined DEBUG_READFRAME
                Serial.println("RECEIVED");
                Serial.println("");
#endif
                return true;
            }
            else
            {
                // packet failed restart
                state = START_BYTE;
#if defined DEBUG_READFRAME
                Serial.println("FAILED");
#endif
            }
            break;
        }
    }
    return false;
}

//
void SerialManager::sendFrame(CAN_Message txCAN)
{
    const byte ID_FILLER = 0x00;
	Serial3.write(STARTING_BYTE);
	Serial3.write(PACKET_SIZE);
	Serial3.write(txCAN.id);
    Serial3.write(0x300);
    //Serial3.write(ID_FILLER);
	for (uint8_t i = 0; i < ARRAY_SIZE; i++)
	{
		Serial3.write(txCAN.data[i]);
	}
	Serial3.write(ENDING_BYTE);
}