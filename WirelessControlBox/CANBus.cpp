// CANBus manages the CAN bus hardware

// 5/20/2021
// CANBus hardware replaced with serial WiFi connection
// This class can be merged with SerialManager in the future

#include "CANBus.h"

// Default Constructor
CANBus::CANBus()
{
    // Currently unused
}

// CAN Bus send message
void CANBus::sendFrame(uint16_t id, byte* frame)
{
    outgoing.id = id;
    outgoing.byte[0] = frame[0];
    outgoing.byte[1] = frame[1];
    outgoing.byte[2] = frame[2];
    outgoing.byte[3] = frame[3];
    outgoing.byte[4] = frame[4];
    outgoing.byte[5] = frame[5];
    outgoing.byte[6] = frame[6];
    outgoing.byte[7] = frame[7];
    Can0.sendFrame(outgoing);
    return;
}

// Get and return message frame from specified rxID
uint8_t* CANBus::getFrame()
{
    return MSGFrame;
}

// Get and return message frame from specified rxID
uint8_t CANBus::processFrame()
{
    // If buffer inbox has a message
    if (Can0.byteInbox() > 0)
    {
        Can0.readFrame(incoming);
        Serial.print("ID: ");
        Serial.println(incoming.id, HEX);
        for (int i = 0; i < 8; i++)
        {
            MSGFrame[i] = incoming.byte[i];
        }
        if (incoming.id == 0xC1)
        {
            Serial.print("Value: ");
            Serial.println((incoming.byte[1]));
            return incoming.byte[1];
        }
        if (incoming.id == 0xC2)
        {
            Serial.print("Value: ");
            Serial.println((incoming.byte[1] + 3));
            return incoming.byte[1] + 3;
        }
    }
    return 0;
}

// Resets objects uint8_t array back to zero
void CANBus::resetMSGFrame()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        MSGFrame[i] = 0x00;
    }
}

// Check if value exists in incoming message, used for confirmation
bool CANBus::msgCheck(uint16_t ID, uint8_t value, int8_t pos)
{
    //Serial.println("msgCheck");
    // If buffer inbox has a message
    if (Can0.byteInbox() > 0)
    {
        Can0.readFrame(incoming);
        if (incoming.id == ID && incoming.byte[pos] == value)
        {
            return true;
        }  
    }
    return false;
}









// --------EVEYTHING BELOW LINE TAGGED TO BE REMOVED-----------
// Get frame ID, used for confirmation
uint16_t CANBus::getFrameID()
{
    // If buffer inbox has a message
    if (Can0.byteInbox() > 0)
    {
        Can0.readFrame(incoming);
    }
    return incoming.id;
}

// return current value and reset hasMSG to true
bool CANBus::hasMSGr() {
    bool temp = hasMSG;
    hasMSG = true;
    return temp;
}


bool CANBus::hasMessage()
{
    return Can0.byteInbox();
}