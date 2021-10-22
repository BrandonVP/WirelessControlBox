/*
 Name:    CANBus.cpp
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/

// 5/20/2021
// CANBus hardware replaced with serial WiFi connection
// This class could be merged with SerialManager in the future
// 9/30/2021
// Merge in progress

#include "CANBus.h"
#include "extern.h"

can_buffer AxisPostionFrame;

// CAN Bus send message
void CANBus::sendFrame(CAN_Message frame)
{
    Serial3.write(0xFE);
    Serial3.write(0x09);
    Serial3.write(frame.id);
    for (uint8_t i = 0; i < ARRAY_SIZE; i++)
    {
        Serial3.write(frame.data[i]);
    }
    Serial3.write(0xFD);

    /*
      outgoing.id = id;
    outgoing.data[0] = frame[0];
    outgoing.data[1] = frame[1];
    outgoing.data[2] = frame[2];
    outgoing.data[3] = frame[3];
    outgoing.data[4] = frame[4];
    outgoing.data[5] = frame[5];
    outgoing.data[6] = frame[6];
    outgoing.data[7] = frame[7];
    Can0.sendFrame(outgoing);
    return;
    */
}

// CAN Bus send message
void CANBus::sendFrame(uint16_t id, byte* frame)
{
    outgoing.id = id;
    outgoing.data[0] = frame[0];
    outgoing.data[1] = frame[1];
    outgoing.data[2] = frame[2];
    outgoing.data[3] = frame[3];
    outgoing.data[4] = frame[4];
    outgoing.data[5] = frame[5];
    outgoing.data[6] = frame[6];
    outgoing.data[7] = frame[7];
    Can0.sendFrame(outgoing);
}

// Get and return message frame from specified rxID
uint8_t* CANBus::getFrame()
{
    return MSGFrame;
}

#define DEBUG_processFrame
// Sorts incoming CAN Bus messages
uint8_t CANBus::processFrame()
{
    bool hasMsg = Can0.readFrame(incoming);

    // If buffer inbox has a message
    if (hasMsg)
    {
#if defined DEBUG_processFrame
        Serial.print("ID: ");
        Serial.println(incoming.id, HEX);
#endif
        
        for (uint8_t i = 0; i < 8; i++)
        {
            MSGFrame[i] = incoming.data[i];
        }
        if (incoming.id == 0xC1)
        {
            // Arm 1 Axis Positions
            AxisPostionFrame.push(incoming);

#if defined DEBUG_processFrame
            Serial.print("Value: ");
            Serial.println((incoming.data[1]));
#endif
            return incoming.data[1];
        }
        if (incoming.id == 0xC2)
        {
            // Arm 2 Axis Positions
            AxisPostionFrame.push(incoming);
#if defined DEBUG_processFrame
            Serial.print("Value: ");
            Serial.println((incoming.data[1] + 3));
#endif
            return incoming.data[1] + 3;
        }
        if (incoming.id == 0xA1)
        {
#if defined DEBUG_processFrame
            Serial.print("Value: ");
            Serial.println(A1);
#endif
            return 0xA1;
        }
        if (incoming.id == 0xA0)
        {
#if defined DEBUG_processFrame
            Serial.print("Value: ");
            Serial.println(A0);
#endif
            return 0xA0;
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
