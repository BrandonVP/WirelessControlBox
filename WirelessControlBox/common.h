#pragma once
#ifndef COMMON_H
#define COMMON_H

//#define DEBUG(x)  SerialUSB.print(x);
//#define DEBUGLN(x)  SerialUSB.println(x);
#define DEBUG(x)  Serial.print(x);
#define DEBUGLN(x)  Serial.println(x);
//#define DEBUG(x)
//#define DEBUGLN(x)

// How many programs can be saved to SD card
#define MAX_PROGRAMS 16

extern String programNames_G[MAX_PROGRAMS];
extern uint8_t numberOfPrograms;
extern char fileList[MAX_PROGRAMS][8];


#include "can_buffer.h"
#include "AxisPos.h"
#include <UTFT.h>

extern can_buffer AxisPostionFrame;
extern UTFT myGLCD;
extern CANBus can1;
extern AxisPos axisPos;
extern SDCard sdCard;

#endif // COMMON_H