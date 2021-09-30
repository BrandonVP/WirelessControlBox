#pragma once
#ifndef COMMON_H
#define COMMON_H

#include "can_buffer.h"
#include "AxisPos.h"
#include <UTFT.h>

extern can_buffer AxisPostionFrame;
extern UTFT myGLCD;
extern CANBus can1;
extern AxisPos axisPos;
extern SDCard sdCard;

#endif // COMMON_H