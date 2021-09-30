#pragma once
#ifndef EXTERN_H
#define EXTERN_H

#include "can_buffer.h"
#include "AxisPos.h"
#include <UTFT.h>

extern can_buffer AxisPostionFrame;
extern UTFT myGLCD;
extern CANBus can1;
extern AxisPos axisPos;
extern SDCard sdCard;

#endif // EXTERN_H