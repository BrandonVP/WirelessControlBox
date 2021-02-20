// 
// 
// 

#include "Program.h"

Program::Program(uint16_t* posArray, uint8_t grip, uint8_t ch)
{
	a1 = posArray[0];
	a2 = posArray[1];
	a3 = posArray[2];
	a4 = posArray[3];
	a5 = posArray[4];
	a6 = posArray[5];
	gripStatus = grip;
	channel = ch;
}

uint16_t Program::getA1()
{
	return a1;
}
uint16_t Program::getA2()
{
	return a2;
}
uint16_t Program::getA3()
{
	return a3;
}
uint16_t Program::getA4()
{
	return a4;
}
uint16_t Program::getA5()
{
	return a5;
}
uint16_t Program::getA6()
{
	return a6;
}
uint8_t Program::getGrip()
{
	return gripStatus;
}
uint16_t Program::getID()
{
	return channel;
}