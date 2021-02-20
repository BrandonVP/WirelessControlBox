// SDCard.h
#include <LinkedList.h>
#include "Program.h"

#ifndef _SDCard_h
#define _SDCard_h
#define SD_CARD_CS 8

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
//class CANBus;

class SDCard
{
 protected:


 public:
	bool startSD();
	void writeFile(String, uint8_t);
	void writeFile(String, String);
	void writeFile(char*, int, int);
	void writeFile(char*, String);
	void writeFileln(String);
	void deleteFile(String);
	void readFile(String, LinkedList<Program*> &);
	bool fileExists(String filename);
};

#endif
