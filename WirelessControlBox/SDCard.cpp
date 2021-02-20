// SDCard class manages the SD card reader hardware

#include <SD.h>
#include "SDCard.h"
#include <string>

// File object
File myFile;

// Called at setup to initialize the SD Card
bool SDCard::startSD()
{
    if (!SD.begin(SD_CARD_CS)) {
        return false;
    }
    return true;
}

/*=========================================================
    Write File Methods
===========================================================*/
// Write string to SD Card
void SDCard::writeFile(char* filename, String incoming)
{
    // File created and opened for writing
    myFile = SD.open(filename, FILE_WRITE);

    // Check if file was sucsefully open
    if (myFile)
    {
        myFile.print(incoming);
        myFile.close();
    }
    return;
}

// Write string to SD Card
void SDCard::writeFile(String filename, String incoming)
{
    // File created and opened for writing
    myFile = SD.open(filename, FILE_WRITE);

    // Check if file was sucsefully open
    if (myFile)
    {
        myFile.print(incoming);
        myFile.close();
    }
    return;
}

void SDCard::writeFile(String filename, uint8_t incoming)
{
    // File created and opened for writing
    myFile = SD.open(filename, FILE_WRITE);

    // Check if file was sucsefully open
    if (myFile)
    {
        myFile.print(incoming);
        myFile.close();
    }
    return;
}

// Write integer and base to SD Card
void SDCard::writeFile(char* filename, int incoming, int base)
{
    // File created and opened for writing
    myFile = SD.open(filename, FILE_WRITE);

    // Check if file was sucsefully open
    if (myFile)
    {
        myFile.print(incoming, base);
        myFile.close();
    }
    return;
}

// Write return to SD Card file
void SDCard::writeFileln(String filename)
{
    // File created and opened for writing
    myFile = SD.open(filename, FILE_WRITE);

    // Check if file was sucsefully open
    if (myFile)
    {
        myFile.println(" ");
        myFile.close();
    }
    return;
}


/*=========================================================
    Delete File Methods
===========================================================*/
// Delete SD Card file
void SDCard::deleteFile(String filename)
{
    //remove any existing file with this name
    SD.remove(filename);
}

// Check if file exists
bool SDCard::fileExists(String filename)
{
    myFile = SD.open(filename);
    bool value = myFile.available();
    myFile.close();
    return value;
}

// Read in program from SD
void SDCard::readFile(String filename, LinkedList<Program*> &runList)
{
    String tempStr;
    uint16_t posArray[6];
    uint8_t channel;
    uint8_t grip;
    char c[20];
    myFile = SD.open(filename);
    myFile.readStringUntil(',');
    while (myFile.available()) {
        for (int i = 0; i < 8; i++)
        {
            tempStr = (myFile.readStringUntil(','));
           strcpy(c, tempStr.c_str());
           if (i < 6)
           {
               posArray[i] = atoi(c);
           }
           if (i == 6)
           {
               channel = atoi(c);
           }
           if (i == 7)
           {
               grip = atoi(c);
           }
           
        }
        for (uint8_t i = 0; i < 6; i++)
        {
            Serial.println(posArray[i]);
        }
            Serial.println(channel);
            Serial.println(grip);
            Serial.println("");
        
            Program* node = new Program(posArray, grip, channel);
            runList.add(node);
        
    }
    myFile.close();
}