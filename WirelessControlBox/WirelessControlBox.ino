/*
 Name:    ControlBoxDue.ino
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/
#pragma once
#include <Wire.h>
#include <LinkedList.h>
#include <memorysaver.h>
#include <SD.h>
#include <SPI.h>
#include <UTFT.h>
#include "LCD.h"
#include "AxisPos.h"
#include "CANBus.h"
#include "SDCard.h"
#include "Program.h"
#include "definitions.h"
#include "icons.h"
#include <stdint.h>

// For touch controls
int x, y;

// External import for fonts
extern uint8_t SmallFont[];
extern uint8_t BigFont[];

// Object to control CAN Bus hardware
CANBus can1;

// Object to control SD Card Hardware
SDCard sdCard;

// AxisPos object used to get current angles of robot arm
AxisPos axisPos;

// Linked list of nodes for a program
LinkedList<Program*> runList = LinkedList<Program*>();

// Current selected program
uint8_t selectedProgram = 0;

// ---THIS CAN BE REWORKED TO SAVE MEMORY---
// CAN message ID and frame, value can be changed in manualControlButtons
uint16_t txIdManual = ARM1_M;

// Execute variables
bool programLoaded = false;
bool loopProgram = true;
bool programRunning = false;
bool Arm1Ready = false;
bool Arm2Ready = false;
// ---THIS WILL LIMIT THE SIZE OF A PROGRAM TO 255 STEPS---
uint8_t programProgress = 0;

// Used hold open a program
bool programOpen = false;

// 0 = open, 1 = close, 2 = no change
int8_t gripStatus = 2;

// Program names
String aList[10] = { "Program1", "Program2", "Program3", "Program4", "Program5", "Program6", "Program7", "Program8", "Program9", "Program10" };

// Page control variables
uint8_t page = 1;
uint8_t oldPage = 1;
bool hasDrawn = false;

// Timer for currant angle updates
uint32_t timer = 0;

/*=========================================================
    Framework Functions
===========================================================*/
//
void bmpDraw(char* filename, int x, int y) {
    File     bmpFile;
    int      bmpWidth, bmpHeight;   // W+H in pixels
    uint8_t  bmpDepth;              // Bit depth (currently must be 24)
    uint32_t bmpImageoffset;        // Start of image data in file
    uint32_t rowSize;               // Not always = bmpWidth; may have padding
    uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
    uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
    uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
    boolean  goodBmp = false;       // Set to true on valid header parse
    boolean  flip = true;        // BMP is stored bottom-to-top
    int      w, h, row, col;
    uint8_t  r, g, b;
    uint32_t pos = 0, startTime = millis();
    uint8_t  lcdidx = 0;
    boolean  first = true;
    int dispx = myGLCD.getDisplayXSize();
    int dispy = myGLCD.getDisplayYSize();

    if ((x >= dispx) || (y >= dispy)) return;

    Serial.println();
    Serial.print(F("Loading image '"));
    Serial.print(filename);
    Serial.println('\'');

    // Open requested file on SD card
    if ((bmpFile = SD.open(filename)) == NULL) {
        Serial.println(F("File not found"));
        return;
    }


    // Parse BMP header
    if (read16(bmpFile) == 0x4D42) { // BMP signature

        Serial.println(read32(bmpFile));
        (void)read32(bmpFile); // Read & ignore creator bytes
        bmpImageoffset = read32(bmpFile); // Start of image data
        Serial.print(F("Image Offset: "));
        Serial.println(bmpImageoffset, DEC);

        // Read DIB header
        Serial.print(F("Header size: "));
        Serial.println(read32(bmpFile));
        bmpWidth = read32(bmpFile);
        bmpHeight = read32(bmpFile);

        if (read16(bmpFile) == 1) { // # planes -- must be '1'
            bmpDepth = read16(bmpFile); // bits per pixel
            Serial.print(F("Bit Depth: "));
            Serial.println(bmpDepth);
            if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
                goodBmp = true; // Supported BMP format -- proceed!
                Serial.print(F("Image size: "));
                Serial.print(bmpWidth);
                Serial.print('x');
                Serial.println(bmpHeight);

                // BMP rows are padded (if needed) to 4-byte boundary
                rowSize = (bmpWidth * 3 + 3) & ~3;

                // If bmpHeight is negative, image is in top-down order.
                // This is not canon but has been observed in the wild.
                if (bmpHeight < 0) {
                    bmpHeight = -bmpHeight;
                    flip = false;
                }

                // Crop area to be loaded
                w = bmpWidth;
                h = bmpHeight;
                if ((x + w - 1) >= dispx)  w = dispx - x;
                if ((y + h - 1) >= dispy) h = dispy - y;

                // Set TFT address window to clipped image bounds
                for (row = 0; row < h; row++) { // For each scanline...
                  // Seek to start of scan line.  It might seem labor-
                  // intensive to be doing this on every line, but this
                  // method covers a lot of gritty details like cropping
                  // and scanline padding.  Also, the seek only takes
                  // place if the file position actually needs to change
                  // (avoids a lot of cluster math in SD library).
                    if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
                        pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
                    else     // Bitmap is stored top-to-bottom
                        pos = bmpImageoffset + row * rowSize;
                    if (bmpFile.position() != pos) { // Need seek?
                        bmpFile.seek(pos);
                        buffidx = sizeof(sdbuffer); // Force buffer reload
                    }
                    for (col = 0; col < w; col++) { // For each column...
                      // Time to read more pixel data?
                        if (buffidx >= sizeof(sdbuffer)) { // Indeed
                          // Push LCD buffer to the display first
                            if (lcdidx > 0) {
                                myGLCD.setColor(lcdbuffer[lcdidx]);
                                myGLCD.drawPixel(col + 250, row + 1);
                                lcdidx = 0;
                                first = false;
                            }
                            bmpFile.read(sdbuffer, sizeof(sdbuffer));
                            buffidx = 0; // Set index to beginning
                        }

                        // Convert pixel from BMP to TFT format
                        b = sdbuffer[buffidx++];
                        g = sdbuffer[buffidx++];
                        r = sdbuffer[buffidx++];
                        myGLCD.setColor(r, g, b);
                        myGLCD.drawPixel(col + 250, row + 1);

                    } // end pixel
                } // end scanline

                // Write any remaining data to LCD
                if (lcdidx > 0) {
                    myGLCD.setColor(lcdbuffer[lcdidx]);
                    myGLCD.drawPixel(col + 250, row + 1);
                }
            } // end goodBmp
        }
    }

    bmpFile.close();
    if (!goodBmp) Serial.println(F("BMP format not recognized."));

}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
uint16_t read16(File f) {
    uint16_t result;
    ((uint8_t*)&result)[0] = f.read(); // LSB
    ((uint8_t*)&result)[1] = f.read(); // MSB
    return result;
}
uint32_t read32(File f) {
    uint32_t result;
    ((uint8_t*)&result)[0] = f.read(); // LSB
    ((uint8_t*)&result)[1] = f.read();
    ((uint8_t*)&result)[2] = f.read();
    ((uint8_t*)&result)[3] = f.read(); // MSB
    return result;
}


// Custom bitmap
void print_icon(int x, int y, const unsigned char icon[]) {
    myGLCD.setColor(menuBtnColor);
    myGLCD.setBackColor(themeBackground);
    int i = 0, row, column, bit, temp;
    int constant = 1;
    for (row = 0; row < 40; row++) {
        for (column = 0; column < 5; column++) {
            temp = icon[i];
            for (bit = 7; bit >= 0; bit--) {

                if (temp & constant) {
                    myGLCD.drawPixel(x + (column * 8) + (8 - bit), y + row);
                }
                else {

                }
                temp >>= 1;
            }
            i++;
        }
    }
}

/***************************************************
*  Draw Round/Square Button                        *
*                                                  *
*  Description:   Draws shapes with/without text   *
*                                                  *
*  Parameters: x start, y start, x stop, y stop    *
*              String: Button text                 *
*              Hex value: Background Color         *
*              Hex value: Border of shape          *
*              Hex value: Color of text            *
*              int: Alignment of text #defined as  *
*                   LEFT, CENTER, RIGHT            *
*                                                  *
***************************************************/
void drawRoundBtn(int x_start, int y_start, int x_stop, int y_stop, String button, int backgroundColor, int btnBorderColor, int btnTxtColor, int align) {
    int size, temp, offset;

    //myGLCD.setColor(backgroundColor);
    myGLCD.setColor(backgroundColor);
    myGLCD.fillRoundRect(x_start, y_start, x_stop, y_stop); // H_Start, V_Start, H_Stop, V_Stop
    myGLCD.setColor(btnBorderColor);
    myGLCD.drawRoundRect(x_start, y_start, x_stop, y_stop);
    myGLCD.setColor(btnTxtColor); // text color
    myGLCD.setBackColor(backgroundColor); // text background
    switch (align)
    {
    case 1:
        myGLCD.print(button, x_start + 5, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 2:
        size = button.length();
        temp = ((x_stop - x_start) / 2);
        offset = x_start + (temp - (8 * size));
        myGLCD.print(button, offset, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 3:
        // Currently hotwired for deg text only
        myGLCD.print(button, x_start + 55, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    default:
        break;
    }

}

void drawSquareBtn(int x_start, int y_start, int x_stop, int y_stop, String button, int backgroundColor, int btnBorderColor, int btnTxtColor, int align) {
    int size, temp, offset;
    myGLCD.setColor(backgroundColor);
    myGLCD.fillRect(x_start, y_start, x_stop, y_stop); // H_Start, V_Start, H_Stop, V_Stop
    myGLCD.setColor(btnBorderColor);
    myGLCD.drawRect(x_start, y_start, x_stop, y_stop);
    myGLCD.setColor(btnTxtColor); // text color
    myGLCD.setBackColor(backgroundColor); // text background
    switch (align)
    {
    case 1:
        myGLCD.print(button, x_start + 5, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 2:
        size = button.length();
        temp = ((x_stop - x_start) / 2);
        offset = x_start + (temp - (8 * size));
        myGLCD.print(button, offset, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 3:
        //align left
        break;
    default:
        break;
    }
}

// Highlights round buttons when selected
void waitForIt(int x1, int y1, int x2, int y2)
{
    myGLCD.setColor(themeBackground);
    myGLCD.drawRoundRect(x1, y1, x2, y2);
    
    uint8_t  ss[1];
    unsigned long timer = millis();
    while ((millis() - timer < 80))
    {
        readGT9271TouchAddr(0x814e, ss, 1);
        if ((ss[0] & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            timer = millis();
        }
    }

    myGLCD.setColor(menuBtnBorder);
    myGLCD.drawRoundRect(x1, y1, x2, y2);
}

// Highlights square buttons when selected
void waitForItRect(int x1, int y1, int x2, int y2)
{
    myGLCD.setColor(themeBackground);
    myGLCD.drawRect(x1, y1, x2, y2);

    uint8_t  ss[1];
    unsigned long timer = millis();
    while ((millis() - timer < 80))
    {
        readGT9271TouchAddr(0x814e, ss, 1);
        if ((ss[0] & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            timer = millis();
        }
    }

    myGLCD.setColor(menuBtnBorder);
    myGLCD.drawRect(x1, y1, x2, y2);
}

// Highlights square buttons when selected and sends CAN message
// This function is used for manual control
void waitForItRect(int x1, int y1, int x2, int y2, int txId, byte data[])
{
    myGLCD.setColor(themeBackground);
    myGLCD.drawRect(x1, y1, x2, y2);

    // Code needs to be tested with CAN Bus working

    
    uint8_t  ss[1];
    unsigned long timer1 = millis();
    unsigned long timer2 = millis();
    while ((millis() - timer1 < 5))
    {
        readGT9271TouchAddr(0x814e, ss, 1);
        if ((ss[0] & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            if ((millis() - timer2 > 70))
            {
                can1.sendFrame(txId, data);
                timer2 = millis();
            }
            timer1 = millis();
        }
    }
    

    //while (myTouch.dataAvailable())
    //{
        //can1.sendFrame(txId, data);
        //myTouch.read();
        //delay(100);
    //}
    myGLCD.setColor(menuBtnBorder);
    myGLCD.drawRect(x1, y1, x2, y2);
}


/*=========================================================
    Manual control Functions
===========================================================*/
// Draw the manual control page
//void drawManualControl(int x = 146, int y = 80)
void drawManualControl(int x = 146, int y = 80, bool drawGrip = true)
{
    // Clear LCD to be written 
    drawSquareBtn(141, 1, 799, 479, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print page title
    drawSquareBtn((x + 34), (y - 70), (x + 254), (y - 35), F("Manual Control"), themeBackground, themeBackground, menuBtnColor, CENTER);

    // Manual control axis labels
    //myGLCD.setColor(VGA_BLACK);
    //myGLCD.setBackColor(VGA_BLACK);
    //myGLCD.print("Axis", CENTER, 60);
    int j = 1;
    for (int i = x; i < (x + 334 - 45); i = i + 54) {
        myGLCD.setColor(menuBtnColor);
        myGLCD.setBackColor(themeBackground);
        myGLCD.printNumI(j, i + 20, y - 20);
        j++;
    }

    // Draw the upper row of movement buttons
        // x_Start, y_start, x_Stop, y_stop
    for (int i = x; i < (x + 334 - 45); i = i + 54) {
        drawSquareBtn(i, y, (i + 54), (y + 60), F("/\\"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }

    // Draw the bottom row of movement buttons
    // x_Start, y_start, x_Stop, y_stop
    for (int i = x; i < (x + 334 - 54); i = i + 54) {
        drawSquareBtn(i, (y + 60), i + 54, (y + 120), F("\\/"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }

    // Draw Select arm buttons
    drawSquareBtn((x + 19), (y + 125), x + 60, (y + 165), F("Arm"), themeBackground, themeBackground, menuBtnColor, CENTER);
    if (txIdManual == ARM1_M)
    {
        drawSquareBtn(x, (y + 160), x + 54, (y + 215), "1", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
        drawSquareBtn((x + 54), (y + 160), x + 108, (y + 215), "2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }
    else if (txIdManual == ARM2_M)
    {
        drawSquareBtn(x, (y + 160), x + 54, (y + 215), "1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        drawSquareBtn((x + 54), (y + 160), x + 108, (y + 215), "2", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
    }

    if (drawGrip)
    {
        // Draw grip buttons
        drawSquareBtn(270, 205, 450, 245, F("Gripper"), themeBackground, themeBackground, menuBtnColor, CENTER);
        drawSquareBtn(270, 240, 360, 295, F("Open"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        drawSquareBtn(360, 240, 450, 295, F("Close"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }

    return;
}

// Draw page button function
void manualControlButtons()
{
    // Mutiply is a future funtion to allow movement of multiple angles at a time instead of just 1
    int multiply = 1;

    // Enables revese
    uint8_t reverse = 0x10;

    byte data[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // LCD touch funtions
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((y >= 80) && (y <= 140))
        {
            // A1 Up
            if ((x >= 146) && (x <= 200))
            {
                data[1] = 1 * multiply;
                waitForItRect(146, 80, 200, 140, txIdManual, data);
                data[1] = 0;
            }
            // A2 Up
            if ((x >= 200) && (x <= 254))
            {
                data[2] = 1 * multiply;
                waitForItRect(200, 80, 254, 140, txIdManual, data);
                data[2] = 0;
            }
            // A3 Up
            if ((x >= 254) && (x <= 308))
            {
                data[3] = 1 * multiply;
                waitForItRect(254, 80, 308, 140, txIdManual, data);
                data[3] = 0;
            }
            // A4 Up
            if ((x >= 308) && (x <= 362))
            {
                data[4] = 1 * multiply;
                waitForItRect(308, 80, 362, 140, txIdManual, data);
                data[4] = 0;
            }
            // A5 Up
            if ((x >= 362) && (x <= 416))
            {
                data[5] = 1 * multiply;
                waitForItRect(362, 80, 416, 140, txIdManual, data);
                data[5] = 0;
            }
            // A6 Up
            if ((x >= 416) && (x <= 470))
            {
                data[6] = 1 * multiply;
                waitForItRect(416, 80, 470, 140, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= 140) && (y <= 200))
        {
            // A1 Down
            if ((x >= 156) && (x <= 200))
            {
                data[1] = (1 * multiply) + reverse;
                waitForItRect(146, 140, 200, 200, txIdManual, data);
                data[1] = 0;
            }
            // A2 Down
            if ((x >= 200) && (x <= 254))
            {
                data[2] = (1 * multiply) + reverse;
                waitForItRect(200, 140, 254, 200, txIdManual, data);
                data[2] = 0;
            }
            // A3 Down
            if ((x >= 254) && (x <= 308))
            {
                data[3] = (1 * multiply) + reverse;
                waitForItRect(254, 140, 308, 200, txIdManual, data);
                data[3] = 0;
            }
            // A4 Down
            if ((x >= 308) && (x <= 362))
            {
                data[4] = (1 * multiply) + reverse;
                waitForItRect(308, 140, 362, 200, txIdManual, data);
                data[4] = 0;
            }
            // A5 Down
            if ((x >= 362) && (x <= 416))
            {
                data[5] = (1 * multiply) + reverse;
                waitForItRect(362, 140, 416, 200, txIdManual, data);
                data[5] = 0;
            }
            // A6 Down
            if ((x >= 416) && (x <= 470))
            {
                data[6] = (1 * multiply) + reverse;
                waitForItRect(416, 140, 470, 200, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= 240) && (y <= 295))
        {
            if ((x >= 146) && (x <= 200))
            {
                // Select arm 1
                drawSquareBtn(146, 240, 200, 295, "1", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
                drawSquareBtn(200, 240, 254, 295, "2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
                txIdManual = ARM1_M;
            }
            if ((x >= 200) && (x <= 254))
            {
                // Select arm 2
                drawSquareBtn(146, 240, 200, 295, "1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
                drawSquareBtn(200, 240, 254, 295, "2", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
                txIdManual = ARM2_M;
            }
            if ((x >= 270) && (x <= 360))
            {
                // Grip open
                data[7] = 1 * multiply;
                waitForItRect(270, 240, 360, 295, txIdManual, data);
                data[7] = 0;
            }
            if ((x >= 360) && (x <= 450))
            {
                // Grip close
                data[7] = (1 * multiply) + reverse;
                waitForItRect(360, 240, 450, 295, txIdManual, data);
                data[7] = 0;
            }
        }
    }
}

/*=========================================================
                    View page
===========================================================*/
// Draw the view page
void drawView()
{
    // Clear LCD to be written 
    drawSquareBtn(141, 1, 799, 479, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Draw row lables
    for (int start = 35, stop = 75, row = 1; start <= 260; start = start + 45, stop = stop + 45, row++)
    {
        String rowLable = "A" + String(row);
        drawRoundBtn(170, start, 190, stop, rowLable, themeBackground, themeBackground, menuBtnColor, CENTER);
    }

    // Boxes for current arm angles
    uint8_t yStart = 35;
    uint8_t yStop = 75;
    // Arm 1
    drawRoundBtn(310, 5, 415, 40, F("Arm2"), themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(310, yStart + 0, 415, yStop + 0, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 45, 415, yStop + 45, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 90, 415, yStop + 90, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 135, 415, yStop + 135, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 180, 415, yStop + 180, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 225, 415, yStop + 225, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    // Arm 2
    drawRoundBtn(205, 5, 305, 40, F("Arm1"), themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(200, yStart + 0, 305, yStop + 0, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 45, 305, yStop + 45, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 90, 305, yStop + 90, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 135, 305, yStop + 135, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 180, 305, yStop + 180, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 225, 305, yStop + 225, DEG, menuBackground, menuBackground, menuBtnColor, RIGHT);
}

void updateViewPage()
{
    if (millis() - timer > REFRESH_RATE)
    {
        axisPos.sendRequest(can1);
        if (page == 1)
        {
            axisPos.drawAxisPos(myGLCD);
        }
        timer = millis();
    }
}

/*==========================================================
                    Program Arm
============================================================*/
// Draws scrollable box that contains 10 slots for programs
void drawProgramScroll(int scroll)
{
    // selected position = scroll * position
    // if selected draw different color border
    int y = 50;
    int x = 150;
    uint8_t height = 40;
    int width = 300;

    for (int i = 0; i < 8; i++)
    {
        if (sdCard.fileExists(aList[i + scroll]))
        {
            drawSquareBtn(x, y, (x + width), (y + height), (aList[i + scroll]), menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }
        else
        {
            drawSquareBtn(x, y, (x + width), (y + height), (aList[i + scroll] + F("-Empty")), menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }

        y = y + height;
        //scroll++;
    }
}

// Draws buttons for program function
void drawProgram(int scroll = 0)
{
    // Clear LCD to be written
    drawSquareBtn(141, 1, 799, 479, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print page title
    drawSquareBtn(180, 10, 400, 45, F("Program"), themeBackground, themeBackground, menuBtnColor, CENTER);

    // Scroll buttons
    myGLCD.setColor(menuBtnColor);
    myGLCD.setBackColor(themeBackground);
    drawSquareBtn(460, 100, 510, 150, F("/\\"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(460, 150, 510, 200, F("\\/"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    // Draws program scroll box with current scroll value
    drawProgramScroll(scroll);

    // Draw program buttons
    drawSquareBtn(150, 430, 250, 470, F("Open"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(255, 430, 355, 470, F("Load"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(360, 430, 460, 470, F("Delete"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(465, 430, 575, 470, F("Rename"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
}

// Deletes current selected program from 
void programDelete()
{
    sdCard.deleteFile(aList[selectedProgram]);
}

// Load selected program from SD card into linked list
void loadProgram()
{
    sdCard.readFile(aList[selectedProgram], runList);
}

/*
// Executes program currently loaded into linked list
void programRun()
{
    // Was this to clear leftover messages in buffer?
    // Does it work? Is it needed?
    can1.getFrameID();

    // Bool control for while loop
    bool isWait = true;

    // Make sure a program was loaded to run
    if (programLoaded == false)
    {
        return;
    }

    // CAN messages for axis movements
    uint8_t bAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t tAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t excMove[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint16_t IDArray[3];
    uint16_t incID;

    for (uint8_t i = 0; i < runList.size(); i++)
    {
        if (runList.get(i)->getID() == ARM1_M)
        {
            IDArray[0] = ARM1_CONTROL;
            IDArray[1] = ARM1_B;
            IDArray[2] = ARM1_T;
            incID = ARM1_RX;
        }
        if (runList.get(i)->getID() == ARM2_M)
        {
            IDArray[0] = ARM2_CONTROL;
            IDArray[1] = ARM2_B;
            IDArray[2] = ARM2_T;
            incID = ARM2_RX;
        }

        // Populate CAN messages with angles from current linkedlist

        // Axis 1
        if (runList.get(i)->getA1() <= 0xFF)
        {
            bAxis[3] = runList.get(i)->getA1();
        }
        else
        {
            bAxis[2] = runList.get(i)->getA1() - 0xFF;
            bAxis[3] = 0xFF;
        }

        // Axis 2
        if (runList.get(i)->getA2() <= 0xFF)
        {
            bAxis[5] = runList.get(i)->getA2();
        }
        else
        {
            bAxis[4] = runList.get(i)->getA2() - 0xFF;
            bAxis[5] = 0xFF;
        }

        // Axis 3
        if (runList.get(i)->getA3() <= 0xFF)
        {
            bAxis[7] = runList.get(i)->getA3();
        }
        else
        {
            bAxis[6] = runList.get(i)->getA3() - 0xFF;
            bAxis[7] = 0xFF;
        }

        // Send first frame with axis 1-3
        can1.sendFrame(IDArray[1], bAxis);

        // Axis 4
        if (runList.get(i)->getA4() <= 0xFF)
        {
            tAxis[3] = runList.get(i)->getA4();
        }
        else
        {
            tAxis[2] = runList.get(i)->getA4() - 0xFF;
            tAxis[3] = 0xFF;
        }

        // Axis 5
        if (runList.get(i)->getA5() <= 0xFF)
        {
            tAxis[5] = runList.get(i)->getA5();
        }
        else
        {
            tAxis[4] = runList.get(i)->getA5() - 0xFF;
            tAxis[5] = 0xFF;
        }

        // Axis 6
        if (runList.get(i)->getA5() <= 0xFF)
        {
            tAxis[7] = runList.get(i)->getA6();
        }
        else
        {
            tAxis[6] = runList.get(i)->getA6() - 0xFF;
            tAxis[7] = 0xFF;
        }

        delay(10);

        // Send second frame with axis 4-6
        can1.sendFrame(IDArray[2], tAxis);

        // Change to array of IDs
        uint8_t ID = runList.get(i)->getID();

        // Grip on/off or hold based on current and next state
        // If there was a change in the grip bool
        excMove[6] = 0x00;
        excMove[7] = 0x00;

        if (runList.get(i)->getGrip() == 0)
        {
            excMove[6] = 0x01;

        }
        else if (runList.get(i)->getGrip() == 1)
        {
            excMove[7] = 0x01;
        }

        delay(10);

        // Send third frame with grip and execute command
        can1.sendFrame(IDArray[0], excMove);

        // Wait for confirmation
        while (isWait)
        {
            if (can1.msgCheck(incID, 0x03, 0x01))
            {
                isWait = false;
            }
        }
        isWait = true;
    }
}
*/

// Button functions for program page 
void programButtons()
{
    // Static so that the position is saved while this method is repeatedly called in a loop
    static int scroll = 0;

    // Touch screen controls
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((x >= 150) && (x <= 410))
        {
            if ((y >= 50) && (y <= 90))
            {
                // 1st position of scroll box
                waitForItRect(150, 50, 450, 90);
                //Serial.println(1 + scroll);
                selectedProgram = 0 + scroll;
            }
            if ((y >= 90) && (y <= 130))
            {
                // 2nd position of scroll box
                waitForItRect(150, 90, 450, 130);
                //Serial.println(2 + scroll);
                selectedProgram = 1 + scroll;
            }
            if ((y >= 130) && (y <= 170))
            {
                // 3d position of scroll box
                waitForItRect(150, 130, 450, 170);
                //Serial.println(3 + scroll);
                selectedProgram = 2 + scroll;
            }
            if ((y >= 170) && (y <= 210))
            {
                // 4th position of scroll box
                waitForItRect(150, 170, 450, 210);
                //Serial.println(4 + scroll);
                selectedProgram = 3 + scroll;
            }
            if ((y >= 210) && (y <= 250))
            {
                // 5th position of scroll box
                waitForItRect(150, 210, 450, 250);
                //Serial.println(5 + scroll);
                selectedProgram = 4 + scroll;
            }
            if ((y >= 250) && (y <= 290))
            {
                // 5th position of scroll box
                waitForItRect(150, 250, 450, 290);
                //Serial.println(6 + scroll);
                selectedProgram = 4 + scroll;
            }
            if ((y >= 290) && (y <= 330))
            {
                // 5th position of scroll box
                waitForItRect(150, 290, 450, 330);
                //Serial.println(7 + scroll);
                selectedProgram = 4 + scroll;
            }
        }
        if ((x >= 460) && (x <= 510))
        {
            if ((y >= 100) && (y <= 150))
            {
                // Scroll up
                waitForIt(460, 100, 510, 150);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramScroll(scroll);
                }
            }
            if ((y >= 150) && (y <= 200))
            {
                // Scroll down
                waitForIt(460, 150, 510, 200);
                if (scroll < 2)
                {
                    scroll++;
                    drawProgramScroll(scroll);
                }
            }
        }
        if ((y >= 430) && (y <= 470))
        {
            if ((x >= 150) && (x <= 250))
            {
                // Open program
                waitForItRect(150, 430, 250, 470);
                runList.clear();
                loadProgram();
                programOpen = true;
                page = 6;
                hasDrawn = false;
            }
            if ((x >= 255) && (x <= 355))
            {
                // Load program
                waitForItRect(255, 430, 355, 470);
                runList.clear();
                loadProgram();
                programLoaded = true;

            }
            if ((x >= 360) && (x <= 460))
            {
                // Delete program
                waitForItRect(360, 430, 460, 470);
                bool result = errorMSG(F("Confirmation"), F("Permanently"), F("Delete File?"));
                if (result)
                {
                    programDelete();
                }
                drawProgram();
            }
            if ((x >= 465) && (x <= 565))
            {
                // Rename program
                waitForItRect(465, 430, 565, 470);

            }
        }
    }
    return;
}


/*==========================================================
                    Edit Program
============================================================*/
// Draws scrollable box that contains all the nodes in a program
void drawProgramEditScroll(uint8_t scroll = 0)
{
    myGLCD.setFont(SmallFont);
    uint8_t nodeSize = runList.size();
    Serial.println(nodeSize);
    int y = 50;
    int x = 150;
    uint8_t height = 40;
    int width = 295;

    // Each node should be listed with all information, might need small text
    for (int i = 0; i < 9; i++)
    {
        String position = String(i + scroll);
        String a = ":";
        String b = " ";
        String label = position + a + String(runList.get(i + scroll)->getA1()) + b + String(runList.get(i + scroll)->getA2())
            + b + String(runList.get(i + scroll)->getA3()) + b + String(runList.get(i + scroll)->getA4()) + b + String(runList.get(i + scroll)->getA5())
            + b + String(runList.get(i + scroll)->getA6()) + b + String(runList.get(i + scroll)->getGrip()) + b + String(runList.get(i + scroll)->getID());
        if (i + scroll < nodeSize)
        {
            drawSquareBtn(x, y, (x + width), (y + height), label, menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }
        else
        {
            drawSquareBtn(x, y, (x + width), (y + height), "", menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }

        y = (y + 40);
        //scroll++;
    }
    // Load linked list from SD card unless already in linked list
    // Need some gui to edit, add and instert nodes
    // Save button will write to SD card and return
    // cancel button will return without saving
    // 5 buttons in total

    // To edit a node just replace with a new position
    myGLCD.setFont(BigFont);
}

// Draws buttons for edit program function
void drawProgramEdit(uint8_t scroll = 0)
{
    // Clear LCD to be written
    drawSquareBtn(141, 1, 799, 479, "", themeBackground, themeBackground, themeBackground, CENTER);
    drawManualControl(460, 80, false);
    // Print page title
    drawSquareBtn(180, 10, 400, 45, F("Edit Program"), themeBackground, themeBackground, menuBtnColor, CENTER);

    // Scroll buttons
    myGLCD.setColor(menuBtnColor);
    myGLCD.setBackColor(themeBackground);
    drawSquareBtn(460, 310, 510, 360, F("/\\"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(460, 360, 510, 410, F("\\/"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    // Draw program edit buttons
    drawSquareBtn(141, 430, 215, 470, F("Add"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(215, 430, 290, 470, F("Ins"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(290, 430, 365, 470, F("Del"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    switch (gripStatus)
    {
    case 0: drawSquareBtn(365, 430, 440, 470, F("Open"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        break;
    case 1: drawSquareBtn(365, 430, 440, 470, F("Close"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        break;
    case 2: drawSquareBtn(365, 430, 440, 470, F("Grip"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        break;
    }
    drawSquareBtn(440, 430, 515, 470, F("Wait"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(515, 430, 590, 470, F("Sens"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(590, 430, 665, 470, F("Loop"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(665, 430, 740, 470, F("Save"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(740, 430, 799, 470, F("X"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
}

// Adds current position to program linked list 
void addNode(int insert = -1)
{
    // Array of arm axis positions
    uint16_t posArray[8] = { 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000 };

    // Update array value with data collected from the axis position update
    if (txIdManual == ARM1_M)
    {
        posArray[0] = axisPos.getA1C1();
        posArray[1] = axisPos.getA2C1();
        posArray[2] = axisPos.getA3C1();
        posArray[3] = axisPos.getA4C1();
        posArray[4] = axisPos.getA5C1();
        posArray[5] = axisPos.getA6C1();
    }
    else if (txIdManual == ARM2_M)
    {
        posArray[0] = axisPos.getA1C2();
        posArray[1] = axisPos.getA2C2();
        posArray[2] = axisPos.getA3C2();
        posArray[3] = axisPos.getA4C2();
        posArray[4] = axisPos.getA5C2();
        posArray[5] = axisPos.getA6C2();
    }

    // Create program object with array positions, grip on/off, and channel
    Program* node = new Program(posArray, gripStatus, txIdManual);

    if (insert < 0)
    {
        // Add created object to linked list
        runList.add(node);
    }
    else
    {
        runList.add(insert, node);
    }
}

// Delete node from linked list
void deleteNode(uint16_t nodeLocation)
{
    runList.remove(nodeLocation);
}

// Writes current selected linked list to SD card
void saveProgram()
{
    // Delimiter 
    String space = ", ";

    // Write out linkedlist data to text file
    for (uint8_t i = 0; i < runList.size(); i++)
    {
        sdCard.writeFile(aList[selectedProgram], ",");
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA1());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA2());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA3());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA4());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA5());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getA6());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getID());
        sdCard.writeFile(aList[selectedProgram], space);
        sdCard.writeFile(aList[selectedProgram], runList.get(i)->getGrip());
        sdCard.writeFileln(aList[selectedProgram]);
    }
}

// Button functions for edit program page
void programEditButtons()
{
    // Current selected node
    static uint8_t selectedNode = 0;

    static uint16_t scroll = 0;

    // Touch screen controls
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((x >= 150) && (x <= 445))
        {
            if ((y >= 50) && (y <= 90))
            {
                waitForItRect(150, 50, 445, 90);
                //Serial.println(1 + scroll);
                selectedNode = 0 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 90) && (y <= 130))
            {
                waitForItRect(150, 90, 445, 130);
                //Serial.println(2 + scroll);
                selectedNode = 1 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 130) && (y <= 170))
            {
                waitForItRect(150, 130, 445, 170);
                //Serial.println(3 + scroll);
                selectedNode = 2 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 170) && (y <= 210))
            {
                waitForItRect(150, 170, 445, 210);
                //Serial.println(4 + scroll);
                selectedNode = 3 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 210) && (y <= 250))
            {
                waitForItRect(150, 210, 445, 250);
                //Serial.println(5 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 250) && (y <= 290))
            {
                waitForItRect(150, 250, 445, 290);
                //Serial.println(6 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 290) && (y <= 330))
            {
                waitForItRect(150, 290, 445, 330);
                //Serial.println(7 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 330) && (y <= 370))
            {
                waitForItRect(150, 330, 445, 370);
                //Serial.println(8 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 370) && (y <= 410))
            {
                waitForItRect(150, 370, 445, 410);
                //Serial.println(9 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
        }
        if ((x >= 460) && (x <= 510))
        {
            if ((y >= 310) && (y <= 360))
            {
                waitForIt(460, 310, 510, 360);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramEditScroll(scroll);
                }
            }
            if ((y >= 360) && (y <= 410))
            {
                waitForIt(460, 360, 510, 410);
                if (scroll < 10)
                {
                    scroll++;
                    drawProgramEditScroll(scroll);
                }
            }
        }
        if ((y >= 430) && (y <= 470))
        {
            if ((x >= 140) && (x <= 215))
            {
                // Add node
                waitForItRect(141, 430, 215, 470);
                addNode();
                drawProgramEditScroll(scroll);
            }
            if ((x >= 215) && (x <= 290))
            {
                // Insert node
                waitForItRect(215, 430, 290, 470);
                addNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 290) && (x <= 365))
            {
                // Delete node
                waitForItRect(290, 430, 365, 470);
                deleteNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 365) && (x <= 440))
            {
                // Grip
                waitForItRect(365, 430, 440, 470);
                if (gripStatus < 2)
                {
                    gripStatus++;
                }
                else
                {
                    gripStatus = 0;
                }
                switch (gripStatus)
                {
                case 0:
                    drawSquareBtn(365, 430, 440, 470, F("Open"), menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    break;
                case 1:
                    drawSquareBtn(365, 430, 440, 470, F("Shut"), menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    break;
                case 2:
                    drawSquareBtn(365, 430, 440, 470, F("Grip"), menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    break;
                }
            }
            if ((x >= 440) && (x <= 515))
            {
                // Wait
                waitForItRect(440, 430, 515, 470);
                //
            }
            if ((x >= 515) && (x <= 590))
            {
                // Sensor
                waitForItRect(515, 430, 590, 470);
                //
            }
            if ((x >= 590) && (x <= 665))
            {
                // Loop
                waitForItRect(590, 430, 665, 470);
                //
            }
            if ((x >= 665) && (x <= 740))
            {
                // Save program
                waitForItRect(665, 430, 740, 470);
                programDelete();
                saveProgram();
                programOpen = false;
                page = 2;
                hasDrawn = false;
            }
            if ((x >= 740) && (x <= 799))
            {
                // Cancel
                waitForItRect(740, 430, 799, 470);
                programOpen = false;
                page = 2;
                hasDrawn = false;
            }
        }
    }
    return;
}


/*==========================================================
                    Configure Arm
============================================================*/
// Draws the config page
void drawConfig()
{
    drawSquareBtn(141, 1, 799, 479, "", themeBackground, themeBackground, themeBackground, CENTER);
    drawSquareBtn(180, 10, 400, 45, F("Configuration"), themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(150, 60, 300, 100, F("Home Ch1"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(310, 60, 460, 100, F("Set Ch1"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(150, 110, 300, 150, F("Home Ch2"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(310, 110, 460, 150, F("Set ch2"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    return;
}

// Sends command to return arm to starting position
void homeArm(uint8_t* armIDArray)
{
    byte data1[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0x5A };
    byte data2[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0xB4 };
    byte data3[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    can1.sendFrame(armIDArray[0], data1);
    can1.sendFrame(armIDArray[1], data2);
    can1.sendFrame(armIDArray[2], data3);
}

// Button functions for config page
void configButtons()
{
    uint8_t setHomeId[8] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t* setHomeIdPtr = setHomeId;

    uint8_t arm1IDArray[3] = { ARM1_B, ARM1_T, ARM1_CONTROL };
    uint8_t arm2IDArray[3] = { ARM2_B, ARM2_T, ARM2_CONTROL };
    uint8_t* arm1IDPtr = arm1IDArray;
    uint8_t* arm2IDPtr = arm2IDArray;
    // Touch screen controls
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((y >= 60) && (y <= 100))
        {
            if ((x >= 150) && (x <= 300))
            {
                waitForIt(150, 60, 300, 100);
                homeArm(arm1IDPtr);
            }
            if ((x >= 310) && (x <= 460))
            {
                waitForIt(310, 60, 460, 100);
                can1.sendFrame(arm1IDArray[2], setHomeIdPtr);
            }
        }
        if ((y >= 110) && (y <= 150))
        {
            if ((x >= 150) && (x <= 300))
            {
                waitForIt(150, 110, 300, 150);
                homeArm(arm2IDPtr);
            }
            if ((x >= 310) && (x <= 460))
            {
                waitForIt(310, 110, 460, 150);
                can1.sendFrame(arm2IDArray[2], setHomeIdPtr);
            }
        }
    }
    return;
}


/*=========================================================
    Framework Functions
===========================================================*/
//Only called once at startup to draw the menu
void drawMenu()
{
    // Draw Layout
    drawSquareBtn(0, 0, 800, 480, "", themeBackground, themeBackground, themeBackground, CENTER);
    drawSquareBtn(0, 0, 140, 480, "", menuBackground, menuBackground, menuBackground, CENTER);

    // Draw Menu Buttons
    drawRoundBtn(10, 10, 130, 65, F("1-VIEW"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 70, 130, 125, F("2-PROG"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 130, 130, 185, F("3-MOVE"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 190, 130, 245, F("4-CONF"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 250, 130, 305, F("5-EXEC"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 310, 130, 365, F("6-STOP"), menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
}

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    Serial3.begin(57600);
    Wire.begin();        // join i2c bus (address optional for master)

    bool hasFailed = sdCard.startSD();
    if (!hasFailed)
    {
        //Serial.println("SD failed");
    }
    else if (hasFailed)
    {
        //Serial.println("SD Running");
    }

    randomSeed(analogRead(0));

    delay(300);
    pinMode(GT9271_RESET, OUTPUT);
    pinMode(GT9271_INT, OUTPUT);
    digitalWrite(GT9271_RESET, LOW);
    delay(20);
    digitalWrite(GT9271_INT, LOW);
    delay(50);
    digitalWrite(GT9271_RESET, HIGH);
    delay(100);
    pinMode(GT9271_INT, INPUT);
    delay(100);

    uint8_t re = GT9271_Send_Cfg((uint8_t*)GTP_CFG_DATA, sizeof(GTP_CFG_DATA));

    pinMode(GT9271_RESET, OUTPUT);
    pinMode(GT9271_INT, OUTPUT);
    digitalWrite(GT9271_RESET, LOW);
    delay(20);
    digitalWrite(GT9271_INT, LOW);
    delay(50);
    digitalWrite(GT9271_RESET, HIGH);
    delay(100);
    pinMode(GT9271_INT, INPUT);
    delay(100);

    re = GT9271_Send_Cfg((uint8_t*)GTP_CFG_DATA, sizeof(GTP_CFG_DATA));

    uint8_t bb[2];
    readGT9271TouchAddr(0x8047, bb, 2);
    while (bb[1] != 32)
    {
        //Serial.println("Capacitive touch screen initialized failure");
        pinMode(GT9271_RESET, OUTPUT);
        pinMode(GT9271_INT, OUTPUT);
        digitalWrite(GT9271_RESET, LOW);
        delay(20);
        digitalWrite(GT9271_INT, LOW);
        delay(50);
        digitalWrite(GT9271_RESET, HIGH);
        delay(100);
        pinMode(GT9271_INT, INPUT);
        delay(100);

        uint8_t re = GT9271_Send_Cfg((uint8_t*)GTP_CFG_DATA, sizeof(GTP_CFG_DATA));
    }

    //Serial.println("Capacitive touch screen initialized success");

    // Setup the LCD
    myGLCD.InitLCD();
    // -------------------------------------------------------------
    pinMode(8, OUTPUT);  //backlight 
    digitalWrite(8, HIGH);//on
  // -------------------------------------------------------------
    myGLCD.setFont(BigFont);
    myGLCD.clrScr();


    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_WHITE);
    drawSquareBtn(1, 1, 800, 600, "", themeBackground, themeBackground, menuBtnColor, CENTER);
    myGLCD.print("Loading...", 320, 290);
    bmpDraw("robotarm.bmp", 0, 0);
    delay(2000);
    drawMenu();
    
    print_icon(48, 420, robotarm);
    timer = millis();
}

// Page control framework
void pageControl()
{
    // Check if button on menu is pushed
    menuButtons();

    // Switch which page to load
    switch (page)
    {
    case 1:
        // Draw page
        if (!hasDrawn)
        {
            drawView();
            axisPos.drawAxisPos(myGLCD);
            axisPos.drawAxisPos(myGLCD);
            hasDrawn = true;
        }
        // Call buttons if any
        break;
    case 2:
        // If program open jump to page 6
        if (programOpen)
        {
            page = 6;
            break;
        }
        // Draw page
        if (!hasDrawn)
        {
            drawProgram();
            hasDrawn = true;
        }
        // Call buttons if any
        programButtons();
        break;
    case 3:
        // Draw page
        if (!hasDrawn)
        {
            drawManualControl();
            hasDrawn = true;
        }
        // Call buttons if any
        manualControlButtons();
        break;
    case 4:
        // Draw page
        if (!hasDrawn)
        {
            drawConfig();
            hasDrawn = true;
        }
        // Call buttons if any
        configButtons();
        break;
    case 5:
        // Draw page
        if (!hasDrawn)
        {
            hasDrawn = true;
            if (programLoaded == true)
            {
                programRunning = true;
                programProgress = 0;
                Arm1Ready = true;
                Arm2Ready = true;
            }
            // ---ERROR MESSAGE---
            page = oldPage;
        }
        // Call buttons if any
        break;
    case 6:
        // Program edit page
        if (!hasDrawn)
        {
            hasDrawn = true;
            programLoaded = true;
            drawProgramEdit();
            drawProgramEditScroll();
        }
        // Call buttons if any
        programEditButtons();
        break;
    }
}

// Error Message function
bool errorMSG(String title, String eMessage1, String eMessage2)
{
    drawSquareBtn(145, 100, 415, 220, "", menuBackground, menuBtnColor, menuBtnColor, CENTER);
    drawSquareBtn(145, 100, 415, 130, title, themeBackground, menuBtnColor, menuBtnBorder, LEFT);
    drawSquareBtn(146, 131, 414, 155, eMessage1, menuBackground, menuBackground, menuBtnText, CENTER);
    drawSquareBtn(146, 155, 414, 180, eMessage2, menuBackground, menuBackground, menuBtnText, CENTER);
    drawRoundBtn(365, 100, 415, 130, "X", menuBtnColor, menuBtnColor, menuBtnText, CENTER);
    drawRoundBtn(155, 180, 275, 215, "Confirm", menuBtnColor, menuBtnColor, menuBtnText, CENTER);
    drawRoundBtn(285, 180, 405, 215, "Cancel", menuBtnColor, menuBtnColor, menuBtnText, CENTER);

    while (true)
    {
        // Touch screen controls
        uint8_t  ss[1];
        readGT9271TouchAddr(0x814e, ss, 1);
        uint8_t status = ss[0];
        if ((status & 0x80) != 0)  // touch status   Software touch interrupt
        {
            readGT9271TouchLocation(touchLocations, 10);
            x = 800 - touchLocations[0].x;
            y = 480 - touchLocations[0].y;

            if ((x >= 365) && (x <= 415))
            {
                if ((y >= 100) && (y <= 130))
                {
                    // 1st position of scroll box
                    waitForItRect(365, 100, 415, 130);
                    return false;
                }
            }
            if ((y >= 180) && (y <= 215))
            {
                if ((x >= 155) && (x <= 275))
                {
                    // 1st position of scroll box
                    waitForItRect(155, 180, 275, 215);
                    return true;
                }
                if ((x >= 285) && (x <= 405))
                {
                    // 1st position of scroll box
                    waitForItRect(285, 180, 405, 215);
                    return false;
                }
            }
        }
    }
}

// Error message without confirmation
uint8_t errorMSGBtn(uint8_t page)
{
    // Touch screen controls
        uint8_t  ss[1];
        readGT9271TouchAddr(0x814e, ss, 1);
        uint8_t status = ss[0];
        if ((status & 0x80) != 0)  // touch status   Software touch interrupt
        {
            readGT9271TouchLocation(touchLocations, 10);
            x = 800 - touchLocations[0].x;
            y = 480 - touchLocations[0].y;

        if ((x >= 400) && (x <= 450))
        {
            if ((y >= 140) && (y <= 170))
            {
                waitForItRect(400, 140, 450, 170);
                page = oldPage;
            }
        }
    }
}

// Buttons for the main menu
void menuButtons()
{
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((x >= 10) && (x <= 130)) 
        {
            if ((y >= 10) && (y <= 65))  
            {
                waitForIt(10, 10, 130, 65);
                page = 1;
                hasDrawn = false;
            }
            if ((y >= 70) && (y <= 125))  
            {
                waitForIt(10, 70, 130, 125);
                page = 2;
                hasDrawn = false;
            }
            if ((y >= 130) && (y <= 185)) 
            {
                waitForIt(10, 130, 130, 185);
                page = 3;
                hasDrawn = false;
            }
            if ((y >= 190) && (y <= 245))
            {
                waitForIt(10, 190, 130, 245);
                page = 4;
                hasDrawn = false;
            }
            if ((y >= 250) && (y <= 305))
            {
                waitForIt(10, 250, 130, 305);
                oldPage = page;
                page = 5;
                hasDrawn = false;
            }
            if ((y >= 310) && (y <= 365))
            {
                waitForIt(10, 310, 130, 365);
                programRunning = false;
            }
        }
    }
}

/*
void testfn()
{
    while (1)
    {
        readGT9271TouchLocation(touchLocations, 10);
        uint8_t  ss[1];
        readGT9271TouchAddr(0x814e, ss, 1);
        uint8_t status = ss[0];
        if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            Serial.println("Touch: ");
            Serial.println(touchLocations[0].x);
            Serial.println(touchLocations[0].y);
            if (touchLocations[0].x > 1 && touchLocations[0].x < 800 && touchLocations[0].y > 1 && touchLocations[0].y < 480)
            {
                Serial.println("Touch: ");
            }
        }
    }
}
*/

void TrafficManager()
{
    uint8_t sw_fn = can1.processFrame();
    switch(sw_fn)
    {
        case 0: // No traffic

        break;
        
        case 1: // C1 lower
            axisPos.updateAxisPos(can1, ARM1_RX);
        break;

        
        case 2: //  C1 Upper
            axisPos.updateAxisPos(can1, ARM1_RX);
        break;

        
        case 3: // C1 Confirmation
            Arm1Ready = true;
            Arm2Ready = true;
            Serial.println("Arm1Ready");
        break;

        
        case 4: // C2 Lower
            axisPos.updateAxisPos(can1, ARM2_RX);
        break;

        
        case 5: // C2 Upper
            axisPos.updateAxisPos(can1, ARM2_RX);
        break;

        
        case 6: // C2 Confirmation
            Arm1Ready = true;
            Arm2Ready = true;
            Serial.println("Arm2Ready");
        break;
    }
}


void executeProgram()
{
    // Return unless enabled
    if (programRunning == false)
    {
        return;
    }

    if (programProgress == runList.size())
    {
        programRunning = false;
    }

    if (Arm1Ready == true && Arm2Ready == true)
    {
        // CAN messages for axis movements
        uint8_t bAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t tAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t excMove[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint16_t IDArray[3];
        uint16_t incID;

        if (runList.get(programProgress)->getID() == ARM1_M)
        {
            IDArray[0] = ARM1_CONTROL;
            IDArray[1] = ARM1_B;
            IDArray[2] = ARM1_T;
            incID = ARM1_RX;
        }
        if (runList.get(programProgress)->getID() == ARM2_M)
        {
            IDArray[0] = ARM2_CONTROL;
            IDArray[1] = ARM2_B;
            IDArray[2] = ARM2_T;
            incID = ARM2_RX;
        }

        // Populate CAN messages with angles from current linkedlist

        // Axis 1
        if (runList.get(programProgress)->getA1() <= 0xFF)
        {
            bAxis[3] = runList.get(programProgress)->getA1();
        }
        else
        {
            bAxis[2] = runList.get(programProgress)->getA1() - 0xFF;
            bAxis[3] = 0xFF;
        }

        // Axis 2
        if (runList.get(programProgress)->getA2() <= 0xFF)
        {
            bAxis[5] = runList.get(programProgress)->getA2();
        }
        else
        {
            bAxis[4] = runList.get(programProgress)->getA2() - 0xFF;
            bAxis[5] = 0xFF;
        }

        // Axis 3
        if (runList.get(programProgress)->getA3() <= 0xFF)
        {
            bAxis[7] = runList.get(programProgress)->getA3();
        }
        else
        {
            bAxis[6] = runList.get(programProgress)->getA3() - 0xFF;
            bAxis[7] = 0xFF;
        }

        // Send first frame with axis 1-3
        can1.sendFrame(IDArray[1], bAxis);

        // Axis 4
        if (runList.get(programProgress)->getA4() <= 0xFF)
        {
            tAxis[3] = runList.get(programProgress)->getA4();
        }
        else
        {
            tAxis[2] = runList.get(programProgress)->getA4() - 0xFF;
            tAxis[3] = 0xFF;
        }

        // Axis 5
        if (runList.get(programProgress)->getA5() <= 0xFF)
        {
            tAxis[5] = runList.get(programProgress)->getA5();
        }
        else
        {
            tAxis[4] = runList.get(programProgress)->getA5() - 0xFF;
            tAxis[5] = 0xFF;
        }

        // Axis 6
        if (runList.get(programProgress)->getA5() <= 0xFF)
        {
            tAxis[7] = runList.get(programProgress)->getA6();
        }
        else
        {
            tAxis[6] = runList.get(programProgress)->getA6() - 0xFF;
            tAxis[7] = 0xFF;
        }

        // Send second frame with axis 4-6
        can1.sendFrame(IDArray[2], tAxis);

        // Change to array of IDs
        uint8_t ID = runList.get(programProgress)->getID();

        // Grip on/off or hold based on current and next state
        // If there was a change in the grip bool
        excMove[6] = 0x00;
        excMove[7] = 0x00;

        if (runList.get(programProgress)->getGrip() == 0)
        {
            excMove[6] = 0x01;

        }
        else if (runList.get(programProgress)->getGrip() == 1)
        {
            excMove[7] = 0x01;
        }

        // Send third frame with grip and execute command
        can1.sendFrame(IDArray[0], excMove);
        

        Arm1Ready = false;
        Arm2Ready = false;
        Serial.print("linkedListSize: ");
        Serial.println(programProgress);
        programProgress++;
    }

    // Loop if enabled
    if (programProgress == runList.size() && loopProgram == true)
    {
        programProgress = 0;
        programRunning = true;
    }
}

void loop()
{
    // GUI
    pageControl();

    // Background Processes
    TrafficManager();
    updateViewPage();
    executeProgram();
}
