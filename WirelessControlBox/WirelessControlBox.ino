/*
 Name:    ControlBoxDue.ino
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/
#pragma once
#include <Wire.h>
#include <LinkedList.h>
//#include <UTouchCD.h>
#include <memorysaver.h>
#include <SD.h>
#include <SPI.h>
#include <UTFT.h>
//#include <UTouch.h>
#include "LCD.h"
#include "AxisPos.h"
#include "CANBus.h"
#include "SDCard.h"
#include "Program.h"
#include "definitions.h"
#include "icons.h"
#include <stdint.h>

// Initialize display
//(byte model, int RS, int WR, int CS, int RST, int SER)
//UTFT myGLCD(ILI9488_16, 7, 38, 9, 10);
//RTP: byte tclk, byte tcs, byte din, byte dout, byte irq
//UTouch  myTouch(2, 6, 3, 4, 5);

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

// Keeps track of current page
uint8_t controlPage = 1;

// Current selected program
uint8_t selectedProgram = 0;

// CAN message ID and frame, value can be changed in manualControlButtons
uint16_t txIdManual = ARM1_M;

// Used to determine if EXEC should run
bool programLoaded = false;

// Used to determine which PROG page should be loaded when button is pressed
bool programOpen = false;

// 0 = open, 1 = close, 2 = no change
int8_t gripStatus = 2;

// Program names
String aList[10] = { "Program1", "Program2", "Program3", "Program4", "Program5", "Program6", "Program7", "Program8", "Program9", "Program10" };

// BITMAP global for bmpDraw()
int dispx, dispy;

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
                                myGLCD.drawPixel(col + 437, row + 1);
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
                        myGLCD.drawPixel(col + 437, row + 1);

                    } // end pixel
                } // end scanline

                // Write any remaining data to LCD
                if (lcdidx > 0) {
                    myGLCD.setColor(lcdbuffer[lcdidx]);
                    myGLCD.drawPixel(col + 437, row + 1);
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
    unsigned long timer = millis();
    while ((millis() - timer < 10))
    {
        readGT9271TouchAddr(0x814e, ss, 1);
        if ((ss[0] & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            can1.sendFrame(txId, data);
            delay(100);
            timer = millis();
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
void drawManualControl()
{
    // Clear LCD to be written 
    drawSquareBtn(141, 1, 478, 319, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print page title
    drawSquareBtn(180, 10, 400, 45, "Manual Control", themeBackground, themeBackground, menuBtnColor, CENTER);

    // Manual control axis labels
    //myGLCD.setColor(VGA_BLACK);
    //myGLCD.setBackColor(VGA_BLACK);
    //myGLCD.print("Axis", CENTER, 60);
    int j = 1;
    for (int i = 146; i < (480 - 45); i = i + 54) {
        myGLCD.setColor(menuBtnColor);
        myGLCD.setBackColor(themeBackground);
        myGLCD.printNumI(j, i + 20, 60);
        j++;
    }

    // Draw the upper row of movement buttons
        // x_Start, y_start, x_Stop, y_stop
    for (int i = 146; i < (480 - 54); i = i + 54) {
        drawSquareBtn(i, 80, i + 54, 140, "/\\", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }

    // Draw the bottom row of movement buttons
    // x_Start, y_start, x_Stop, y_stop
    for (int i = 146; i < (480 - 54); i = i + 54) {
        drawSquareBtn(i, 140, i + 54, 200, "\\/", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }

    // Draw Select arm buttons
    drawSquareBtn(165, 225, 220, 265, "Arm", themeBackground, themeBackground, menuBtnColor, CENTER);
    if (txIdManual == ARM1_M)
    {
        drawSquareBtn(146, 260, 200, 315, "1", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
        drawSquareBtn(200, 260, 254, 315, "2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    }
    else if (txIdManual == ARM2_M)
    {
        drawSquareBtn(146, 260, 200, 315, "1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
        drawSquareBtn(200, 260, 254, 315, "2", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
    }

    // Draw grip buttons
    drawSquareBtn(270, 225, 450, 265, "Gripper", themeBackground, themeBackground, menuBtnColor, CENTER);
    drawSquareBtn(270, 260, 360, 315, "Open", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(360, 260, 450, 315, "Close", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

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
        if ((y >= 260) && (y <= 315))
        {
            if ((x >= 146) && (x <= 200))
            {
                // Select arm 1
                drawSquareBtn(146, 260, 200, 315, "1", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
                drawSquareBtn(200, 260, 254, 315, "2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
                txIdManual = ARM1_M;
            }
            if ((x >= 200) && (x <= 254))
            {
                // Select arm 2
                drawSquareBtn(146, 260, 200, 315, "1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
                drawSquareBtn(200, 260, 254, 315, "2", menuBtnText, menuBtnBorder, menuBtnColor, CENTER);
                txIdManual = ARM2_M;
            }
            if ((x >= 270) && (x <= 360))
            {
                // Grip open
                data[7] = 1 * multiply;
                waitForItRect(270, 260, 360, 315, txIdManual, data);
                data[7] = 0;
            }
            if ((x >= 360) && (x <= 450))
            {
                // Grip close
                data[7] = (1 * multiply) + reverse;
                waitForItRect(360, 260, 450, 315, txIdManual, data);
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
    drawSquareBtn(141, 1, 478, 319, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print arm logo
    

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
    drawRoundBtn(310, 5, 415, 40, "Arm2", themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(310, yStart + 0, 415, yStop + 0, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 45, 415, yStop + 45, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 90, 415, yStop + 90, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 135, 415, yStop + 135, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 180, 415, yStop + 180, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(310, yStart + 225, 415, yStop + 225, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    // Arm 2
    drawRoundBtn(205, 5, 305, 40, "Arm1", themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(200, yStart + 0, 305, yStop + 0, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 45, 305, yStop + 45, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 90, 305, yStop + 90, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 135, 305, yStop + 135, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 180, 305, yStop + 180, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
    drawRoundBtn(200, yStart + 225, 305, yStop + 225, "deg", menuBackground, menuBackground, menuBtnColor, RIGHT);
}

// No buttons, consider adding a refresh mechanism 


/*==========================================================
                    Program Arm
============================================================*/
// Draws scrollable box that contains 10 slots for programs
void drawProgramScroll(int scroll)
{
    // selected position = scroll * position
    // if selected draw different color border
    int y = 50;

    for (int i = 0; i < 5; i++)
    {
        if (sdCard.fileExists(aList[i + scroll]))
        {
            drawSquareBtn(150, y, 410, y + 40, (aList[i + scroll]), menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }
        else
        {
            drawSquareBtn(150, y, 410, y + 40, (aList[i + scroll] + "-Empty"), menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }

        y = y + 40;
        //scroll++;
    }
}

// Draws buttons for program function
void drawProgram(int scroll = 0)
{
    // Clear LCD to be written
    drawSquareBtn(141, 1, 478, 319, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print page title
    drawSquareBtn(180, 10, 400, 45, "Program", themeBackground, themeBackground, menuBtnColor, CENTER);

    // Scroll buttons
    myGLCD.setColor(menuBtnColor);
    myGLCD.setBackColor(themeBackground);
    drawSquareBtn(420, 100, 470, 150, "/\\", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(420, 150, 470, 200, "\\/", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    // Draws program scroll box with current scroll value
    drawProgramScroll(scroll);

    // Draw program buttons
    drawSquareBtn(150, 260, 250, 300, "Open", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(255, 260, 355, 300, "Load", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(360, 260, 460, 300, "Delete", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
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
            Serial.print("Waiting on isWait");
            if (can1.msgCheck(incID, 0x03, 0x01))
            {
                isWait = false;
            }
        }
        isWait = true;
    }
}

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
                waitForItRect(150, 50, 410, 90);
                //Serial.println(1 + scroll);
                selectedProgram = 0 + scroll;
            }
            if ((y >= 90) && (y <= 130))
            {
                // 2nd position of scroll box
                waitForItRect(150, 90, 410, 130);
                //Serial.println(2 + scroll);
                selectedProgram = 1 + scroll;
            }
            if ((y >= 130) && (y <= 170))
            {
                // 3d position of scroll box
                waitForItRect(150, 130, 410, 170);
                //Serial.println(3 + scroll);
                selectedProgram = 2 + scroll;
            }
            if ((y >= 170) && (y <= 210))
            {
                // 4th position of scroll box
                waitForItRect(150, 170, 410, 210);
                //Serial.println(4 + scroll);
                selectedProgram = 3 + scroll;
            }
            if ((y >= 210) && (y <= 250))
            {
                // 5th position of scroll box
                waitForItRect(150, 210, 410, 250);
                //Serial.println(5 + scroll);
                selectedProgram = 4 + scroll;
            }
        }
        if ((x >= 420) && (x <= 470))
        {
            if ((y >= 100) && (y <= 150))
            {
                // Scroll up
                waitForIt(420, 100, 470, 150);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramScroll(scroll);
                }
            }
        }
        if ((x >= 420) && (x <= 470))
        {
            if ((y >= 150) && (y <= 200))
            {
                // Scroll down
                waitForIt(420, 150, 470, 200);
                if (scroll < 5)
                {
                    scroll++;
                    drawProgramScroll(scroll);
                }
            }
        }

        if ((y >= 260) && (y <= 300))
        {
            if ((x >= 150) && (x <= 250))
            {
                // Open program
                waitForItRect(150, 260, 250, 300);
                runList.clear();
                loadProgram();
                programOpen = true;
                pageControl(6, 0);
            }
            if ((x >= 255) && (x <= 355))
            {
                // Load program
                waitForItRect(255, 260, 355, 300);
                runList.clear();
                loadProgram();

            }
            if ((x >= 360) && (x <= 460))
            {
                // Delete program
                waitForItRect(360, 260, 460, 300);
                bool result = errorMSG("Confirmation", "Permanently", "Delete File?");
                if (result)
                {
                    programDelete();
                }
                drawProgram();
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
    // Each node should be listed with all information, might need small text
    int row = 50;
    for (int i = 0; i < 5; i++)
    {
        String position = String(i + scroll);
        String a = ":";
        String b = " ";
        String label = position + a + String(runList.get(i + scroll)->getA1()) + b + String(runList.get(i + scroll)->getA2())
            + b + String(runList.get(i + scroll)->getA3()) + b + String(runList.get(i + scroll)->getA4()) + b + String(runList.get(i + scroll)->getA5())
            + b + String(runList.get(i + scroll)->getA6()) + b + String(runList.get(i + scroll)->getGrip()) + b + String(runList.get(i + scroll)->getID());
        if (i + scroll < nodeSize)
        {
            drawSquareBtn(150, row, 410, row + 40, label, menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }
        else
        {
            drawSquareBtn(150, row, 410, row + 40, "", menuBackground, menuBtnBorder, menuBtnText, LEFT);
        }

        row = row + 40;
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
    drawSquareBtn(141, 1, 478, 319, "", themeBackground, themeBackground, themeBackground, CENTER);

    // Print page title
    drawSquareBtn(180, 10, 400, 45, "Edit", themeBackground, themeBackground, menuBtnColor, CENTER);

    // Scroll buttons
    myGLCD.setColor(menuBtnColor);
    myGLCD.setBackColor(themeBackground);
    drawSquareBtn(420, 100, 470, 150, "/\\", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(420, 150, 470, 200, "\\/", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    // Draw program edit buttons
    drawSquareBtn(150, 260, 230, 310, "Add", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(230, 260, 310, 310, "Ins", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(310, 260, 390, 310, "Del", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawSquareBtn(390, 260, 470, 310, "Save", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    drawSquareBtn(420, 50, 470, 90, "X", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);

    myGLCD.setFont(SmallFont);
    switch (gripStatus)
    {
    case 0: drawSquareBtn(420, 210, 470, 250, " Open", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
        break;
    case 1: drawSquareBtn(420, 210, 470, 250, " Close", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
        break;
    case 2: drawSquareBtn(420, 210, 470, 250, " Grip", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
        break;
    }
    myGLCD.setFont(BigFont);
}

// Adds current position to program linked list 
void addNode(int insert = -1)
{
    // Array of arm axis positions
    uint16_t posArray[8] = { 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000 };

    // Request and update 
    axisPos.updateAxisPos();

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

        if ((x >= 150) && (x <= 410))
        {
            if ((y >= 50) && (y <= 90))
            {
                waitForItRect(150, 50, 410, 90);
                //Serial.println(1 + scroll);
                selectedNode = 0 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 90) && (y <= 130))
            {
                waitForItRect(150, 90, 410, 130);
                //Serial.println(2 + scroll);
                selectedNode = 1 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 130) && (y <= 170))
            {
                waitForItRect(150, 130, 410, 170);
                //Serial.println(3 + scroll);
                selectedNode = 2 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 170) && (y <= 210))
            {
                waitForItRect(150, 170, 410, 210);
                //Serial.println(4 + scroll);
                selectedNode = 3 + scroll;
                drawProgramEditScroll(scroll);
            }
            if ((y >= 210) && (y <= 250))
            {
                waitForItRect(150, 210, 410, 250);
                //Serial.println(5 + scroll);
                selectedNode = 4 + scroll;
                drawProgramEditScroll(scroll);
            }
        }
        if ((x >= 420) && (x <= 470))
        {
            if ((y >= 50) && (y <= 90))
            {
                // Cancel
                waitForItRect(420, 50, 470, 90);
                programOpen = false;
                pageControl(2, false);
            }
            if ((y >= 100) && (y <= 150))
            {
                waitForIt(420, 100, 470, 150);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramEditScroll(scroll);
                }
            }
            if ((y >= 150) && (y <= 200))
            {
                waitForIt(420, 150, 470, 200);
                if (scroll < 10)
                {
                    scroll++;
                    drawProgramEditScroll(scroll);
                }
            }
            if ((y >= 210) && (y <= 250))
            {
                // Grip
                waitForItRect(420, 210, 470, 250);
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
                    myGLCD.setFont(SmallFont);
                    drawSquareBtn(420, 210, 470, 250, " Open", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    myGLCD.setFont(BigFont);
                    break;
                case 1:
                    myGLCD.setFont(SmallFont);
                    drawSquareBtn(420, 210, 470, 250, "Close", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    myGLCD.setFont(BigFont);
                    break;
                case 2:
                    myGLCD.setFont(SmallFont);
                    drawSquareBtn(420, 210, 470, 250, " Grip", menuBtnColor, menuBtnBorder, menuBtnText, LEFT);
                    myGLCD.setFont(BigFont);
                    break;
                }
            }
        }

        if ((y >= 260) && (y <= 310))
        {
            if ((x >= 150) && (x <= 230))
            {
                // Add node
                waitForItRect(150, 260, 230, 310);
                addNode();
                drawProgramEditScroll(scroll);
            }
            if ((x >= 230) && (x <= 310))
            {
                // Insert node
                waitForItRect(230, 260, 310, 310);
                addNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 310) && (x <= 390))
            {
                // Delete node
                waitForItRect(310, 260, 390, 310);
                deleteNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 390) && (x <= 470))
            {
                // Save program
                waitForItRect(390, 260, 470, 310);
                programDelete();
                saveProgram();
                programOpen = false;
                pageControl(2, false);
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
    drawSquareBtn(180, 10, 400, 45, "Configuration", themeBackground, themeBackground, menuBtnColor, CENTER);
    drawSquareBtn(141, 1, 478, 319, "", themeBackground, themeBackground, themeBackground, CENTER);
    drawSquareBtn(180, 10, 400, 45, "Configuration", themeBackground, themeBackground, menuBtnColor, CENTER);
    drawRoundBtn(150, 60, 300, 100, "Home Ch1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(310, 60, 460, 100, "Set Ch1", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(150, 110, 300, 150, "Home Ch2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(310, 110, 460, 150, "Set ch2", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    return;
}

// Sends command to return arm to starting position
void homeArm(uint8_t* armIDArray)
{
    byte data1[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0x5A };
    byte data2[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0xB4 };
    byte data3[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    can1.sendFrame(armIDArray[0], data1);
    delay(150);
    can1.sendFrame(armIDArray[1], data2);
    delay(150);
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
    drawRoundBtn(10, 10, 130, 65, "1-VIEW", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 70, 130, 125, "2-PROG", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 130, 130, 185, "3-MOVE", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 190, 130, 245, "4-CONF", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
    drawRoundBtn(10, 250, 130, 305, "5-EXEC", menuBtnColor, menuBtnBorder, menuBtnText, CENTER);
}

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    Serial3.begin(57600);
    Wire.begin();        // join i2c bus (address optional for master)

    can1.startCAN();
    bool hasFailed = sdCard.startSD();
    if (!hasFailed)
    {
        Serial.println("SD failed");
    }
    else if (hasFailed)
    {
        Serial.println("SD Running");
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
        Serial.println("Capacitive touch screen initialized failure");
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

    Serial.println("Capacitive touch screen initialized success");


    // Setup the LCD
    myGLCD.InitLCD();
    // -------------------------------------------------------------
    pinMode(8, OUTPUT);  //backlight 
    digitalWrite(8, HIGH);//on
  // -------------------------------------------------------------
    myGLCD.setFont(BigFont);
    myGLCD.clrScr();
    dispx = myGLCD.getDisplayXSize();
    dispy = myGLCD.getDisplayYSize();
    // Draw the Hypertech logo
    //bmpDraw("robotarm.bmp", 0, 0);
    //bmpDraw("System/HYPER.bmp", 0, 0);
    
}

// Page control framework
void pageControl(int page, bool value = false)
{
    // Static bool ensures the page is drawn only once while the loop is running
    static bool hasDrawn;
    // Seperated because compiler produces error with 1 line
    hasDrawn = value;

    while (true)
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
                axisPos.drawAxisPos(myGLCD, CHANNEL1);
                axisPos.drawAxisPos(myGLCD, CHANNEL2);
                hasDrawn = true;
                controlPage = page;
            }
            // Call buttons if any
            break;
        case 2:
            if (programOpen)
            {
                pageControl(6);
            }
            // Draw page
            if (!hasDrawn)
            {
                drawProgram();
                hasDrawn = true;
                controlPage = page;
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
                controlPage = page;
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
                controlPage = page;
            }
            // Call buttons if any
            configButtons();
            break;
        case 5:
            // Draw page
            if (!hasDrawn)
            {
                hasDrawn = true;
                programLoaded = true;
                programRun();
                pageControl(controlPage, true);
            }
            page = 2;
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
                controlPage = page;
            }
            // Call buttons if any
            programEditButtons();
            break;
        }
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
                pageControl(page);
            }
        }
    }
}

// Buttons for the main menu
void menuButtons()
{
    while (true)
    {
        uint8_t  ss[1];
        readGT9271TouchAddr(0x814e, ss, 1);
        uint8_t status = ss[0];
        if ((status & 0x80) != 0)  // touch status   Software touch interrupt  
        {
            readGT9271TouchLocation(touchLocations, 10);
            x = 800 - touchLocations[0].x;
            y = 480 - touchLocations[0].y;

            // Menu
            if ((x >= 10) && (x <= 130))  // Button: 1
            {
                if ((y >= 10) && (y <= 65))  // Upper row
                {
                    waitForIt(10, 10, 130, 65);
                    pageControl(1);
                }
                if ((y >= 70) && (y <= 125))  // Upper row
                {

                    // X_Start, Y_Start, X_Stop, Y_Stop
                    waitForIt(10, 70, 130, 125);
                    pageControl(2);

                }
                if ((y >= 130) && (y <= 185))  // Upper row
                {
                    // X_Start, Y_Start, X_Stop, Y_Stop
                    waitForIt(10, 130, 130, 185);
                    pageControl(3);
                }
                // Settings touch button
                if ((y >= 190) && (y <= 245))
                {

                    // X_Start, Y_Start, X_Stop, Y_Stop
                    waitForIt(10, 190, 130, 245);
                    pageControl(4);
                }
                if ((y >= 250) && (y <= 305))
                {

                    // X_Start, Y_Start, X_Stop, Y_Stop
                    waitForIt(10, 250, 130, 305);
                    pageControl(5);
                }

            }
        }
        return;
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

// Calls pageControl with a value of 1 to set view page as the home page
void loop()
{
    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_WHITE);
    myGLCD.print("Loading...", 290, 290);
    delay(4000);
    drawMenu();
    print_icon(48, 420, robotarm);
    pageControl(controlPage);
}