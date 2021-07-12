/*
 Name:    ControlBoxDue.ino
 Created: 11/15/2020 8:27:18 AM
 Author:  Brandon Van Pelt
*/

/*=========================================================
    Todo List
===========================================================
TX CAN Buffer / Scheduler

Program Edit
-Add wait
-Add Sensors

New non-blocking design for the 3 waitForIt functions

Disable program edit when program is running
===========================================================
    End Todo List
=========================================================*/

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
uint16_t x, y;

// Declare fonts
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

// CAN message ID and frame, value can be changed in manualControlButtons
uint16_t txIdManual = ARM1_MANUAL;

// Execute variables
bool programLoaded = false;
bool loopProgram = true;
bool programRunning = false;
bool Arm1Ready = false;
bool Arm2Ready = false;
// THIS (uint8_t) WILL LIMIT THE SIZE OF A PROGRAM TO 255 MOVEMENTS
uint8_t programProgress = 0;

// Used hold open a program
bool programOpen = false;

// 0 = open, 1 = close, 2 = no change
int8_t gripStatus = 2;

// Program names
// Future version should Initialize list from SD card for custom program naming
String aList[10] = { "Program1", "Program2", "Program3", "Program4", "Program5", "Program6", "Program7", "Program8", "Program9", "Program10" };

// Page control variables
uint8_t currentPage = 1;
uint8_t lastPage = 1;
bool hasDrawn = false;

// Timer for current angle updates
uint32_t timer = 0;

uint8_t errorMessageReturn = 2;


/*=========================================================
    Framework Functions
===========================================================*/
// Script to print bitmap, I forget where I found it at 
void bmpDraw(char* filename, int x, int y) {
    File     bmpFile;
    int      bmpWidth, bmpHeight;   // W+H in pixels
    uint8_t  bmpDepth;              // Bit depth (currently must be 24)
    uint32_t bmpImageoffset;        // Start of image data in file
    uint32_t rowSize;               // Not always = bmpWidth; may have padding
    uint8_t  sdbuffer[3 * PIXEL_BUFFER]; // pixel in buffer (R+G+B per pixel)
    uint16_t lcdbuffer[PIXEL_BUFFER];  // pixel out buffer (16-bit per pixel)
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

// Print Robot Arm bitmap
void print_icon(int x, int y, const unsigned char icon[]) 
{
    myGLCD.setColor(MENU_BUTTON_COLOR);
    myGLCD.setBackColor(THEME_BACKGROUND);
    int i = 0, row, column, bit, temp;
    int constant = 1;
    for (row = 0; row < 40; row++) 
    {
        for (column = 0; column < 5; column++) 
        {
            temp = icon[i];
            for (bit = 7; bit >= 0; bit--) 
            {
                if (temp & constant) {myGLCD.drawPixel(x + (column * 8) + (8 - bit), y + row);}
                temp >>= 1;
            }
            i++;
        }
    }
}

/***************************************************
*  Draw Round/Square Button                        *
*                                                  *
*  Description: Draws shapes with/without text   *
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
void drawRoundBtn(uint16_t x_start, uint16_t y_start, uint16_t x_stop, uint16_t y_stop, String button, int backgroundColor, int btnBorderColor, int btnTxtColor, uint8_t align) {
    myGLCD.setColor(backgroundColor);
    myGLCD.fillRoundRect(x_start, y_start, x_stop, y_stop); 
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
        uint16_t size, temp, offset;
        size = button.length();
        temp = ((x_stop - x_start) / 2);
        offset = x_start + (temp - (8 * size));
        myGLCD.print(button, offset, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 3:
        // Currently hotwired for deg text only
        myGLCD.print(button, x_start + 55, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    }
}

void drawSquareBtn(uint16_t x_start, uint16_t y_start, uint16_t x_stop, uint16_t y_stop, String button, int backgroundColor, int btnBorderColor, int btnTxtColor, uint8_t align) {
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
        uint16_t size, temp, offset;
        size = button.length();
        temp = ((x_stop - x_start) / 2);
        offset = x_start + (temp - (8 * size));
        myGLCD.print(button, offset, y_start + ((y_stop - y_start) / 2) - 8); // hor, ver
        break;
    case 3:
        //align left
        break;
    }
}

// Highlights round buttons when selected
void waitForIt(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    myGLCD.setColor(THEME_BACKGROUND);
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

    myGLCD.setColor(MENU_BUTTON_BORDER);
    myGLCD.drawRoundRect(x1, y1, x2, y2);
}

// Highlights square buttons when selected
void waitForItRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    myGLCD.setColor(THEME_BACKGROUND);
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

    myGLCD.setColor(MENU_BUTTON_BORDER);
    myGLCD.drawRect(x1, y1, x2, y2);
}

// Highlights square buttons when selected and sends CAN message, used by manual control
void waitForItRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t txId, byte data[])
{
    myGLCD.setColor(THEME_BACKGROUND);
    myGLCD.drawRect(x1, y1, x2, y2);

    uint8_t  ss[1];
    unsigned long timer1 = millis();
    unsigned long timer2 = millis();

    // Hacked work around for not fullying understanding the Touch LCD hardware, needs reworked
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

    myGLCD.setColor(MENU_BUTTON_BORDER);
    myGLCD.drawRect(x1, y1, x2, y2);
}


/*=========================================================
    Manual control Functions
===========================================================*/
// Draw the manual control page
void drawManualControl(uint16_t x = 146, uint16_t y = 80, bool drawGrip = true)
{
    // Clear LCD to be written 
    drawSquareBtn(141, 1, 799, 479, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);

    // Print page title
    drawSquareBtn((x + 34), (y - 70), (x + 254), (y - 35), F("Manual Control"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);

    // Manual control axis labels
    uint16_t xPos = 1;
    for (uint16_t yPos = x; yPos < (x + 334 - 45); yPos = yPos + 54)
    {
        myGLCD.setColor(MENU_BUTTON_COLOR);
        myGLCD.setBackColor(THEME_BACKGROUND);
        myGLCD.printNumI(xPos, yPos + 20, y - 20);
        xPos++;
    }

    // Draw the upper row of movement buttons
    for (uint16_t i = x; i < (x + 334 - 45); i = i + 54)
    {
        drawSquareBtn(i, y, (i + 54), (y + 60), F("/\\"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }

    // Draw the bottom row of movement buttons
    for (uint16_t i = x; i < (x + 334 - 54); i = i + 54)
    {
        drawSquareBtn(i, (y + 60), i + 54, (y + 120), F("\\/"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }

    // Draw Select arm buttons
    drawSquareBtn((x + 19), (y + 125), x + 60, (y + 165), F("Arm"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    if (txIdManual == ARM1_MANUAL)
    {
        drawSquareBtn(x, (y + 160), x + 54, (y + 215), "1", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
        drawSquareBtn((x + 54), (y + 160), x + 108, (y + 215), "2", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }
    else if (txIdManual == ARM2_MANUAL)
    {
        drawSquareBtn(x, (y + 160), x + 54, (y + 215), "1", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        drawSquareBtn((x + 54), (y + 160), x + 108, (y + 215), "2", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
    }

    if (drawGrip)
    {
        // Draw grip buttons
        drawSquareBtn(270, 205, 450, 245, F("Gripper"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
        drawSquareBtn(270, 240, 360, 295, F("Open"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        drawSquareBtn(360, 240, 450, 295, F("Close"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }
}

// Draw page button function
void manualControlButtons(uint16_t x1 = 146, uint16_t y1 = 80, bool drawGrip = true)
{
    // Mutiply is a future funtion to allow movement of multiple angles at a time instead of just 1
    uint8_t multiply = 1;

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

        if ((y >= y1) && (y <= (y1 + 60)))
        {
            // Axis 1 Up
            if ((x >= 146) && (x <= 200))
            {
                data[1] = 1 * multiply;
                waitForItRect(146, 80, 200, 140, txIdManual, data);
                data[1] = 0;
            }
            // Axis 2 Up
            if ((x >= 200) && (x <= 254))
            {
                data[2] = 1 * multiply;
                waitForItRect(200, 80, 254, 140, txIdManual, data);
                data[2] = 0;
            }
            // Axis 3 Up
            if ((x >= 254) && (x <= 308))
            {
                data[3] = 1 * multiply;
                waitForItRect(254, 80, 308, 140, txIdManual, data);
                data[3] = 0;
            }
            // Axis 4 Up
            if ((x >= 308) && (x <= 362))
            {
                data[4] = 1 * multiply;
                waitForItRect(308, 80, 362, 140, txIdManual, data);
                data[4] = 0;
            }
            // Axis 5 Up
            if ((x >= 362) && (x <= 416))
            {
                data[5] = 1 * multiply;
                waitForItRect(362, 80, 416, 140, txIdManual, data);
                data[5] = 0;
            }
            // Axis 6 Up
            if ((x >= 416) && (x <= 470))
            {
                data[6] = 1 * multiply;
                waitForItRect(416, 80, 470, 140, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= (y1 + 60)) && (y <= (y1 + 120)))
        {
            // Axis 1 Down
            if ((x >= 156) && (x <= 200))
            {
                data[1] = (1 * multiply) + reverse;
                waitForItRect(146, 140, 200, 200, txIdManual, data);
                data[1] = 0;
            }
            // Axis 2 Down
            if ((x >= 200) && (x <= 254))
            {
                data[2] = (1 * multiply) + reverse;
                waitForItRect(200, 140, 254, 200, txIdManual, data);
                data[2] = 0;
            }
            // Axis 3 Down
            if ((x >= 254) && (x <= 308))
            {
                data[3] = (1 * multiply) + reverse;
                waitForItRect(254, 140, 308, 200, txIdManual, data);
                data[3] = 0;
            }
            // Axis 4 Down
            if ((x >= 308) && (x <= 362))
            {
                data[4] = (1 * multiply) + reverse;
                waitForItRect(308, 140, 362, 200, txIdManual, data);
                data[4] = 0;
            }
            // Axis 5 Down
            if ((x >= 362) && (x <= 416))
            {
                data[5] = (1 * multiply) + reverse;
                waitForItRect(362, 140, 416, 200, txIdManual, data);
                data[5] = 0;
            }
            // Axis 6 Down
            if ((x >= 416) && (x <= 470))
            {
                data[6] = (1 * multiply) + reverse;
                waitForItRect(416, 140, 470, 200, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= (y1 + 160)) && (y <= (y1 + 215)))
        {
            if ((x >= 146) && (x <= 200))
            {
                // Select arm 1
                drawSquareBtn(146, 240, 200, 295, "1", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
                drawSquareBtn(200, 240, 254, 295, "2", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
                txIdManual = ARM1_MANUAL;
            }
            if ((x >= 200) && (x <= 254))
            {
                // Select arm 2
                drawSquareBtn(146, 240, 200, 295, "1", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
                drawSquareBtn(200, 240, 254, 295, "2", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
                txIdManual = ARM2_MANUAL;
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
    drawSquareBtn(141, 1, 799, 479, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);

    // Draw row lables
    for (int start = 35, stop = 75, row = 1; start <= 260; start = start + 45, stop = stop + 45, row++)
    {
        String rowLable = "A" + String(row);
        drawRoundBtn(170, start, 190, stop, rowLable, THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    }

    // Boxes for current arm angles

    uint8_t const yStart = 35;
    uint8_t const yStop = 75;

    // Arm 1
    drawRoundBtn(310, 5, 415, 40, F("Arm2"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    drawRoundBtn(310, yStart + 0, 415, yStop + 0, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(310, yStart + 45, 415, yStop + 45, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(310, yStart + 90, 415, yStop + 90, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(310, yStart + 135, 415, yStop + 135, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(310, yStart + 180, 415, yStop + 180, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(310, yStart + 225, 415, yStop + 225, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    // Arm 2
    drawRoundBtn(205, 5, 305, 40, F("Arm1"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    drawRoundBtn(200, yStart + 0, 305, yStop + 0, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(200, yStart + 45, 305, yStop + 45, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(200, yStart + 90, 305, yStop + 90, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(200, yStart + 135, 305, yStop + 135, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(200, yStart + 180, 305, yStop + 180, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
    drawRoundBtn(200, yStart + 225, 305, yStop + 225, DEG, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_RIGHT);
}

void updateViewPage()
{
    if (millis() - timer > REFRESH_RATE)
    {
        axisPos.sendRequest(can1);
        if (currentPage == 1)
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
void drawProgramScroll(uint16_t scroll)
{
    uint16_t y = 50;
    const uint16_t x = 150;
    const uint16_t width = 300;
    const uint8_t height = 40;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (sdCard.fileExists(aList[i + scroll]))
        {
            drawSquareBtn(x, y, (x + width), (y + height), (aList[i + scroll]), MENU_BACKGROUND, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
        }
        else
        {
            drawSquareBtn(x, y, (x + width), (y + height), (aList[i + scroll] + F("-Empty")), MENU_BACKGROUND, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
        }

        y = y + height;
    }
}

// Draws buttons for program function
void drawProgram(uint16_t scroll = 0)
{
    // Clear LCD to be written
    drawSquareBtn(141, 1, 799, 479, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);

    // Print page title
    drawSquareBtn(180, 10, 400, 45, F("Program"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);

    // Scroll buttons
    myGLCD.setColor(MENU_BUTTON_COLOR);
    myGLCD.setBackColor(THEME_BACKGROUND);
    drawSquareBtn(460, 110, 510, 210, F("/\\"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(460, 210, 510, 310, F("\\/"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);

    // Draws program scroll box with current scroll value
    drawProgramScroll(scroll);

    // Draw program buttons
    drawSquareBtn(150, 430, 250, 470, F("Open"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(255, 430, 355, 470, F("Load"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(360, 430, 460, 470, F("Delete"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(465, 430, 575, 470, F("Rename"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
}

// Deletes current selected program
void programDelete()
{
    sdCard.deleteFile(aList[selectedProgram]);
}

// Load selected program from SD card into linked list
void loadProgram()
{
    sdCard.readFile(aList[selectedProgram], runList);
}

// Button functions for program page 
void programButtons()
{
    // Static so that the position is saved while this method is repeatedly called in the loop
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
            if ((y >= 110) && (y <= 210))
            {
                // Scroll up
                waitForIt(460, 110, 510, 210);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramScroll(scroll);
                }
            }
            if ((y >= 210) && (y <= 310))
            {
                // Scroll down
                waitForIt(460, 210, 510, 310);
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
                currentPage = 6;
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
                errorMessageReturn = 2;
                drawErrorMSG(F("Confirmation"), F("Permanently"), F("Delete File?"));
                lastPage = currentPage;
                currentPage = 7;
                hasDrawn = false;
            }
            if ((x >= 465) && (x <= 565))
            {
                // Rename program
                //waitForItRect(465, 430, 565, 470);
                // TODO
            }
        }
    }
}


/*==========================================================
                    Edit Program
============================================================*/
// Draws scrollable box that contains all the nodes in a program
void drawProgramEditScroll(uint8_t scroll = 0)
{
    myGLCD.setFont(SmallFont);
    uint8_t nodeSize = runList.size();
    int y = 50;
    int x = 150;
    uint8_t height = 40;
    int width = 295;

    for (int i = 0; i < 9; i++)
    {
        String position = String(i + scroll);
        String a = ":";
        String b = " ";
        String label = position + a + String(runList.get(i + scroll)->getA1()) + b + String(runList.get(i + scroll)->getA2())
            + b + String(runList.get(i + scroll)->getA3()) + b + String(runList.get(i + scroll)->getA4()) + b + String(runList.get(i + scroll)->getA5())
            + b + String(runList.get(i + scroll)->getA6()) + b + String(runList.get(i + scroll)->getGrip()) + b + String(runList.get(i + scroll)->getID());
        (i + scroll < nodeSize) ? drawSquareBtn(x, y, (x + width), (y + height), label, MENU_BACKGROUND, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT) : drawSquareBtn(x, y, (x + width), (y + height), "", MENU_BACKGROUND, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
        y += 40;
    }
    myGLCD.setFont(BigFont);
}

// Draws buttons for edit program function
void drawProgramEdit(uint8_t scroll = 0)
{
    // Clear LCD to be written
    drawSquareBtn(141, 1, 799, 479, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);
    
    // Print page title
    drawSquareBtn(180, 10, 400, 45, F("Edit Program"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);

    // Scroll buttons
    myGLCD.setColor(MENU_BUTTON_COLOR);
    myGLCD.setBackColor(THEME_BACKGROUND);
    drawSquareBtn(450, 130, 500, 230, F("/\\"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(450, 230, 500, 330, F("\\/"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);

    uint16_t x = 505;
    uint16_t y = 170;

    // Draw Select arm buttons
    drawSquareBtn((x + 100), (y - 140), x + 180, (y - 100), F("Select Arm"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    if (txIdManual == ARM1_MANUAL)
    {
        drawSquareBtn(x + 90, (y - 95), x + 140, (y - 40), "1", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
        drawSquareBtn((x + 140), (y - 95), x + 190, (y - 40), "2", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }
    else if (txIdManual == ARM2_MANUAL)
    {
        drawSquareBtn(x + 90, (y - 95), x + 140, (y - 40), "1", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        drawSquareBtn((x + 140), (y - 95), x + 190, (y - 40), "2", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
    }

    // Axis numbering
    uint16_t j = 1;
    for (uint16_t i = x - 5; i <= 752; i = i + 48)
    {
        myGLCD.setColor(MENU_BUTTON_COLOR);
        myGLCD.setBackColor(THEME_BACKGROUND);
        myGLCD.printNumI(j, i + 20, y - 20);
        j++;
    }

    // Draw the upper row of movement buttons
    for (uint16_t i = x; i <= 752; i = i + 48)
    {
        drawSquareBtn(i, y, (i + 48), (y + 60), F("/\\"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }

    // Draw the bottom row of movement buttons
    for (uint16_t i = x; i <= 752; i = i + 48)
    {
        drawSquareBtn(i, (y + 60), (i + 48), (y + 120), F("\\/"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    }

    // Draw program edit buttons
    drawSquareBtn(141, 430, 225, 470, F("Add"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(225, 430, 310, 470, F("Ins"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(310, 430, 395, 470, F("Del"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    switch (gripStatus)
    {
    case 0: drawSquareBtn(395, 430, 480, 470, F("Open"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        break;
    case 1: drawSquareBtn(395, 430, 480, 470, F("Close"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        break;
    case 2: drawSquareBtn(395, 430, 480, 470, F("Grip"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
        break;
    }
    drawSquareBtn(480, 430, 565, 470, F("Wait"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(565, 430, 650, 470, F("Sens"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(650, 430, 735, 470, F("Save"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(735, 430, 799, 470, F("X"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
}

// Adds current position to program linked list 
void addNode(uint16_t insert = -1)
{
    // Array of arm axis positions
    uint16_t posArray[8] = { 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000 };

    // Update array value with data collected from the axis position update
    if (txIdManual == ARM1_MANUAL)
    {
        posArray[0] = axisPos.getA1C1();
        posArray[1] = axisPos.getA2C1();
        posArray[2] = axisPos.getA3C1();
        posArray[3] = axisPos.getA4C1();
        posArray[4] = axisPos.getA5C1();
        posArray[5] = axisPos.getA6C1();
    }
    else if (txIdManual == ARM2_MANUAL)
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

    (insert < 0) ? runList.add(node) : runList.add(insert, node);
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
    const String space = ", ";

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
    static uint8_t selectedNode = 0;
    static uint16_t scroll = 0;

    // Mutiply is a future funtion to allow movement of multiple angles at a time instead of just 1
    uint8_t multiply = 1;

    // Enables revese
    uint8_t reverse = 0x10;

    byte data[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
        if ((x >= 450) && (x <= 500))
        {
            if ((y >= 130) && (y <= 230))
            {
                waitForIt(450, 130, 500, 230);
                if (scroll > 0)
                {
                    scroll--;
                    drawProgramEditScroll(scroll);
                }
            }
            if ((y >= 230) && (y <= 330))
            {
                waitForIt(450, 230, 500, 330);
                if (scroll < 10)
                {
                    scroll++;
                    drawProgramEditScroll(scroll);
                }
            }
        }
        if ((y >= 75) && (y <= 130))
        {
            if ((x >= 595) && (x <= 645))
            {
                //Select Arm 1
                waitForIt(595, 75, 645, 130);
                drawSquareBtn(595, 75, 645, 130, "1", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
                drawSquareBtn(645, 75, 695, 130, "2", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
                txIdManual = ARM1_MANUAL;
            }
            if ((x >= 645) && (x <= 695))
            {
                // Select Arm 2
                waitForIt(645, 75, 695, 130);
                drawSquareBtn(595, 75, 645, 130, "1", MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
                drawSquareBtn(645, 75, 695, 130, "2", MENU_BUTTON_TEXT, MENU_BUTTON_BORDER, MENU_BUTTON_COLOR, ALIGN_CENTER);
                txIdManual = ARM2_MANUAL;
            }
        }
        if ((y >= 170) && (y <= 230))
        {
            // A1 Up
            if ((x >= 505) && (x <= 553))
            {
                data[1] = 1 * multiply;
                waitForItRect(505, 170, 553, 230, txIdManual, data);
                data[1] = 0;
            }
            // A2 Up
            if ((x >= 553) && (x <= 601))
            {
                data[2] = 1 * multiply;
                waitForItRect(553, 170, 601, 230, txIdManual, data);
                data[2] = 0;
            }
            // A3 Up
            if ((x >= 601) && (x <= 649))
            {
                data[3] = 1 * multiply;
                waitForItRect(601, 170, 649, 230, txIdManual, data);
                data[3] = 0;
            }
            // A4 Up
            if ((x >= 649) && (x <= 697))
            {
                data[4] = 1 * multiply;
                waitForItRect(649, 170, 697, 230, txIdManual, data);
                data[4] = 0;
            }
            // A5 Up
            if ((x >= 697) && (x <= 745))
            {
                data[5] = 1 * multiply;
                waitForItRect(697, 170, 745, 230, txIdManual, data);
                data[5] = 0;
            }
            // A6 Up
            if ((x >= 745) && (x <= 793))
            {
                data[6] = 1 * multiply;
                waitForItRect(745, 170, 793, 230, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= 230) && (y <= 290))
        {
            // A1 Down
            if ((x >= 505) && (x <= 553))
            {
                data[1] = (1 * multiply) + reverse;
                waitForItRect(505, 230, 553, 290, txIdManual, data);
                data[1] = 0;
            }
            // A2 Down
            if ((x >= 553) && (x <= 601))
            {
                data[2] = (1 * multiply) + reverse;
                waitForItRect(553, 230, 601, 290, txIdManual, data);
                data[2] = 0;
            }
            // A3 Down
            if ((x >= 601) && (x <= 649))
            {
                data[3] = (1 * multiply) + reverse;
                waitForItRect(601, 230, 649, 290, txIdManual, data);
                data[3] = 0;
            }
            // A4 Down
            if ((x >= 649) && (x <= 697))
            {
                data[4] = (1 * multiply) + reverse;
                waitForItRect(649, 230, 679, 290, txIdManual, data);
                data[4] = 0;
            }
            // A5 Down
            if ((x >= 697) && (x <= 745))
            {
                data[5] = (1 * multiply) + reverse;
                waitForItRect(679, 230, 745, 290, txIdManual, data);
                data[5] = 0;
            }
            // A6 Down
            if ((x >= 745) && (x <= 793))
            {
                data[6] = (1 * multiply) + reverse;
                waitForItRect(745, 230, 793, 290, txIdManual, data);
                data[6] = 0;
            }
        }
        if ((y >= 430) && (y <= 470))
        {
            if ((x >= 140) && (x <= 225))
            {
                // Add node
                waitForItRect(141, 430, 225, 470);
                addNode();
                drawProgramEditScroll(scroll);
            }
            if ((x >= 225) && (x <= 310))
            {
                // Insert node
                waitForItRect(225, 430, 310, 470);
                addNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 310) && (x <= 395))
            {
                // Delete node
                waitForItRect(310, 430, 395, 470);
                deleteNode(selectedNode);
                drawProgramEditScroll(scroll);
            }
            if ((x >= 395) && (x <= 480))
            {
                // Grip
                waitForItRect(395, 430, 480, 470);
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
                    drawSquareBtn(395, 430, 480, 470, F("Open"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
                    break;
                case 1:
                    drawSquareBtn(395, 430, 480, 470, F("Shut"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
                    break;
                case 2:
                    drawSquareBtn(395, 430, 480, 470, F("Grip"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_LEFT);
                    break;
                }
            }
            if ((x >= 480) && (x <= 565))
            {
                // Wait
                waitForItRect(480, 430, 565, 470);
                // TODO
            }
            if ((x >= 565) && (x <= 650))
            {
                // Sensor
                waitForItRect(565, 430, 650, 470);
                // TODO
            }
            if ((x >= 650) && (x <= 735))
            {
                // Save program
                waitForItRect(650, 430, 735, 470);
                programDelete();
                saveProgram();
                programOpen = false;
                currentPage = 2;
                hasDrawn = false;
            }
            if ((x >= 735) && (x <= 799))
            {
                // Cancel
                waitForItRect(735, 430, 799, 470);
                programOpen = false;
                currentPage = 2;
                hasDrawn = false;
            }
        }
    }
}


/*==========================================================
                    Configure Arm
============================================================*/
// Draws the config page
void drawConfig()
{
    drawSquareBtn(141, 1, 799, 479, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);
    drawSquareBtn(180, 10, 400, 45, F("Configuration"), THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
    drawRoundBtn(150, 60, 300, 100, F("Home Ch1"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(310, 60, 460, 100, F("Set Ch1"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(150, 110, 300, 150, F("Home Ch2"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(310, 110, 460, 150, F("Set Ch2"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(150, 160, 300, 200, F("Loop On"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(310, 160, 460, 200, F("Loop Off"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
}

// Sends command to return arm to starting position
void homeArm(uint8_t* armIDArray)
{
    byte lowerPositions[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0x5A };
    byte upperPositions[8] = { 0x00, 0x00, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0xB4 };
    byte executeMove[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    can1.sendFrame(armIDArray[0], lowerPositions);
    can1.sendFrame(armIDArray[1], upperPositions);
    can1.sendFrame(armIDArray[2], executeMove);
}

// Button functions for config page
void configButtons()
{
    uint8_t setHomeID[8] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t* setHomeIDPtr = setHomeID;
    uint8_t arm1IDArray[3] = { ARM1_LOWER, ARM1_UPPER, ARM1_CONTROL };
    uint8_t arm2IDArray[3] = { ARM2_LOWER, ARM2_UPPER, ARM2_CONTROL };
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
                can1.sendFrame(arm1IDArray[2], setHomeIDPtr);
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
                can1.sendFrame(arm2IDArray[2], setHomeIDPtr);
            }
        }
        if ((y >= 160) && (y <= 200))
        {
            if ((x >= 150) && (x <= 300))
            {
                waitForIt(150, 160, 300, 200);
                loopProgram = true;
            }
            if ((x >= 310) && (x <= 460))
            {
                waitForIt(310, 160, 460, 200);
                loopProgram = false;
            }
        }
    }
}


/*=========================================================
    Framework Functions
===========================================================*/
//Only called once at startup to draw the menu
void drawMenu()
{
    // Draw Layout
    drawSquareBtn(0, 0, 800, 480, "", THEME_BACKGROUND, THEME_BACKGROUND, THEME_BACKGROUND, ALIGN_CENTER);
    drawSquareBtn(0, 0, 140, 480, "", MENU_BACKGROUND, MENU_BACKGROUND, MENU_BACKGROUND, ALIGN_CENTER);

    // Draw Menu Buttons
    drawRoundBtn(10, 10, 130, 65, F("1-VIEW"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(10, 70, 130, 125, F("2-PROG"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(10, 130, 130, 185, F("3-MOVE"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(10, 190, 130, 245, F("4-CONF"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(10, 250, 130, 305, F("5-EXEC"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(10, 310, 130, 365, F("6-STOP"), MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, MENU_BUTTON_TEXT, ALIGN_CENTER);
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
    drawSquareBtn(1, 1, 800, 600, "", THEME_BACKGROUND, THEME_BACKGROUND, MENU_BUTTON_COLOR, ALIGN_CENTER);
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
    switch (currentPage)
    {
    case 1: // View Page
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
    case 2: // Program page
        // If program open jump to page 6
        if (programOpen)
        {
            currentPage = 6;
            break;
        }
        if (errorMessageReturn == 1)
        {
            programDelete();
            errorMessageReturn = 2;
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
    case 3: // Move page
        // Draw page
        if (!hasDrawn)
        {
            drawManualControl();
            hasDrawn = true;
        }
        // Call buttons if any
        manualControlButtons();
        break;
    case 4: // Configuation page
        // Draw page
        if (!hasDrawn)
        {
            drawConfig();
            hasDrawn = true;
        }
        // Call buttons if any
        configButtons();
        break;
    case 5: // Execute
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
            currentPage = lastPage;
        }
        // Call buttons if any
        break;
    case 6: // Program edit page
        // Draw page
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
    case 7: // Error page
        // Draw page
        if (!hasDrawn)
        {
            hasDrawn = true;
        }
        if (errorMessageReturn == 0 || errorMessageReturn == 1)
        {
            currentPage = lastPage;
            hasDrawn = false;
        }
        // Call buttons if any
        errorMSGButtons();
        break;
    }
}

// errorMSGReturn
// 0 = false
// 1 = true
// 2 = no value
// Error Message function
bool drawErrorMSG(String title, String eMessage1, String eMessage2)
{
    drawSquareBtn(145, 100, 415, 220, "", MENU_BACKGROUND, MENU_BUTTON_COLOR, MENU_BUTTON_COLOR, ALIGN_CENTER);
    drawSquareBtn(145, 100, 415, 130, title, THEME_BACKGROUND, MENU_BUTTON_COLOR, MENU_BUTTON_BORDER, ALIGN_LEFT);
    drawSquareBtn(146, 131, 414, 155, eMessage1, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawSquareBtn(146, 155, 414, 180, eMessage2, MENU_BACKGROUND, MENU_BACKGROUND, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(365, 100, 415, 130, "X", MENU_BUTTON_COLOR, MENU_BUTTON_COLOR, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(155, 180, 275, 215, "Confirm", MENU_BUTTON_COLOR, MENU_BUTTON_COLOR, MENU_BUTTON_TEXT, ALIGN_CENTER);
    drawRoundBtn(285, 180, 405, 215, "Cancel", MENU_BUTTON_COLOR, MENU_BUTTON_COLOR, MENU_BUTTON_TEXT, ALIGN_CENTER);
}

void errorMSGButtons()
{
    // Touch screen controls
    uint8_t  ss[1];
    readGT9271TouchAddr(0x814e, ss, 1);
    uint8_t status = ss[0];
    if ((status & 0x80) != 0) 
    {
        readGT9271TouchLocation(touchLocations, 10);
        x = 800 - touchLocations[0].x;
        y = 480 - touchLocations[0].y;

        if ((x >= 365) && (x <= 415))
        {
            if ((y >= 100) && (y <= 130))
            {
                waitForItRect(365, 100, 415, 130);
                errorMessageReturn = 0;
            }
        }
        if ((y >= 180) && (y <= 215))
        {
            if ((x >= 155) && (x <= 275))
            {
                waitForItRect(155, 180, 275, 215);
                errorMessageReturn = 1;

            }
            if ((x >= 285) && (x <= 405))
            {
                waitForItRect(285, 180, 405, 215);
                errorMessageReturn = 0;
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
                page = lastPage;
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
                currentPage = 1;
                hasDrawn = false;
            }
            if ((y >= 70) && (y <= 125))  
            {
                waitForIt(10, 70, 130, 125);
                lastPage = currentPage;
                currentPage = 2;
                hasDrawn = false;
            }
            if ((y >= 130) && (y <= 185)) 
            {
                waitForIt(10, 130, 130, 185);
                currentPage = 3;
                hasDrawn = false;
            }
            if ((y >= 190) && (y <= 245))
            {
                waitForIt(10, 190, 130, 245);
                currentPage = 4;
                hasDrawn = false;
            }
            if ((y >= 250) && (y <= 305))
            {
                waitForIt(10, 250, 130, 305);
                lastPage = currentPage;
                currentPage = 5;
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

// Watch for incoming CAN traffic
void TrafficManager()
{
    uint8_t sw_fn = can1.processFrame();
    switch (sw_fn)
    {
    case 0: // No traffic

        break;

    case 1: // C1 lower
        axisPos.updateAxisPos(can1, ARM1_RX);
        if (currentPage == 1)
        {
            axisPos.drawAxisPos(myGLCD);
        }
        break;

    case 2: //  C1 Upper
        axisPos.updateAxisPos(can1, ARM1_RX);
        if (currentPage == 1)
        {
            axisPos.drawAxisPos(myGLCD);
        }
        break;

    case 3: // C1 Confirmation
        Arm1Ready = true;
        Arm2Ready = true;
        Serial.println("Arm1Ready");
        break;

    case 4: // C2 Lower
        axisPos.updateAxisPos(can1, ARM2_RX);
        if (currentPage == 1)
        {
            axisPos.drawAxisPos(myGLCD);
        }
        break;

    case 5: // C2 Upper
        axisPos.updateAxisPos(can1, ARM2_RX);
        if (currentPage == 1)
        {
            axisPos.drawAxisPos(myGLCD);
        }
        break;

    case 6: // C2 Confirmation
        Arm1Ready = true;
        Arm2Ready = true;
        Serial.println("Arm2Ready");
        break;
    }
}

//
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
        uint8_t lowerAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t upperAxis[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t executeMove[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint16_t IDArray[3];
        uint16_t incomingID;

        if (runList.get(programProgress)->getID() == ARM1_MANUAL)
        {
            IDArray[0] = ARM1_CONTROL;
            IDArray[1] = ARM1_LOWER;
            IDArray[2] = ARM1_UPPER;
            incomingID = ARM1_RX;
        }
        if (runList.get(programProgress)->getID() == ARM2_MANUAL)
        {
            IDArray[0] = ARM2_CONTROL;
            IDArray[1] = ARM2_LOWER;
            IDArray[2] = ARM2_UPPER;
            incomingID = ARM2_RX;
        }

        // Populate CAN messages with angles from current linkedlist
        // Axis 1
        if (runList.get(programProgress)->getA1() <= 0xFF)
        {
            lowerAxis[3] = runList.get(programProgress)->getA1();
        }
        else
        {
            lowerAxis[2] = runList.get(programProgress)->getA1() - 0xFF;
            lowerAxis[3] = 0xFF;
        }
        // Axis 2
        if (runList.get(programProgress)->getA2() <= 0xFF)
        {
            lowerAxis[5] = runList.get(programProgress)->getA2();
        }
        else
        {
            lowerAxis[4] = runList.get(programProgress)->getA2() - 0xFF;
            lowerAxis[5] = 0xFF;
        }
        // Axis 3
        if (runList.get(programProgress)->getA3() <= 0xFF)
        {
            lowerAxis[7] = runList.get(programProgress)->getA3();
        }
        else
        {
            lowerAxis[6] = runList.get(programProgress)->getA3() - 0xFF;
            lowerAxis[7] = 0xFF;
        }

        // Send first frame with axis 1-3
        can1.sendFrame(IDArray[1], lowerAxis);

        // Axis 4
        if (runList.get(programProgress)->getA4() <= 0xFF)
        {
            upperAxis[3] = runList.get(programProgress)->getA4();
        }
        else
        {
            upperAxis[2] = runList.get(programProgress)->getA4() - 0xFF;
            upperAxis[3] = 0xFF;
        }
        // Axis 5
        if (runList.get(programProgress)->getA5() <= 0xFF)
        {
            upperAxis[5] = runList.get(programProgress)->getA5();
        }
        else
        {
            upperAxis[4] = runList.get(programProgress)->getA5() - 0xFF;
            upperAxis[5] = 0xFF;
        }
        // Axis 6
        if (runList.get(programProgress)->getA5() <= 0xFF)
        {
            upperAxis[7] = runList.get(programProgress)->getA6();
        }
        else
        {
            upperAxis[6] = runList.get(programProgress)->getA6() - 0xFF;
            upperAxis[7] = 0xFF;
        }

        // Send second frame with axis 4-6
        can1.sendFrame(IDArray[2], upperAxis);

        // Change to array of IDs
        uint8_t ID = runList.get(programProgress)->getID();

        // Grip on/off or hold based on current and next state
        // If there was a change in the grip bool
        executeMove[6] = 0x00;
        executeMove[7] = 0x00;

        if (runList.get(programProgress)->getGrip() == 0)
        {
            executeMove[6] = 0x01;
        }
        else if (runList.get(programProgress)->getGrip() == 1)
        {
            executeMove[7] = 0x01;
        }

        // Send third frame with grip and execute command
        can1.sendFrame(IDArray[0], executeMove);
        
        Arm1Ready = false;
        Arm2Ready = false;
        //Serial.print("linkedListSize: ");
        //Serial.println(programProgress);
        programProgress++;
    }

    // Loop if enabled
    if (programProgress == runList.size() && loopProgram == true)
    {
        programProgress = 0;
        programRunning = true;
    }
}

//
void loop()
{
    // GUI
    pageControl();

    // Background Processes
    TrafficManager();
    //updateViewPage();
    executeProgram();
}