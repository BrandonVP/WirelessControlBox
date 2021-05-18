#pragma once
#include <stdint.h>
#include <UTFT.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>

uint8_t addr = 0x5d;  //CTP IIC ADDRESS
// Declare which fonts we will be using
extern uint8_t SmallFont[];

// Set the pins to the correct ones for your development shield
// Standard Arduino Mega/Due shield       

#define GT9271_RESET 41   //CTP RESET
#define GT9271_INT   48   //CTP  INT

UTFT myGLCD(SSD1963_800480, 38, 39, 40, 41);  //(byte model, int RS, int WR, int CS, int RST)

const PROGMEM unsigned char GTP_CFG_DATA[] =
{

0x00,0x20,0x03,0xE0,0x01,0x0A,0x0D,0x00,
0x01,0x0A,0x28,0x0F,0x50,0x32,0x03,0x08,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x8E,0x2E,0x88,0x23,0x21,
0x31,0x0D,0x00,0x00,0x00,0x01,0x03,0x1D,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x1E,0x50,0x94,0xC5,0x02,
0x07,0x00,0x00,0x04,0x80,0x21,0x00,0x6B,
0x28,0x00,0x59,0x31,0x00,0x4B,0x3B,0x00,
0x3F,0x48,0x00,0x3F,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x15,0x14,0x11,0x10,0x0F,0x0E,0x0D,0x0C,
0x09,0x08,0x07,0x06,0x05,0x04,0x01,0x00,
0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
0x10,0x11,0x12,0x13,0x19,0x1B,0x1C,0x1E,
0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,
0x27,0x28,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x4D,0x01
};


struct TouchLocation
{
    uint16_t x;
    uint16_t y;
};
TouchLocation touchLocations[10];

void inttostr(uint16_t value, uint8_t* str);
uint8_t GT9271_Send_Cfg(uint8_t* buf, uint16_t cfg_len);
void writeGT9271TouchRegister(uint16_t regAddr, uint8_t* val, uint16_t cnt);
uint8_t readGT9271TouchAddr(uint16_t regAddr, uint8_t* pBuf, uint8_t len);
uint8_t readGT9271TouchLocation(TouchLocation* pLoc, uint8_t num);
uint32_t dist(const TouchLocation& loc);
uint32_t dist(const TouchLocation& loc1, const TouchLocation& loc2);
bool sameLoc(const TouchLocation& loc, const TouchLocation& loc2);


uint8_t buf[80];


uint8_t GT9271_Send_Cfg(uint8_t* buf, uint16_t cfg_len)
{
    //uint8_t ret=0;
    uint8_t retry = 0;
    for (retry = 0; retry < 2; retry++)
    {
        writeGT9271TouchRegister(0x8047, buf, cfg_len);
        //if(ret==0)break;
        delay(10);
    }
    //return ret;
}


void writeGT9271TouchRegister(uint16_t regAddr, uint8_t* val, uint16_t cnt)
{
    uint16_t i = 0;
    Wire.beginTransmission(addr);
    Wire.write(regAddr >> 8);  // register 0
    Wire.write(regAddr);  // register 0 
    for (i = 0; i < cnt; i++, val++)//data
    {
        Wire.write(*val);  // value
    }
    uint8_t retVal = Wire.endTransmission();
}


uint8_t readGT9271TouchAddr(uint16_t regAddr, uint8_t* pBuf, uint8_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(regAddr >> 8);  // register 0
    Wire.write(regAddr);  // register 0  
    uint8_t retVal = Wire.endTransmission();

    uint8_t returned = Wire.requestFrom(addr, len);    // request 1 bytes from slave device #2

    uint8_t i;
    for (i = 0; (i < len) && Wire.available(); i++)

    {
        pBuf[i] = Wire.read();
    }

    return i;
}


uint8_t readGT9271TouchLocation(TouchLocation* pLoc, uint8_t num)
{
    uint8_t retVal;
    uint8_t i;
    uint8_t k;
    uint8_t  ss[1];

    do
    {

        if (!pLoc) break; // must have a buffer
        if (!num)  break; // must be able to take at least one
        ss[0] = 0;
        readGT9271TouchAddr(0x814e, ss, 1);
        uint8_t status = ss[0];

        if ((status & 0x0f) == 0) break; // no points detected
        uint8_t hitPoints = status & 0x0f;

        //Serial.print("number of hit points = ");
        //Serial.println(hitPoints);

        uint8_t tbuf[32]; uint8_t tbuf1[32]; uint8_t tbuf2[16];
        readGT9271TouchAddr(0x8150, tbuf, 32);
        readGT9271TouchAddr(0x8150 + 32, tbuf1, 32);
        readGT9271TouchAddr(0x8150 + 64, tbuf2, 16);

        if (hitPoints <= 4)
        {
            for (k = 0, i = 0; (i < 4 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf[i + 1] << 8 | tbuf[i + 0];
                pLoc[k].y = tbuf[i + 3] << 8 | tbuf[i + 2];
            }
        }
        if (hitPoints > 4)
        {
            for (k = 0, i = 0; (i < 4 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf[i + 1] << 8 | tbuf[i + 0];
                pLoc[k].y = tbuf[i + 3] << 8 | tbuf[i + 2];
            }

            for (k = 4, i = 0; (i < 4 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf1[i + 1] << 8 | tbuf1[i + 0];
                pLoc[k].y = tbuf1[i + 3] << 8 | tbuf1[i + 2];
            }
        }

        if (hitPoints > 8)
        {
            for (k = 0, i = 0; (i < 4 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf[i + 1] << 8 | tbuf[i + 0];
                pLoc[k].y = tbuf[i + 3] << 8 | tbuf[i + 2];
            }

            for (k = 4, i = 0; (i < 4 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf1[i + 1] << 8 | tbuf1[i + 0];
                pLoc[k].y = tbuf1[i + 3] << 8 | tbuf1[i + 2];
            }

            for (k = 8, i = 0; (i < 2 * 8) && (k < num); k++, i += 8)
            {
                pLoc[k].x = tbuf2[i + 1] << 8 | tbuf2[i + 0];
                pLoc[k].y = tbuf2[i + 3] << 8 | tbuf2[i + 2];
            }
        }



        retVal = hitPoints;

    } while (0);

    ss[0] = 0;
    writeGT9271TouchRegister(0x814e, ss, 1);
    delay(2);
    return retVal;
}


uint32_t dist(const TouchLocation& loc)
{
    uint32_t retVal = 0;

    uint32_t x = loc.x;
    uint32_t y = loc.y;

    retVal = x * x + y * y;

    return retVal;
}


uint32_t dist(const TouchLocation& loc1, const TouchLocation& loc2)
{
    uint32_t retVal = 0;

    uint32_t x = loc1.x - loc2.x;
    uint32_t y = loc1.y - loc2.y;

    retVal = sqrt(x * x + y * y);

    return retVal;
}


bool sameLoc(const TouchLocation& loc, const TouchLocation& loc2)
{
    return dist(loc, loc2) < 50;
}



    /*
    int buf[798];
    int x, x2;
    int y, y2;
    int r;

    static uint16_t w = 800;
    static uint16_t h = 480;

    float xScale = 1024.0F/w;
    float yScale = 1024.0F/h;
    */
    /* Wait around for touch events */

  /*  pinMode     (GT9271_INT, INPUT);
    uint8_t st=digitalRead(GT9271_INT);
    if(!st)   //Hardware touch interrupt
    */

    //uint8_t  ss[1];
    //readGT9271TouchAddr( 0x814e, ss, 1);
    //uint8_t status=ss[0];
    //if ((status & 0x80) != 0)  // touch status   Software touch interrupt 
    //uint8_t count = readGT9271TouchLocation( touchLocations, 10 );
    //lastTouch = touchLocations[0];
    //if(touchLocations[i].x>=3&&touchLocations[i].x<=40&&touchLocations[i].y>=450&&touchLocations[i].y<=480 )flag=0;
    //myGLCD.fillCircle(800-touchLocations[i].x,480-touchLocations[i].y, 4); 
