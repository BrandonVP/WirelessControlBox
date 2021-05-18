#pragma once
#include <avr/pgmspace.h>
// Robotic Arm bitmap
// 5 columns * 8 bits = 40 x 40 rows
const PROGMEM unsigned char robotarm[] =
{
  0x00,0x00,0xC0,0x01,0x00,
  0x00,0x00,0xF0,0x07,0x00,
  0x00,0x00,0x30,0x06,0x00,
  0x00,0x00,0xDA,0x0D,0x00,
  0x00,0x00,0xDB,0x0D,0x00,
  0x00,0xC0,0x37,0x06,0x00,
  0x00,0xF0,0xF7,0x17,0x00,
  0x00,0xFC,0xCF,0x19,0x00,
  0x00,0xFE,0x03,0x3E,0x00,
  0x00,0xFF,0x01,0x7F,0x00,
  0xFC,0xFE,0x00,0xFE,0x00,
  0xFE,0x7D,0x00,0xCE,0x01,
  0x86,0x3D,0x00,0xB6,0x01,
  0x7B,0x1B,0x00,0xB6,0x01,
  0x7B,0x03,0x00,0xCE,0x07,
  0x7B,0x03,0x00,0xFC,0x0F,
  0x86,0x0D,0x00,0x70,0x1E,
  0xFE,0x1D,0x00,0x38,0x1C,
  0xFC,0x3E,0x00,0x38,0x0C,
  0x00,0x7F,0x00,0x78,0x0C,
  0xC0,0xFF,0x00,0xF0,0x01,
  0x80,0x7F,0x00,0xC0,0x01,
  0x00,0xBF,0x1F,0x00,0x00,
  0x00,0xDE,0x3F,0x00,0x00,
  0x00,0xEC,0x70,0x00,0x00,
  0x00,0x60,0xEF,0x00,0x00,
  0x00,0xB0,0xDF,0x00,0x00,
  0x00,0xB0,0xDF,0x00,0x00,
  0x00,0xB0,0xDF,0x00,0x00,
  0x00,0x70,0xEF,0x00,0x00,
  0x00,0xE0,0x70,0x00,0x00,
  0x00,0xC0,0x3F,0x00,0x00,
  0x00,0xB0,0xDF,0x00,0x00,
  0x00,0x78,0xE0,0x01,0x00,
  0x00,0xFC,0xFF,0x03,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0xC0,0xFF,0xFF,0x3F,0x00,
  0xC0,0xFF,0xFF,0x3F,0x00,
  0xC0,0xFF,0xFF,0x3F,0x00,
  0x00,0x00,0x00,0x00,0x00 
};