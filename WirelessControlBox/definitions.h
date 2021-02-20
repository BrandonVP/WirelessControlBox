#pragma once
// Global LCD theme color variables
#define themeBackground 0xFFFFFF // White
#define menuBtnText 0xFFFFFF // White
#define menuBtnBorder 0x000000 // Black
#define menuBtnColor 0x87005f // Orange
#define menuBackground 0xB5B5B5 //Silver

// For the draw shape functions
#define LEFT 1
#define CENTER 2
#define RIGHT 3

// Arm 1 IDs
constexpr auto ARM1_RX = 0x0C1;
constexpr auto ARM1_M = 0x0A3;
constexpr auto ARM1_T = 0x0A2;
constexpr auto ARM1_B = 0x0A1;
constexpr auto ARM1_CONTROL = 0x0A0;
#define CHANNEL1 0

// Arm 2 IDs
constexpr auto ARM2_RX = 0x0C2;
constexpr auto ARM2_M = 0x0B3;
constexpr auto ARM2_T = 0x0B2;
constexpr auto ARM2_B = 0x0B1;
constexpr auto ARM2_CONTROL = 0x0B0;
#define CHANNEL2 1

// Bitmap
#define BUFFPIXEL 20

// Used to prevent physical button doubletap
#define BUTTON_DELAY 200

// Declaring a method because the compiler was crying about not finding it...
void pageControl(int page, bool value);
