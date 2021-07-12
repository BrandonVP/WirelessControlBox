#pragma once
// Global LCD theme color variables
#define THEME_BACKGROUND 0xFFFFFF // White
#define MENU_BUTTON_TEXT 0xFFFFFF // White
#define MENU_BUTTON_BORDER 0x000000 // Black
#define MENU_BUTTON_COLOR 0x87005f // Orange
#define MENU_BACKGROUND 0xB5B5B5 //Silver

// For the draw shape functions
#define ALIGN_LEFT 1
#define ALIGN_CENTER 2
#define ALIGN_RIGHT 3

// Arm 1 IDs
constexpr auto ARM1_RX = 0x0C1;
constexpr auto ARM1_MANUAL = 0x0A3;
constexpr auto ARM1_UPPER = 0x0A2;
constexpr auto ARM1_LOWER = 0x0A1;
constexpr auto ARM1_CONTROL = 0x0A0;
constexpr auto CHANNEL_1 = 1;

// Arm 2 IDs
constexpr auto ARM2_RX = 0x0C2;
constexpr auto ARM2_MANUAL = 0x0B3;
constexpr auto ARM2_UPPER = 0x0B2;
constexpr auto ARM2_LOWER = 0x0B1;
constexpr auto ARM2_CONTROL = 0x0B0;
constexpr auto CHANNEL_2 = 2;

// Bitmap
#define PIXEL_BUFFER 20

#define DEG "deg"
#define REFRESH_RATE 400
