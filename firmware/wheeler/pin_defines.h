#pragma once

#define USE_HAT 0

#define ENCODER_CLK 16 //WHITE
#define ENCODER_DT  17 //GREEN

#define BRAKE 22
#define ACC 28
#define L_ANALOG_STICK_X 27
#define L_ANALOG_STICK_Y 26

#define B1 5    //A
#define B2 6    //B
#define B3 7    //X
#define B4 8    //Y
#define B5 9    //LB
#define B6 10   //RB
#define B7 11   //LT
#define B8 12   //RT
#define B9 13   //start
#define B10 14  //back
#define B11 15  //guide

#if USE_HAT
#define HAT_UP 22
#define HAT_DOWN 28
#define HAT_LEFT 27
#define HAT_RIGHT 26
#endif

#define ENCODER_RESET 21
#define DEBUG_PIN 3
#define RANGE_SELECT_PIN 4

#define RSTICK_SWAP 15

#define BOOTSEL_PIN 0
