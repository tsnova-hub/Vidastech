#ifndef _MES_PIXEL_LED_COLOR_H_
#define _MES_PIXEL_LED_COLOR_H_


#include "MES_Config.h"

#include "./PixelLed/CPixelLED.h"


#define RGB_PROGRAM_ONE									(CRGB::Red)					//0xFF0000
#define RGB_PROGRAM_TWO	 								(CRGB::Blue)				//0x0000FF
#define RGB_MANUAL_ONE									(CRGB::Green)				//0x00FF00

#define RGB_SETUP_ENTRY_BLINK_BG						(CRGB::DimGrey)				//0x696969

// #define RGB_BREW_PROGRAM								(CRGB::IndianRed)			//0xCD5C5C
#define RGB_BREW_INJECTION								(CRGB::Sienna)				//0xA0522D
#define RGB_BREW_EXTRACTION								(CRGB::Crimson)				//0xDC143C
#define RGB_BREW_PROGRESS_BG							(0x0E0C1C)					//DarkSlateBlue x 0.2

#define RGB_BREW_SECOND_DIGIT_FG						(CRGB::DarkRed)				//0x8B0000
#define RGB_BREW_FIRST_DIGIT_BG							(RGB_BREW_PROGRESS_BG)
#define RGB_BREW_SECOND_DIGIT_BG						(RGB_BREW_PROGRESS_BG)

#define RGB_CLEANING_MACHINE							(CRGB::DeepSkyBlue) 		//0x00BFFF
#define RGB_CLEANING_PROGRESS_BG						(0x0E0C1C)					//DarkSlateBlue x 0.2

#define RGB_AIR_VENTILATION_STAGE_FG					(CRGB::DeepSkyBlue)
#define RGB_AIR_VENTILATION_STAGE_BG 					(CRGB::DimGrey)				//0x696969

#define RGB_ACTIVATION_PROGRESS_FG						(CRGB::DeepSkyBlue)			//0x00BFFF
#define RGB_ACTIVATION_PROGRESS_BG						(CRGB::RoyalBlue)			//0x4169E1

#define RGB_SETUP_EXIT_PROGRESS_FG						(CRGB::GreenYellow)			//0xADFF2F
#define RGB_LONG_PUSH_PROGRESS_FG						(CRGB::GreenYellow)
#define RGB_LONG_PUSH_PROGRESS_BG						(0x322B61)					//DarkSlateBlue x 0.7

#define RGB_VLONG_PUSH_FG								(CRGB::MediumTurquoise)		//0x48D1CC

#define RGB_BREATHING_SLEEP								(CRGB::DimGrey)				//0x696969

#define RGB_SYSTEM_ERROR_BLINK_FG						(CRGB::Yellow)				//0xFFFF00
#define RGB_SYSTEM_ERROR_BLINK_BG						(0x151515)					//DimGrey x 0.2


#endif //_MES_PIXEL_LED_COLOR_H_