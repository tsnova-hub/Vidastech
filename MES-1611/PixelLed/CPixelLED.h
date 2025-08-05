#ifndef _CPIXEL_LED_H_
#define _CPIXEL_LED_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <FastLED.h>


// #include "../MES_Config.h"
#include "../MES_Hardware_Map.h"


using namespace std;


#define NUM_OF_PIXELS									(16)
#define FIRST_PIXEL										(0)
#define LAST_PIXEL										(NUM_OF_PIXELS-1)


#define RGB_BRIGHTNESS									(64)						// tune brightness (dedicated to RGBLEDStick16x5050), (96)

#define RGB_INIT_BRIGHTNESS								(RGB_BRIGHTNESS)
#define RGB_INIT_BACKGROUND								(CRGB::RoyalBlue)			//0x4169E1
#define RGB_BLANK 										(CRGB::Black)				//0x000000

#define INIT_PIXEL_BRIGHTNESS							(RGB_BRIGHTNESS)
#define INIT_PIXEL_COLOR1								(CRGB::MediumSlateBlue)
#define INIT_PIXEL_COLOR2								(CRGB::Black)

#define MAX_DIM_BRIGHTNESS								(RGB_BRIGHTNESS)
#define MIN_DIM_BRIGHTNESS								(24)
#define HOLD_DIM_FACTOR									(0)
#define DEC_DIM_FACTOR									(-1)
#define INC_DIM_FACTOR									(1)

#define LED_TIMELAPSE_FIRST_DIGIT_NUM					(10)
#define LED_TIMELAPSE_SECOND_DIGIT_NUM					(6)
#define FIRST_DIGIT_START_PIXEL							(LED_TIMELAPSE_SECOND_DIGIT_NUM+1)
#define SECOND_DIGIT_END_PIXEL							(LED_TIMELAPSE_SECOND_DIGIT_NUM-1)


#define ON_STATIC_DRAW_INTERVAL							(30*1000UL)					// 30 sec
#define ON_BLINK_DRAW_INTERVAL 							500UL						// 500 millisec

#define ON_PUSHING_DRAW_INTERVAL						64UL 						// 64 millisec
#define ON_PUSHING_HOLD_INTERVAL						1250UL						// 1250 millisec

#define ON_TIMELAPSE_HOLD_INTERVAL						1500UL						// < 2000 millisec

#define ON_BREATH_INC_INTERVAL							20UL 						// 20 millisec
#define ON_BREATH_DEC_INTERVAL							10UL 						// 10 millisec
#define ON_BREATH_MIN_INTERVAL							2000UL 						// 2 sec
#define ON_BREATH_MAX_INTERVAL							500UL						// 500 millisec


#define SHOW_IMMEDIATELY								true
#define SHOW_LATER										false

#define UPDATE_LED_PALETT								true
#define NOT_UPDATE_LED_PALETT							false


class CPixelLED
{
public:
	enum LedMode {STATIC_MODE=0, BLINK_MODE, LPUSHING_PROGRESS_MODE, VARIOUS_PROGRESS_MODE, BREATHE_MODE, HOLDING_ON_STATIC_MODE, HOLDING_ON_BLINK_MODE};

	CRGB _ledPalett[NUM_OF_PIXELS];
	CRGB _ledPalettSwap[NUM_OF_PIXELS];

	CPixelLED(const unsigned char nDataPin);

	void init(void);

	void setBrightness(const unsigned char nBrightness = RGB_BRIGHTNESS);
	void setForegroundColor(const CRGB rgbColor, const bool bUpdate);
	void setBackgroundColor(const CRGB rgbColor, const bool bUpdate);
	void setInterval(const unsigned long lInterval) { _lShowInterval = lInterval; }
	void setLasttime(const unsigned long lLastTime) { _lLastShowTime = lLastTime; }
	void setPalettMode(const enum LedMode nLedMode);
	// void setProgressColorChanged(const bool bChanged) { _bProgressColorChanged = bChanged; }

	void drawPixels(const CRGB rgbColor, const unsigned char nBrightness = RGB_BRIGHTNESS);
	void drawPixelPos(const unsigned char nPixel, const CRGB rgbColor, const unsigned char nBrightness = RGB_BRIGHTNESS);

	void drawPalettMode(void);
	void drawProgressPalettOnLongPush(const unsigned int nDrawInterval);

	void makeVariousProgressPalett(const unsigned int nCurrProgressData, const unsigned int nMaxProgressData);
	void makeFixedProgressPalett(const unsigned int nCurrProgressData, const unsigned int nMaxProgressData);
	void makeTimelapsePalett(const unsigned int nFirstDigitData, const unsigned int nSecondDigitData);
	void makeBreathingPalett(void);
	void makeBlinkPalett(void);

	unsigned char getBrightness(void) const { return _nBrightness; }
	enum LedMode getPalettMode(void) const { return _LedMode; }
	unsigned long getShowInterval(void) const { return _lShowInterval; }
	unsigned long getShowLasttime(void) const { return _lLastShowTime; }

private:
	enum LedMode _LedMode;

	unsigned char _nDataPin;

	CRGB _rgbFgColor;
	CRGB _rgbBgColor;
	unsigned char _nBrightness;

	unsigned char _nBreatheBrightness;
	char _nBreatheDimming;

	int _nUpperPixel;
	unsigned int _nPastProgressData;
	unsigned int _nPastFirstDigit;
	unsigned int _nPastSecondDigit;

	bool _bPalettChanged;
	bool _bBlinkState;
	bool _bProgressColorChanged;
	
	unsigned long _lShowInterval;
	unsigned long _lLastShowTime;

	void show(void);
	void clear(const bool bIsWriteColor = false);
	void set_Brightness(const unsigned char nBrightness);

	// void swap_Palett(const bool bRecovery);
};


#endif //_CPIXEL_LED_H_

