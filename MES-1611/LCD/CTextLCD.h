#ifndef _CTEXT_LCD_H_
#define _CTEXT_LCD_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <LiquidTWI.h>
#include <Wire.h>


using namespace std;


#define NUM_COLUMNS									(20)
#define NUM_LINES									(4)

#define SCHAR_DESELECTED							(0)
#define SCHAR_SELECTED								(1)


class CTextLCD: public LiquidTWI
{
public:	
	enum Backlight {BLUE=0, GREEN, PURPLE, RED};

	CTextLCD(const unsigned char nBLlsb, const unsigned char nBLmsb, const enum Backlight nBLcolor = GREEN, const unsigned char nI2CAddr = 0);

	void init(const unsigned char nMaxPos = NUM_COLUMNS, const unsigned char nMaxLine = NUM_LINES);

	void setBacklightMode(const bool bBacklight);
	void setBacklightColor(const enum Backlight nColor);
	void setCursorMode(const bool bCursor);
	void setBlinkMode(const bool bBlink);

	void homeCursor(void);
	void moveCursor(const unsigned char nPos, const unsigned char nLine);

	void clear(void);
	void writePos(const unsigned char nPos, const unsigned char nLine, const unsigned char nValue);
	void printPos(const unsigned char nPos, const unsigned char nLine, const char* pString);
	void fillBlank(const unsigned char nPos, const unsigned char nLine, unsigned char nLen);

	bool isBacklightOn(void) const { return _bIsBacklightOn; }
	bool isCursorOn(void) const { return _bIsCursorOn; }
	bool isBlinkOn(void) const { return _bIsBlinkOn; }

private:
	unsigned char _nBacklightLSB;
	unsigned char _nBacklightMSB;

	unsigned char _nMaxPos;
	unsigned char _nMaxLine;

	bool _bIsBacklightOn;
	bool _bIsCursorOn;
	bool _bIsBlinkOn;
};


#endif //_CTEXT_LCD_H_

