

#include "CTextLCD.h"


// User-defined Fonts
unsigned char schar_deselected[8] = {
	B11000,
	B10100,
	B10010,
	B10001,
	B10010,
	B10100,
	B11000,
};

unsigned char schar_selected[8] = {
	B11000,
	B11100,
	B11110,
	B11111,
	B11110,
	B11100,
	B11000,
};


CTextLCD::CTextLCD(const unsigned char nBLlsb, const unsigned char nBLmsb, const enum Backlight nBLcolor, const unsigned char nI2CAddr)
: LiquidTWI(nI2CAddr)
{
	_nMaxPos = NUM_COLUMNS;
	_nMaxLine = NUM_LINES;
	
	_bIsBacklightOn = true;
	_bIsCursorOn = false;
	_bIsBlinkOn = false;

	_nBacklightLSB = nBLlsb;
	_nBacklightMSB = nBLmsb;

	pinMode(_nBacklightLSB, OUTPUT);
	pinMode(_nBacklightMSB, OUTPUT);

	setBacklightColor(nBLcolor);
}

void CTextLCD::init(const unsigned char nMaxPos, const unsigned char nMaxLine)
{
	_nMaxPos = nMaxPos;
	_nMaxLine = nMaxLine;

	this->begin(_nMaxPos, _nMaxLine);

	this->createChar(SCHAR_DESELECTED, schar_deselected);
	this->createChar(SCHAR_SELECTED, schar_selected);

	this->setBacklight(true);
	// this->noCursor();
	// this->noBlink();
	this->clear();

	this->noDisplay();

	// I2C Clock FREQ = 200 [KHz]
	bitSet(TWSR, TWPS0);
	TWBR = (((F_CPU / (12500UL * 16UL)) - 16) / 2) / 4;
	
	this->display();
}

void CTextLCD::setBacklightMode(const bool bBacklight)
{
	_bIsBacklightOn = bBacklight;

	this->setBacklight(_bIsBacklightOn);
}

void CTextLCD::setBacklightColor(const enum Backlight nColor)
{
	digitalWrite(_nBacklightLSB, nColor & 1);
	digitalWrite(_nBacklightMSB, nColor >> 1);
}

void CTextLCD::setCursorMode(const bool bCursor)
{
	_bIsCursorOn = bCursor;

	if (_bIsCursorOn) {

		this->cursor();

	}
	else {

		this->noCursor();

	}
}

void CTextLCD::setBlinkMode(const bool bBlink)
{
	_bIsBlinkOn = bBlink;

	if (_bIsBlinkOn) {

		this->blink();

	}
	else {

		this->noBlink();

	}
}

void CTextLCD::homeCursor(void)
{
	this->home();
}

void CTextLCD::moveCursor(const unsigned char nPos, const unsigned char nLine)
{
	this->setCursor(nPos, nLine);
}

void CTextLCD::clear(void)
{
	LiquidTWI::clear();
}

void CTextLCD::writePos(const unsigned char nPos, const unsigned char nLine, const unsigned char nValue)
{
	if (nPos < _nMaxPos && nLine < _nMaxLine) {

		this->setCursor(nPos, nLine);
		this->write(nValue);

	}
}

void CTextLCD::printPos(const unsigned char nPos, const unsigned char nLine, const char* pString)
{
	char* strBuf = new char [_nMaxPos+1];
	
	int nStrLen = int(strlen(pString));
	
	if (nPos+nStrLen > _nMaxPos){
		nStrLen = _nMaxPos - nPos;
	}
	memcpy(strBuf, pString, nStrLen);

	// appending a terminating null character after the content.
	strBuf[nStrLen] = 0;//'\0'

	this->setCursor(nPos, nLine);
	this->print(strBuf);

	delete [] strBuf;
}

void CTextLCD::fillBlank(const unsigned char nPos, const unsigned char nLine, unsigned char nLen)
{
	char* strBlank = new char [_nMaxPos+1];

	if (nPos+nLen > _nMaxPos) {
		nLen = _nMaxPos - nPos;
	}
	memset(strBlank, ' ', sizeof(char)*nLen);

	// appending a terminating null character after the content.
	strBlank[nLen+1] = 0;//'\0'
	
	this->setCursor(nPos, nLine);
	this->print(strBlank);

	delete [] strBlank;
}

