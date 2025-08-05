

#include "CPixelLED.h"


CPixelLED::CPixelLED(const unsigned char nDataPin)
{
	_nDataPin = nDataPin;

	_nBrightness = RGB_BRIGHTNESS;

	_nBreatheBrightness = MIN_DIM_BRIGHTNESS;
	_nBreatheDimming = HOLD_DIM_FACTOR;

	_nUpperPixel = NUM_OF_PIXELS;
	_nPastProgressData = 0UL;
	_nPastFirstDigit = 0UL;
	_nPastSecondDigit = 0UL;

	_bPalettChanged = false;
	_bBlinkState = false;
	_bProgressColorChanged = false;

	_LedMode = CPixelLED::STATIC_MODE;
	_rgbFgColor = RGB_BLANK;
	_rgbBgColor = RGB_BLANK;
	_lShowInterval = 0UL;
	_lLastShowTime = 0UL;

	pinMode(_nDataPin, OUTPUT);
	digitalWrite(_nDataPin, LOW);
}

void CPixelLED::init(void)
{
	FastLED.addLeds<NEOPIXEL, NEOPIXEL_DIN>(this->_ledPalett, NUM_OF_PIXELS);

	FastLED.clear(true);
	FastLED.show();

	for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++)

		_ledPalett[nPixel] = INIT_PIXEL_COLOR1;

	for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++)

		_ledPalettSwap[nPixel] = INIT_PIXEL_COLOR2;

	FastLED.setBrightness(INIT_PIXEL_BRIGHTNESS);
	FastLED.show();
}

void CPixelLED::drawPixels(const CRGB rgbColor, const unsigned char nBrightness)
{
	_nBrightness = nBrightness;

	for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++)

		_ledPalett[nPixel] = rgbColor;

	FastLED.setBrightness(_nBrightness);
	FastLED.show();
}

void CPixelLED::drawPixelPos(const unsigned char nPixel, const CRGB rgbColor, const unsigned char nBrightness)
{
	unsigned char nPixelPos = constrain(nPixel, FIRST_PIXEL, LAST_PIXEL);

	_nBrightness = nBrightness;

	_ledPalett[nPixelPos] = rgbColor;

	FastLED.setBrightness(_nBrightness);
	FastLED.show();
}

void CPixelLED::setBrightness(const unsigned char nBrightness)
{
	_nBrightness = nBrightness;

	FastLED.setBrightness(_nBrightness);
	FastLED.show();
}

void CPixelLED::setForegroundColor(const CRGB rgbColor, const bool bUpdate)
{
	_rgbFgColor = rgbColor;

	if (bUpdate == UPDATE_LED_PALETT) {

		for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

			_ledPalett[nPixel] = _rgbFgColor;

		}

		_lLastShowTime = 0UL;

	}
	else {

		_bProgressColorChanged = true;				// bug fixed : version 1.1r

	}
}

void CPixelLED::setBackgroundColor(const CRGB rgbColor, const bool bUpdate)
{
	_rgbBgColor = rgbColor;

	if (bUpdate == UPDATE_LED_PALETT) {

		for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

			_ledPalett[nPixel] = _rgbBgColor;

		}

		_lLastShowTime = 0UL;

	}
}

void CPixelLED::setPalettMode(const enum LedMode nLedMode)
{
	_LedMode = nLedMode;

	_nBrightness = RGB_BRIGHTNESS;

	switch (_LedMode) {

		case CPixelLED::VARIOUS_PROGRESS_MODE:
		case CPixelLED::LPUSHING_PROGRESS_MODE:

			_lShowInterval = 0UL;
			_lLastShowTime = 0UL;

			_nPastProgressData = 0UL;
			_nPastFirstDigit = 0UL;
			_nPastSecondDigit = 0UL;
			_nUpperPixel = NUM_OF_PIXELS;

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalett[nPixel] = _rgbBgColor;

			}

		break;

		case CPixelLED::BREATHE_MODE:

			_lLastShowTime = 0UL;

			_nBreatheBrightness = MIN_DIM_BRIGHTNESS;
			_nBreatheDimming = HOLD_DIM_FACTOR;

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalett[nPixel] = _rgbFgColor;

			}

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalettSwap[nPixel] = _rgbBgColor;

			}

		break;

		case CPixelLED::BLINK_MODE:

			_lLastShowTime = 0UL;

		case CPixelLED::HOLDING_ON_BLINK_MODE:

			_bBlinkState = false;

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalett[nPixel] = _rgbFgColor;

			}

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalettSwap[nPixel] = _rgbBgColor;

			}

			if (_LedMode == CPixelLED::HOLDING_ON_BLINK_MODE) 

				return;

		break;

		case CPixelLED::STATIC_MODE:
		default:

			_lLastShowTime = 0UL;

		case CPixelLED::HOLDING_ON_STATIC_MODE:

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalett[nPixel] = _rgbFgColor;

			}

			for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

				_ledPalettSwap[nPixel] = _rgbBgColor;

			}

			if (_LedMode == CPixelLED::HOLDING_ON_STATIC_MODE)

				return;

		break;

	}

	FastLED.setBrightness(_nBrightness);
	FastLED.show();
}

void CPixelLED::drawPalettMode(void)
{
	switch (_LedMode) {

		case CPixelLED::VARIOUS_PROGRESS_MODE:

			if (_bPalettChanged == true) {

				_bPalettChanged = false;

				FastLED.show();

			}

		break;

		case CPixelLED::LPUSHING_PROGRESS_MODE:

			drawProgressPalettOnLongPush(ON_PUSHING_DRAW_INTERVAL);

		break;

		case CPixelLED::BREATHE_MODE:

			if (millis() - _lLastShowTime >= _lShowInterval) {

				_lLastShowTime = millis();

				makeBreathingPalett();

				FastLED.show();

			}

		break;

		case CPixelLED::BLINK_MODE:

			if (millis() - _lLastShowTime >= _lShowInterval) {

				_lLastShowTime = millis();

				makeBlinkPalett();

				FastLED.show();

			}

		break;

		case CPixelLED::HOLDING_ON_STATIC_MODE:
		case CPixelLED::HOLDING_ON_BLINK_MODE:

			if (millis() - _lLastShowTime >= _lShowInterval) {

				_lLastShowTime = 0UL;

				if (_LedMode == CPixelLED::HOLDING_ON_STATIC_MODE) {

					_lShowInterval = ON_STATIC_DRAW_INTERVAL;
					_LedMode = CPixelLED::STATIC_MODE;

				}
				else {

					_lShowInterval = ON_BLINK_DRAW_INTERVAL;
					_LedMode = CPixelLED::BLINK_MODE;

				}

			}

		break;

		case CPixelLED::STATIC_MODE:
		default:

			if (millis() - _lLastShowTime >= _lShowInterval) {

				_lLastShowTime = millis();

				FastLED.show();

			}
			else {

				if (millis() < _lShowInterval && _lLastShowTime == 0ul) {

					_lLastShowTime = millis();

					FastLED.show();

				}

			}

		break;

	}
}

void CPixelLED::drawProgressPalettOnLongPush(const unsigned int nDrawInterval)
{
	int nPixel = NUM_OF_PIXELS;

	// noInterrupts();

	while (--nPixel >= FIRST_PIXEL) {

		unsigned long ulLastShow;

		_ledPalett[nPixel] = _rgbFgColor;

		FastLED.show();

		ulLastShow = millis();

		while (millis() - ulLastShow < nDrawInterval);

	}

	// interrupts();
}

void CPixelLED::makeVariousProgressPalett(const unsigned int nCurrProgressData, const unsigned int nMaxProgressData)
{
	if ((_bProgressColorChanged || _nPastProgressData != nCurrProgressData) && nCurrProgressData) {

		unsigned int nPixelCount = (unsigned int)(nCurrProgressData / (nMaxProgressData / float(NUM_OF_PIXELS)));

		int nLowerPixel = constrain(NUM_OF_PIXELS - nPixelCount, FIRST_PIXEL, LAST_PIXEL);

		for (int nPixel = nLowerPixel; nPixel <= LAST_PIXEL; nPixel++) {

			_ledPalett[nPixel] = _rgbFgColor;

		}

		if (nLowerPixel > _nUpperPixel) {

			for (int nPixel = nLowerPixel-1; nPixel >= _nUpperPixel; nPixel--) {

				_ledPalett[nPixel] = _rgbBgColor;

			}

		}

		_nUpperPixel = nLowerPixel;

		_bProgressColorChanged = false;

		_bPalettChanged = true;

	}

	_nPastProgressData = nCurrProgressData;
}

void CPixelLED::makeFixedProgressPalett(const unsigned int nCurrProgressData, const unsigned int nMaxProgressData)
{
	if ((_bProgressColorChanged || _nPastProgressData != nCurrProgressData) && nCurrProgressData) {

		unsigned int nPixelCount = (unsigned int)(nCurrProgressData / (nMaxProgressData / float(NUM_OF_PIXELS)));

		int nLowerPixel = constrain(LAST_PIXEL - nPixelCount, FIRST_PIXEL, LAST_PIXEL);

		if (nLowerPixel < _nUpperPixel) {

			for (int nPixel = nLowerPixel; nPixel < _nUpperPixel; nPixel++)

				_ledPalett[nPixel] = _rgbFgColor;

			_nUpperPixel = nLowerPixel;

			_bProgressColorChanged = false;

			_bPalettChanged = true;

		}
		else {

			if (_nUpperPixel < nLowerPixel) 
				
				_nUpperPixel = NUM_OF_PIXELS;

		}

	}

	_nPastProgressData = nCurrProgressData;	
}

void CPixelLED::makeTimelapsePalett(const unsigned int nFirstDigitData, const unsigned int nSecondDigitData)
{
	if (_nPastFirstDigit != nFirstDigitData) {

		if (_nPastSecondDigit != nSecondDigitData) {

			if (nSecondDigitData == 1) {

				for (int nPixel = FIRST_PIXEL; nPixel < SECOND_DIGIT_END_PIXEL; nPixel++)

					_ledPalett[nPixel] = _rgbBgColor;

				_ledPalett[SECOND_DIGIT_END_PIXEL] = CRGB::DarkRed;

			} 
			else {

				int nLowerPixel = constrain((nSecondDigitData ? SECOND_DIGIT_END_PIXEL - (nSecondDigitData-1) : FIRST_PIXEL) , FIRST_PIXEL, SECOND_DIGIT_END_PIXEL);

				_ledPalett[nLowerPixel] = CRGB::DarkRed;

			}

		}

		if (!nFirstDigitData) {

			for (int nPixel = FIRST_DIGIT_START_PIXEL; nPixel < NUM_OF_PIXELS; nPixel++)

				_ledPalett[nPixel] = _rgbBgColor;

		}
		else {

			int nLowerPixel = constrain(LAST_PIXEL - (nFirstDigitData-1), FIRST_DIGIT_START_PIXEL, LAST_PIXEL);

			_ledPalett[nLowerPixel] = _rgbFgColor;

		}

		_bProgressColorChanged = false;

		_bPalettChanged = true;

	}

	_nPastFirstDigit = nFirstDigitData;
	_nPastSecondDigit = nSecondDigitData;
}

void CPixelLED::makeBreathingPalett(void)
{
	_nBreatheBrightness += _nBreatheDimming;

	FastLED.setBrightness(_nBreatheBrightness);

	switch (_nBreatheBrightness) {

		case MIN_DIM_BRIGHTNESS :

			if (_nBreatheDimming == DEC_DIM_FACTOR) {

				_nBreatheDimming = HOLD_DIM_FACTOR;

				_lShowInterval = ON_BREATH_MIN_INTERVAL;

			}
			else {

				_nBreatheDimming = INC_DIM_FACTOR;

				_lShowInterval = ON_BREATH_INC_INTERVAL;

			}

		break;

		case MAX_DIM_BRIGHTNESS :

			_nBreatheDimming = DEC_DIM_FACTOR;

			_lShowInterval = ON_BREATH_MAX_INTERVAL;

		break;

		default :

			_lShowInterval = _nBreatheDimming == INC_DIM_FACTOR ? ON_BREATH_INC_INTERVAL : ON_BREATH_DEC_INTERVAL;

		break;

	}
}

void CPixelLED::makeBlinkPalett(void)
{
	CRGB rgbColor = RGB_BLANK;

	// swap_Palett((_bBlinkState = _bBlinkState ? false : true));
	if (_bBlinkState) {

		_bBlinkState = false;
		rgbColor = _rgbBgColor;

	}
	else {

		_bBlinkState = true;
		rgbColor = _rgbFgColor;

	}

	for (int nPixel = 0; nPixel < NUM_OF_PIXELS; nPixel++) {

		_ledPalett[nPixel] = rgbColor;

	}
}

// void CPixelLED::swap_Palett(const bool bRecovery)
// {
// 	CRGB* pDestPalett = bRecovery ? _ledPalett : _ledPalettSwap;
// 	CRGB* pSrcPalett = bRecovery ? _ledPalettSwap : _ledPalett;

// 	// noInterrutps();

// 	memcpy(pDestPalett, pSrcPalett, sizeof(CRGB)*NUM_OF_PIXELS);

// 	// interrupts();
// }

void CPixelLED::show(void)
{
	// noInterrutps();

	FastLED.show();

	// interrupts();
}

void CPixelLED::clear(const bool bIsWriteColor)
{
	// noInterrutps();

	FastLED.clear(bIsWriteColor);

	// interrupts();
}

void CPixelLED::set_Brightness(const unsigned char nBrightness)
{
	// noInterrutps();

	FastLED.setBrightness(nBrightness);

	// interrupts();
}

