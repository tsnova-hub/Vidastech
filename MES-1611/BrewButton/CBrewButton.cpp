

#include "CBrewButton.h"


CBrewButton::CBrewButton(const unsigned char nBTNpin, const unsigned char nLEDpin)
: CButton(nBTNpin)
{
	_nLEDpin = nLEDpin;

	pinMode(_nLEDpin, OUTPUT);
	digitalWrite(_nLEDpin, LOW);
}

void CBrewButton::initButton(void)
{
	_bButtonEventMask = true;
	_ulMaskingStartTime = 0UL;

	CButton::init();
}

void CBrewButton::setButtonLEDState(const bool bTurnOn)
{
	digitalWrite(_nLEDpin, bTurnOn);
}

bool CBrewButton::isButtonMaskedEventOccured(void)
{
	if (!_bButtonEventMask) {

		if (millis() - _ulMaskingStartTime < TIME_TO_MASK_FALSE_EVENT) {

			CButton::clearButton();

		}
		else

			_bButtonEventMask = true;

	}

	return _bButtonEventMask & CButton::isButtonEventOccured();
}

void CBrewButton::startMaskingEvent(void)
{
	_bButtonEventMask = false;
	_ulMaskingStartTime = millis();
}

