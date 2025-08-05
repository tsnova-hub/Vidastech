#ifndef _CBREWBUTTON_H_
#define _CBREWBUTTON_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include "CButton.h"


#ifndef TIME_TO_MASK_FALSE_EVENT
	#define TIME_TO_MASK_FALSE_EVENT				(250UL) // for 250 msec since last closing solenoid valve event
#endif


class CBrewButton: public CButton
{
	friend bool operator == (CBrewButton& obj1, CBrewButton& obj2) { return &obj1 == &obj2; }

public:
	CBrewButton(const unsigned char nBTNpin, const unsigned char nLEDpin);

	void initButton(void);

	void setButtonLEDState(const bool bTurnOn = true);

	bool isButtonMaskedEventOccured(void);

	void startMaskingEvent(void);
	void stopMaskingEvent(void) { _bButtonEventMask = true; }

private:
	unsigned char _nLEDpin;

	bool _bButtonEventMask;
	unsigned long _ulMaskingStartTime;
};


#endif //_CBREWBUTTON_H_

