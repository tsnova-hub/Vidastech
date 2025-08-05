#ifndef _CBUTTON_H_
#define _CBUTTON_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


// #include "../MES_Config.h"


using namespace std;


// #if defined(_USE_APPLICATION_DEBUG_MSG_)
	// #define _TEST_CBUTTON_
// #endif


#define BTN_PUSHED									LOW
#define BTN_PULLED									HIGH


#define TIME_SCLICKED_BUTTON						250UL 	// > 250 millisec
#define TIME_DCLICKED_BUTTON						250UL	// < 250 millisec
// #define TIME_SCLICKED_BUTTON						150UL 	// > 150 millisec
// #define TIME_DCLICKED_BUTTON						150UL	// < 150 millisec
#define TIME_LPUSHING_BUTTON						2000UL	// 2000 millisec (2 sec)
#define TIME_LCLICKED_BUTTON						2000UL	// 2000 millisec (2 sec)
#define TIME_VCLICKED_BUTTON						4200UL	// 4000 millisec (4 sec)

#define BUTTON_NOT_PUSHED							-2
#define BUTTON_ON_PUSHING							-1
#define BUTTON_ON_LPUSHING							0
#define BUTTON_PUSHED_SHORT							1
#define BUTTON_PUSHED_LONG							2
#define BUTTON_PUSHED_VLONG							3
#define BUTTON_PUSHED_DCLK							4


#define SHORT_PUSHED_MASK							0b0001
#define LONG_PUSHED_MASK							0b0010
#define VLONG_PUSHED_MASK							0b0100
#define DOUBLE_PUSHED_MASK							0b1000

#define INIT_BUTTON_STATE							(SHORT_PUSHED_MASK)
#define LONG_VLONG_PUSHED_MASK						(LONG_PUSHED_MASK | VLONG_PUSHED_MASK)
#define SHORT_LONG_VLONG_PUSHED_MASK				(SHORT_PUSHED_MASK | LONG_PUSHED_MASK | VLONG_PUSHED_MASK)


class CButton
{
public:
	CButton(const unsigned char nBTNpin);

	void init(void);

	void getButtonState(void);

	unsigned char getButton(void) const;
	void clearButton(void);
	bool isButtonEventOccured(void);

	void disableButton(void) { _bDisableButton = true; }
	void enableButton(void) { _bDisableButton = false; }
	bool isButtonDisabled(void) const { return _bDisableButton; }

private:
	unsigned char _nBTNpin;

	volatile int _nPastButton;

	volatile unsigned char _nButton;
	volatile bool _bButtonEvent;

	bool _bDisableButton;

	volatile unsigned char _u8ButtonState;

	bool _bGuess2DClick;
	bool _bGuess2LClick;

	unsigned long _ulTime_last_pushed;
	unsigned long _ulTime_last_pulled;


	void update_Button(const unsigned char nButton) { _nButton = nButton; }
	void set_ButtonEvent(void) { _bButtonEvent = true; }
	void clear_ButtonEvent(void) { _bButtonEvent = false; }
	bool is_ButtonEventSet(void) const { return _bButtonEvent; }
};


#endif //_CBUTTON_H_

