

#include "CButton.h"


CButton::CButton(const unsigned char nBTNpin)
{
	_nBTNpin = nBTNpin;

	pinMode(_nBTNpin, INPUT_PULLUP);
}

void CButton::init(void)
{
	_nButton = BUTTON_NOT_PUSHED;

	_bButtonEvent = false;

	_bDisableButton = false;


	_nPastButton = BTN_PULLED;

	_u8ButtonState = INIT_BUTTON_STATE;

	_bGuess2DClick = false;
	_bGuess2LClick = false;

	_ulTime_last_pushed = 0UL;
	_ulTime_last_pulled = 0UL;
}

//////////////////////////////////////////////////////////////////////////////////////////
//			PUSH-BUTTON SWITCH ACCESS(READ) FOR USER-INTERFACE
//
// 			Switch State Value
// 			0b0001 : Initial State
// 			0b0001 : bShortPushed = true
// 			0b0010 : bLongPushed = true
// 			0b0100 : bVLongPushed = true
// 			0b1000 : bDoublePushed = true
//

void CButton::getButtonState(void)
{
	if (isButtonDisabled() == true)

		return;


	int nCurrButton = digitalRead(_nBTNpin);


	if (is_ButtonEventSet() == true) {

		_nPastButton = nCurrButton;
		return;

	}

	if (nCurrButton != _nPastButton && nCurrButton == BTN_PUSHED)
	{

		_ulTime_last_pushed = millis();

		_bGuess2DClick = (_ulTime_last_pushed - _ulTime_last_pulled <= TIME_DCLICKED_BUTTON) ? true : false;

		_u8ButtonState = 0x00;

		update_Button(BUTTON_ON_PUSHING);
		clear_ButtonEvent();

	}
	else if (nCurrButton != _nPastButton && nCurrButton == BTN_PULLED)
	{

		_ulTime_last_pulled = millis();

		if (_bGuess2DClick && !(_u8ButtonState & LONG_VLONG_PUSHED_MASK)) {

			_u8ButtonState |= DOUBLE_PUSHED_MASK;

			update_Button(BUTTON_PUSHED_DCLK);
			set_ButtonEvent();

			#if defined(_TEST_CBUTTON_)
			cout << F("BTN : Double Pushed.") << endl;
			#endif

		}	
	
	}
	else {

		if (nCurrButton == BTN_PUSHED) {

			if (!_bGuess2DClick && !(_u8ButtonState & LONG_VLONG_PUSHED_MASK)) {

				if (millis() - _ulTime_last_pushed >= TIME_LPUSHING_BUTTON) {

					// if (_GET_SYSTEM_STATE != TAP_OPEN_STATE) {

						if (!_bGuess2LClick) {

							_bGuess2LClick = true;

							update_Button(BUTTON_ON_LPUSHING);
							set_ButtonEvent();

							#if defined(_TEST_CBUTTON_)
							cout << F("BTN : On Pushing Long.") << endl;
							#endif

						}

					// }
					// else

						// _ulTime_last_pushed = millis();

				}

			}
			else {

				if (is_ButtonEventSet() == false)

					update_Button(BUTTON_ON_PUSHING);

			}

		}
		else {

			if (!_bGuess2DClick && !(_u8ButtonState & SHORT_LONG_VLONG_PUSHED_MASK)) {

				if (!_bGuess2LClick) {

					if (millis() - _ulTime_last_pulled > TIME_SCLICKED_BUTTON) {

						_u8ButtonState |= SHORT_PUSHED_MASK;

						update_Button(BUTTON_PUSHED_SHORT);
						set_ButtonEvent();

						#if defined(_TEST_CBUTTON_)
						cout << F("BTN : Short Pushed.") << endl;
						#endif

					}

				}
				else {

					_bGuess2LClick = false;

					unsigned long ulPushedTime = millis() - _ulTime_last_pushed;

					if (ulPushedTime >= TIME_LCLICKED_BUTTON && ulPushedTime < TIME_VCLICKED_BUTTON) {

						_u8ButtonState |= LONG_PUSHED_MASK;

						update_Button(BUTTON_PUSHED_LONG);

						#if defined(_TEST_CBUTTON_)
						cout << F("BTN : Long Pushed.") << endl;
						#endif

					}
					else /*if (ulPushedTime >= TIME_VCLICKED_BUTTON)*/ {

						_u8ButtonState |= VLONG_PUSHED_MASK;

						update_Button(BUTTON_PUSHED_VLONG);

						#if defined(_TEST_CBUTTON_)
						cout << F("BTN : VLong Pushed.") << endl;
						#endif

					}

					set_ButtonEvent();

				}

			}
			else {

				if (is_ButtonEventSet() == false)

					update_Button(BUTTON_NOT_PUSHED);

			}

		}

	}

	_nPastButton = nCurrButton;
}

unsigned char CButton::getButton(void) const
{
	unsigned char nButton;

	noInterrupts();

	nButton = _nButton;
	// _nButton = BUTTON_NOT_PUSHED;

	interrupts();

	return nButton;
}

void CButton::clearButton(void)
{
	noInterrupts();

	_bButtonEvent = false;
	_nButton = BUTTON_NOT_PUSHED;

	interrupts();
}

bool CButton::isButtonEventOccured(void)
{
	bool bButtonEvent;

	noInterrupts();

	bButtonEvent = _bButtonEvent;
	_bButtonEvent = false;

	interrupts();

	return bButtonEvent;
}

