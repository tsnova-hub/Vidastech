

#include "CRotaryEncoder.h"


CRotaryEncoder::CRotaryEncoder(const unsigned char nSTpin, const unsigned char nDIpin, const unsigned char nSWpin)
: StopWatch(StopWatch::MICROS)
{
	_nSTpin = nSTpin;
	_nDIpin = nDIpin;
	_nSWpin = nSWpin;

	// must pull-down with (<)100 Ohm at pin 'ST'
	pinMode(_nSTpin, INPUT /*INPUT_PULLUP*/);
	pinMode(_nDIpin, INPUT /*INPUT_PULLUP*/);
	pinMode(_nSWpin, INPUT /*INPUT_PULLUP*/);
	// DigitalWrite(_nSTpin, LOW);
	// DigitalWrite(_nDIpin, LOW);
	// DigitalWrite(_nSWpin, LOW);
}

void CRotaryEncoder::init(void)
{
	_nSwitch = SWITCH_NOT_PUSHED;
	_nDirection = ENCODER_NO_ROTATION;

	_bSwitchEvent = false;
	_bDirectionEvent = false;

	_bDisableRotaryEncoder = false;
	_bDisableDirectionEvent = false;

	memset(_bEventMask, true, sizeof(bool)*2);
	memset(_ulMaskingStartTime, 0UL, sizeof(unsigned long)*2);

	_nPastSwitch = SW_PULLED;

	_u8SwitchState = INIT_SWITCH_STATE;

	_bGuess2DClick = false;
	_bGuess2LClick = false;

	_ulTime_last_pushed = 0UL;
	_ulTime_last_pulled = 0UL;

	_nAccelerationFactor = 1;
}

void CRotaryEncoder::startACC(void)
{
#if defined(_ROTARY_ENCODER_ACCELERATION_)
	if (!StopWatch::isRunning()) {

		_nAccelerationFactor = 1;

		StopWatch::reset();
		StopWatch::start();

	}
#endif
}

void CRotaryEncoder::stopACC(void)
{
#if defined(_ROTARY_ENCODER_ACCELERATION_)
	if (StopWatch::isRunning()) {

		_nAccelerationFactor = 1;

		StopWatch::stop();

	}
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
//			ROTARY ENCODER / SWITCH ACCESS(READ) FOR USER-INTERFACE
//
// 			Switch State Value
// 			0b0001 : Initial State
// 			0b0001 : bShortPushed = true
// 			0b0010 : bLongPushed = true
// 			0b0100 : bVLongPushed = true
// 			0b1000 : bDoublePushed = true
//

void CRotaryEncoder::getRotaryEncoderSwitchState(void)
{
	if (isRotaryEncoderDisabled() == true)

		return;


	int nCurrSwitch = digitalRead(_nSWpin);


	if (is_SwitchEventSet() == true) {

		_nPastSwitch = nCurrSwitch;
		return;

	}

	if (nCurrSwitch != _nPastSwitch && nCurrSwitch == SW_PUSHED)
	{

		disableDirectionEvent();

		_ulTime_last_pushed = millis();

		_bGuess2DClick = (_ulTime_last_pushed - _ulTime_last_pulled <= TIME_DCLICKED_SWITCH) ? true : false;

		_u8SwitchState = 0x00;

		update_Switch(SWITCH_ON_PUSHING);
		clear_SwitchEvent();

	}
	else if (nCurrSwitch != _nPastSwitch && nCurrSwitch == SW_PULLED)
	{

		enableDirectionEvent();

		_ulTime_last_pulled = millis();

		if (_bGuess2DClick && !(_u8SwitchState & LONG_VLONG_PUSHED_MASK)) {

			_u8SwitchState |= DOUBLE_PUSHED_MASK;

			update_Switch(SWITCH_PUSHED_DCLK);
			set_SwitchEvent();

			#if defined(_TEST_CROTARYENCODER_)
			cout << "SW : Double Pushed." << endl;
			#endif

		}	
	
	}
	else {

		if (nCurrSwitch == SW_PUSHED) {

			if (!_bGuess2DClick && !(_u8SwitchState & LONG_VLONG_PUSHED_MASK)) {

				if (millis() - _ulTime_last_pushed >= TIME_LPUSHING_SWITCH) {

					// if (_GET_SYSTEM_STATE != TAP_OPEN_STATE) {

						if (!_bGuess2LClick) {

							_bGuess2LClick = true;

							update_Switch(SWITCH_ON_LPUSHING);
							set_SwitchEvent();

							#if defined(_TEST_CROTARYENCODER_)
							cout << F("SW : On Pushing Long.") << endl;
							#endif

						}

					// }
					// else

						// _ulTime_last_pushed = millis();

				}

			}
			else {

				if (is_SwitchEventSet() == false)

					update_Switch(SWITCH_ON_PUSHING);

			}

		}
		else {

			if (!_bGuess2DClick && !(_u8SwitchState & SHORT_LONG_VLONG_PUSHED_MASK)) {

				if (!_bGuess2LClick) {

					if (millis() - _ulTime_last_pulled > TIME_SCLICKED_SWITCH) {

						_u8SwitchState |= SHORT_PUSHED_MASK;

						update_Switch(SWITCH_PUSHED_SHORT);
						set_SwitchEvent();

						#if defined(_TEST_CROTARYENCODER_)
						cout << F("SW : Short Pushed.") << endl;
						#endif

					}

				}
				else {

					_bGuess2LClick = false;

					unsigned long ulPushedTime = millis() - _ulTime_last_pushed;

					if (ulPushedTime >= TIME_LCLICKED_SWITCH && ulPushedTime < TIME_VCLICKED_SWITCH) {

						_u8SwitchState |= LONG_PUSHED_MASK;

						update_Switch(SWITCH_PUSHED_LONG);

						#if defined(_TEST_CROTARYENCODER_)
						cout << F("SW : Long Pushed.") << endl;
						#endif

					}
					else /*if (ulPushedTime >= TIME_VCLICKED_SWITCH)*/ {

						_u8SwitchState |= VLONG_PUSHED_MASK;

						update_Switch(SWITCH_PUSHED_VLONG);

						#if defined(_TEST_CROTARYENCODER_)
						cout << F("SW : VLong Pushed.") << endl;
						#endif

					}

					set_SwitchEvent();

				}

			}
			else {

				if (is_SwitchEventSet() == false)

					update_Switch(SWITCH_NOT_PUSHED);

			}

		}

	}

	_nPastSwitch = nCurrSwitch;
}

void CRotaryEncoder::getRotaryEncoderDirectionState(void)
{
	if (isRotaryEncoderDisabled() == false && isDirectionEventDisabled() == false) {

#if !defined(_MODULO_DIRECTION_COUNT_)
		int nDir = digitalRead(_nDIpin) ? ENCODER_CW_ROTATION : ENCODER_CCW_ROTATION;
		// int nDir = digitalRead(_nDIpin) ? ENCODER_CCW_ROTATION : ENCODER_CW_ROTATION;

		update_Direction(nDir);

		set_DirectionEvent();

		#if defined(_ROTARY_ENCODER_ACCELERATION_)
		set_RotaryAccelerationFactor();
		#endif
#else
		static int _nNegativeDirMod = 0;
		static int _nPositiveDirMod = 0;

		int nDir = digitalRead(_nDIpin) ? ENCODER_CCW_ROTATION : ENCODER_CW_ROTATION;

		if (nDir > 0) {

			_nPositiveDirMod = (_nPositiveDirMod + nDir) % MODULO_NUM;

			if (!_nPositiveDirMod) {

				update_Direction(nDir);

				set_DirectionEvent();

			}

		}
		else {

			_nNegativeDirMod = (_nNegativeDirMod + nDir) % MODULO_NUM;

			if (!_nNegativeDirMod) {

				update_Direction(nDir);

				set_DirectionEvent();

			}

		}
#endif

	}
}

int CRotaryEncoder::getSwitch(void) const
{
	int nSwitch;

	noInterrupts();

	nSwitch = _nSwitch;
	// _nSwitch = SWITCH_NOT_PUSHED;

	interrupts();

	return nSwitch;
}

int CRotaryEncoder::getDirection(void)
{
	int nDirection;

	noInterrupts();

	nDirection = _nDirection;
	_nDirection = ENCODER_NO_ROTATION;

	interrupts();

	return nDirection;
}

int CRotaryEncoder::getAccFactor(const unsigned int nSensibility)
{
	unsigned int nAccelerationFactor;

	// noInterrupts();

	nAccelerationFactor = _nAccelerationFactor;

	// interrupts();

	return nAccelerationFactor > 1 ? nAccelerationFactor * nSensibility : 1;
}

void CRotaryEncoder::clearSwitch(void)
{
	noInterrupts();

	_bSwitchEvent = false;
	_nSwitch = SWITCH_NOT_PUSHED;

	interrupts();
}

void CRotaryEncoder::clearDirection(void)
{
	noInterrupts();

	_bDirectionEvent = false;
	_nDirection = ENCODER_NO_ROTATION;

	interrupts();
}

void CRotaryEncoder::clearRotaryEncoderAll(void)
{
	noInterrupts();

	_bSwitchEvent = false;
	_bDirectionEvent = false;

	_nSwitch = SWITCH_NOT_PUSHED;
	_nDirection = ENCODER_NO_ROTATION;

	interrupts();
}

bool CRotaryEncoder::isSwitchEventOccured(void)
{
	bool bSwitchEvent;

	noInterrupts();

	bSwitchEvent = _bSwitchEvent;
	_bSwitchEvent = false;

	interrupts();

	return bSwitchEvent;
}

bool CRotaryEncoder::isSwitchMaskedEventOccured(void)
{
	if (!_bEventMask[0]) {

		if (millis() - _ulMaskingStartTime[0] < TIME_TO_MASK_FALSE_EVENT) {

			clearSwitch();

		}
		else

			_bEventMask[0] = true;

	}

	return _bEventMask[0] & isSwitchEventOccured();
}

bool CRotaryEncoder::isDirectionEventOccured(void)
{
	bool bDirectionEvent;

	noInterrupts();

	bDirectionEvent = _bDirectionEvent;
	_bDirectionEvent = false;

	interrupts();

	return bDirectionEvent;
}

bool CRotaryEncoder::isDirectionMaskedEventOccured(void)
{
	if (!_bEventMask[1]) {

		if (millis() - _ulMaskingStartTime[1] < TIME_TO_MASK_FALSE_EVENT) {

			clearDirection();

		}
		else

			_bEventMask[1] = true;

	}

	return _bEventMask[1] & isDirectionEventOccured();	
}

void CRotaryEncoder::startMaskingEvent(const unsigned char nMask)
{
	_bEventMask[nMask] = false;
	_ulMaskingStartTime[nMask] = millis();

	// if (!nMask) {

	// 	clearSwitch();

	// }
	// else {

	// 	clearDirection();
		
	// }
}

#if defined(_ROTARY_ENCODER_ACCELERATION_)
float CRotaryEncoder::calculate_RotaryEncoderRPM(const unsigned long t_duration)
{
	float fAveragedRPM = 0.0F;

	if (t_duration > 0UL) {

		fAveragedRPM = float(60.0F * (1000000.0F / float(t_duration * RE_PULSE_PER_REVOLUTION)));
		
	}

	return constrain(fAveragedRPM, 0.0F, RE_RPM_LIMIT);
}

void CRotaryEncoder::set_RotaryAccelerationFactor(void)
{
	int nAccelerationFactor = 1;

	if (StopWatch::isRunning()) {

		float fRotaryEncoderRPM = 0.0f;
		unsigned long time_duration = StopWatch::elapsed();

		StopWatch::reset();
		StopWatch::start();

		fRotaryEncoderRPM = calculate_RotaryEncoderRPM(time_duration);

		nAccelerationFactor = RE_ACC_MULTIPLE * int((fRotaryEncoderRPM + RE_RPM_OFFSET) / RE_ACC_RANGE);

		// if (nAccelerationFactor == 0) {

		// 	nAccelerationFactor = 1;

		// }

	}

	_nAccelerationFactor = nAccelerationFactor;
}
#endif

