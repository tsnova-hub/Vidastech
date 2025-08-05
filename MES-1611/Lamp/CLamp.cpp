

#include "CLamp.h"


CLamp::CLamp(const unsigned char nLampPin)
: StopWatch(StopWatch::MILLIS)
{
	_nLampPin = nLampPin;

	pinMode(_nLampPin, OUTPUT);
	analogWrite(_nLampPin, LAMP_TURN_OFF);
}

void CLamp::init(void)
{
	_bIsLampOn = false;

	_eLampMode = CLamp::TURNON_SHOT;
	_eLampIntensity = CLamp::HIGH_INTENSITY;

	analogWrite(_nLampPin, LAMP_TURN_OFF);
}

void CLamp::turnon(void)
{
	if (_eLampMode > CLamp::TURNOFF_ALWAYS) {

		#if defined(_LAMP_DELAYED_OFF_)
		StopWatch::stop();
		#endif

		analogWrite(_nLampPin, (_eLampIntensity == CLamp::LOW_INTENSITY ? LAMP_INTENSITY_LOW : LAMP_INTENSITY_HIGH));

		_bIsLampOn = true;

	}
}

void CLamp::turnoff(void)
{
	if (_eLampMode < CLamp::TURNON_ALWAYS) {

		#if !defined(_LAMP_DELAYED_OFF_)
		analogWrite(_nLampPin, LAMP_TURN_OFF);
		#else
		StopWatch::reset();
		StopWatch::start();
		#endif

		_bIsLampOn = false;

	}
}

void CLamp::turnoffDELAYED(void)
{
#if defined(_LAMP_DELAYED_OFF_)
	if (StopWatch::isRunning()) {

		if (StopWatch::elapsed() >= DELAY_TIME_TO_TURNOFF) {

			StopWatch::stop();

			analogWrite(_nLampPin, LAMP_TURN_OFF);

		}
		
	}
#endif
}

void CLamp::turnoffEMG(void)
{
	analogWrite(_nLampPin, LAMP_TURN_OFF);

	_bIsLampOn = false;
}

void CLamp::setMode(const enum LampMode eMode)
{
	_eLampMode = eMode;

	if (_eLampMode == CLamp::TURNON_ALWAYS) {

		turnon();

	}
	else {

		turnoff();

	}
}

void CLamp::setIntensity(const enum LampIntensity eIntensity)
{
	_eLampIntensity = eIntensity;

	if (_bIsLampOn) {

		analogWrite(_nLampPin, (_eLampIntensity == CLamp::LOW_INTENSITY ? LAMP_INTENSITY_LOW : LAMP_INTENSITY_HIGH));

	}
}

#if defined(_TEST_CLAMP_)
void CLamp::printStatus(void)
{
	cout << F("CLamp::LOW_INTENSITY: ") << LOWER_INTENSITY_RATE
		 << F(", PWM: ") << LAMP_INTENSITY_LOW
		 << F(", Rate: ") << float(LAMP_INTENSITY_LOW / float(255UL)) << endl;

	cout << F("CLamp::HIGH_INTENSITY: ") << UPPER_INTENSITY_RATE
		 << F(", PWM: ") << LAMP_INTENSITY_HIGH
		 << F(", Rate: ") << float(LAMP_INTENSITY_HIGH / float(255UL)) << endl;
}
#endif


