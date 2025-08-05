

#include "CSolenoidValve.h"


CSolenoidValve::CSolenoidValve(const unsigned char nSignalPin, const enum SVType nType)
{
	_nSignalPin = nSignalPin;

	if (nType == CSolenoidValve::NORMAL_CLOSE) {

		_nSVCmd[SV_CLOSE] = LOW;
		_nSVCmd[SV_OPEN] = HIGH;

	}
	else {

		_nSVCmd[SV_CLOSE] = HIGH;
		_nSVCmd[SV_OPEN] = LOW;
	}

	_bIsOpened = false;

#if defined(_USE_SOLENOID_VALVE_PROC_MODE_)
	_preProcessing = NULL;
	_postProcessing = NULL;
#endif

	pinMode(_nSignalPin, OUTPUT);
	digitalWrite(_nSignalPin, _nSVCmd[SV_CLOSE]);
}

void CSolenoidValve::init(const unsigned char nSVState)
{
	unsigned char nSVCmd = constrain(nSVState, SV_CLOSE, SV_OPEN);

	_bIsOpened = nSVCmd == SV_OPEN ? true : false;

	digitalWrite(_nSignalPin, _nSVCmd[nSVCmd]);
}

void CSolenoidValve::open(void)
{	
	digitalWrite(_nSignalPin, _nSVCmd[SV_OPEN]);

	_bIsOpened = true;
}

void CSolenoidValve::close(void)
{
	digitalWrite(_nSignalPin, _nSVCmd[SV_CLOSE]);

	_bIsOpened = false;
}

#if defined(_USE_SOLENOID_VALVE_PROC_MODE_)
void CSolenoidValve::setPreProcessing(void (*preProcessing)(void))
{
	if (preProcessing != NULL) {

		_preProcessing = preProcessing;

	}
}

void CSolenoidValve::setPostProcessing(void (*postProcessing)(void))
{
	if (postProcessing != NULL) {

		_postProcessing = postProcessing;
		
	}
}

void CSolenoidValve::openProc(void)
{
	if (_preProcessing) {

		_preProcessing();

	}

	digitalWrite(_nSignalPin, _nSVCmd[SV_OPEN]);

	_bIsOpened = true;

	if (_postProcessing) {

		_postProcessing();

	}
}

void CSolenoidValve::closeProc(void)
{
	if (_preProcessing) {

		_preProcessing();

	}

	digitalWrite(_nSignalPin, _nSVCmd[SV_CLOSE]);

	_bIsOpened = false;

	if (_postProcessing) {

		_postProcessing();

	}
}
#endif

