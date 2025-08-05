

#include "COpticalFiber.h"


COpticalFiber::COpticalFiber(const unsigned char nPowerPin, const unsigned char nSignalPin, const unsigned char nDiagnosisPin, const enum ActiveType nActiveType)
{
	_nPowerPin = nPowerPin;
	_nSignalPin = nSignalPin;
	_nDiagnosisPin = nDiagnosisPin;

#if USE_NPN_TYPE_OPTICAL_FIBER_AMP
	_nLight = nActiveType == COpticalFiber::LIGHT_ON ? LOW : HIGH;
	_nDark = nActiveType == COpticalFiber::LIGHT_ON ? HIGH : LOW;
#else
	_nLight = nActiveType == COpticalFiber::LIGHT_ON ? HIGH : LOW;
	_nDark = nActiveType == COpticalFiber::LIGHT_ON ? LOW : HIGH;
#endif

	_bIsRunning = false;
	_bIsEnabled = false;
	_bIsUnstable = false;

	_ulDarkDuration = DEFAULT_SIGNAL_DURATION;
	_ulLightDuration = DEFAULT_SIGNAL_DURATION;

#if defined(_USE_OPTICAL_FIBER_PROC_MODE_)
	_darkProcessing = NULL;
	_lightProcessing = NULL;
#endif

	pinMode(_nPowerPin, OUTPUT);
	digitalWrite(_nPowerPin, LOW);

	pinMode(_nSignalPin, INPUT);
	pinMode(_nDiagnosisPin, INPUT);
}

#if defined(_USE_OPTICAL_FIBER_PROC_MODE_)
void COpticalFiber::setDarkProcessing(void (*darkProcessing)(void))
{
	if (darkProcessing != NULL) {

		_darkProcessing = darkProcessing;

	}
}

void COpticalFiber::setLightProcessing(void (*lightProcessing)(void))
{
	if (lightProcessing != NULL) {

		_lightProcessing = lightProcessing;
		
	}
}
#endif

void COpticalFiber::init(void)
{
	_nCurrSignal = _nDark;  /*LOW;*/
	_nLastSignal = _nLight; /*HIGH;*/

	_ulDarkOnTime = 0UL;
	_ulLightOnTime = 0UL;
}

void COpticalFiber::setActiveType(const enum ActiveType nActiveType)
{
#if USE_NPN_TYPE_OPTICAL_FIBER_AMP
	_nLight = nActiveType == COpticalFiber::LIGHT_ON ? LOW : HIGH;
	_nDark = nActiveType == COpticalFiber::LIGHT_ON ? HIGH : LOW;
#else
	_nLight = nActiveType == COpticalFiber::LIGHT_ON ? HIGH : LOW;
	_nDark = nActiveType == COpticalFiber::LIGHT_ON ? LOW : HIGH;
#endif
}

void COpticalFiber::setDuration(const unsigned long ulDarkDuration, const unsigned long ulLightDuration)
{
	_ulDarkDuration = ulDarkDuration;
	_ulLightDuration = ulLightDuration;
}

void COpticalFiber::turnon(void)
{
	digitalWrite(_nPowerPin, HIGH);

	_bIsRunning = true;

}

void COpticalFiber::turnoff(void)
{
	digitalWrite(_nPowerPin, LOW);

	_bIsRunning = false;
}

// void COpticalFiber::powerOnReset(void)
// {
// 	unsigned long ulStartTime;

// 	turnoff();

// 	ulStartTime = millis();

// 	while (millis() - ulStartTime < OPTICAL_FIBER_AMP_RESET_DURATION);

// 	turnon();
// }

int COpticalFiber::readSignal(void)
{
	if (_bIsRunning == false)

		return -1;

	return digitalRead(_nSignalPin);
}

void COpticalFiber::pollingSignal(void)
{
	if (_bIsRunning == false || _bIsEnabled == false)

		return;

	int nCurrSignal = digitalRead(_nSignalPin);

	if (nCurrSignal == _nDark && nCurrSignal != _nLastSignal) {
		
		_ulDarkOnTime = millis();

	}
	else {

		if (nCurrSignal == _nDark) {

			if (_nCurrSignal != _nDark) {

				if (millis() - _ulDarkOnTime >= _ulDarkDuration) {

					#if defined(_USE_OPTICAL_FIBER_PROC_MODE_)
					if (_darkProcessing) {

						_darkProcessing();

					}
					#endif

					_nCurrSignal = _nDark;

				}

			}

		}

	}

	if (nCurrSignal == _nLight && nCurrSignal != _nLastSignal) {

		_ulLightOnTime = millis();

	}
	else {

		if (nCurrSignal == _nLight) {

			if (_nCurrSignal != _nLight) {

				if (millis() - _ulLightOnTime >= _ulLightDuration) {

					#if defined(_USE_OPTICAL_FIBER_PROC_MODE_)
					if (_lightProcessing) {

						_lightProcessing();

					}
					#endif

					_nCurrSignal = _nLight;

				}

			}

		}

	}

	_nLastSignal = nCurrSignal;

	diagnosis();
}

void COpticalFiber::diagnosis(void)
{
	if (digitalRead(_nDiagnosisPin) == _nLight) {

		_bIsUnstable = true;

	}
	else {

		_bIsUnstable = false;

	}
}

