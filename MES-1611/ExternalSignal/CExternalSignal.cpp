

#include "CExternalSignal.h"


CExternalSignal::CExternalSignal(const unsigned char nSignalPin, const enum ActiveSigType nSigType)
{
	_nSignalPin = nSignalPin;

	_nActiveSignal = nSigType == CExternalSignal::ACTIVE_HIGH ? HIGH : LOW;

#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
	_preProcessing = NULL;
	_postProcessing = NULL;
#endif

	pinMode(_nSignalPin, INPUT_PULLUP);
}

#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
void CExternalSignal::setPreProcessing(void (*preProcessing)(void))
{
	if (preProcessing != NULL) {

		_preProcessing = preProcessing;

	}
}

void CExternalSignal::setPostProcessing(void (*postProcessing)(void))
{
	if (postProcessing != NULL) {

		_postProcessing = postProcessing;
		
	}
}
#endif

void CExternalSignal::clear(void)
{
	_u16SignalCNT = 0UL;

	_nCurrSignal = LOW;
	_nLastSignal = LOW;
}

void CExternalSignal::pollingSignal(void)
{
	_nCurrSignal = digitalRead(_nSignalPin);

	if (_nCurrSignal == _nActiveSignal && _nCurrSignal != _nLastSignal) {

		#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
		if (_preProcessing) {

			_preProcessing();

		}
		#endif
		
		if (_u16SignalCNT >= CExternalSignal::_nMaxCount) {

			_u16SignalCNT = 0UL;

		}
		else {

			_u16SignalCNT++;

		}

		#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
		if (_postProcessing) {

			_postProcessing();

		}
		#endif

	}

	_nLastSignal = _nCurrSignal;
}

