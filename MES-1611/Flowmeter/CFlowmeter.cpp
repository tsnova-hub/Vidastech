

#include "CFlowmeter.h"


CFlowmeter::CFlowmeter(const unsigned char nSensorPin, const int nTriggerMode)
: StopWatch(StopWatch::MICROS)
{
	_nFlowmeterPin = nSensorPin;
	_nTriggerMode = nTriggerMode;

	_fFlowQuantityFactor = WATERFLOW_QUANTITY_FACTOR;

	_fMinTimeToDecayFlowrate = MIN_TIME_TO_DECAY_FLOWRATE;

	_fLowerFlowmeterRPM = LOWER_WATERFLOW_RPM;
	_fUpperFlowmeterRPM = UPPER_WATERFLOW_RPM;

#if defined(_USE_FLOWMETER_PROC_MODE_)
	_postProcessing = NULL;
#endif

	pinMode(_nFlowmeterPin, INPUT_PULLUP);
}

bool CFlowmeter::tune(const float fFactor)
{
	if (_fFlowQuantityFactor != fFactor) {

		_fFlowQuantityFactor = fFactor;

		_fMinTimeToDecayFlowrate = fFactor / float(MIN_WATERFLOW_PER_SECOND);

		_fLowerFlowmeterRPM = LOWER_WATERFLOW_RPM;
		// _fLowerFlowmeterRPM = MIN_WATERFLOW_PER_SECOND * 60.0F / (fFactor * PULSE_PER_REVOLUTION);
		_fUpperFlowmeterRPM = MAX_WATERFLOW_PER_SECOND * 60.0F / (fFactor * PULSE_PER_REVOLUTION);

		return true;

	}

	return false;
}

void CFlowmeter::clear(void)
{
	_u16FlowmeterCNT = 0;
	_fFlowmeterRPM = 0.0f;

	_nLast_signal = LOW;

	_nNum_of_samples = 0;
	_nIndex_of_sample = 0;
	_fSum_of_rpm = 0.0f;

	memset(_fRPMSample, 0.0f, sizeof(float)*NUM_OF_RPM_SAMPLES);
}

void CFlowmeter::stop(void)
{
	if (StopWatch::isRunning()) {

		StopWatch::stop();

	}
}

#if defined(_USE_FLOWMETER_PROC_MODE_)
void CFlowmeter::setPostProcessing(void (*callback)(void))
{
	if (callback != NULL) {

		_postProcessing = callback;

	}
}
#endif

float CFlowmeter::getWaterFlowQTY(const enum WaterUnit nUnit)
{
	float fWaterFlowQty;

	fWaterFlowQty = nUnit == CFlowmeter::UNIT_MILLIS ? _u16FlowmeterCNT * _fFlowQuantityFactor : float((_u16FlowmeterCNT * _fFlowQuantityFactor) / 1000.0f);

	return fWaterFlowQty;
}

float CFlowmeter::getFlowmeterQTY(const enum FlowUnit nUnit)
{
	float fFlowmeterQTY;

	fFlowmeterQTY = nUnit == CFlowmeter::UNIT_LPS ? (_fFlowmeterRPM * PULSE_PER_REVOLUTION * _fFlowQuantityFactor) / 60.0F : _fFlowmeterRPM;

	return fFlowmeterQTY;
}

void CFlowmeter::measureFlowmeter(void)
{
	if (_nTriggerMode == CHANGE) {	// FLOWMETER_TRIGGER_TYPE == CHANGE

		int nCurrent_signal;

		nCurrent_signal = digitalRead(_nFlowmeterPin);

		if (nCurrent_signal == HIGH && nCurrent_signal != _nLast_signal)
		{
			if (!StopWatch::isRunning()) {

				StopWatch::reset();
				StopWatch::start();

			}
			else {

				unsigned long time_duration = StopWatch::elapsed();

				StopWatch::reset();
				StopWatch::start();

				_u16FlowmeterCNT++;

				calculate_FlowmeterRPM(time_duration);

			}
		}

		_nLast_signal = nCurrent_signal;

	}
	else {	// FLOWMETER_TRIGGER_TYPE == RISING OR FALLING

		if (!StopWatch::isRunning()) {

			StopWatch::reset();
			StopWatch::start();

		} else {
			
			unsigned long time_duration = StopWatch::elapsed();

			StopWatch::reset();
			StopWatch::start();

			_u16FlowmeterCNT++;

			calculate_FlowmeterRPM(time_duration);

		}

	}

#if defined(_USE_FLOWMETER_PROC_MODE_)
	if (_postProcessing) {

		_postProcessing();
		
	}
#endif

}

void CFlowmeter::decayFlowmeterRPM(void)
{
	if (StopWatch::isRunning()) {

		unsigned long ulPulseTimeDuration = StopWatch::elapsed();

		if (ulPulseTimeDuration >= _fMinTimeToDecayFlowrate * 1000000UL) {

			float fAveragedRPM = 0.0F;

			_fSum_of_rpm -= _fRPMSample[_nIndex_of_sample];

			_fRPMSample[_nIndex_of_sample] = 0.0F;

			_fSum_of_rpm += _fRPMSample[_nIndex_of_sample];

			_nIndex_of_sample = ++_nIndex_of_sample % NUM_OF_RPM_SAMPLES;
			
			if (_nNum_of_samples < NUM_OF_RPM_SAMPLES) {

				_nNum_of_samples++;
				
			}

			fAveragedRPM = _fSum_of_rpm / float(_nNum_of_samples);

			_fFlowmeterRPM = constrain(fAveragedRPM, LOWER_WATERFLOW_RPM, UPPER_WATERFLOW_RPM);
			_fFlowmeterRPM = constrain(fAveragedRPM, _fLowerFlowmeterRPM, _fUpperFlowmeterRPM);

		}

	}
}

void CFlowmeter::calculate_FlowmeterRPM(const unsigned long t_duration)
{
	float fAveragedRPM = 0.0F;

	if (t_duration > 0UL) {

		_fSum_of_rpm -= _fRPMSample[_nIndex_of_sample];

		_fRPMSample[_nIndex_of_sample] = float(60.0F * (1000000.0F / float(t_duration * PULSE_PER_REVOLUTION)));

		_fSum_of_rpm += _fRPMSample[_nIndex_of_sample];

		_nIndex_of_sample = ++_nIndex_of_sample % NUM_OF_RPM_SAMPLES;
		
		if (_nNum_of_samples < NUM_OF_RPM_SAMPLES) {

			_nNum_of_samples++;
			
		}

		fAveragedRPM = _fSum_of_rpm / float(_nNum_of_samples);
		
	}

	_fFlowmeterRPM = constrain(fAveragedRPM, _fLowerFlowmeterRPM, _fUpperFlowmeterRPM);
}

