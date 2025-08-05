

#include "CAnalogSensor.h"


CAnalogSensor::CAnalogSensor(const unsigned char nSensorPin, const float fVoltageFactor, const float fVoltageTuneValue)
{
	_nAnalogPin = nSensorPin;

	_fVoltageFactor = fVoltageFactor;
	_fVoltageTuneValue = fVoltageTuneValue;

	pinMode(_nAnalogPin, INPUT);
}

void CAnalogSensor::clear(void)
{
	_fAnalogValue = 0.0f;

	_nNum_of_samples = 0;
	_nIndex_of_sample = 0;
	_fSum_of_samples = 0.0f;
	
	memset(_nAnalogSample, 0l, sizeof(int)*NUM_OF_ANALOG_SENSOR_SAMPLES);
}

void CAnalogSensor::measureAnalogSensor(void)
{
	int nPresentAnalog = read_AnalogSensor();

	calculate_AveragedAnalogValue(nPresentAnalog);
}

unsigned int CAnalogSensor::read_AnalogSensor(void)
{
	unsigned int u16ADCDigit;
	unsigned int u16PresentVal_Analog;
	unsigned int u16PresentVal_Mapped;

#if !defined(_USE_ANALOG_GAUSSIAN_NOISE_REDUCTION_)
	u16ADCDigit = analogRead(_nAnalogPin);
#else
	u16ADCDigit = reduce_AnalogGaussianNoise(analogRead(_nAnalogPin), ANALOG_GAUSSIAN_NOISE_EPSILON);
#endif

	u16PresentVal_Analog = constrain(u16ADCDigit, MIN_VOLTAGE_ANALOG, MAX_VOLTAGE_ANALOG);

	u16PresentVal_Mapped = map(u16PresentVal_Analog, MIN_VOLTAGE_ANALOG, MAX_VOLTAGE_ANALOG, MIN_VOLTAGE_MAPPED, MAX_VOLTAGE_MAPPED);

	return u16PresentVal_Mapped;
}

unsigned int CAnalogSensor::reduce_AnalogGaussianNoise(const unsigned int nADCDigit, const unsigned int nEpsilon)
{
	static unsigned int _prevADCDigit = 0;

	if (nEpsilon == 0 || ((nADCDigit - _prevADCDigit) & 0x7FFF) > nEpsilon)
	{
		_prevADCDigit = nADCDigit;
	}

	return _prevADCDigit;
}

void CAnalogSensor::calculate_AveragedAnalogValue(const int nPresentAnalog)
{
	float fAveragedValue = 0.0F;

	_fSum_of_samples -= _nAnalogSample[_nIndex_of_sample];

	_nAnalogSample[_nIndex_of_sample] = nPresentAnalog;

	_fSum_of_samples += _nAnalogSample[_nIndex_of_sample];

	_nIndex_of_sample = ++_nIndex_of_sample % NUM_OF_ANALOG_SENSOR_SAMPLES;
	
	if (_nNum_of_samples < NUM_OF_ANALOG_SENSOR_SAMPLES) {

		_nNum_of_samples++;
		
	}

	fAveragedValue = _fSum_of_samples / float(_nNum_of_samples);

	_fAnalogValue = constrain(fAveragedValue, LOWER_VOLTAGE_VALUE, UPPER_VOLTAGE_VALUE);
}
