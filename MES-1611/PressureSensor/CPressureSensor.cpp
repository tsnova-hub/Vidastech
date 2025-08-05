

#include "CPressureSensor.h"


CPressureSensor::CPressureSensor(const unsigned char nSensorPin, const float fTuneValue)
{
	_nPressurePin = nSensorPin;

	_fZeroShift = float(DEFAULT_ZERO_SHIFT);
	_fSpanFactor = float(DEFAULT_SPAN_FACTOR);

	_fPressureTuneValue = fTuneValue;

	pinMode(_nPressurePin, INPUT);
}

void CPressureSensor::clear(void)
{
	_fPressureValue = 0.0f;

	_nAnalogEstimatedValue = 0;

	_nAnalogRawValue = 0;

	_nIndex_of_sample = 0;
	_nNum_of_samples = 0;

	_nSum_of_samples = 0l;

	memset(_nAnalogSample, 0l, sizeof(int)*NUM_OF_ANALOG_PRESS_SAMPLES);
}

void CPressureSensor::setPressureZeroShift(void)
{
	_fZeroShift = constrain(float(ZERO_EXACT_VALUE - _nAnalogRawValue), -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);

#if defined(_TEST_CPRESSURE_SENSOR_)
	cout << F("(Pressure Sensor ZeroShift Set:: ") << _fZeroShift << F(")") << endl;
#endif
}

void CPressureSensor::setPressureSpanFactor(void)
{
	_fSpanFactor = constrain(float(SPAN_EXACT_VALUE / float(_nAnalogRawValue + _fZeroShift)), 0.001F, 2.000F);

#if defined(_TEST_CPRESSURE_SENSOR_)
	cout << F("(Pressure Sensor Span Set:: ") << _fSpanFactor << F(")") << endl;
#endif
}

void CPressureSensor::resetPressureAdjustment(void)
{
	_fZeroShift = float(DEFAULT_ZERO_SHIFT);
	_fSpanFactor = float(DEFAULT_SPAN_FACTOR);
}

void CPressureSensor::measurePressure(void)
{
	unsigned int nPresentAnalogPressure = read_PressureSensor();

	convert_PressureValueBAR(nPresentAnalogPressure);
}

unsigned int CPressureSensor::read_PressureSensor(void)
{
	unsigned int u16ADCDigit;
	unsigned int u16PressureVal_Analog;

#if !defined(_USE_GAUSSIAN_NOISE_REDUCTION_)
	u16ADCDigit = analogRead(_nPressurePin);
#else
	u16ADCDigit = reduce_AnalogGaussianNoise(analogRead(_nPressurePin), GAUSSIAN_NOISE_EPSILON);
#endif

	calculate_AveragedAnalogValue(u16ADCDigit);

	// _nAnalogEstimatedValue = int(float(_nAnalogRawValue + _fZeroShift) * _fSpanFactor);
	_nAnalogEstimatedValue = int(roundf((_nAnalogRawValue + _fZeroShift) * _fSpanFactor));

	u16PressureVal_Analog = constrain(_nAnalogEstimatedValue, MIN_PRESSURE_ANALOG, MAX_PRESSURE_ANALOG);	

	return u16PressureVal_Analog;
}

unsigned int CPressureSensor::reduce_AnalogGaussianNoise(const unsigned int nADCDigit, const unsigned int nEpsilon)
{
	static unsigned int _prevADCDigit = 0;

	if (nEpsilon == 0 || ((nADCDigit - _prevADCDigit) & 0x7FFF) > nEpsilon)
	{
		_prevADCDigit = nADCDigit;
	}

	return _prevADCDigit;
}

void CPressureSensor::convert_PressureValueBAR(const unsigned int nAnalogPressureValue)
{
	unsigned char u8PressureVal_Mapped;

	u8PressureVal_Mapped = map(nAnalogPressureValue, MIN_PRESSURE_ANALOG, MAX_PRESSURE_ANALOG, MIN_PRESSURE_MAPPED, MAX_PRESSURE_MAPPED);

	_fPressureValue = float(u8PressureVal_Mapped / 10.0f) + _fPressureTuneValue;
}

void CPressureSensor::calculate_AveragedAnalogValue(const int nAnalogValue)
{
	int nAveragedValue = 0L;

	_nSum_of_samples -= _nAnalogSample[_nIndex_of_sample];

	_nAnalogSample[_nIndex_of_sample] = nAnalogValue;

	_nSum_of_samples += _nAnalogSample[_nIndex_of_sample];

	_nIndex_of_sample = ++_nIndex_of_sample % NUM_OF_ANALOG_PRESS_SAMPLES;
	
	if (_nNum_of_samples < NUM_OF_ANALOG_PRESS_SAMPLES) {

		_nNum_of_samples++;

	}

	nAveragedValue = _nSum_of_samples / _nNum_of_samples;

	_nAnalogRawValue = constrain(nAveragedValue, 0, MAX_PRESSURE_ANALOG);
}

