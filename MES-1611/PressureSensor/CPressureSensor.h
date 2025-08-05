#ifndef _CPRESSURE_SENSOR_H_
#define _CPRESSURE_SENSOR_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include "../MES_Config.h"


using namespace std;


#if defined(_USE_APPLICATION_DEBUG_MSG_)
	#define _TEST_CPRESSURE_SENSOR_
#endif

// #define _USE_GAUSSIAN_NOISE_REDUCTION_

#define GAUSSIAN_NOISE_EPSILON 						(5)			// 0..255 (<= 10)
#define NUM_OF_ANALOG_PRESS_SAMPLES					(20)		// number of Samples <= 20

#define MIN_PRESSURE_ANALOG							(205)		// Delta = 204.6 : Min. 1 Volt, if 250 Ohm, 4 mA
#define MAX_PRESSURE_ANALOG							(1023)		// 				   Max. 5 Volt, if 250 Ohm, 20 mA
#define MIN_PRESSURE_MAPPED							(0) 		// Sensor Spec. : Min. 0 bar
#define MAX_PRESSURE_MAPPED							(160)		// 				: Max. 16 bar

#define LOWER_PRESSURE_VALUE						(MIN_PRESSURE_MAPPED / 10.0F)
#define UPPER_PRESSURE_VALUE						(MAX_PRESSURE_MAPPED / 10.0F)

#define ZERO_EXACT_VALUE							(204.6F)	// ExactValue: 204.6F = 4.00mA (0.0bar, 1.0V @ 250Ohm)
#define SPAN_EXACT_VALUE							(716.1F)	// ExactValue: 716.1F = 14.0mA (10.0bar, 3.5V @ 250Ohm)

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
// PTE5000-016-1-B-1-B
#define DEFAULT_ZERO_SHIFT							(16.982F)	// (ZERO_EXACT_VALUE - 187.618F) 	// (ExactValue - MeasureValue)
#define DEFAULT_SPAN_FACTOR							(1.094F)	// (SPAN_EXACT_VALUE / 654.720F)	// (ExactValue / MeasureValue)
#else
// SPT-I2 (16bar), PRIGNITZ
#define DEFAULT_ZERO_SHIFT							(10.600F)	// (ZERO_EXACT_VALUE - 194.000F) 	// (ExactValue - MeasureValue)
#define DEFAULT_SPAN_FACTOR							(1.043F)	// (SPAN_EXACT_VALUE / 686.577F)	// (ExactValue / MeasureValue)
#endif


class CPressureSensor
{
	friend bool operator == (CPressureSensor& obj1, CPressureSensor& obj2) { return &obj1 == &obj2; }

public:
	CPressureSensor(const unsigned char nSensorPin, const float fTuneValue = 0.0f);

	void clear(void);

	void measurePressure(void);

	void setPressureZeroShift(void);
	void setPressureSpanFactor(void);
	void resetPressureAdjustment(void);

	float getPressureValue(void) const { return _fPressureValue; }
	void setPressureTuneValue(const float fTuneValue) { _fPressureTuneValue = fTuneValue; }
	void setZeroShiftValue(const float fZeroShift) { _fZeroShift = fZeroShift; }
	void setSpanFactorValue(const float fSpanFactor) { _fSpanFactor = fSpanFactor; }
	float getZeroShiftValue(void) const { return _fZeroShift; }
	float getSpanFactorValue(void) const { return _fSpanFactor; }
	int getAnalogEstimatedValue(void) const { return _nAnalogEstimatedValue; }
	
	int getAnalogRawValue(void) const { return _nAnalogRawValue; }

private:
	unsigned char _nPressurePin;
	
	volatile int _nAnalogRawValue;
	volatile int _nAnalogEstimatedValue;
	volatile float _fPressureValue;

	float _fZeroShift;
	float _fSpanFactor;
	float _fPressureTuneValue;

	int _nSum_of_samples;
	int _nAnalogSample[NUM_OF_ANALOG_PRESS_SAMPLES];

	unsigned int _nNum_of_samples;
	unsigned int _nIndex_of_sample;

	unsigned int read_PressureSensor(void);
	unsigned int reduce_AnalogGaussianNoise(const unsigned int nADCDigit, const unsigned int nEpsilon);
	void convert_PressureValueBAR(const unsigned int nAnalogPressureValue);
	void calculate_AveragedAnalogValue(const int nAnalogValue);
};


#endif //_CPRESSURE_SENSOR_H_


