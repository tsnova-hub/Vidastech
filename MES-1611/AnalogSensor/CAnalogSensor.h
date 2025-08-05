#ifndef _CANALOG_SENSOR_H_
#define _CANALOG_SENSOR_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


using namespace std;


// #define _USE_ANALOG_GAUSSIAN_NOISE_REDUCTION_

#define ANALOG_GAUSSIAN_NOISE_EPSILON 				(10)		// 0..255 (<= 10)
#define NUM_OF_ANALOG_SENSOR_SAMPLES				(10)		// number of Samples <= 10

// 1 digit = about 0.004888V (4.888mV), 1.0 scaling factor @ 5V (NOTICE: must connect to pin header in controllino maxi)
#define VOLTAGE_DIVIDE_FACTOR5						(0.004888F)
// 1 digit = about 0.015V (15mV), 3.06 scaling factor @ 12V (NOTICE: 12V power supply)
#define VOLTAGE_DIVIDE_FACTOR12     				(0.0150F)
// 1 digit = about 0.03V (30mV), 6.14 scaling factor @ 24V (NOTICE: 24V power supply)
#define VOLTAGE_DIVIDE_FACTOR24     				(0.0300F)

#define MIN_VOLTAGE_ANALOG							(0)			
#define MAX_VOLTAGE_ANALOG							(1023)		
#define MIN_VOLTAGE_MAPPED							(0) 		
#define MAX_VOLTAGE_MAPPED							(1023)		

#define LOWER_VOLTAGE_VALUE							(MIN_VOLTAGE_MAPPED)
#define UPPER_VOLTAGE_VALUE							(MAX_VOLTAGE_MAPPED)

#define BIAS_VOLTAGE_OFFSET							(0.0F) 		


class CAnalogSensor
{
	friend bool operator == (CAnalogSensor& obj1, CAnalogSensor& obj2) { return &obj1 == &obj2; }

public:
	CAnalogSensor(const unsigned char nSensorPin, const float fVoltageFactor = VOLTAGE_DIVIDE_FACTOR24, const float fVoltageTuneValue = 0.0f);

	void clear(void);

	void measureAnalogSensor(void);

	float getAnalogValue(void) const { return _fAnalogValue; }
	float getVoltageValue(void) { return (_fAnalogValue * _fVoltageFactor + BIAS_VOLTAGE_OFFSET + _fVoltageTuneValue); }
	void setVoltageTuneValue(const float fTuneValue) { _fVoltageTuneValue = fTuneValue; }

private:
	unsigned char _nAnalogPin;
	
	volatile float _fAnalogValue;

	float _fVoltageFactor;
	float _fVoltageTuneValue;
	
	float _fSum_of_samples;
	int _nAnalogSample[NUM_OF_ANALOG_SENSOR_SAMPLES];

	unsigned int _nNum_of_samples;
	unsigned int _nIndex_of_sample;

	unsigned int read_AnalogSensor(void);
	unsigned int reduce_AnalogGaussianNoise(const unsigned int nADCDigit, const unsigned int nEpsilon);
	void calculate_AveragedAnalogValue(const int nPresentAnalog);
};


#endif //_CANALOG_SENSOR_H_

