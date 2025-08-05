#ifndef _CFLOWMETER_H_
#define _CFLOWMETER_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <StopWatch.h>


using namespace std;


// #define _USE_FLOWMETER_PROC_MODE_


#define NUM_OF_RPM_SAMPLES							(6) 		// Number of Samples to calc. Moving Average of Flowrate (number of Samples < max. 10)

#define PULSE_PER_REVOLUTION						(2)

#define WATERFLOW_QUANTITY_FACTOR					(0.5F)		// 0.5 ml/pulse (ZICAR : 2000 pulse/liter, +/- 2 % accuracy)
#define MIN_WATERFLOW_PER_SECOND					(0.5F) 		// < 0.5 ml/sec
#define MAX_WATERFLOW_PER_SECOND					(60.0F)		// 60.0 ml/sec

#define MIN_TIME_TO_DECAY_FLOWRATE					(WATERFLOW_QUANTITY_FACTOR / MIN_WATERFLOW_PER_SECOND)	// > 1 sec

#define MIN_WATERFLOW_RPM							(MIN_WATERFLOW_PER_SECOND * 60.0F / (WATERFLOW_QUANTITY_FACTOR * PULSE_PER_REVOLUTION))
#define MAX_WATERFLOW_RPM							(MAX_WATERFLOW_PER_SECOND * 60.0F / (WATERFLOW_QUANTITY_FACTOR * PULSE_PER_REVOLUTION))

#define LOWER_WATERFLOW_RPM							(0.0F)
#define UPPER_WATERFLOW_RPM							(MAX_WATERFLOW_RPM)


class CFlowmeter: public StopWatch
{
	friend bool operator == (CFlowmeter& obj1, CFlowmeter& obj2) { return &obj1 == &obj2; }
	
public:
	enum FlowUnit {UNIT_RPM=0, UNIT_LPS};			// UNIT = 0 : rpm, 1 : ml/sec
	enum WaterUnit {UNIT_MILLIS=0, UNIT_LITERS};	// UNIT = 0 : milliliter, 1 : liter

	CFlowmeter(const unsigned char nSensorPin, const int nTriggerMode = CHANGE);

	bool tune(const float fFactor);

	void clear(void);
	void start(void) { clear(); }
	void stop(void);

#ifdef _USE_FLOWMETER_PROC_MODE_
	void setPostProcessing(void (*callback)(void));
#endif

	void measureFlowmeter(void);
	void decayFlowmeterRPM(void);

	float getTuningFactor(void) const { return _fFlowQuantityFactor; }
	
	unsigned int getFlowmeterCNT(void) const { return _u16FlowmeterCNT; }
	float getWaterFlowQTY(const enum WaterUnit nUnit = UNIT_LITERS);
	float getFlowmeterQTY(const enum FlowUnit nUnit = UNIT_RPM);

private:
	unsigned char _nFlowmeterPin;

	int _nTriggerMode;

	volatile int _nLast_signal;
	volatile unsigned int _u16FlowmeterCNT;

	unsigned int _nNum_of_samples;
	unsigned int _nIndex_of_sample;

	volatile float _fFlowmeterRPM;
	float _fSum_of_rpm;
	float _fRPMSample[NUM_OF_RPM_SAMPLES];

	float _fFlowQuantityFactor;
	float _fMinTimeToDecayFlowrate;
	float _fLowerFlowmeterRPM;
	float _fUpperFlowmeterRPM;

#ifdef _USE_FLOWMETER_PROC_MODE_
	void (*_postProcessing)(void);
#endif

	void calculate_FlowmeterRPM(const unsigned long t_duration);
};


#endif //_CFLOWMETER_H_

