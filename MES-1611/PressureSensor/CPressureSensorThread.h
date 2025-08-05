#ifndef _CPRESSURE_SENSOR_THREAD_H_
#define _CPRESSURE_SENSOR_THREAD_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <Thread.h>


#include "CPressureSensor.h"


using namespace std;


// #define _PS_THREAD_DEBUG_MSG_

#define _PS_EVENT_DRIVEN_ACCESS_

#define MAX_PRESSURE_SENSORS				(6)
#define PRESSURE_THREAD_INTERVAL			(100UL) 	// default interval


typedef vector<CPressureSensor*> VSensorList_t;


class CPressureSensorThread: public Thread
{
public:
	CPressureSensorThread();

	void begin(const long lInterval = PRESSURE_THREAD_INTERVAL);
	void stop(void);

	bool add(CPressureSensor* phSensor);
	void remove(const unsigned char u8Index);
	void remove(const CPressureSensor* phSensor);

	unsigned char getSensorListSize(void) const { return _vSensorList.size(); }
	CPressureSensor* getSensorObject(const unsigned char u8Index) const;

	bool isTriggered(void);

	// 'shouldRun' method is overrided.
	bool shouldRun(void);
	bool shouldRun(long time);
	// 'run' method is overrided.
	void run(void);

private:
	VSensorList_t _vSensorList;

#if defined(_PS_EVENT_DRIVEN_ACCESS_)
	volatile bool _bEventTriggered;
#endif

	bool _bRunned;

	void measure_PressureSensorList(void);
};


#endif //_CPRESSURE_SENSOR_THREAD_H_

