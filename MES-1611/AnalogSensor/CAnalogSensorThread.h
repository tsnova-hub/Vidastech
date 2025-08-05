#ifndef _CANALOG_SENSOR_THREAD_H_
#define _CANALOG_SENSOR_THREAD_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <Thread.h>


#include "CAnalogSensor.h"


using namespace std;


// #define _AS_THREAD_DEBUG_MSG_

#define _AS_EVENT_DRIVEN_ACCESS_

#define MAX_ANALOG_SENSORS					(7)

#define ANALOG_THREAD_INTERVAL				(100UL) 	// default interval


typedef vector<CAnalogSensor*> VAnalogSensorList_t;


class CAnalogSensorThread: public Thread
{
public:
	CAnalogSensorThread();

	void begin(const long lInterval = ANALOG_THREAD_INTERVAL);
	void stop(void);

	bool add(CAnalogSensor* phSensor);
	void remove(const unsigned char u8Index);
	void remove(const CAnalogSensor* phSensor);

	unsigned char getSensorListSize(void) const { return _vSensorList.size(); }
	CAnalogSensor* getSensorObject(const unsigned char u8Index) const;

	bool isTriggered(void);

	// 'shouldRun' method is overrided.
	bool shouldRun(void);
	bool shouldRun(long time);
	// 'run' method is overrided.
	void run(void);

private:
	VAnalogSensorList_t _vSensorList;

#if defined(_AS_EVENT_DRIVEN_ACCESS_)
	volatile bool _bEventTriggered;
#endif

	bool _bRunned;

	void measure_AnalogSensorList(void);
};


#endif //_CANALOG_SENSOR_THREAD_H_

