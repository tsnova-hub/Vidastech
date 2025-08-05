#ifndef _CMOTORPUMP_H_
#define _CMOTORPUMP_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


// #define _MEASURE_PRESENT_RPM_
#ifdef _MEASURE_PRESENT_RPM_
	#include <StopWatch.h>
#endif


#include "../MES_Config.h"


using namespace std;


// #if defined(_USE_APPLICATION_DEBUG_MSG_)
// 	#define _TEST_CMOTORPUMP_
// #endif


// #define _USE_MOTOR_PUMP_ALARM_PROC_MODE_
// #define _RESET_ON_ALARM_							(v1.2.2r-180220)


#ifndef __U8_MAX__
#define __U8_MAX__									(255UL)
#endif

#ifndef __U10_MAX__
#define __U10_MAX__									(1023UL)
#endif

#define _MIN_DUTY_VALUE_	 						(0UL)
#define _MAX_DUTY_VALUE_							(__U8_MAX__)

#define _MIN_DUTY_RATE_								(0.0F)	// rate=0%   : value=0
#define _MAX_DUTY_RATE_								(1.0F)	// rate=100% : value=255

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	// Pseudo Definitions to build when _USE_OZBV10AD2M2_MOTOR_CONTROLLER_ is not defined
	#define _MIN_ACC_TIME_							(1UL)				// 0.001 [sec] @ 1000 [rpm]
	#define _MAX_ACC_TIME_							(20000UL)			// 20 [sec] @ 1000 [rpm]
	#define _MIN_DEC_TIME_							(_MIN_ACC_TIME_)
	#define _MAX_DEC_TIME_							(_MAX_ACC_TIME_)
#endif


#define MOTOR_RESET_DURATION						(50UL)

#define MOTOR_PPR 									(4)

#define NUM_OF_RPM_SAMPLES							(5) 		// Number of Samples to calc. Moving Average of Flowrate (number of Samples = max. 10)


class CMotorPump
{
public:
	CMotorPump(const unsigned char nSpeedPin, const unsigned char nStopPin, const unsigned char nResetPin, const unsigned char nAlarmPin, const unsigned char nSpdoutPin);
	~CMotorPump();

	void clear(void);

	void pollingAlarm(void);

	void setDutyRange(const float fLowerRate, const float fUpperRate);

#ifdef _USE_MOTOR_PUMP_ALARM_PROC_MODE_
	void setPostAlarm(void (*postAlarm)(void));
#endif

	void reset(void);
	void start(void);
	void start(const unsigned int nDutyValue);
	void start(const float fDutyRate);
	void stop(void);
	void stopEMG(void) { stop(); }
	void setDuty(const unsigned int nDutyValue);
	void setDuty(const float fDutyRate);

	bool isRunning(void) const { return _bIsRunning; }
	bool isAlarm(void) const { return _bIsAlarm; }

	unsigned int getDutyValue(void) const { return _u16SetDutyValue; }
	float getDutyRate(void) const { return _fSetDutyRate; }

#ifdef _MEASURE_PRESENT_RPM_
	float getPresentRPM(void) const;
#endif

private:
	unsigned char _nSpeedPin;
	unsigned char _nStopPin;
	unsigned char _nResetPin;
	unsigned char _nSpdoutPin;
	unsigned char _nAlarmPin;

	bool _bIsAlarm;
	bool _bIsRunning;

	unsigned int _u16SetDutyValue;
	float _fSetDutyRate;

	static unsigned int _nLowerDutyValue;
	static unsigned int _nUpperDutyValue;

#ifdef _USE_MOTOR_PUMP_ALARM_PROC_MODE_
	void (*_postAlarm)(void);
#endif

#ifdef _MEASURE_PRESENT_RPM_
	volatile int _nLast_signal;
	volatile unsigned int _u16PulseCNT;

	unsigned int _nNum_of_samples;
	unsigned int _nIndex_of_sample;

	float _fPresentMotorRPM;
	float _fSum_of_rpm;
	float _fRPMSample[NUM_OF_RPM_SAMPLES];

	StopWatch* _phStopwatch;

	void measure_PresentMotorRPM(void);
	void calculate_MotorRPM(const unsigned long t_duration);

	void start_StopWatch(void);
	void restart_StopWatch(void);
	void stop_StopWatch(void);
#endif
};

#endif //_CMOTORPUMP_H_

