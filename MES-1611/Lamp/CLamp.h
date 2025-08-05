#ifndef _CLAMP_H_
#define _CLAMP_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <StopWatch.h>


#include "../MES_Config.h"


using namespace std;


// #if defined(_USE_APPLICATION_DEBUG_MSG_)
// 	#define _TEST_CLAMP_
// #endif


#define MIN_INTENSITY_RATE							(0.25F)				// >  6.0 V
#define LOWER_INTENSITY_RATE						MIN_INTENSITY_RATE
#define UPPER_INTENSITY_RATE 						(0.50F)				// = 12.0 V
#define MAX_INTENSITY_RATE							(1.00F) 			// = 24.0 V (DO NOT USE 24.0V because it use lamp device for 12.0V)

#define LAMP_TURN_OFF 								(0UL)
#define LAMP_INTENSITY_LOW							((unsigned int)(LOWER_INTENSITY_RATE * 255UL))
#define LAMP_INTENSITY_HIGH							((unsigned int)(UPPER_INTENSITY_RATE * 255UL))

#define DELAY_TIME_TO_TURNOFF						(1500UL)


class CLamp: public StopWatch
{
public:
	enum LampMode {TURNOFF_ALWAYS=0, TURNON_SHOT, TURNON_ALWAYS};
	enum LampIntensity {LOW_INTENSITY=0, HIGH_INTENSITY};

	CLamp(const unsigned char nLampPin);

	void init(void);

	void turnon(void);
	void turnoff(void);
	void turnoffDELAYED(void);
	void turnoffEMG(void);

	void setMode(const enum LampMode eMode);
	void setIntensity(const enum LampIntensity eIntensity);

	bool isTurnon(void) const { return _bIsLampOn; }
	enum LampMode getMode(void) const { return _eLampMode; }
	enum LampIntensity getIntensity(void) const { return _eLampIntensity; }

#ifdef _TEST_CLAMP_
	void printStatus(void);
#endif

private:
	unsigned char _nLampPin;

	bool _bIsLampOn;
	enum LampMode _eLampMode;
	enum LampIntensity _eLampIntensity;
};

#endif //_CLAMP_H_

