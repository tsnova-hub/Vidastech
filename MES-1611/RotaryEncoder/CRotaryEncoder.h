#ifndef _CROTARYENCODER_H_
#define _CROTARYENCODER_H_


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


#include <StopWatch.h>


// #if defined(_USE_APPLICATION_DEBUG_MSG_)
	// #define _TEST_CROTARYENCODER_
// #endif


#ifdef _ROTARY_ENCODER_ACCELERATION_
	#define RE_PULSE_PER_REVOLUTION					(12)
	#define RE_RPM_LIMIT							500.0F
	#define RE_RPM_OFFSET							0
	#define RE_ACC_RANGE							50
	#define RE_ACC_MULTIPLE							5
#endif


// #define _MODULO_DIRECTION_COUNT_
#ifdef _MODULO_DIRECTION_COUNT_
	#define MODULO_NUM								(4)
#endif

#define ENCODER_CW_ROTATION 						1
#define ENCODER_CCW_ROTATION						-1
#define ENCODER_NO_ROTATION							0

#define SW_PUSHED									HIGH
#define SW_PULLED									LOW


// #define TIME_SCLICKED_SWITCH						250UL 	// > 250 millisec
// #define TIME_DCLICKED_SWITCH						250UL	// < 250 millisec
#define TIME_SCLICKED_SWITCH						150UL 	// > 150 millisec
#define TIME_DCLICKED_SWITCH						150UL	// < 150 millisec
#define TIME_LPUSHING_SWITCH						2000UL	// 2000 millisec (2 sec)
#define TIME_LCLICKED_SWITCH						2000UL	// 2000 millisec (2 sec)
#define TIME_VCLICKED_SWITCH						4200UL	// 4000 millisec (4 sec)

#define SWITCH_NOT_PUSHED							-2
#define SWITCH_ON_PUSHING							-1
#define SWITCH_ON_LPUSHING							0
#define SWITCH_PUSHED_SHORT							1
#define SWITCH_PUSHED_LONG							2
#define SWITCH_PUSHED_VLONG							3
#define SWITCH_PUSHED_DCLK							4


#define SHORT_PUSHED_MASK							0b0001
#define LONG_PUSHED_MASK							0b0010
#define VLONG_PUSHED_MASK							0b0100
#define DOUBLE_PUSHED_MASK							0b1000

#define INIT_SWITCH_STATE							(SHORT_PUSHED_MASK)
#define LONG_VLONG_PUSHED_MASK						(LONG_PUSHED_MASK | VLONG_PUSHED_MASK)
#define SHORT_LONG_VLONG_PUSHED_MASK				(SHORT_PUSHED_MASK | LONG_PUSHED_MASK | VLONG_PUSHED_MASK)


#ifndef TIME_TO_MASK_FALSE_EVENT
	#define TIME_TO_MASK_FALSE_EVENT				(250UL)
#endif


class CRotaryEncoder: public StopWatch
{
	friend bool operator == (CRotaryEncoder& obj1, CRotaryEncoder& obj2) { return &obj1 == &obj2; }

public:
	CRotaryEncoder(const unsigned char nSTpin, const unsigned char nDIpin, const unsigned char nSWpin);

	void init(void);
	
	void startACC(void);
	void stopACC(void);

	void getRotaryEncoderDirectionState(void);
	void getRotaryEncoderSwitchState(void);

	int getSwitch(void) const;
	int getDirection(void);

	int getAccFactor(const unsigned int nSensibility = 1);

	bool isSwitchEventOccured(void);
	bool isDirectionEventOccured(void);

	bool isSwitchMaskedEventOccured(void);
	bool isDirectionMaskedEventOccured(void);

	void startMaskingEvent(const unsigned char nMask);
	void stopMaskingEvent(const unsigned char nMask) { _bEventMask[nMask] = true; }

	void clearSwitch(void);
	void clearDirection(void);
	void clearRotaryEncoderAll(void);

	void disableDirectionEvent(void) { _bDisableDirectionEvent = true; }
	void enableDirectionEvent(void) { _bDisableDirectionEvent = false; }
	bool isDirectionEventDisabled(void) const { return _bDisableDirectionEvent; }

	void disableRotaryEncoder(void) { _bDisableRotaryEncoder = true; }
	void enableRotaryEncoder(void) { _bDisableRotaryEncoder = false; }
	bool isRotaryEncoderDisabled(void) const { return _bDisableRotaryEncoder; }

private:
	unsigned char _nSTpin;
	unsigned char _nDIpin;
	unsigned char _nSWpin;

	volatile int _nPastSwitch;

	volatile int _nDirection;
	volatile int _nSwitch;

	volatile bool _bDirectionEvent;
	volatile bool _bSwitchEvent;

	volatile int _nAccelerationFactor;

	bool _bDisableRotaryEncoder;
	bool _bDisableDirectionEvent;


	bool _bEventMask[2];							// 0 : Switch, 1 : Direction
	unsigned long _ulMaskingStartTime[2];			// 0 : Switch, 1 : Direction


	volatile unsigned char _u8SwitchState;

	bool _bGuess2DClick;
	bool _bGuess2LClick;

	unsigned long _ulTime_last_pushed;
	unsigned long _ulTime_last_pulled;


	void update_Direction(const int nDirection) { _nDirection = nDirection; }
	void set_DirectionEvent(void) { _bDirectionEvent = true; }
	void clear_DirectionEvent(void) { _bDirectionEvent = false; }

	void update_Switch(const int nSwitch) { _nSwitch = nSwitch; }
	void set_SwitchEvent(void) { _bSwitchEvent = true;/* stopACC();*/ }
	void clear_SwitchEvent(void) { _bSwitchEvent = false; }
	bool is_SwitchEventSet(void) const { return _bSwitchEvent; }

#ifdef _ROTARY_ENCODER_ACCELERATION_
	float calculate_RotaryEncoderRPM(const unsigned long t_duration);
	void set_RotaryAccelerationFactor(void);
#endif
};


#endif //_CROTARYENCODER_H_

