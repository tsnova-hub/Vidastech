#ifndef _COPTICAL_FIBER_H_
#define _COPTICAL_FIBER_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


using namespace std;


// Configuration of Optical Fiber Amp BF4RP
// - Sensibility : 'Maximum' set or 'calibration' each
// - Detection : 'Light ON' mode
// - OFF Delay : 'OFF' mode
// - Set / Lock : 'Lock' mode


// #define _USE_OPTICAL_FIBER_PROC_MODE_


#define USE_NPN_TYPE_OPTICAL_FIBER_AMP				(false)				// BF4R : NPN, BF4RP : PNP

#define OPTICAL_FIBER_AMP_RESET_DURATION			(250UL)				// > 250 msec

#define DEFAULT_SIGNAL_DURATION						(1000UL)			// 1000 msec


class COpticalFiber
{
	friend bool operator == (COpticalFiber& obj1, COpticalFiber& obj2) { return &obj1 == &obj2; }

public:
	enum ActiveType {DARK_ON=0, LIGHT_ON};

	COpticalFiber(const unsigned char nPowerPin, const unsigned char nSignalPin, const unsigned char nDiagnosisPin, const enum ActiveType nActiveType = LIGHT_ON);

	void init(void);
	void turnon(void);
	void turnoff(void);
	// void powerOnReset(void);

	void setActiveType(const enum ActiveType nActiveType);
	void setDuration(const unsigned long ulDarkDuration, const unsigned long ulLightDuration);

	int readSignal(void);										// directly, read the signal from optical fiber amplifier
	void pollingSignal(void);									// periodically, read and evaluate the signal from optical fiber amplifier
	void diagnosis(void);										// diagnose the stability of optical fiber amplifier after read the signal from optical fiber

#ifdef _USE_OPTICAL_FIBER_PROC_MODE_
	void setDarkProcessing(void (*darkProcessing)(void));
	void setLightProcessing(void (*lightProcessing)(void));
#endif

	int getSignal(void) const { return _nCurrSignal; }			// get the value of signal from optical fiber amplifier
	int Light(void) const { return _nLight; }					// get the logic value of 'Light' state according to a type of optical fiber amplifier
	int Dark(void) const { return _nDark; }						// get the logic value of 'Dark' state according to a type of optical fiber amplifier

	void enable(void) { _bIsEnabled = true; }
	void disable(void) { _bIsEnabled = false; }

	bool isRunning(void) const { return _bIsRunning; }
	bool isEnabled(void) const { return _bIsEnabled; }
	bool isUnstable(void) const { return _bIsUnstable; }

	// bool isLight(void) const { return _nCurrSignal == _nLight; }
	// bool isDark(void) const { return _nCurrSignal == _nDark; }

private:
	unsigned char _nPowerPin;
	unsigned char _nSignalPin;
	unsigned char _nDiagnosisPin;

	bool _bIsRunning;
	bool _bIsEnabled;
	bool _bIsUnstable;

	int _nLight;
	int _nDark;

	volatile int _nCurrSignal;
	volatile int _nLastSignal;

	volatile unsigned long _ulDarkOnTime = 0ul;
	volatile unsigned long _ulLightOnTime = 0ul;

	unsigned long _ulDarkDuration;
	unsigned long _ulLightDuration;

#ifdef _USE_OPTICAL_FIBER_PROC_MODE_
    // Processing callback function; gets called when a Optical Fiber Signal is 'dark'.
	void (*_darkProcessing)(void);
    // Processing callback function; gets called when a Optical Fiber Signal is 'light'.
    void (*_lightProcessing)(void);
#endif
};


#endif //_COPTICAL_FIBER_H_

