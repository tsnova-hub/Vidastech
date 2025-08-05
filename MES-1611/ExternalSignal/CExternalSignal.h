#ifndef _CEXTERNAL_SIGNAL_H_
#define _CEXTERNAL_SIGNAL_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


using namespace std;


// #define _USE_EXTERNAL_SIGNAL_PROC_MODE_


#ifndef __U8_MAX__
#define __U8_MAX__									(255UL)
#endif

#ifndef __U16_MAX__
#define __U16_MAX__									(65535UL)
#endif


class CExternalSignal
{
	friend bool operator == (CExternalSignal& obj1, CExternalSignal& obj2) { return &obj1 == &obj2; }
	
public:
	enum ActiveSigType {ACTIVE_LOW=0, ACTIVE_HIGH};

	CExternalSignal(const unsigned char nSignalPin, const enum ActiveSigType nSigType = ACTIVE_HIGH);

	void clear(void);

	void pollingSignal(void);

#ifdef _USE_EXTERNAL_SIGNAL_PROC_MODE_
	void setPreProcessing(void (*preProcessing)(void));
	void setPostProcessing(void (*postProcessing)(void));
#endif

	int getSignal(void) const { return _nCurrSignal; }
	unsigned int getSignalCNT(void) const { return _u16SignalCNT; }

private:
	unsigned char _nSignalPin;

	int _nActiveSignal;

	volatile int _nCurrSignal;
	volatile int _nLastSignal;
	volatile unsigned int _u16SignalCNT;

	static const unsigned int _nMaxCount = __U16_MAX__;

#ifdef _USE_EXTERNAL_SIGNAL_PROC_MODE_
    // preProcessing callback function; gets called before a External Signal has been received.
	void (*_preProcessing)(void);
    // postProcessing callback function; gets called after a External Signal has been received.
    void (*_postProcessing)(void);
#endif
};


#endif //_CEXTERNAL_SIGNAL_H_

