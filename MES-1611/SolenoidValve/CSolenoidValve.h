#ifndef _CSOLENOID_VALVE_H_
#define _CSOLENOID_VALVE_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


using namespace std;


// #define _USE_SOLENOID_VALVE_PROC_MODE_


#define SV_CLOSE									(0)
#define SV_OPEN										(1)
#define SV_CMD_NUM									(2)


class CSolenoidValve
{
	friend bool operator == (CSolenoidValve& obj1, CSolenoidValve& obj2) { return &obj1 == &obj2; }

public:
	enum SVType {NORMAL_CLOSE=0, NORMAL_OPEN};

	CSolenoidValve(const unsigned char nSignalPin, const enum SVType nType = NORMAL_CLOSE);

	void init(const unsigned char nSVState);

	void open(void);
	void close(void);

#ifdef _USE_SOLENOID_VALVE_PROC_MODE_
	void setPreProcessing(void (*preProcessing)(void));
	void setPostProcessing(void (*postProcessing)(void));

	void openProc(void);
	void closeProc(void);
#endif

	bool isOpening(void) const { return _bIsOpened; }

private:
	unsigned char _nSignalPin;

	unsigned char _nSVCmd[SV_CMD_NUM];

	bool _bIsOpened;

#ifdef _USE_SOLENOID_VALVE_PROC_MODE_
    // preProcessing callback function; gets called before a Solenoid Valve has been opened/closed.
	void (*_preProcessing)(void);
    // postProcessing callback function; gets called after a Solenoid Valve has been opened/closed.
    void (*_postProcessing)(void);
#endif
};

#endif //_CSOLENOID_VALVE_H_

