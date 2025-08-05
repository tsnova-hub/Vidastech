#ifndef _CESPRESSO_H_
#define _CESPRESSO_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>
// #include <string>
// #include <vector>
// #include <queue>
// #include <stack>
// #include <iterator>

// #include <cmath>
// #include <math.h>


#include <StopWatch.h>


#include "../MES_Config.h"
#include "../MES_Hardware_Map.h"
#include "../MES_Peripheral_Interface.h"
#include "../MES_DataSet_Definition.h"

#include "../MES_SetupMenu.h"


using namespace std;


#ifndef _FlashString
	#define _FlashString 										__FlashStringHelper		// char
#endif


#if defined(_USE_APPLICATION_DEBUG_MSG_)
	#define _USE_ESPRESSO_DEBUG_MSG_
	#define LOG_BREWING_OBSERBATION_PERIOD						(500UL)					// < 1 sec
#else
	#if defined(_USE_BREW_PROFILE_ACQUISITION_)
		#define LOG_BREWING_OBSERBATION_PERIOD					(500UL)					// < 1 sec
	#endif
#endif


#define TIME_FOR_PRESSURE_STABILIZATION 						(200UL)					// < 200 msec (candidate : 180UL)

#define MIN_BREWING_FLOWCNT 									(2UL)					// >= 1 ml
#define MAX_BREWING_FLOWCNT										(4000UL)				// <= 2000 ml
#define MIN_BREWING_TIME 										(1*1000UL)				// >= 1 sec
#define MAX_BREWING_TIME										(6*60000UL)				// <= 6 min

#define MIN_INJECTION_FLOWCNT 									(0L)
#define MAX_INJECTION_FLOWCNT									(MAX_BREWING_FLOWCNT/2UL)
#define MIN_EXTRACTION_FLOWCNT 									(MIN_BREWING_FLOWCNT)
#define MAX_EXTRACTION_FLOWCNT									(MAX_BREWING_FLOWCNT/2UL)

#define MIN_INJECTION_TIME 										(0UL)
#define MAX_INJECTION_TIME										(MAX_BREWING_TIME/2UL)
#define MIN_EXTRACTION_TIME 									(MIN_BREWING_TIME)
#define MAX_EXTRACTION_TIME										(MAX_BREWING_TIME/2UL)

#define MIN_EFFECTIVE_BREW_TIME_FOR_FLUSHING_DETECTION			(10.5F)					// <= 10.5 sec
#define MIN_EFFECTIVE_BREW_PRES_FOR_FLUSHING_DETECTION			(2.5F)					// <= 2.5 bar

#define OBSERVATION_DURATION_FOR_FLUSHING_DETECTION				(2000UL)				// 2000 msec, ex: 5 ~ 7 sec @ set = 5 sec 


class CEspresso: public StopWatch
{
public:
	enum BrewMode {BREW_MODE=0, SETUP_MODE, FREE_MODE};
	enum BrewBase {BREW_FLOWCNT=0, BREW_TIMECNT};
	enum BrewState {BREW_STOP=0, BREW_INJECTION, BREW_EXTRACTION, BREW_FINISH};

	CEspresso();

	void init(stDataSet_t* pstDataSet);

	void setIdleProcessing(void (*callback)(void));

	void prepareToBrew(void);
	void postBrew(void);
	bool prepareToSave(void);

	void startBrewing(void);
	void stopBrewing(void);
	enum BrewState onBrewing(void);

	void setBrewMode(enum BrewMode nBrewMode) { _BrewMode = nBrewMode; }
	void setBrewBase(enum BrewBase nBrewBase) { _BrewBase = nBrewBase; }
	void setBrewState(enum BrewState nBrewState) { _BrewState = nBrewState; _bManualStateTransition = true; }

	enum BrewMode getBrewMode(void) const { return _BrewMode; }
	enum BrewBase getBrewBase(void) const { return _BrewBase; }
	enum BrewState getBrewState(void) const { return _BrewState; }

	bool isTimeover(void) const { return _bIsTimeover; }
	bool isFlushingDetected(void) const { return _bIsFlushingDetected; }
	bool isPrepared(void) const { return _bIsBrewPrepared; }
	bool isChanged(void) const { return _bIsStateChanged; }

	float getInjectionPower(void) const { return _fInjectionPower; }
	float getExtractionPower(void) const { return _fExtractionPower; }

	void setProgramNum(const unsigned char nProgramNum) { _nProgramNum = nProgramNum; }
	unsigned char getProgramNum(void) const { return _nProgramNum; }

	void clearProgramSet(void);
	float getInjectionSet(enum BrewBase nBrewBase) const { return _fInjectionSet[nBrewBase]; }
	float getExtractionSet(enum BrewBase nBrewBase) const { return _fExtractionSet[nBrewBase] - _fInjectionSet[nBrewBase]; }
	float getMaxBrewingSet(enum BrewBase nBrewBase) const { return _fExtractionSet[nBrewBase]; }

	bool invalidateCondition(const unsigned int nInjectionFlow, const unsigned int nExtractionFlow, const float fInjectionTime, const float fExtractionTime);

private:
	enum BrewMode _BrewMode;
	enum BrewBase _BrewBase;
	enum BrewState _BrewState;

	bool _bIsTimeover;
	bool _bIsFlushingDetected;
	bool _bIsBrewPrepared;
	bool _bIsStateChanged;
	bool _bManualStateTransition;

	stDataSet_t* _pstDataSet;

	unsigned char _nProgramNum;

	float _fInjectionSet[2];
	float _fExtractionSet[2];

	float _fInjectionSetQueue[2];
	float _fExtractionSetQueue[2];

	float _fInjectionPower;
	float _fExtractionPower;

	void clear_Condition(void);
	void clear_ConditionQueue(void);
	void load_Condition(void);
	void update_Condition(void);
	void clear_Observation(void);
	void update_Observation(void);

	float calculate_AveragedFlowRate(const bool bReset);
	float calculate_AveragedBrewPressure(const bool bReset);
#if PUMP_PRESSURE_SENSOR_ATTACHED
	float calculate_AveragedPumpPressure(const bool bReset);
#endif

	void start_Brewing(void);
	void stop_Brewing(void);

	void stabilize_BrewPressure(void);

	void (*_idleProcessing)(void);

#ifdef _USE_ESPRESSO_DEBUG_MSG_
	unsigned long _ulLogPrintTime;
	void print_PreparedBrewCondition(const _FlashString* pstrHeader);
	void print_BrewingInitObservation(const bool bStandby);
	void print_BrewingObservation(void);
	void print_PostBrewCondition(const _FlashString* pstrHeader);
	void print_UpdatedBrewCondition(const _FlashString* pstrHeader);
#else
	#ifdef _USE_BREW_PROFILE_ACQUISITION_
	unsigned long _ulLogPrintTime;
	void print_PreparedBrewCondition(void);
	void print_BrewingInitObservation(const bool bStandby);
	void print_BrewingObservation(void);
	void print_PostBrewCondition(void);
	#endif
#endif
};


#endif //_CESPRESSO_H_

