#ifndef _CRTClOCK_H_
#define _CRTClOCK_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <Controllino.h>


using namespace std;


// #define _RTC_DEBUG_MSG_

#define RTC_YEAR_PREFIX									(2000)

typedef struct stTimeDateRTC_ {
	unsigned char day;
	unsigned char weekday; 		// weekday = SUN(0), MON(1), TUE(2),...,SAT(6)
	unsigned char month;
	unsigned char year;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
} stTimeDate_t;


class CRTClock
{
public:
	enum WeekDay {SUN=0, MON, TUE, WED, THU, FRI, SAT};

	CRTClock();

	void init(const unsigned char nChipSelect = 0x00);
	void clear(void);

	char readTimeDate(void);

	void getTimeDate(stTimeDate_t& stTimeDate);
	char setTimeDate(const stTimeDate_t& stSetTimeDate);

	enum WeekDay getWeekday(const stTimeDate_t& stTimeDate);

private:
	stTimeDate_t _stTimeDateRTC;

	unsigned char get_DayOfWeek(const stTimeDate_t& stTimeDate);

#if defined(_RTC_DEBUG_MSG_)
	void print_TimeDate(void);
#endif // _RTC_DEBUG_MSG_
};


#endif //_CRTClOCK_H_

