

#include "CRTClock.h"


#if defined(CONTROLLINO_MINI) || defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
#define _init_rtc(_cs)										Controllino_RTC_init(_cs)
#define _set_timedate(_d,_w,_m,_y,_hh,_mm,_ss)				Controllino_SetTimeDate(_d, _w, _m, _y, _hh, _mm, _ss)
#define _get_timedate(_pd,_pw,_pm,_py,_phh,_pmm,_pss)		Controllino_ReadTimeDate(_pd, _pw, _pm, _py, _phh, _pmm, _pss)
#else
#define _init_rtc
#define _set_timedate(_d,_w,_m,_y,_hh,_mm,_ss)				(-1)
#define _get_timedate(_pd,_pw,_pm,_py,_phh,_pmm,_pss)		(-1)
#endif


CRTClock::CRTClock()
{
	clear();
}

void CRTClock::init(const unsigned char nChipSelect)
{
	_init_rtc(nChipSelect);
}

void CRTClock::clear(void)
{
	memset(&_stTimeDateRTC, 0x00, sizeof(stTimeDate_t));
}

void CRTClock::getTimeDate(stTimeDate_t& stTimeDate)
{
	stTimeDate.day = _stTimeDateRTC.day;
	stTimeDate.weekday = _stTimeDateRTC.weekday;
	stTimeDate.month = _stTimeDateRTC.month;
	stTimeDate.year = _stTimeDateRTC.year;
	stTimeDate.hour = _stTimeDateRTC.hour;
	stTimeDate.minute = _stTimeDateRTC.minute;
	stTimeDate.second = _stTimeDateRTC.second;

#if defined(_RTC_DEBUG_MSG_)
	print_TimeDate();
#endif // _RTC_DEBUG_MSG_
}

char CRTClock::setTimeDate(const stTimeDate_t& stSetTimeDate)
{
	unsigned char weekday = get_DayOfWeek(stSetTimeDate);

	if (_set_timedate(stSetTimeDate.day, weekday, stSetTimeDate.month, stSetTimeDate.year, stSetTimeDate.hour, stSetTimeDate.minute, stSetTimeDate.second) < 0) {

#if defined(_RTC_DEBUG_MSG_)		
		cout << F("RTC chip was not initialized properly.\n\r") << endl;
#endif // _RTC_DEBUG_MSG_

		return -1;

	}

	return 0;
}

enum CRTClock::WeekDay CRTClock::getWeekday(const stTimeDate_t& stTimeDate)
{
	enum CRTClock::WeekDay eWeekday;

	eWeekday = static_cast<enum CRTClock::WeekDay>(get_DayOfWeek(stTimeDate));

	return eWeekday;
}

char CRTClock::readTimeDate(void)
{
	unsigned char day, weekday, month, year, hour, minute, second;

	if (_get_timedate(&day, &weekday, &month, &year, &hour, &minute, &second) >= 0) {

		_stTimeDateRTC.day = day;
		_stTimeDateRTC.weekday = weekday;
		_stTimeDateRTC.month = month;
		_stTimeDateRTC.year = year;
		_stTimeDateRTC.hour = hour;
		_stTimeDateRTC.minute = minute;
		_stTimeDateRTC.second = second;

		return 0;

	}
	else {

#if defined(_RTC_DEBUG_MSG_)
		cout << F("RTC chip : failed to read.\n\r") << endl;
#endif // _RTC_DEBUG_MSG_

		return -1;

	}
}

// Notice : Gregorian dates only (began on 14, 9, 1752)
#if 1
// Optimized implementation
unsigned char CRTClock::get_DayOfWeek(const stTimeDate_t& stTimeDate)
{
	static int _nMonthCode[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

	int nYear = stTimeDate.year + RTC_YEAR_PREFIX;

	nYear -= stTimeDate.month < 3;

	return (nYear + int(nYear / 4) - int(nYear / 100) + int(nYear / 400) + _nMonthCode[abs(stTimeDate.month-1)] + stTimeDate.day) % 7;
}
#else
// Theoretical implementation
unsigned char CRTClock::get_DayOfWeek(const stTimeDate_t& stTimeDate)
{
	static int _nMonthCode[] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5};
	static int _nCenturyCode[] = {4, 2, 0, 6, 4, 2, 0};

	int nDay = stTimeDate.day;
	int nMonth = _nMonthCode[stTimeDate.month-1];
	int nYear = (stTimeDate.year + int(stTimeDate.year / 4)) % 7;

	int nCentury = _nCenturyCode[int(RTC_YEAR_PREFIX / 100) - 17];
	int nLeapYear = 0;

	if (stTimeDate.month < 3) {

		int year = stTimeDate.year + RTC_YEAR_PREFIX;

		nLeapYear = !(year % 4) && (year % 100) ? 1 : (!(year % 400) ? 1 : 0);

	}

	return (nYear + nMonth + nDay + nCentury - nLeapYear) % 7;
}
#endif

#if defined(_RTC_DEBUG_MSG_)
void CRTClock::print_TimeDate(void)
{
	cout << F("\n\rTime and Date: ");
	cout << int(_stTimeDateRTC.year) << "/" << int(_stTimeDateRTC.month) << "/" << int(_stTimeDateRTC.day) << "   ";
	cout << int(_stTimeDateRTC.hour) << ":" << int(_stTimeDateRTC.minute) << ":" << int(_stTimeDateRTC.second) << endl;
}
#endif // _RTC_DEBUG_MSG_

