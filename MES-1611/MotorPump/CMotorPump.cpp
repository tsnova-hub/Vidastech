

#include "CMotorPump.h"


#if defined(_TEST_CMOTORPUMP_)

static inline void printStatus(const unsigned int nSetDutyValue, const float fSetDutyRate, const unsigned int nUpperDutyValue)
{
	cout.precision(6);	// ex) 3 : 0.123, 6 : 0.123456

	cout << F("(Motor Duty Set:: ") << fSetDutyRate 
		 << F(" PWM:: ") << nSetDutyValue 
		 << F(" Rate:: ") << float(nSetDutyValue / float(nUpperDutyValue)) 
		 << F(")") << endl;

	cout.precision(FLOAT_POINT_PRECISION_FOR_DATA_ACQ);
}

#endif


#if defined(_CMOTORPUMP_H_)

static inline unsigned int calc_DutyValue(const float fRate, const unsigned int nUpperValue)
{
	float fDutyRate = constrain(fRate, 0.0F, 1.0F);

	return (unsigned int)(fDutyRate * nUpperValue);
}

static inline float calc_DutyRate(const unsigned int nValue, const unsigned int nUpperValue)
{
	float fDutyPercentage = round((nValue / float(nUpperValue)) * 100ul);

	return float(fDutyPercentage / 100.0f);
}

#endif


unsigned int CMotorPump::_nLowerDutyValue = _MIN_DUTY_VALUE_;
unsigned int CMotorPump::_nUpperDutyValue = _MAX_DUTY_VALUE_;


CMotorPump::CMotorPump(const unsigned char nSpeedPin, const unsigned char nStopPin, const unsigned char nResetPin, const unsigned char nAlarmPin, const unsigned char nSpdoutPin)
{
	_nSpeedPin = nSpeedPin;
	_nStopPin = nStopPin;
	_nResetPin = nResetPin;
	_nAlarmPin = nAlarmPin;
	_nSpdoutPin = nSpdoutPin;

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
	_postAlarm = NULL;
#endif

	pinMode(_nSpeedPin, OUTPUT);
	analogWrite(_nSpeedPin, 0UL);

	pinMode(_nStopPin, OUTPUT);
	digitalWrite(_nStopPin, LOW);

	pinMode(_nResetPin, OUTPUT);
	digitalWrite(_nResetPin, LOW);

	pinMode(_nAlarmPin, INPUT_PULLUP);
	pinMode(_nSpdoutPin, INPUT_PULLUP);

#if defined(_MEASURE_PRESENT_RPM_)
	_phStopwatch = new StopWatch(StopWatch::MICROS);
#endif
}

CMotorPump::~CMotorPump()
{
#if defined(_MEASURE_PRESENT_RPM_)
	delete _phStopwatch;
#endif
}

void CMotorPump::clear(void)
{
	_bIsAlarm = false;
	_bIsRunning = false;

	_u16SetDutyValue = 0UL;
	_fSetDutyRate = 0.0f;

#if defined(_MEASURE_PRESENT_RPM_)
	_u16PulseCNT = 0;
	_fPresentMotorRPM = 0.0f;

	_nNum_of_samples = 0;
	_nIndex_of_sample = 0;
	_fSum_of_rpm = 0.0f;
	
	memset(_fRPMSample, 0.0f, sizeof(float)*NUM_OF_RPM_SAMPLES);
#endif
}

void CMotorPump::setDutyRange(const float fLowerRate, const float fUpperRate)
{
	float fLower = constrain(fLowerRate, _MIN_DUTY_RATE_, _MAX_DUTY_RATE_);
	float fUpper = constrain(fUpperRate, _MIN_DUTY_RATE_, _MAX_DUTY_RATE_);

	CMotorPump::_nLowerDutyValue = (unsigned int)(fLower * _MAX_DUTY_VALUE_);
	CMotorPump::_nUpperDutyValue = (unsigned int)(fUpper * _MAX_DUTY_VALUE_);
}

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
void CMotorPump::setPostAlarm(void (*postAlarm)(void))
{
	if (postAlarm != NULL) {

		_postAlarm = postAlarm;

	}
}
#endif

void CMotorPump::pollingAlarm(void)
{
	int nAlarm;

	nAlarm = digitalRead(_nAlarmPin);

	if (!nAlarm) {

		_bIsAlarm = true;

		if (_bIsRunning == true) {

			stop();

		}

#if defined(_RESET_ON_ALARM_)
		reset();
#endif

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
		if (_postAlarm) {

			_postAlarm();

		}
#endif

	}
	else {

		_bIsAlarm = false;

	}
}

void CMotorPump::reset(void)
{
	unsigned long ulStartTime;

	ulStartTime = millis();

	digitalWrite(_nResetPin, HIGH);

	while (millis() - ulStartTime < MOTOR_RESET_DURATION);

	digitalWrite(_nResetPin, LOW);
}

void CMotorPump::stop(void)
{
	analogWrite(_nSpeedPin, 0UL);

	digitalWrite(_nStopPin, LOW);

	_bIsRunning = false;

	_u16SetDutyValue = 0UL;

	_fSetDutyRate = 0.0F;

#if defined(_MEASURE_PRESENT_RPM_)
	if (_phStopwatch->isRunning())

		_phStopwatch->stop();
#endif
}

void CMotorPump::start(void)
{
	if (_bIsRunning) 

		return;

	analogWrite(_nSpeedPin, CMotorPump::_nUpperDutyValue);

	digitalWrite(_nStopPin, HIGH);

	_bIsRunning = true;

	_u16SetDutyValue = CMotorPump::_nUpperDutyValue;

	_fSetDutyRate = _MAX_DUTY_RATE_;

#if defined(_TEST_CMOTORPUMP_)
	printStatus(_u16SetDutyValue, _fSetDutyRate, CMotorPump::_nUpperDutyValue);
#endif
}

void CMotorPump::start(const unsigned int nDutyValue)
{
	if (_bIsRunning)

		return;

	_u16SetDutyValue = constrain(nDutyValue, CMotorPump::_nLowerDutyValue, CMotorPump::_nUpperDutyValue);

	analogWrite(_nSpeedPin, _u16SetDutyValue);

	digitalWrite(_nStopPin, HIGH);

	_bIsRunning = true;

	_fSetDutyRate = calc_DutyRate(_u16SetDutyValue, CMotorPump::_nUpperDutyValue);

#if defined(_TEST_CMOTORPUMP_)
	printStatus(_u16SetDutyValue, _fSetDutyRate, CMotorPump::_nUpperDutyValue);
#endif
}

void CMotorPump::start(const float fDutyRate)
{
	if (_bIsRunning)

		return;
	
	_u16SetDutyValue = calc_DutyValue(fDutyRate, CMotorPump::_nUpperDutyValue);

	analogWrite(_nSpeedPin, _u16SetDutyValue);

	digitalWrite(_nStopPin, HIGH);

	_bIsRunning = true;

	_fSetDutyRate = fDutyRate;//calc_DutyRate(_u16SetDutyValue, CMotorPump::_nUpperDutyValue);

#if defined(_TEST_CMOTORPUMP_)
	printStatus(_u16SetDutyValue, _fSetDutyRate, CMotorPump::_nUpperDutyValue);
#endif
}

void CMotorPump::setDuty(const unsigned int nDutyValue)
{
	_u16SetDutyValue = constrain(nDutyValue, CMotorPump::_nLowerDutyValue, CMotorPump::_nUpperDutyValue);

	analogWrite(_nSpeedPin, _u16SetDutyValue);

	_fSetDutyRate = calc_DutyRate(_u16SetDutyValue, CMotorPump::_nUpperDutyValue);

#if defined(_TEST_CMOTORPUMP_)
	printStatus(_u16SetDutyValue, _fSetDutyRate, CMotorPump::_nUpperDutyValue);
#endif
}

void CMotorPump::setDuty(const float fDutyRate)
{
	_u16SetDutyValue = calc_DutyValue(fDutyRate, CMotorPump::_nUpperDutyValue);

	analogWrite(_nSpeedPin, _u16SetDutyValue);

	_fSetDutyRate = fDutyRate;//calc_DutyRate(_u16SetDutyValue, CMotorPump::_nUpperDutyValue);

#if defined(_TEST_CMOTORPUMP_)
	printStatus(_u16SetDutyValue, _fSetDutyRate, CMotorPump::_nUpperDutyValue);
#endif
}


#if defined(_MEASURE_PRESENT_RPM_)
float CMotorPump::getPresentRPM(void) const
{
	float fPresentRPM;

	noInterrupts();

	fPresentRPM = _fPresentMotorRPM;

	interrupts();

	return fPresentRPM;
}

void CMotorPump::measure_PresentMotorRPM(void)
{
	// MOTORPUMP_SPDOUT_TRIGGER_TYPE == RISING OR FALLING

	if (!_phStopwatch->isRunning()) {

		_phStopwatch->reset();
		_phStopwatch->start();

	} else {
		
		unsigned long time_duration = _phStopwatch->elapsed();

		_phStopwatch->reset();
		_phStopwatch->start();

		_u16PulseCNT++;

		calculate_MotorRPM(time_duration);

	}
}

void CMotorPump::calculate_MotorRPM(const unsigned long t_duration)
{
	if (t_duration > 0UL) {

		_fSum_of_rpm -= _fRPMSample[_nIndex_of_sample];

		_fRPMSample[_nIndex_of_sample] = 60UL * (1000000UL / float(t_duration * MOTOR_PPR));

		_fSum_of_rpm += _fRPMSample[_nIndex_of_sample];

		_nIndex_of_sample += 1;
		_nIndex_of_sample %= NUM_OF_RPM_SAMPLES;
		
		if (_nNum_of_samples < NUM_OF_RPM_SAMPLES) {

			_nNum_of_samples++;
			
		}

		_fPresentMotorRPM = _fSum_of_rpm / float(_nNum_of_samples);
		
	}
}
#endif //_MEASURE_PRESENT_RPM_

