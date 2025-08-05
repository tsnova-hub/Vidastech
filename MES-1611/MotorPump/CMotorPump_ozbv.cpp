

#include "CMotorPump_ozbv.h"


#if defined(_TEST_CMOTORPUMP_BLDC_)

static inline void printSpeed(const unsigned int nSetRPMValue, const float fSetRPMRate)
{
	cout.precision(6);	// ex) 3 : 0.123, 6 : 0.123456

	cout << F("(Motor RPM Set:: ") << fSetRPMRate 
		 << F(" RPM:: ") << nSetRPMValue 
		 << F(" Rate:: ") << (nSetRPMValue ? float((nSetRPMValue - _MIN_RPM_VALUE_) / float(_MAX_RPM_VALUE_ - _MIN_RPM_VALUE_)) : _ZERO_RPM_RATE_)
		 << F(")") << endl;

	cout.precision(FLOAT_POINT_PRECISION_FOR_DATA_ACQ);
}

static inline void printAcctime(const unsigned int nACCTime)
{
	cout.precision(6);

	cout << F("(Motor ACC Set:: ") << float(nACCTime / 1000.0f) << F(" [sec])") << endl;

	cout.precision(FLOAT_POINT_PRECISION_FOR_DATA_ACQ);
}

static inline void printDectime(const unsigned int nDECTime)
{
	cout.precision(6);

	cout << F("(Motor DEC Set:: ") << float(nDECTime / 1000.0f) << F(" [sec])") << endl; 

	cout.precision(FLOAT_POINT_PRECISION_FOR_DATA_ACQ);
}

#endif


#if defined(_CMOTORPUMP_OZBV10A_H_)
/*
cout.setf(ios::hex, ios::basefield);
cout.setf(ios::showbase);
cout.unsetf(ios::showbase);
//(or)
cout << hex << showbase << cPacketBuf[0];
*/
static inline void printState(const int nStateCode)
{
	cout << F("\n[STATE] Motor Controller State : ") << hex << showbase << nStateCode << endl;
	cout << F("\t\t");

	if (nStateCode & MOTOR_BRAKE_STATE) {

		cout << F("BRAKE");

	}

	if (nStateCode & MOTOR_FREE_STATE) {

		cout << F("|FREE");

	}

	if (nStateCode & MOTOR_ALARM_STATE) {

		cout << F("|Alarm");

	}

	if (nStateCode & MOTOR_EMERGENCY_STOP_STATE) {

		cout << F("|EMG STOP");

	}

	if (nStateCode & MOTOR_DEC_STATE) {

		cout << F("|DEC");

	}

	if (nStateCode & MOTOR_ACC_STATE) {

		cout << F("|ACC");

	}

	if (nStateCode & MOTOR_CCW_STATE) {

		cout << F("|CCW");

	}
	else {

		cout << F("|CW");

	}

	if (nStateCode & MOTOR_RUN_STATE) {

		cout << F("|RUN");

	}
	else {

		cout << F("|STOP");

	}
}

static inline void printAlarm(const int nAlarmCode)
{
	const _FlashString* pstrError;

	switch (nAlarmCode) {

		case MOTOR_EMERGENCY_STOP:
			pstrError = F("\t\tEmergency Stopped.");
		break;

		case MOTOR_OVERCURRENT:
			pstrError = F("\t\tOvercurrent Detected.");
		break;

		case MOTOR_OVERHEAT:
			pstrError = F("\t\tOverheat Detected.(> 70[deg C])");
		break;

		case MOTOR_HALLSENSOR_ABNORMAL:
			pstrError = F("\t\tHall Sensor Fault.");
		break;

		case MOTOR_OVERVOLTAGE:
			pstrError = F("\t\tOvervoltage Detected.");
		break;

		case MOTOR_UNDERVOLTAGE:
			pstrError = F("\t\tUndervoltage Detected.");
		break;

		case MOTOR_OVERLOAD:
			pstrError = F("\t\tOverload Detected.");
		break;

		case MOTOR_SHORT_DRIVE:
			pstrError = F("\t\tShort-term Driving Error Detected.");
		break;

		case MOTOR_NORMAL:
		default:
			// pstError = F("\t\tMotor Normal.");
			return;
		break;

	}

	cout << F("\n[ERROR] Motor Controller Error : ") << nAlarmCode << endl;
	cout << pstrError << endl;
}

static inline unsigned int calc_RPMValue(const float fRate)
{
	float fRPMRate = constrain(fRate, _ZERO_RPM_RATE_, _MAX_RPM_RATE_);

	unsigned int nRPMValue = fRPMRate >= _MIN_RPM_RATE_ ? (unsigned int)((_MAX_RPM_VALUE_ - _MIN_RPM_VALUE_) * fRPMRate + _MIN_RPM_VALUE_) : _ZERO_RPM_VALUE_;

	return nRPMValue;
}

static inline float calc_RPMRate(const unsigned int nValue)
{
	float fRPMPercentage = nValue ? round(((nValue - _MIN_RPM_VALUE_) / float(_MAX_RPM_VALUE_ - _MIN_RPM_VALUE_)) * 100UL) : _ZERO_RPM_RATE_;

	return float(fRPMPercentage / 100.0f);
}

static inline void wait_Time(const unsigned long ulWaitTime)
{
	unsigned long ulStartTime = millis();

	while (millis() - ulStartTime < ulWaitTime);
}

#endif


CMotorPumpBLDC::CMotorPumpBLDC(): CModbusRTU()
{
#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
	_postAlarm = NULL;
#endif

	clear();
}

void CMotorPumpBLDC::setModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr)
{
	this->set_ModbusNode(rSerial, nMBSlaveAddr);
}

#if OZBV10A_RS232_MODEL == false
void CMotorPumpBLDC::setPreTransmission(void (*preTransmission)(void))
{
	this->set_PreTransmission(preTransmission);
}

void CMotorPumpBLDC::setPostTransmission(void (*postTransmission)(void))
{
	this->set_PostTransmission(postTransmission);
}
#endif

void CMotorPumpBLDC::clear(void)
{
	_bIsAlarm = false;
	_bIsRunning = false;
	_bIsPseudoRunning = false;

	_u16SetRPMValue = _ZERO_RPM_VALUE_;
	_fSetRPMRate = _ZERO_RPM_RATE_;

	_u16ACCTime = _DEFAULT_ACC_TIME_;
	_u16DECTime = _DEFAULT_DEC_TIME_;
}

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
void CMotorPumpBLDC::setPostAlarm(void (*postAlarm)(void))
{
	if (postAlarm != NULL) {

		_postAlarm = postAlarm;

	}
}
#endif

bool CMotorPumpBLDC::init(void)
{
	bool bSuccess = true;

	// bSuccess &= write_ControlParams(OZBV_COMM_SLAVE_ID_REG_ADDR, _DEFAULT_COMM_ID_);

	// bSuccess &= write_ControlParams(OZBV_COMM_BAUDRATE_REG_ADDR, _DEFAULT_COMM_BAUDRATE_);

	bSuccess &= write_ControlParams(OZBV_CURRENT_P_GAIN_REG_ADDR, _DEFAULT_CURRENT_P_GAIN_);

	bSuccess &= write_ControlParams(OZBV_CURRENT_I_GAIN_REG_ADDR, _DEFAULT_CURRENT_I_GAIN_);

	bSuccess &= write_ControlParams(OZBV_SPEED_P_GAIN_REG_ADDR, _DEFAULT_SPEED_P_GAIN_);

	bSuccess &= write_ControlParams(OZBV_SPEED_I_GAIN_REG_ADDR, _DEFAULT_SPEED_I_GAIN_);

	bSuccess &= write_ControlParams(OZBV_ACC_TIME_REG_ADDR, _DEFAULT_ACC_TIME_);

	bSuccess &= write_ControlParams(OZBV_DEC_TIME_REG_ADDR, _DEFAULT_DEC_TIME_);

	bSuccess &= write_ControlParams(OZBV_MIN_SPEED_REG_ADDR, _DEFAULT_MIN_SPEED_);

	bSuccess &= write_ControlParams(OZBV_MAX_SPEED_REG_ADDR, _DEFAULT_MAX_SPEED_);

	bSuccess &= write_ControlParams(OZBV_CURRENT_LIMIT_REG_ADDR, _DEFAULT_CURRENT_LIMIT_);

	bSuccess &= write_ControlParams(OZBV_CURRENT_LIMIT_TIME_REG_ADDR, _DEFAULT_CURRENT_LIMIT_TIME_);

	bSuccess &= write_ControlParams(OZBV_OVERHEAT_LIMIT_REG_ADDR, _DEFAULT_OVERHEAT_LIMIT_);

	bSuccess &= write_ControlParams(OZBV_OVERHEAT_LIMIT_TIME_REG_ADDR, _DEFAULT_OVERHEAT_LIMIT_TIME_);

	// bSuccess &= write_ControlParams(OZBV_POWER_DIFF_LIMIT_TIME_REG_ADDR, _DEFAULT_POWER_DIFF_LIMIT_TIME_);

	bSuccess &= write_ControlParams(OZBV_MOTOR_STOP_MODE_REG_ADDR, _DEFAULT_MOTOR_STOP_MODE_);

	bSuccess &= write_ControlParams(OZBV_MOTOR_EMG_STOP_MODE_REG_ADDR, _DEFAULT_MOTOR_EMG_STOP_MODE_);

	// bSuccess &= write_ControlParams(OZBV_MONITORING_MODE_REG_ADDR, _DEFAULT_MONITORING_MODE_);

	bSuccess &= write_ControlParams(OZBV_SPEED_PERMISSIBLE_RANGE_REG_ADDR, _DEFAULT_SPEED_PERMISSIBLE_RANGE_);

	// bSuccess &= write_ControlParams(OZBV_IN_POLARITY_SEL_REG_ADDR, _DEFAULT_IN_POLARITY_);

	// bSuccess &= write_ControlParams(OZBV_OUT_POLARITY_SEL_REG_ADDR, _DEFAULT_OUT_POLARITY_);

	bSuccess &= write_ControlParams(OZBV_MOTOR_RUN_MODE_REG_ADDR, _DEFAULT_MOTOR_RUN_MODE_);

	bSuccess &= write_ControlParams(OZBV_IN1_FUNCTION_REG_ADDR, _DEFAULT_IN1_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_IN2_FUNCTION_REG_ADDR, _DEFAULT_IN2_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_IN3_FUNCTION_REG_ADDR, _DEFAULT_IN3_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_IN4_FUNCTION_REG_ADDR, _DEFAULT_IN4_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_OUT1_FUNCTION_REG_ADDR, _DEFAULT_OUT1_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_OUT2_FUNCTION_REG_ADDR, _DEFAULT_OUT2_FUNCTION_);

	bSuccess &= write_ControlParams(OZBV_OUT3_FUNCTION_REG_ADDR, _DEFAULT_OUT3_FUNCTION_);

	// bSuccess &= write_ControlParams(OZBV_ANALOG_INPUT_POLARITY_REG_ADDR, _DEFAULT_ANALOG_IN_POLARITY_);

	// bSuccess &= write_ControlParams(OZBV_ANALOG_INPUT_MINIMUM_REG_ADDR, _DEFAULT_ANALOG_IN_MINIMUM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE_CHECK_REG_ADDR, _DEFAULT_TORQUE_CHECK_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE00_RPM_REG_ADDR, _DEFAULT_TORQUE00_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE00_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE00_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE00_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE00_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE01_RPM_REG_ADDR, _DEFAULT_TORQUE01_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE01_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE01_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE01_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE01_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE10_RPM_REG_ADDR, _DEFAULT_TORQUE10_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE10_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE10_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE10_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE10_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE11_RPM_REG_ADDR, _DEFAULT_TORQUE11_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE11_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE11_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE11_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE11_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE20_RPM_REG_ADDR, _DEFAULT_TORQUE20_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE20_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE20_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE20_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE20_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE21_RPM_REG_ADDR, _DEFAULT_TORQUE21_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE21_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE21_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE21_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE21_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE30_RPM_REG_ADDR, _DEFAULT_TORQUE30_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE30_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE30_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE30_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE30_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE31_RPM_REG_ADDR, _DEFAULT_TORQUE31_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE31_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE31_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE31_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE31_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE40_RPM_REG_ADDR, _DEFAULT_TORQUE40_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE40_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE40_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE40_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE40_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE41_RPM_REG_ADDR, _DEFAULT_TORQUE41_RPM_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE41_CURRENT_BASE_REG_ADDR, _DEFAULT_TORQUE41_CURRENT_BASE_);

	// bSuccess &= write_ControlParams(OZBV_MOTOR_TORQUE41_CURRENT_LIMIT_REG_ADDR, _DEFAULT_TORQUE41_CURRENT_LIMIT_);

	// bSuccess &= write_ControlParams(OZBV_SPEED_OFFSET0_REG_ADDR, _DEFAULT_SPEED_OFFSET0_);

	// bSuccess &= write_ControlParams(OZBV_SPEED_OFFSET1_REG_ADDR, _DEFAULT_SPEED_OFFSET1_);

	// bSuccess &= write_ControlParams(OZBV_SPEED_OFFSET2_REG_ADDR, _DEFAULT_SPEED_OFFSET2_);

	// bSuccess &= write_ControlParams(OZBV_TORQUE_CURRENT_LIMIT_TIME_REG_ADDR, _DEFAULT_TORQUE_CURRENT_LIMIT_TIME_);

	bSuccess &= write_ControlParams(OZBV_MOTOR_RUN_DIRECTION_REG_ADDR, _DEFAULT_RUN_DIRECTION_);

	bSuccess &= write_ControlParams(OZBV_MOTOR_POLARITY_NUMBER_REG_ADDR, _DEFAULT_POLARITY_NUMBER_);

	bSuccess &= write_ControlParams(OZBV_CURRENT_FILTER_REG_ADDR, _DEFAULT_CURRENT_FILTER_);

	bSuccess &= write_ControlParams(OZBV_SPEED_FILTER_REG_ADDR, _DEFAULT_SPEED_FILTER_);

	// bSuccess &= write_ControlParams(OZBV_EXTERNAL_VOLUME_FILTER_REG_ADDR, _DEFAULT_EXTERNAL_VOLUME_FILTER_);

	// bSuccess &= write_ControlParams(OZBV_CURRENT_SENSOR_CAPACITY_REG_ADDR, _DEFAULT_CURRENT_SENSOR_CAPACITY_);

	bSuccess &= write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_PARAMS_SAVE);

	// wait_Time(MOTOR_PARAMS_SAVE_LATENCY);

	return bSuccess;
}

void CMotorPumpBLDC::reset(void)
{
	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RESET);

	wait_Time(MOTOR_RESET_DURATION);
}

void CMotorPumpBLDC::stop(void)
{
	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_STOP);

// 	write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _ZERO_RPM_VALUE_);

	_bIsRunning = false;

	_bIsPseudoRunning = false;

	_u16SetRPMValue = 0UL;

	_fSetRPMRate = 0.0F;
}

void CMotorPumpBLDC::stopEMG(void)
{
	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_EMG_STOP);

// 	write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _ZERO_RPM_VALUE_);

	_bIsRunning = false;

	_bIsPseudoRunning = false;

	_u16SetRPMValue = 0UL;

	_fSetRPMRate = 0.0F;
}

void CMotorPumpBLDC::start(void)
{
	if (_bIsRunning)

		return;

	write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _MAX_RPM_VALUE_EFF_);

	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RUN);

	_bIsRunning = true;

	_u16SetRPMValue = _MAX_RPM_VALUE_EFF_;

	_fSetRPMRate = _MAX_RPM_RATE_;

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printSpeed(_u16SetRPMValue, _fSetRPMRate);
#endif
}

void CMotorPumpBLDC::start(const unsigned int nRPMValue)
{
	if (_bIsRunning)

		return;

	if (nRPMValue > _ZERO_RPM_VALUE_) {

		_u16SetRPMValue = constrain(nRPMValue, _MIN_RPM_VALUE_EFF_, _MAX_RPM_VALUE_EFF_);

		write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _u16SetRPMValue);

		write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RUN);

	}
	else {

		_bIsPseudoRunning = true;

	}

	_bIsRunning = true;

	_fSetRPMRate = calc_RPMRate(_u16SetRPMValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printSpeed(_u16SetRPMValue, _fSetRPMRate);
#endif
}

void CMotorPumpBLDC::start(const float fRPMRate)
{
	if (_bIsRunning)

		return;

	_u16SetRPMValue = calc_RPMValue(fRPMRate);

	if (_u16SetRPMValue > _ZERO_RPM_VALUE_) {

		write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _u16SetRPMValue);

		write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RUN);

	}
	else {

		_bIsPseudoRunning = true;

	}

	_bIsRunning = true;

	_fSetRPMRate = fRPMRate;//calc_RPMRate(_u16SetRPMValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printSpeed(_u16SetRPMValue, _fSetRPMRate);
#endif
}

void CMotorPumpBLDC::setRPM(const unsigned int nRPMValue)
{
	if (nRPMValue > _ZERO_RPM_VALUE_) {

		_u16SetRPMValue = constrain(nRPMValue, _MIN_RPM_VALUE_EFF_, _MAX_RPM_VALUE_EFF_);

		write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _u16SetRPMValue);

		if (_bIsPseudoRunning) {

			write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RUN);

		}

	}
	else {

		write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_STOP);

		_bIsPseudoRunning = true;

	}

	_fSetRPMRate = calc_RPMRate(_u16SetRPMValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printSpeed(_u16SetRPMValue, _fSetRPMRate);
#endif
}

void CMotorPumpBLDC::setRPM(const float fRPMRate)
{
	_u16SetRPMValue = calc_RPMValue(fRPMRate);

	if (_u16SetRPMValue > _ZERO_RPM_VALUE_) {

		write_SpeedSet(OZBV_SPEED_SETPOINT_REG_ADDR, _u16SetRPMValue);

		if (_bIsPseudoRunning) {

			write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_RUN);

		}

	}
	else {

		write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_STOP);

		_bIsPseudoRunning = true;

	}

	_fSetRPMRate = fRPMRate;//calc_RPMRate(_u16SetRPMValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printSpeed(_u16SetRPMValue, _fSetRPMRate);
#endif
}

// Params : nACCTime [msec] @ 1000 [rpm]
void CMotorPumpBLDC::setACC(const unsigned int nACCTime)
{
	_u16ACCTime = nACCTime;

	write_ControlParams(OZBV_ACC_TIME_REG_ADDR, _u16ACCTime);

	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_PARAMS_SAVE);

	// wait_Time(MOTOR_PARAMS_SAVE_LATENCY);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printAcctime(_u16ACCTime);
#endif
}

// Params : nDECTime [msec] @ 1000 [rpm]
void CMotorPumpBLDC::setDEC(const unsigned int nDECTime)
{
	_u16DECTime = nDECTime;

	write_ControlParams(OZBV_DEC_TIME_REG_ADDR, _u16DECTime);

	write_CommandSet(OZBV_COMMAND_REG_ADDR, COMMAND_PARAMS_SAVE);

	// wait_Time(MOTOR_PARAMS_SAVE_LATENCY);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	printDectime(_u16DECTime);
#endif
}

int CMotorPumpBLDC::getParams(const unsigned int nStartAddr)
{
	int nParams = OZBV_INVALID_CONTROL_PARAMS;

	int nControlParams = read_ControlParams(nStartAddr);

	if (nControlParams != OZBV_INVALID_CONTROL_PARAMS) {

		nParams = nControlParams & 0xFFFF;

	}

	return nParams;
}

int CMotorPumpBLDC::getSpeed(void)
{
	int nSpeedValue[2];

	read_PresentElec(OZBV_PRESENT_SPEED_SETPOINT_REG_ADDR, nSpeedValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	// cout << F("[SET.SPEED] Motor Speed: ") << nSpeedValue[0] << F(" [rpm]") << endl;
	// cout << F("[PRESENT.SPEED] Motor Speed : ") << nSpeedValue[1] << F(" [rpm]") << endl;
#endif

	return nSpeedValue[1];
}

int CMotorPumpBLDC::getCurrent(void)
{
	int nCurrentValue[2];

	read_PresentElec(OZBV_PRESENT_CURRENT_SETPOINT_REG_ADDR, nCurrentValue);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	// cout << F("[SET.CURRENT] Motor Current : ") << nCurrentValue[0] << F(" [mA]") << endl;
	// cout << F("[PRESENT.CURRENT] Motor Current : ") << nCurrentValue[1] << F(" [mA]") << endl;
#endif

	return nCurrentValue[1];
}

int CMotorPumpBLDC::getVersion(int* pVersion)
{
	if (!pVersion) {
		return 0;
	}

	read_ControllerVersion(pVersion);

#if defined(_TEST_CMOTORPUMP_BLDC_)
	// cout << F("[OZBV10A.VERSION] H/W ver.") << *pVersion << endl;
	// cout << F("[OZBV10A.VERSION] S/W ver.") << *(pVersion+1) << endl;
#endif

	return *(pVersion+1);
}

unsigned char CMotorPumpBLDC::getState(void)
{
	unsigned char nStateCode = OZBV_INVALID_PRESENT_STATUS & 0x00FF;

	int nPresentState = read_PresentStatus(OZBV_PRESENT_STATE_REG_ADDR);

	if (nPresentState != OZBV_INVALID_PRESENT_STATUS) {

		nStateCode = nPresentState & 0xFF;

#if defined(_TEST_CMOTORPUMP_BLDC_)
		printState(nStateCode);
#endif

	}

	return nStateCode;
}

unsigned char CMotorPumpBLDC::getAlarmCode(void)
{
	unsigned char nAlarmCode = OZBV_INVALID_PRESENT_STATUS & 0x00FF;

	int nPresentAlarm = read_PresentStatus(OZBV_PRESENT_ALARM_REG_ADDR);

	if (nPresentAlarm != OZBV_INVALID_PRESENT_STATUS) {

		nAlarmCode = nPresentAlarm & 0x0F;

#if defined(_TEST_CMOTORPUMP_BLDC_)
		printAlarm(nAlarmCode);
#endif

	}
	else {

		nAlarmCode = MOTOR_NORMAL;

	}

	return nAlarmCode;
}

void CMotorPumpBLDC::pollingAlarm(void)
{
	unsigned char nAlarmCode = getAlarmCode();

	if (nAlarmCode != MOTOR_NORMAL) {

		_bIsAlarm = true;

		if (_bIsRunning == true) {

			stopEMG();

		}

		reset();

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

// Utilities
unsigned int CMotorPumpBLDC::verifyACCTime(void)
{
	unsigned int nACCTime = getParams(OZBV_ACC_TIME_REG_ADDR);

	if (nACCTime != OZBV_INVALID_CONTROL_PARAMS) {

		_u16ACCTime = nACCTime;

		return nACCTime;

	}

	return 0;
}

unsigned int CMotorPumpBLDC::convACCTimeWithRPM(const unsigned int nACCTime, const float fRPMRate)
{
	float fTimeRate = float(calc_RPMValue(fRPMRate) / float(_ACC_TIME_BASIS_RPM_));

	return constrain((unsigned int)(round(nACCTime * fTimeRate)), _MIN_ACC_TIME_, _MAX_ACC_TIME_ * (_MAX_RPM_VALUE_ / float(_MIN_RPM_VALUE_)));
}

unsigned int CMotorPumpBLDC::convDECTimeWithRPM(const unsigned int nDECTime, const float fRPMRate)
{
	float fTimeRate = float(calc_RPMValue(fRPMRate) / float(_DEC_TIME_BASIS_RPM_));

	return constrain((unsigned int)(round(nDECTime * fTimeRate)), _MIN_DEC_TIME_, _MAX_DEC_TIME_ * (_MAX_RPM_VALUE_ / float(_MIN_RPM_VALUE_)));
}


// OUT1 / OUT2 / OUT3 : Alarm Out, Speed Out, Dir Out
int CMotorPumpBLDC::read_OutputStatus(const unsigned int nStartAddr)
{
	unsigned int nReceivedStatus;

	int nOutputIOStatus = OZBV_INVALID_OUT_STATUS;

	if (this->read_Coils(nStartAddr, 3, &nReceivedStatus) == MODBUS_RTU_SUCCESS) {

		nOutputIOStatus = nReceivedStatus & 0x07;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedStatus);

	}
	#endif

	return nOutputIOStatus;
}

// OUT1 / OUT2 / OUT3 : Alarm Out, Speed Out, Dir Out
bool CMotorPumpBLDC::write_SignleOutputStatus(const unsigned int nStartAddr, const unsigned char nOutputStatus)
{
	return (this->write_SingleCoil(nStartAddr, nOutputStatus) ? false : true);
}

// IN1 / IN2 / IN3 / IN4 : Run/Stop, Dir, Reset, EMG
int CMotorPumpBLDC::read_InputStatus(const unsigned int nStartAddr)
{
	unsigned int nReceivedStatus;

	int nInputIOStatus = OZBV_INVALID_IN_STATUS;

	if (this->read_DiscreteInputs(nStartAddr, 3, &nReceivedStatus) == MODBUS_RTU_SUCCESS) {

		nInputIOStatus = nReceivedStatus & 0x0F;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedStatus);

	}
	#endif

	return nInputIOStatus;
}

// Present State of Motor Controller
int CMotorPumpBLDC::read_PresentStatus(const unsigned int nStartAddr)
{
	unsigned int nReceivedData;

	int nPresentStatus = OZBV_INVALID_PRESENT_STATUS;

	if (this->read_InputRegisters(nStartAddr, 1, &nReceivedData) == MODBUS_RTU_SUCCESS) {

		nPresentStatus = nReceivedData & 0xFFFF;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nPresentStatus;
}

int CMotorPumpBLDC::read_PresentElec(const unsigned int nStartAddr, int* pElecValBuf)
{
	unsigned int nReceivedData[2];

	if (!pElecValBuf) {
		return 0;
	}

	pElecValBuf[0] = 0;
	pElecValBuf[1] = 0;

	if (this->read_InputRegisters(nStartAddr, 2, nReceivedData) == MODBUS_RTU_SUCCESS) {

		pElecValBuf[0] = (nReceivedData[0] & 0xFFFF) * _DEFAULT_DIRECTION_;
		pElecValBuf[1] = (nReceivedData[1] & 0xFFFF) * _DEFAULT_DIRECTION_;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData[1]);

	}
	#endif

	return pElecValBuf[1];
}

// Control Setting Parameters of Motor Controller
int CMotorPumpBLDC::read_ControlParams(const unsigned int nStartAddr)
{
	unsigned int nReceivedData;

	int nControlParams = OZBV_INVALID_CONTROL_PARAMS;

	if (this->read_HoldingRegisters(nStartAddr, 1, &nReceivedData) == MODBUS_RTU_SUCCESS) {

		nControlParams = nReceivedData & 0xFFFF;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nControlParams;
}

bool CMotorPumpBLDC::write_ControlParams(const unsigned int nStartAddr, const int nSetParams)
{
	return (this->write_SingleRegister(nStartAddr, nSetParams) ? false : true);
}

// Command Group of Motor Controller
bool CMotorPumpBLDC::write_CommandSet(const unsigned int nStartAddr, const unsigned char nCommand)
{
	return (this->write_SingleRegister(nStartAddr, nCommand) ? false : true);
}

bool CMotorPumpBLDC::write_SpeedSet(const unsigned int nStartAddr, const int nSpeedRPM)
{
	int nSetSpeed = nSpeedRPM * _DEFAULT_DIRECTION_;

	return (this->write_SingleRegister(nStartAddr, nSetSpeed) ? false : true);
}

int CMotorPumpBLDC::read_ControllerVersion(int* pVersionBuf)
{
	unsigned int nReceivedData[2];

	if (!pVersionBuf) {
		return 0;
	}

	pVersionBuf[0] = 0;
	pVersionBuf[1] = 0;

	if (this->read_HoldingRegisters(OZBV_HARDWARE_VERSION_REG_ADDR, 2, nReceivedData) == MODBUS_RTU_SUCCESS) {

		pVersionBuf[0] = nReceivedData[0] & 0xFFFF;
		pVersionBuf[1] = nReceivedData[1] & 0xFFFF;

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData[1]);
		
	}
	#endif

	return pVersionBuf[1];
}

