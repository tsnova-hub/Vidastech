

#include "CTM4Controller.h"


CTM4Controller::CTM4Controller(): CModbusRTU()
{
	clear();
}

void CTM4Controller::setModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr)
{
	this->set_ModbusNode(rSerial, nMBSlaveAddr);
}

void CTM4Controller::setPreTransmission(void (*preTrasmission)(void))
{
	this->set_PreTransmission(preTrasmission);
}

void CTM4Controller::setPostTransmission(void (*postTransmission)(void))
{
	this->set_PostTransmission(postTransmission);
}

void CTM4Controller::clear(void)
{
	memset(_stThermoSensor.fPresentTemperature, 0.0f, sizeof(float)*TM4_TOTAL_NO_OF_CH);
	memset(_stThermoSensor.fSetTemperature, 0.0f, sizeof(float)*TM4_TOTAL_NO_OF_CH);
	memset(_stThermoSensor.nSensorErrorCode, 0, sizeof(int)*TM4_TOTAL_NO_OF_CH);

	memset(_fPresentHeatingMV, 0.0f, sizeof(float)*TM4_TOTAL_NO_OF_CH);

#if defined(_USE_MOVING_AVERAGE_)
	memset(_fTempSamples[CTM4Controller::TM4CH1-1], 0.0f, sizeof(float)*NUM_OF_TEMPERATURE_SAMPLES);
	memset(_fTempSamples[CTM4Controller::TM4CH2-1], 0.0f, sizeof(float)*NUM_OF_TEMPERATURE_SAMPLES);
	memset(_fTempSamples[CTM4Controller::TM4CH3-1], 0.0f, sizeof(float)*NUM_OF_TEMPERATURE_SAMPLES);
	memset(_fTempSamples[CTM4Controller::TM4CH4-1], 0.0f, sizeof(float)*NUM_OF_TEMPERATURE_SAMPLES);

	memset(_fSum_of_samples, 0.0f, sizeof(float)*TM4_TOTAL_NO_OF_CH);

	memset(_nIndex_of_sample, 0, sizeof(unsigned int)*TM4_TOTAL_NO_OF_CH);
	memset(_nNum_of_samples, 0, sizeof(unsigned int)*TM4_TOTAL_NO_OF_CH);
#endif
}

#if defined(_USE_MOVING_AVERAGE_)
float CTM4Controller::calculate_AveragedTemperature(const int nChannel, const float fPresentTemperature)
{
	_fSum_of_samples[nChannel] -= _fTempSamples[nChannel][_nIndex_of_sample[nChannel]];

	_fTempSamples[nChannel][_nIndex_of_sample[nChannel]] = fPresentTemperature;

	_fSum_of_samples[nChannel] += fPresentTemperature;

	_nIndex_of_sample[nChannel] = ++_nIndex_of_sample[nChannel] % NUM_OF_TEMPERATURE_SAMPLES;

	if (_nNum_of_samples[nChannel] < NUM_OF_TEMPERATURE_SAMPLES) {

		_nNum_of_samples[nChannel]++;

	}

	return (_fSum_of_samples[nChannel] / float(_nNum_of_samples[nChannel]));
}
#endif

bool CTM4Controller::resetFactoryDefault(void)
{
	bool bResult = false;
	bool bRetry = true;
	unsigned char nRetryNum = 0;

	while (bRetry && ++nRetryNum < 4) {

		unsigned char cResult = MODBUS_RTU_SUCCESS;

		for (int nChannel = CTM4Controller::TM4CH1; nChannel <= CTM4Controller::TM4CH4; nChannel++) {

			enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

			cResult |= disableOutputControl(eChannel);
			cResult |= disableAutoTuning(eChannel);

			cResult |= writeHeatingControlType(eChannel, DEFAULT_HEATING_CONTROL_TYPE);
			cResult |= writeMultiSetPointNum(eChannel, DEFAULT_MULTI_SETPOINT_NUM);

			cResult |= writeSensorInputType(eChannel, DEFAULT_SENSOR_INPUT_TYPE);
			cResult |= writeSensorInputUnit(eChannel, DEFAULT_SENSOR_INPUT_UNIT);
			cResult |= writeSVLowLimit(eChannel, DEFAULT_HEATING_SV_LOW_LIMIT);
			cResult |= writeSVHighLimit(eChannel, DEFAULT_HEATING_SV_HIGH_LIMIT);
			cResult |= writeOperatingType(eChannel, DEFAULT_OPERATING_TYPE);
			cResult |= writeControlMethod(eChannel, DEFAULT_CONTROL_METHOD);
			cResult |= writeAutotuningMode(eChannel, DEFAULT_AUTOTUNING_MODE);

			cResult |= writeNumOfSetPoint(eChannel, DEFAULT_NUM_OF_MULTI_SETPOINT);
			cResult |= writeInitialManualMV(eChannel, DEFAULT_INITIAL_MANUAL_MV);
			cResult |= writePresetManualMV(eChannel, DEFAULT_PRESET_MANUAL_MV);
			cResult |= writeSensorErrorMV(eChannel, DEFAULT_SENSOR_ERROR_MV);
			cResult |= writeStopMV(eChannel, DEFAULT_STOP_MV);
			cResult |= writeStopAlarmOut(eChannel, DEFAULT_STOP_ALARM_OUT);

		}

		// Channel 1
		cResult |= writeSensorInputBias(CTM4Controller::TM4CH1, DEFAULT_CH1_SENSOR_INPUT_BIAS);
		cResult |= writeSensorDigitalFilter(CTM4Controller::TM4CH1, DEFAULT_CH1_SENSOR_DIGITAL_FILTER);
		cResult |= writeHeatingControlTime(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_CONTROL_TIME);

		cResult |= writeHeatingProportionalBand(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_PROPORTIONAL_BAND);
		cResult |= writeHeatingIntegralTime(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_INTEGRAL_TIME);
		cResult |= writeHeatingDerivationTime(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_DERIVATION_TIME);
		cResult |= writeMVLowLimit(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_MV_LOW_LIMIT);
		cResult |= writeMVHighLimit(CTM4Controller::TM4CH1, DEFAULT_CH1_HEATING_MV_HIGH_LIMIT);
		cResult |= writeRampUpRate(CTM4Controller::TM4CH1, DEFAULT_CH1_RAMP_UP_RATE);
		cResult |= writeRampDownRate(CTM4Controller::TM4CH1, DEFAULT_CH1_RAMP_DOWN_RATE);
		cResult |= writeRampTimeUnit(CTM4Controller::TM4CH1, DEFAULT_CH1_RAMP_TIME_UNIT);

		// Channel 2
		cResult |= writeSensorInputBias(CTM4Controller::TM4CH2, DEFAULT_CH2_SENSOR_INPUT_BIAS);
		cResult |= writeSensorDigitalFilter(CTM4Controller::TM4CH2, DEFAULT_CH2_SENSOR_DIGITAL_FILTER);
		cResult |= writeHeatingControlTime(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_CONTROL_TIME);

		cResult |= writeHeatingProportionalBand(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_PROPORTIONAL_BAND);
		cResult |= writeHeatingIntegralTime(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_INTEGRAL_TIME);
		cResult |= writeHeatingDerivationTime(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_DERIVATION_TIME);
		cResult |= writeMVLowLimit(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_MV_LOW_LIMIT);
		cResult |= writeMVHighLimit(CTM4Controller::TM4CH2, DEFAULT_CH2_HEATING_MV_HIGH_LIMIT);
		cResult |= writeRampUpRate(CTM4Controller::TM4CH2, DEFAULT_CH2_RAMP_UP_RATE);
		cResult |= writeRampDownRate(CTM4Controller::TM4CH2, DEFAULT_CH2_RAMP_DOWN_RATE);
		cResult |= writeRampTimeUnit(CTM4Controller::TM4CH2, DEFAULT_CH2_RAMP_TIME_UNIT);

		// Channel 3
		cResult |= writeSensorInputBias(CTM4Controller::TM4CH3, DEFAULT_CH3_SENSOR_INPUT_BIAS);
		cResult |= writeSensorDigitalFilter(CTM4Controller::TM4CH3, DEFAULT_CH3_SENSOR_DIGITAL_FILTER);
		cResult |= writeHeatingControlTime(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_CONTROL_TIME);

		cResult |= writeHeatingProportionalBand(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_PROPORTIONAL_BAND);
		cResult |= writeHeatingIntegralTime(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_INTEGRAL_TIME);
		cResult |= writeHeatingDerivationTime(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_DERIVATION_TIME);
		cResult |= writeMVLowLimit(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_MV_LOW_LIMIT);
		cResult |= writeMVHighLimit(CTM4Controller::TM4CH3, DEFAULT_CH3_HEATING_MV_HIGH_LIMIT);
		cResult |= writeRampUpRate(CTM4Controller::TM4CH3, DEFAULT_CH3_RAMP_UP_RATE);
		cResult |= writeRampDownRate(CTM4Controller::TM4CH3, DEFAULT_CH3_RAMP_DOWN_RATE);
		cResult |= writeRampTimeUnit(CTM4Controller::TM4CH3, DEFAULT_CH3_RAMP_TIME_UNIT);

		// Channel 4
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	#if defined(_USE_BREW_WATER_TEMP_SENSOR_K_L_TYPE_FOR_TEST_)
		cResult |= writeSensorInputType(CTM4Controller::TM4CH4, 1);	// K.L type temperature sensor
	#endif
#endif
		cResult |= writeSensorInputBias(CTM4Controller::TM4CH4, DEFAULT_CH4_SENSOR_INPUT_BIAS);
		cResult |= writeSensorDigitalFilter(CTM4Controller::TM4CH4, DEFAULT_CH4_SENSOR_DIGITAL_FILTER);
		cResult |= writeHeatingControlTime(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_CONTROL_TIME);

		cResult |= writeHeatingProportionalBand(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_PROPORTIONAL_BAND);
		cResult |= writeHeatingIntegralTime(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_INTEGRAL_TIME);
		cResult |= writeHeatingDerivationTime(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_DERIVATION_TIME);
		cResult |= writeMVLowLimit(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_MV_LOW_LIMIT);
		cResult |= writeMVHighLimit(CTM4Controller::TM4CH4, DEFAULT_CH4_HEATING_MV_HIGH_LIMIT);
		cResult |= writeRampUpRate(CTM4Controller::TM4CH4, DEFAULT_CH4_RAMP_UP_RATE);
		cResult |= writeRampDownRate(CTM4Controller::TM4CH4, DEFAULT_CH4_RAMP_DOWN_RATE);
		cResult |= writeRampTimeUnit(CTM4Controller::TM4CH4, DEFAULT_CH4_RAMP_TIME_UNIT);


		if (cResult == MODBUS_RTU_SUCCESS) {

			bRetry = false;
			bResult = true;

		}
		else {

			bRetry = true;
			delay(10UL);

		}

	}

	return bResult;
}

void CTM4Controller::readPresentTemperaturesAll(void)
{
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	for (int nChannel = CTM4Controller::TM4CH1; nChannel <= CTM4Controller::TM4CH4; nChannel++) {

		readPresentTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

	}
#else
	for (int nChannel = CTM4Controller::TM4CH1; nChannel < CTM4Controller::TM4CH4; nChannel++) {

		readPresentTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

	}
#endif
}

float CTM4Controller::readPresentTemperature(const enum TM4Channel eChannel)
{
	float fPresentTemperature = TM4_INVALID_TEMPERATURE;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_InputRegisters(_TM4_INPUT_REG_ADDR(eChannel,0), 1, &nReceivedData) == MODBUS_RTU_SUCCESS) {

		if ((_stThermoSensor.nSensorErrorCode[eChannel-1] = handle_InvalidSensorResponse(nReceivedData)) == TM4_SENSOR_ERROR_NONE)
		{
			fPresentTemperature = float(int(nReceivedData) / 10.0f);

			// noInterrupts();

			#if !defined(_USE_MOVING_AVERAGE_)
			_stThermoSensor.fPresentTemperature[eChannel-1] = fPresentTemperature;
			#else
			_stThermoSensor.fPresentTemperature[eChannel-1] = calculate_AveragedTemperature(eChannel-1, fPresentTemperature);
			#endif

			// interrupts();
		}

	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fPresentTemperature;
}

float CTM4Controller::readSetTemperature(const enum TM4Channel eChannel)
{
	float fSetTemperature = TM4_INVALID_TEMPERATURE;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_MONITOR_REG_ADDR(eChannel,0), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSetTemperature = float(int(nReceivedData) / 10.0f);

		// noInterrupts();

		_stThermoSensor.fSetTemperature[eChannel-1] = fSetTemperature;

		// interrupts();
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSetTemperature;
}

unsigned char CTM4Controller::writeSetTemperature(const enum TM4Channel eChannel, const float fSetTemperature)
{
	unsigned int nSetValue;

	// noInterrupts();

	_stThermoSensor.fSetTemperature[eChannel-1] = fSetTemperature;

	nSetValue = lowWord(int(_stThermoSensor.fSetTemperature[eChannel-1] * 10));

	// interrupts();

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,2), nSetValue);
}

unsigned char CTM4Controller::enableOutputControl(const enum TM4Channel eChannel)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0), TM4_OUTPUT_CONTROL_RUN);
}

unsigned char CTM4Controller::disableOutputControl(const enum TM4Channel eChannel)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0), TM4_OUTPUT_CONTROL_STOP);
}

unsigned char CTM4Controller::enableAutoTuning(const enum TM4Channel eChannel)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x32), TM4_AUTOTUNING_ON);
}

unsigned char CTM4Controller::disableAutoTuning(const enum TM4Channel eChannel)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x32), TM4_AUTOTUNING_OFF);
}

int CTM4Controller::readOutputControlStatus(const enum TM4Channel eChannel)
{
	int nOutputControlStatus = TM4_INVALID_CONTROL_STATUS;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nOutputControlStatus = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nOutputControlStatus;
}

int CTM4Controller::readAutotuningStatus(const enum TM4Channel eChannel)
{
	int nAutotuneRunStatus = TM4_INVALID_CONTROL_STATUS;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x32), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nAutotuneRunStatus = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nAutotuneRunStatus;
}

unsigned char CTM4Controller::startOutputControl(const enum TM4Channel eChannel)
{
	return this->write_SingleCoil(_TM4_COILSTATUS_REG_ADDR(eChannel,0), TM4_OUTPUT_CONTROL_RUN);
}

unsigned char CTM4Controller::stopOutputControl(const enum TM4Channel eChannel)
{
	return this->write_SingleCoil(_TM4_COILSTATUS_REG_ADDR(eChannel,0), TM4_OUTPUT_CONTROL_STOP);
}

unsigned char CTM4Controller::startAutotuning(const enum TM4Channel eChannel)
{
	return this->write_SingleCoil(_TM4_COILSTATUS_REG_ADDR(eChannel,1), TM4_AUTOTUNING_ON);
}

unsigned char CTM4Controller::stopAutotuning(const enum TM4Channel eChannel)
{
	return this->write_SingleCoil(_TM4_COILSTATUS_REG_ADDR(eChannel,1), TM4_AUTOTUNING_OFF);
}

int CTM4Controller::readCoilStatusOfOutputControl(const enum TM4Channel eChannel)
{
	int nOutputControlStatus = TM4_INVALID_COIL_STATUS;

	unsigned int nReceivedStatus = TM4_RXBUF_RESET;

	if (this->read_Coils(_TM4_CONTROL_REG_ADDR(eChannel,0), 1, &nReceivedStatus) == MODBUS_RTU_SUCCESS)
	{
		nOutputControlStatus = nReceivedStatus & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedStatus);

	}
	#endif

	return nOutputControlStatus;
}

int CTM4Controller::readCoilStatusOfAutotuning(const enum TM4Channel eChannel)
{
	int nAutotuneRunStatus = TM4_INVALID_COIL_STATUS;

	unsigned int nReceivedStatus = TM4_RXBUF_RESET;

	if (this->read_Coils(_TM4_CONTROL_REG_ADDR(eChannel,0x32), 1, &nReceivedStatus) == MODBUS_RTU_SUCCESS)
	{
		nAutotuneRunStatus = nReceivedStatus & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedStatus);

	}
	#endif

	return nAutotuneRunStatus;
}


unsigned char CTM4Controller::readHeatingControlType(const enum TM4Channel eChannel)
{
	unsigned char nHeatingControlType = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_MONITOR_REG_ADDR(eChannel,3), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nHeatingControlType = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nHeatingControlType;
}

unsigned char CTM4Controller::writeHeatingControlType(const enum TM4Channel eChannel, const unsigned char nControlType)
{
	return this->write_SingleRegister(_TM4_MONITOR_REG_ADDR(eChannel,3), lowWord(nControlType & 0x01));
}

void CTM4Controller::readPresentHeatingMVAll(void)
{
	for (int eChannel = CTM4Controller::TM4CH1; eChannel <= CTM4Controller::TM4CH4; eChannel++) {

		readPresentHeatingMV(static_cast<enum CTM4Controller::TM4Channel>(eChannel));

	}
}

float CTM4Controller::readPresentHeatingMV(const enum TM4Channel eChannel)
{
	float fHeatingMV = 0.0f;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_MONITOR_REG_ADDR(eChannel,1), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fHeatingMV = float(int(nReceivedData) / 10.0f);

		// noInterrupts();

		_fPresentHeatingMV[eChannel-1] = fHeatingMV;

		// interrupts();
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fHeatingMV;
}

unsigned char CTM4Controller::readMultiSetPointNum(const enum TM4Channel eChannel)
{
	unsigned char nMultiSetPointNum = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,1), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nMultiSetPointNum = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nMultiSetPointNum;
}

unsigned char CTM4Controller::writeMultiSetPointNum(const enum TM4Channel eChannel, const unsigned char nSetPointNum)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,1), lowWord(nSetPointNum));
}

float CTM4Controller::readHeatingProportionalBand(const enum TM4Channel eChannel)
{
	float fHeatingProportionalBand = TM4_INVALID_TEMPERATURE;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x33), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fHeatingProportionalBand = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fHeatingProportionalBand;
}

unsigned char CTM4Controller::writeHeatingProportionalBand(const enum TM4Channel eChannel, const float fProportionalBand)
{
	unsigned int nHeatingProportionalBand = lowWord((unsigned int)(fProportionalBand * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x33), nHeatingProportionalBand);
}

unsigned int CTM4Controller::readHeatingIntegralTime(const enum TM4Channel eChannel)
{
	unsigned int nHeatingIntegralTime = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x35), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nHeatingIntegralTime = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nHeatingIntegralTime;
}

unsigned char CTM4Controller::writeHeatingIntegralTime(const enum TM4Channel eChannel, const unsigned int nIntegralTime)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x35), lowWord(nIntegralTime));
}

unsigned int CTM4Controller::readHeatingDerivationTime(const enum TM4Channel eChannel)
{
	unsigned int nHeatingDerivationTime = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x37), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nHeatingDerivationTime = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nHeatingDerivationTime;
}

unsigned char CTM4Controller::writeHeatingDerivationTime(const enum TM4Channel eChannel, const unsigned int nDerivationTime)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x37), lowWord(nDerivationTime));
}

float CTM4Controller::readMVLowLimit(const enum TM4Channel eChannel)
{
	float fMVLowLimit = 0.0F;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x3F), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fMVLowLimit = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fMVLowLimit;
}

unsigned char CTM4Controller::writeMVLowLimit(const enum TM4Channel eChannel, const float fMVLowLimit)
{
	unsigned int nMVLowLimit = lowWord((unsigned int)(fMVLowLimit * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x3F), nMVLowLimit);
}

float CTM4Controller::readMVHighLimit(const enum TM4Channel eChannel)
{
	float fMVHighLimit = 0.0F;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x40), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fMVHighLimit = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fMVHighLimit;
}

unsigned char CTM4Controller::writeMVHighLimit(const enum TM4Channel eChannel, const float fMVHighLimit)
{
	unsigned int nMVHighLimit = lowWord((unsigned int)(fMVHighLimit * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x40), nMVHighLimit);
}

unsigned int CTM4Controller::readRampUpRate(const enum TM4Channel eChannel)
{
	unsigned int nRampUpRate = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x41), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nRampUpRate = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nRampUpRate;
}

unsigned char CTM4Controller::writeRampUpRate(const enum TM4Channel eChannel, const unsigned int nRampUpRate)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x41), lowWord(nRampUpRate));
}

unsigned int CTM4Controller::readRampDownRate(const enum TM4Channel eChannel)
{
	unsigned int nRampDownRate = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x42), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nRampDownRate = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nRampDownRate;
}

unsigned char CTM4Controller::writeRampDownRate(const enum TM4Channel eChannel, const unsigned int nRampDownRate)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x42), lowWord(nRampDownRate));
}

unsigned char CTM4Controller::readRampTimeUnit(const enum TM4Channel eChannel)
{
	unsigned char nRampTimeUnit = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x43), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nRampTimeUnit = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nRampTimeUnit;
}

unsigned char CTM4Controller::writeRampTimeUnit(const enum TM4Channel eChannel, const unsigned char nRampTimeUnit)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x43), lowWord(nRampTimeUnit));
}

unsigned char CTM4Controller::readNumOfSetPoint(const enum TM4Channel eChannel)
{
	unsigned char nNumOfMultiSetPoint = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x96), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nNumOfMultiSetPoint = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nNumOfMultiSetPoint;
}

unsigned char CTM4Controller::writeNumOfSetPoint(const enum TM4Channel eChannel, const unsigned char nNumOfSetPoint)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x96), lowWord(nNumOfSetPoint));
}

unsigned char CTM4Controller::readInitialManualMV(const enum TM4Channel eChannel)
{
	unsigned char nInitialManualMV = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x97), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nInitialManualMV = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nInitialManualMV;
}

unsigned char CTM4Controller::writeInitialManualMV(const enum TM4Channel eChannel, const unsigned char nInitialManualMVMode)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x97), lowWord(nInitialManualMVMode & 0x01));
}

float CTM4Controller::readPresetManualMV(const enum TM4Channel eChannel)
{
	float fPresetManualMV = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x98), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fPresetManualMV = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fPresetManualMV;
}

unsigned char CTM4Controller::writePresetManualMV(const enum TM4Channel eChannel, const float fPresetManualMV)
{
	unsigned int nPresetManualMV = lowWord((unsigned int)(fPresetManualMV * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x98), nPresetManualMV);
}

float CTM4Controller::readSensorErrorMV(const enum TM4Channel eChannel)
{
	float fSensorErrorMV = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x99), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSensorErrorMV = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSensorErrorMV;
}

unsigned char CTM4Controller::writeSensorErrorMV(const enum TM4Channel eChannel, const float fSensorErrorMV)
{
	unsigned int nSensorErrorMV = lowWord((unsigned int)(fSensorErrorMV * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x99), nSensorErrorMV);
}

float CTM4Controller::readStopMV(const enum TM4Channel eChannel)
{
	float fStopMV = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x9A), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fStopMV = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fStopMV;
}

unsigned char CTM4Controller::writeStopMV(const enum TM4Channel eChannel, const float fStopMV)
{
	unsigned int nStopMV = lowWord((unsigned int)(fStopMV * 10));

	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x9A), nStopMV);
}

unsigned char CTM4Controller::readStopAlarmOut(const enum TM4Channel eChannel)
{
	unsigned char nAlarmOut = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_CONTROL_REG_ADDR(eChannel,0x9B), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nAlarmOut = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nAlarmOut;
}

unsigned char CTM4Controller::writeStopAlarmOut(const enum TM4Channel eChannel, const unsigned char nStopAlarmOut)
{
	return this->write_SingleRegister(_TM4_CONTROL_REG_ADDR(eChannel,0x9B), lowWord(nStopAlarmOut & 0x01));
}

unsigned char CTM4Controller::readBaudrate(void)
{
	unsigned char nBaudrateIndex = TM4_INVALID_COMM_STATUS;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_COMM_REG_ADDR(0), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nBaudrateIndex = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nBaudrateIndex;
}

unsigned char CTM4Controller::writeBaudrate(const unsigned int nBaudrate)
{
	unsigned int nBaudrateIndex;

	switch (nBaudrate)
	{
		case 2400:
			nBaudrateIndex = TM4_BAUDRATE_2400;
			break;
		case 4800:
			nBaudrateIndex = TM4_BAUDRATE_4800;
			break;
		case 9600:
			nBaudrateIndex = TM4_BAUDRATE_9600;
			break;
		case 38400:
			nBaudrateIndex = TM4_BAUDRATE_38400;
			break;
		case 19200:
		default:
			nBaudrateIndex = TM4_BAUDRATE_19200;
			break;
	}

	return this->write_SingleRegister(_TM4_COMM_REG_ADDR(0), lowWord(nBaudrateIndex));
}

unsigned char CTM4Controller::readParityBit(void)
{
	unsigned char nParityBitIndex = TM4_INVALID_COMM_STATUS;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_COMM_REG_ADDR(1), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nParityBitIndex = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nParityBitIndex;
}

unsigned char CTM4Controller::writeParityBit(const unsigned char nParityBit)
{
	unsigned int nParityBitIndex = nParityBit <= TM4_PARITY_BIT_ODD ? lowWord(nParityBit) : TM4_PARITY_BIT_NONE;

	return this->write_SingleRegister(_TM4_COMM_REG_ADDR(1), nParityBitIndex);
}

unsigned char CTM4Controller::readStopBit(void)
{
	unsigned char nStopBitIndex = TM4_INVALID_COMM_STATUS;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_COMM_REG_ADDR(2), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nStopBitIndex = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nStopBitIndex;
}

unsigned char CTM4Controller::writeStopBit(const unsigned char nStopBit)
{
	unsigned int nStopBitIndex = nStopBit == 1 ? TM4_STOPBIT_ONE : TM4_STOPBIT_TWO;

	return this->write_SingleRegister(_TM4_COMM_REG_ADDR(2), lowWord(nStopBitIndex));
}

unsigned char CTM4Controller::readSensorInputType(const enum TM4Channel eChannel)
{
	unsigned char nSensorInputType = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,0), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nSensorInputType = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nSensorInputType;
}

unsigned char CTM4Controller::writeSensorInputType(const enum TM4Channel eChannel, const unsigned char nSensorType)
{
	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,0), lowWord(nSensorType));
}

unsigned char CTM4Controller::readSensorInputUnit(const enum TM4Channel eChannel)
{
	unsigned char nSensorInputUnit = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,1), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nSensorInputUnit = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nSensorInputUnit;
}

unsigned char CTM4Controller::writeSensorInputUnit(const enum TM4Channel eChannel, const unsigned char nSensorUnit)
{
	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,1), lowWord(nSensorUnit & 0x01));
}

float CTM4Controller::readSensorInputBias(const enum TM4Channel eChannel)
{
	float fSensorInputBias = TM4_INVALID_TEMPERATURE;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,2), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSensorInputBias = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSensorInputBias;
}

unsigned char CTM4Controller::writeSensorInputBias(const enum TM4Channel eChannel, const float fSensorInputBias)
{
	unsigned int nSensorInputBias = lowWord(int(fSensorInputBias * 10));

	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,2), nSensorInputBias);
}

float CTM4Controller::readSensorDigitalFilter(const enum TM4Channel eChannel)
{
	float fSensorDigitalFilter = 0.0F;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,3), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSensorDigitalFilter = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSensorDigitalFilter;
}

unsigned char CTM4Controller::writeSensorDigitalFilter(const enum TM4Channel eChannel, const float fSensorDigitalFilter)
{
	unsigned int nSensorDigitalFilter = lowWord((unsigned int)(fSensorDigitalFilter * 10));

	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,3), nSensorDigitalFilter);
}

float CTM4Controller::readSVLowLimit(const enum TM4Channel eChannel)
{
	float fSVLowLimt = 0.0F;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,4), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSVLowLimt = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSVLowLimt;
}

unsigned char CTM4Controller::writeSVLowLimit(const enum TM4Channel eChannel, const float fSVLowLimit)
{
	unsigned int nSetPointValueLowLimit = lowWord(int(fSVLowLimit * 10));

	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,4), nSetPointValueLowLimit);
}

float CTM4Controller::readSVHighLimit(const enum TM4Channel eChannel)
{
	float fSVHighLimit = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,5), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fSVHighLimit = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fSVHighLimit;
}

unsigned char CTM4Controller::writeSVHighLimit(const enum TM4Channel eChannel, const float fSVHighLimit)
{
	unsigned int nSetPointValueHighLimit = lowWord(int(fSVHighLimit * 10));

	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,5), nSetPointValueHighLimit);	
}

unsigned char CTM4Controller::readOperatingType(const enum TM4Channel eChannel)
{
	unsigned char nOperatingType = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,6), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nOperatingType = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nOperatingType;
}

unsigned char CTM4Controller::writeOperatingType(const enum TM4Channel eChannel, const unsigned char nOperatingType)
{
	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,6), lowWord(nOperatingType));
}

unsigned char CTM4Controller::readControlMethod(const enum TM4Channel eChannel)
{
	unsigned char nControlOperatingMethod = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,7), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nControlOperatingMethod = nReceivedData;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nControlOperatingMethod;
}

unsigned char CTM4Controller::writeControlMethod(const enum TM4Channel eChannel, const unsigned char nControlMethod)
{
	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,7), lowWord(nControlMethod));
}

unsigned char CTM4Controller::readAutotuningMode(const enum TM4Channel eChannel)
{
	unsigned char nHeaterAutotuningMode = 0;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,8), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		nHeaterAutotuningMode = nReceivedData & 0x01;
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return nHeaterAutotuningMode;
}

unsigned char CTM4Controller::writeAutotuningMode(const enum TM4Channel eChannel, const unsigned char nAutotuningMode)
{
	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,8), lowWord(nAutotuningMode & 0x01));
}

float CTM4Controller::readHeatingControlTime(const enum TM4Channel eChannel)
{
	float fHeatingControlTime = 0.0F;

	unsigned int nReceivedData = TM4_RXBUF_RESET;

	if (this->read_HoldingRegisters(_TM4_INITIAL_REG_ADDR(eChannel,9), 1, &nReceivedData) == MODBUS_RTU_SUCCESS)
	{
		fHeatingControlTime = float(int(nReceivedData) / 10.0f);
	}
	#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
	else {

		this->handle_InvalidResponse(nReceivedData);

	}
	#endif

	return fHeatingControlTime;
}

unsigned char CTM4Controller::writeHeatingControlTime(const enum TM4Channel eChannel, const float fHeatingControlTime)
{
	unsigned int nHeatingControlTime = lowWord((unsigned int)(fHeatingControlTime * 10));

	return this->write_SingleRegister(_TM4_INITIAL_REG_ADDR(eChannel,9), nHeatingControlTime);
}

int CTM4Controller::handle_InvalidSensorResponse(const int nResponse)
{
	int nErrorCode = TM4_SENSOR_ERROR_NONE;

	#if defined(_USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_)
	const _FlashString* pstrErrorMsg = NULL;
	#endif

	switch (nResponse)
	{
		case TM4_RXBUF_ERROR1 :
			nErrorCode = TM4_SENSOR_ERROR_OPENED;
			#if defined(_USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_)
			pstrErrorMsg = F("Sensor input Opened (not connected).");
			#endif
		break;
		case TM4_RXBUF_ERROR2 :
			nErrorCode = TM4_SENSOR_ERROR_UPPER_RANGE;
			#if defined(_USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_)
			pstrErrorMsg = F("Sensor input out of range (upper limit).");
			#endif
		break;
		case TM4_RXBUF_ERROR3 :
			nErrorCode = TM4_SENSOR_ERROR_LOWER_RANGE;
			#if defined(_USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_)
			pstrErrorMsg = F("Sensor input out of range (lower limit).");
			#endif
		break;
	}

	#if defined(_USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_)
	if (pstrErrorMsg != NULL) {
		cout << F("[SENSOR.ERR] ") << pstrErrorMsg << endl;
	}
	#endif

	return nErrorCode;
}

