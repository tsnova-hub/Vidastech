#ifndef _CTM4_CONTROLLER_H_
#define _CTM4_CONTROLLER_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include "../ModbusRTU/CModbusRTU.h"


#include "../MES_Config.h"


using namespace std;


#ifndef _FlashString
	#define _FlashString 							__FlashStringHelper	// char
#endif


#if defined(_USE_APPLICATION_DEBUG_MSG_)
	#define _USE_CTM4CONTROLLER_SENSOR_RESPONSE_HANDLER_
#endif


#if defined(_USE_MOVING_AVERAGED_TEMPERATURE_)
	#define _USE_MOVING_AVERAGE_
#endif

#define NUM_OF_TEMPERATURE_SAMPLES					(5) 			// Number of Samples to calc. Moving Average of Present Temperature (max. 10)


// SERIAL COMM. BAUDRATE FOR TEMPERATURE CONTROLLER (RS485-MODBUS RTU)
#define DEFAULT_TM4_BAUD_RATE						(9600)
#define DEFAULT_TM4_COMM_CONFIG						(SERIAL_8N2)
#define TM4_BAUD_RATE								(19200)			// Baud Rate of TM4 : 19200 [bps]
#define TM4_COMM_CONFIG								(SERIAL_8N2)	// 8 sets data, Non parity bits, 2 stop bits
// #define TM4_RCV_WAIT_TIME						(20)			// 20 msec, Max. Time to wait for serial incoming data


//////////////////////////////////////////////////////////////////////////////////////////
// DEFAULT REGISTER VALUE OF TM4 CONTROLLER

// COMMON INITIAL VALUE : Channel 1 ~ 4
#define DEFAULT_HEATING_CONTROL_TYPE				(0)				// AUTO
#define DEFAULT_MULTI_SETPOINT_NUM					(0)				// SV-0

#define DEFAULT_SENSOR_INPUT_TYPE					(20)			// JPt100.L
#define DEFAULT_SENSOR_INPUT_UNIT					(0)				// Celsius
#define DEFAULT_SENSOR_INPUT_BIAS					(1.0F)			// +1.0 deg C

#define DEFAULT_MIN_SENSOR_INPUT_BIAS				(-10.0F)		// -10.0 deg C (Spec : -999.0 ~ 999.9)
#define DEFAULT_MAX_SENSOR_INPUT_BIAS				(10.0F)			// +10.0 deg C (Spec : -999.0 ~ 999.9)

#define DEFAULT_HEATING_MIN_TEMPERATURE				(70.0F)			// 70.0 Degree C
#define DEFAULT_HEATING_MAX_TEMPERATURE				(96.0F)			// 96.0 Degree C
#define DEFAULT_HEATING_SV_TEMPERATURE				(92.0F)			// 92.0 Degree C

#define DEFAULT_HEATING_SV_LOW_LIMIT				(0.0F)			// 0.0 Degree C
#define DEFAULT_HEATING_SV_HIGH_LIMIT				(100.0F)		// 100.0 Degree C
#define DEFAULT_OPERATING_TYPE						(0)				// HEATING
#define DEFAULT_CONTROL_METHOD						(0)				// PID
#define DEFAULT_AUTOTUNING_MODE						(0)				// TUN1

#define DEFAULT_NUM_OF_MULTI_SETPOINT				(0)				// 1EA
#define DEFAULT_INITIAL_MANUAL_MV					(0)				// AUTO-MV
#define DEFAULT_PRESET_MANUAL_MV					(0.0F)			// 0.0 % (heating, cooling)
#define DEFAULT_SENSOR_ERROR_MV						(0.0F)			// 0.0 ~ 100.0 % (heating, cooling and PID)
#define DEFAULT_STOP_MV								(0.0F)			// 0.0 ~ 100.0 % (heating, cooling and PID)
#define DEFAULT_STOP_ALARM_OUT						(0)				// CONTINUE

// Channel 1 : PRE-HEAT BOILER
#define DEFAULT_CH1_SENSOR_INPUT_BIAS				(3.0F)			// -999.9 ~ 999.9 (L), -999.0 ~ 999.0 (H)
#define DEFAULT_CH1_SENSOR_DIGITAL_FILTER			(0.1F)			// 0.1 sec
#define DEFAULT_CH1_HEATING_CONTROL_TIME			(1.0F)			// 2.0 sec : SSR Default

#define DEFAULT_CH1_HEATING_PROPORTIONAL_BAND		(2.9F)
#define DEFAULT_CH1_HEATING_INTEGRAL_TIME			(90)
#define DEFAULT_CH1_HEATING_DERIVATION_TIME			(25)
#define DEFAULT_CH1_HEATING_MV_LOW_LIMIT			(0.0F)			// 0.0 ~ MV High Limit - 0.1 % (heating, cooling)
#define DEFAULT_CH1_HEATING_MV_HIGH_LIMIT			(80.0F)			// MV Low Limt + 0.1 ~ 100.0 % (heating, cooling)
#define DEFAULT_CH1_RAMP_UP_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH1_RAMP_DOWN_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH1_RAMP_TIME_UNIT					(1)				// MINUNTE

// Channel 2 : BREW BOILER
#define DEFAULT_CH2_SENSOR_INPUT_BIAS				(0.0F)			// -999.9 ~ 999.9 (L), -999.0 ~ 999.0 (H)
#define DEFAULT_CH2_SENSOR_DIGITAL_FILTER			(0.1F)			// 0.1 sec
#define DEFAULT_CH2_HEATING_CONTROL_TIME			(0.1F)			// 2.0 sec : SSR Default

#define DEFAULT_CH2_HEATING_PROPORTIONAL_BAND		(2.0F)
#define DEFAULT_CH2_HEATING_INTEGRAL_TIME			(30)
#define DEFAULT_CH2_HEATING_DERIVATION_TIME			(10)
#define DEFAULT_CH2_HEATING_MV_LOW_LIMIT			(0.0F)			// 0.0 ~ MV High Limit - 0.1 % (heating, cooling)
#define DEFAULT_CH2_HEATING_MV_HIGH_LIMIT			(80.0F)			// MV Low Limt + 0.1 ~ 100.0 % (heating, cooling)
#define DEFAULT_CH2_RAMP_UP_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH2_RAMP_DOWN_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH2_RAMP_TIME_UNIT					(1)				// MINUNTE

// Channel 3 : GROUPHEAD
#define DEFAULT_CH3_SENSOR_INPUT_BIAS				(0.0F)			// -999.9 ~ 999.9 (L), -999.0 ~ 999.0 (H)
#define DEFAULT_CH3_SENSOR_DIGITAL_FILTER			(0.1F)			// 0.1 sec
#define DEFAULT_CH3_HEATING_CONTROL_TIME			(0.1F)			// 2.0 sec : SSR Default

#define DEFAULT_CH3_HEATING_PROPORTIONAL_BAND		(24.0F)
#define DEFAULT_CH3_HEATING_INTEGRAL_TIME			(80)
#define DEFAULT_CH3_HEATING_DERIVATION_TIME			(15)
#define DEFAULT_CH3_HEATING_MV_LOW_LIMIT			(0.0F)			// 0.0 ~ MV High Limit - 0.1 % (heating, cooling)
#define DEFAULT_CH3_HEATING_MV_HIGH_LIMIT			(90.0F)			// MV Low Limt + 0.1 ~ 100.0 % (heating, cooling)
#define DEFAULT_CH3_RAMP_UP_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH3_RAMP_DOWN_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH3_RAMP_TIME_UNIT					(1)				// MINUNTE

// Channel 4 : BREWING WATER
#define DEFAULT_CH4_SENSOR_INPUT_BIAS				(0.0F)			// -999.9 ~ 999.9 (L), -999.0 ~ 999.0 (H)
#define DEFAULT_CH4_SENSOR_DIGITAL_FILTER			(0.1F)			// 0.1 sec
#define DEFAULT_CH4_HEATING_CONTROL_TIME			(1.0F)			// 2 : SSR Default

#define DEFAULT_CH4_HEATING_PROPORTIONAL_BAND		(10.0F)
#define DEFAULT_CH4_HEATING_INTEGRAL_TIME			(0)
#define DEFAULT_CH4_HEATING_DERIVATION_TIME			(0)
#define DEFAULT_CH4_HEATING_MV_LOW_LIMIT			(0.0F)			// 0.0 ~ MV High Limit - 0.1 % (heating, cooling)
#define DEFAULT_CH4_HEATING_MV_HIGH_LIMIT			(100.0F)		// MV Low Limt + 0.1 ~ 100.0 % (heating, cooling)
#define DEFAULT_CH4_RAMP_UP_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH4_RAMP_DOWN_RATE					(0)				// 0(OFF) ~ 9999
#define DEFAULT_CH4_RAMP_TIME_UNIT					(1)				// MINUNTE


//////////////////////////////////////////////////////////////////////////////////////////
// MODBUS RTU LIBRARY RELATED CONSTANTS DEFINITION

// TM4 RELATED DEFINITIONS
#define TM4_TOTAL_CH_PER_UNIT						(4)
#define TM4_TOTAL_NO_OF_CH							(TM4_TOTAL_CH_PER_UNIT*1)
#define TM4_UNIT_BASE_ADDR							(0x00)
#define TM4_MASTER_ADDR 							(TM4_UNIT_BASE_ADDR+1)

#define TM4_INITIAL_GRP_REG_OFFSET					(0x03E8)
#define TM4_CH1_INITIAL_REG_ADDR					(0x0096)
#define TM4_CH2_INITIAL_REG_ADDR					(0x047E)
#define TM4_CH3_INITIAL_REG_ADDR					(0x0866)
#define TM4_CH4_INITIAL_REG_ADDR					(0x0C4E)
#define _TM4_CH_INITIAL_GRP_START_ADDR(_ch)			(TM4_CH1_INITIAL_REG_ADDR+TM4_INITIAL_GRP_REG_OFFSET*((_ch-1)%TM4_TOTAL_CH_PER_UNIT))
#define _TM4_INITIAL_REG_ADDR(_ch,_offset)			(_TM4_CH_INITIAL_GRP_START_ADDR(_ch) + _offset)

#define TM4_INPUT_GRP_REG_OFFSET					(0x0006)
#define TM4_CH1_INPUT_REG_ADDR						(0x03E8)
#define TM4_CH2_INPUT_REG_ADDR						(0x03EE)
#define TM4_CH3_INPUT_REG_ADDR						(0x03F4)
#define TM4_CH4_INPUT_REG_ADDR						(0x03FA)
#define _TM4_CH_INPUT_GRP_START_ADDR(_ch)			(TM4_CH1_INPUT_REG_ADDR+TM4_INPUT_GRP_REG_OFFSET*((_ch-1)%TM4_TOTAL_CH_PER_UNIT))
#define _TM4_INPUT_REG_ADDR(_ch,_offset)			(_TM4_CH_INPUT_GRP_START_ADDR(_ch) + _offset)

#define TM4_MONITOR_GRP_REG_OFFSET					(0x03E8)
#define TM4_CH1_MONITOR_REG_START_ADDR				(0x0000)
#define TM4_CH2_MONITOR_REG_START_ADDR				(0x03E8)
#define TM4_CH3_MONITOR_REG_START_ADDR				(0x07D0)
#define TM4_CH4_MONITOR_REG_START_ADDR				(0x0BB8)
#define _TM4_CH_MONITOR_GRP_START_ADDR(_ch)			(TM4_CH1_MONITOR_REG_START_ADDR+TM4_MONITOR_GRP_REG_OFFSET*((_ch-1)%TM4_TOTAL_CH_PER_UNIT))
#define _TM4_MONITOR_REG_ADDR(_ch,_offset)			(_TM4_CH_MONITOR_GRP_START_ADDR(_ch) + _offset)

#define TM4_CONTROL_GRP_REG_OFFSET					(0x03E8)
#define TM4_CH1_CONTROL_REG_START_ADDR				(0x0032)
#define TM4_CH2_CONTROL_REG_START_ADDR				(0x041A)
#define TM4_CH3_CONTROL_REG_START_ADDR				(0x0802)
#define TM4_CH4_CONTROL_GRP_START_ADDR				(0x0BEA)
#define _TM4_CONTROL_GRP_START_ADDR(_ch)			(TM4_CH1_CONTROL_REG_START_ADDR+TM4_CONTROL_GRP_REG_OFFSET*((_ch-1)%TM4_TOTAL_CH_PER_UNIT))
#define _TM4_CONTROL_REG_ADDR(_ch,_offset)			(_TM4_CONTROL_GRP_START_ADDR(_ch) + _offset)

#define TM4_COILSTATUS_GRP_REG_OFFSET				(0x0002)
#define TM4_CH1_COILSTATUS_REG_START_ADDR			(0x0000)
#define TM4_CH2_COILSTATUS_REG_START_ADDR			(0x0002)
#define TM4_CH3_COILSTATUS_REG_START_ADDR			(0x0004)
#define TM4_CH4_COILSTATUS_GRP_START_ADDR			(0x0006)
#define _TM4_COILSTATUS_GRP_START_ADDR(_ch)			(TM4_CH1_COILSTATUS_REG_START_ADDR+TM4_COILSTATUS_GRP_REG_OFFSET*((_ch-1)%TM4_TOTAL_CH_PER_UNIT))
#define _TM4_COILSTATUS_REG_ADDR(_ch,_offset)		(_TM4_COILSTATUS_GRP_START_ADDR(_ch) + _offset)

#define TM4_COMM_REG_START_ADDR						(0x012C)
#define _TM4_COMM_REG_ADDR(_offset)					(TM4_COMM_REG_START_ADDR + _offset)

#define TM4_BAUDRATE_2400							(0)
#define TM4_BAUDRATE_4800							(1)
#define TM4_BAUDRATE_9600							(2)
#define TM4_BAUDRATE_19200							(3)
#define TM4_BAUDRATE_38400							(4)

#define TM4_PARITY_BIT_NONE							(0)
#define TM4_PARITY_BIT_EVEN							(1)
#define TM4_PARITY_BIT_ODD							(2)

#define TM4_STOPBIT_ONE								(0)
#define TM4_STOPBIT_TWO								(1)

#define TM4_OUTPUT_CONTROL_RUN						(0x00)
#define TM4_OUTPUT_CONTROL_STOP						(0x01)

#define TM4_AUTOTUNING_OFF							(0x00)
#define TM4_AUTOTUNING_ON							(0x01)

#define TM4_RXBUF_RESET								(MODBUS_RXBUF_RESET)
#define TM4_RXBUF_ERROR1							31000		// when Sensor Input : Opened
#define TM4_RXBUF_ERROR2							30000		// when Sensor Input : Out of Range(+)
#define TM4_RXBUF_ERROR3							-30000		// when Sensor Input : Out of Range(-)

#define TM4_SENSOR_ERROR_NONE						(0)			// No Sensor Error
#define TM4_SENSOR_ERROR_OPENED						(-1)		// TM4_RXBUF_ERROR1
#define TM4_SENSOR_ERROR_UPPER_RANGE				(-2)		// TM4_RXBUF_ERROR2
#define TM4_SENSOR_ERROR_LOWER_RANGE				(-3)		// TM4_RXBUF_ERROR3

#define TM4_INVALID_TEMPERATURE						(0.0F)		// when failed to read TM4 Register (Temperature Data)
#define TM4_INVALID_COIL_STATUS						(-1)		// when failed to read TM4 Coil Status Register
#define TM4_INVALID_CONTROL_STATUS					(-1)		// when failed to read TM4 Control Operation Status Register (Output Control / Autotuning)
#define TM4_INVALID_COMM_STATUS						(0xFF)		// when failed to read TM4 Communication Setting Register


//////////////////////////////////////////////////////////////
// STRUCTURE DATA TYPE DEFINITION

typedef struct _ST_TM4_TEMPERATURE_T {
	float fPresentTemperature[TM4_TOTAL_NO_OF_CH];
	float fSetTemperature[TM4_TOTAL_NO_OF_CH];
	int nSensorErrorCode[TM4_TOTAL_NO_OF_CH];
} stTM4Temperature_t;


class CTM4Controller : public CModbusRTU
{
public:
	enum TM4Channel {TM4ALL=0, TM4CH1=1, TM4CH2, TM4CH3, TM4CH4};

	CTM4Controller();

	void setModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr = TM4_MASTER_ADDR);
	void setPreTransmission(void (*preTrasmission)(void));
	void setPostTransmission(void (*PostTrasmission)(void));

	void clear(void);
#ifdef _USE_MOVING_AVERAGE_
	float calculate_AveragedTemperature(const int nChannel, const float fPresentTemperature);
#endif
	bool resetFactoryDefault(void);

	// for Setup-Menu and Data-Acquisition
	void readPresentTemperaturesAll(void);
	float readPresentTemperature(const enum TM4Channel eChannel);
	float readSetTemperature(const enum TM4Channel eChannel);
	unsigned char writeSetTemperature(const enum TM4Channel eChannel, const float fSetTemperature);

	unsigned char enableOutputControl(const enum TM4Channel eChannel);
	unsigned char disableOutputControl(const enum TM4Channel eChannel);
	unsigned char enableAutoTuning(const enum TM4Channel eChannel);
	unsigned char disableAutoTuning(const enum TM4Channel eChannel);
	int readOutputControlStatus(const enum TM4Channel eChannel);
	int readAutotuningStatus(const enum TM4Channel eChannel);

	unsigned char startOutputControl(const enum TM4Channel eChannel);
	unsigned char stopOutputControl(const enum TM4Channel eChannel);
	unsigned char startAutotuning(const enum TM4Channel eChannel);
	unsigned char stopAutotuning(const enum TM4Channel eChannel);
	int readCoilStatusOfOutputControl(const enum TM4Channel eChannel);
	int readCoilStatusOfAutotuning(const enum TM4Channel eChannel);

	void readPresentHeatingMVAll(void);
	float readPresentHeatingMV(const enum TM4Channel eChannel);

	// only for factory default reset
	unsigned char readHeatingControlType(const enum TM4Channel eChannel);
	unsigned char writeHeatingControlType(const enum TM4Channel eChannel, const unsigned char nControlType);

	unsigned char readMultiSetPointNum(const enum TM4Channel eChannel);
	unsigned char writeMultiSetPointNum(const enum TM4Channel eChannel, const unsigned char nSetPointNum);

	float readHeatingProportionalBand(const enum TM4Channel eChannel);
	unsigned char writeHeatingProportionalBand(const enum TM4Channel eChannel, const float fProportionalBand);
	unsigned int readHeatingIntegralTime(const enum TM4Channel eChannel);
	unsigned char writeHeatingIntegralTime(const enum TM4Channel eChannel, const unsigned int nIntegralTime);
	unsigned int readHeatingDerivationTime(const enum TM4Channel eChannel);
	unsigned char writeHeatingDerivationTime(const enum TM4Channel eChannel, const unsigned int nDerivationTime);
	float readMVLowLimit(const enum TM4Channel eChannel);
	unsigned char writeMVLowLimit(const enum TM4Channel eChannel, const float fMVLowLimit);
	float readMVHighLimit(const enum TM4Channel eChannel);
	unsigned char writeMVHighLimit(const enum TM4Channel eChannel, const float fMVHighLimit);

	unsigned int readRampUpRate(const enum TM4Channel eChannel);
	unsigned char writeRampUpRate(const enum TM4Channel eChannel, const unsigned int nRampUpRate);
	unsigned int readRampDownRate(const enum TM4Channel eChannel);
	unsigned char writeRampDownRate(const enum TM4Channel eChannel, const unsigned int nRampDownRate);
	unsigned char readRampTimeUnit(const enum TM4Channel eChannel);
	unsigned char writeRampTimeUnit(const enum TM4Channel eChannel, const unsigned char nRampTimeUnit);

	unsigned char readNumOfSetPoint(const enum TM4Channel eChannel);
	unsigned char writeNumOfSetPoint(const enum TM4Channel eChannel, const unsigned char nNumOfSetPoint);
	unsigned char readInitialManualMV(const enum TM4Channel eChannel);
	unsigned char writeInitialManualMV(const enum TM4Channel eChannel, const unsigned char nInitialManualMV);
	float readPresetManualMV(const enum TM4Channel eChannel);
	unsigned char writePresetManualMV(const enum TM4Channel eChannel, const float fPresetManualMV);
	float readSensorErrorMV(const enum TM4Channel eChannel);
	unsigned char writeSensorErrorMV(const enum TM4Channel eChannel, const float fSensorErrorMV);
	float readStopMV(const enum TM4Channel eChannel);
	unsigned char writeStopMV(const enum TM4Channel eChannel, const float fStopMV);
	unsigned char readStopAlarmOut(const enum TM4Channel eChannel);
	unsigned char writeStopAlarmOut(const enum TM4Channel eChannel, const unsigned char nStopAlarmOut);

	unsigned char readBaudrate(void);
	unsigned char writeBaudrate(const unsigned int nBaudrate);
	unsigned char readParityBit(void);
	unsigned char writeParityBit(const unsigned char nParityBit);
	unsigned char readStopBit(void);
	unsigned char writeStopBit(const unsigned char nStopBit);

	unsigned char readSensorInputType(const enum TM4Channel eChannel);
	unsigned char writeSensorInputType(const enum TM4Channel eChannel, const unsigned char nSensorType);
	unsigned char readSensorInputUnit(const enum TM4Channel eChannel);
	unsigned char writeSensorInputUnit(const enum TM4Channel eChannel, const unsigned char nSensorUnit);
	float readSensorInputBias(const enum TM4Channel eChannel);
	unsigned char writeSensorInputBias(const enum TM4Channel eChannel, const float fSensorInputTune);
	float readSensorDigitalFilter(const enum TM4Channel eChannel);
	unsigned char writeSensorDigitalFilter(const enum TM4Channel eChannel, const float fSensorDigitalFilter);

	float readSVLowLimit(const enum TM4Channel eChannel);
	unsigned char writeSVLowLimit(const enum TM4Channel eChannel, const float fSVLowLimit);
	float readSVHighLimit(const enum TM4Channel eChannel);
	unsigned char writeSVHighLimit(const enum TM4Channel eChannel, const float nSVHighLimit);
	unsigned char readOperatingType(const enum TM4Channel eChannel);
	unsigned char writeOperatingType(const enum TM4Channel eChannel, const unsigned char nOperatingType);
	unsigned char readControlMethod(const enum TM4Channel eChannel);
	unsigned char writeControlMethod(const enum TM4Channel eChannel, const unsigned char nControlMethod);
	unsigned char readAutotuningMode(const enum TM4Channel eChannel);
	unsigned char writeAutotuningMode(const enum TM4Channel eChannel, const unsigned char nAutotuningMode);
	float readHeatingControlTime(const enum TM4Channel eChannel);
	unsigned char writeHeatingControlTime(const enum TM4Channel eChannel, const float fHeatingControlTime);

	float getPresentTemperatureValue(const enum TM4Channel eChannel) const { return _stThermoSensor.fPresentTemperature[eChannel-1]; }
	float getSetTemperatureValue(const enum TM4Channel eChannel) const { return _stThermoSensor.fSetTemperature[eChannel-1]; }
	void updateSetTemperatureValue(const enum TM4Channel eChannel, const float fSetTemp) { _stThermoSensor.fSetTemperature[eChannel-1] = fSetTemp; }

	float getPresentHeatingMV(const enum TM4Channel eChannel) const { return _fPresentHeatingMV[eChannel-1]; }

	bool isSensorAlarm(const enum TM4Channel eChannel) const { return _stThermoSensor.nSensorErrorCode[eChannel-1] < 0; }
	int getSensorErrorCode(const enum TM4Channel eChannel) const { return _stThermoSensor.nSensorErrorCode[eChannel-1]; }

private:
	stTM4Temperature_t _stThermoSensor;

	float _fPresentHeatingMV[TM4_TOTAL_NO_OF_CH];

#ifdef _USE_MOVING_AVERAGE_
	float _fTempSamples[TM4_TOTAL_NO_OF_CH][NUM_OF_TEMPERATURE_SAMPLES];
	float _fSum_of_samples[TM4_TOTAL_NO_OF_CH];
	
	unsigned int _nIndex_of_sample[TM4_TOTAL_NO_OF_CH];
	unsigned int _nNum_of_samples[TM4_TOTAL_NO_OF_CH];
#endif

	int handle_InvalidSensorResponse(const int nRespose);
};


#endif //_CTM4_CONTROLLER_H_

