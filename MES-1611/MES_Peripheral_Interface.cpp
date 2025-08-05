

#include "MES_SetupMenu.h"

#include "MES_Peripheral_Interface.h"


//////////////////////////////////////////////////////////////////////////////////////////
//			EXTERNAL PERIPHERAL CONTROL ACCESS
///////////////////////////////////////////////////////////////////////con///////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// INSTANTIATIONS OF HANDLE FOR EXTERNAL PERIPHERALS

CRTClock hRTClock = CRTClock();

CTM4Controller hTM4Controller = CTM4Controller();

CFlowmeter hFlowmeter(WATER_FLOWMETER, FLOWMETER_TRIGGER_TYPE);

CSolenoidValve hBrewValve(BREW_SOLENOID_VALVE, SOLENOID_VALVE_TYPE);

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
CMotorPump hMotorPump(BLDC_MOTOR_SPEED, BLDC_MOTOR_STOP, BLDC_MOTOR_RESET, BLDC_MOTOR_ALARM, BLDC_MOTOR_SPEEDOUT);
#else
CMotorPumpBLDC hMotorPump = CMotorPumpBLDC();
#endif

CPressureSensor hPressureBrewBoiler(BREW_PRESSURE_SENSOR);
#if PUMP_PRESSURE_SENSOR_ATTACHED
CPressureSensor hPressureFeed(FEED_PRESSURE_SENSOR);
#endif
#if AUX_CURRENT_SENSOR_ATTACHED
CPressureSensor hPressureAUX(AUX_ANALOG_CH1);
#endif

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
CAnalogSensor hAnalogInput(AUX_ANALOG_CH2, ANALOG_SIGNAL_CONDITION);
#endif
#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
CExternalSignal hExternalSignal(AUX_EXTERNAL_INTERRUPT, EXTERNAL_SIGNAL_ACTIVE_TYPE);
#endif

#if AUTOMATION_SENSOR_ATTACHED
COpticalFiber hOpticalFiber(OPTICAL_FIBER_AMP_POWER, OPTICAL_FIBER_SIGNAL, OPTICAL_FIBER_DIAGNOSIS, OPTICAL_FIBER_AMP_ACTIVE_TYPE);
#endif

CBrewButton hBrewButton(BREW_BUTTON_SWITCH, BREW_BUTTON_LED);
CRotaryEncoder hRotaryEncoder(ROTARY_ENCODER_ST, ROTARY_ENCODER_DI, ROTARY_ENCODER_SW);

CLamp hLamp(GROUPHEAD_LAMP);

CTextLCD hTextLCD(LCD_BACKLIGHT_LSB, LCD_BACKLIGHT_MSB, LCD_BACKLIGHT_COLOR, LCD_I2C_ADDR);

CPixelLED hPixelLED(NEOPIXEL_DIN);



//////////////////////////////////////////////////////////////////////////////////////////
// WATER FLOWMETER SENSOR ACCESS(READ/CALCUATION) FOR DATA ACQUISITION

void initialize_Flowmeter(void)
{
	hFlowmeter.clear();

	// flowmeter tuning factor : use default value
	// hFlowmeter.tune(WATERFLOW_QUANTITY_FACTOR);

#if defined(_USE_FLOWMETER_PROC_MODE_)
	hFlowmeter.setPostProcessing(postProcessing_Flowmeter);
#endif	
}

void tune_Flowmeter(const float fFactor)
{
	noInterrupts();

	hFlowmeter.tune(fFactor);

	interrupts();
}

void start_Flowmeter(void)
{
	hFlowmeter.start();

	enable_FlowmeterInterrupt();
}

void stop_Flowmeter(void)
{
	disable_FlowmeterInterrupt();

	hFlowmeter.stop();
}

void reset_Flowmeter(void)
{
	noInterrupts();

	hFlowmeter.clear();

	interrupts();
}

unsigned int get_FlowmeterCNT(void)
{
	unsigned int unWaterFlowCount;

	noInterrupts();

	unWaterFlowCount = hFlowmeter.getFlowmeterCNT();

	interrupts();

	return unWaterFlowCount;
}

float get_WaterFlowQuantity(const enum CFlowmeter::WaterUnit nUnit)
{
	float fWaterFlowQTY;

	noInterrupts();

	fWaterFlowQTY = hFlowmeter.getWaterFlowQTY(nUnit);

	interrupts();

	return fWaterFlowQTY;
}

float get_AveragedFlowmeterQTY(const enum CFlowmeter::FlowUnit nUnit)
{
	float fFlowmeterQTY;

	noInterrupts();

	fFlowmeterQTY = hFlowmeter.getFlowmeterQTY(nUnit);

	interrupts();

	return fFlowmeterQTY;
}

float get_FlowmeterFactor(void)
{
	float fTuningFactor;

	// noInterrupts();

	fTuningFactor = hFlowmeter.getTuningFactor();

	// interrupts();

	return fTuningFactor;
}

#if defined(_USE_FLOWMETER_PROC_MODE_)
void postProcessing_Flowmeter(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[POST_PROC] Flowmeter::External Interrupt[#18]" << endl;
#endif
}
#endif

// Polling Water Flow Status to decay flowmeter RPM
void PollingWaterFlowStatus_Callback(void)
{
	hFlowmeter.decayFlowmeterRPM();
}

// Interrupt Handler : Trigger of Water Flowmeter Sensor
void TriggerWaterFlowmeter_Callback(void)
{
	hFlowmeter.measureFlowmeter();
}

void enable_FlowmeterInterrupt(void)
{
	// delay(FLOWMETER_NOISE_REDUCTION_TIME);

	enable_ExternalInterrupt(TriggerWaterFlowmeter_Callback, PIN_INTERRUPT_NUM_5, FLOWMETER_TRIGGER_TYPE);
}

void disable_FlowmeterInterrupt(void)
{
	disable_ExternalInterrupt(PIN_INTERRUPT_NUM_5);
}


//////////////////////////////////////////////////////////////////////////////////////////
// PRESSURE SENSOR ACCESS(READ/CALCUATION) FOR DATA ACQUISITION

void initialize_PressureSensor(void)
{
	hPressureBrewBoiler.clear();
	hPressureBrewBoiler.setPressureTuneValue(PRESSURE_TUNE_FACTOR);

#if PUMP_PRESSURE_SENSOR_ATTACHED
	hPressureFeed.clear();
	hPressureFeed.setPressureTuneValue(PRESSURE_TUNE_FACTOR);
#endif

#if AUX_CURRENT_SENSOR_ATTACHED
	hPressureAUX.clear();
	hPressureAUX.setPressureTuneValue(PRESSURE_TUNE_FACTOR);
#endif
}

void set_PressureSensorZeroShift(CPressureSensor& rhPressureSensor)
{
	noInterrupts();

	rhPressureSensor.setPressureZeroShift();

	interrupts();
}

void set_PressureSensorSpanFactor(CPressureSensor& rhPressureSensor)
{
	noInterrupts();

	rhPressureSensor.setPressureSpanFactor();

	interrupts();
}

void reset_PressureSensorAdjustment(CPressureSensor& rhPressureSensor)
{
	noInterrupts();

	rhPressureSensor.resetPressureAdjustment();

	interrupts();
}

void set_PressureSensorZeroShiftWithValue(CPressureSensor& rhPressureSensor, const float fZeroShift)
{
	float fZeroShiftValue = constrain(fZeroShift, -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);

	rhPressureSensor.setZeroShiftValue(fZeroShiftValue);
}

void set_PressureSensorSpanFactorWithValue(CPressureSensor& rhPressureSensor, const float fSpanFactor)
{
	float fSpanFactorValue = constrain(fSpanFactor, 0.001f, 2.000f);

	rhPressureSensor.setSpanFactorValue(fSpanFactorValue);
}

void reset_PressureSensor(CPressureSensor& rhPressureSensor)
{
	noInterrupts();

	rhPressureSensor.clear();

	interrupts();
}

void reset_PressureSensor(void)
{
	noInterrupts();

	hPressureBrewBoiler.clear();
#if PUMP_PRESSURE_SENSOR_ATTACHED
	hPressureFeed.clear();
#endif
#if AUX_CURRENT_SENSOR_ATTACHED
	hPressureAUX.clear();
#endif

	interrupts();
}

float get_PressureSensorValue(CPressureSensor& rhPressureSensor)
{
	float fPressureValue;

	noInterrupts();

	fPressureValue = rhPressureSensor.getPressureValue();

	interrupts();

	return fPressureValue;
}

float get_BrewBoilerPressureValue(void)
{
	float fPressureValue;

	noInterrupts();

	fPressureValue = hPressureBrewBoiler.getPressureValue();

	interrupts();

	return fPressureValue;
}

#if PUMP_PRESSURE_SENSOR_ATTACHED
float get_PumpPressureValue(void)
{
	float fPressureValue;

	noInterrupts();

	fPressureValue = hPressureFeed.getPressureValue();

	interrupts();

	return fPressureValue;
}
#endif

#if AUX_CURRENT_SENSOR_ATTACHED
float get_AuxPressureValue(void)
{
	float fPressureValue;

	noInterrupts();

	fPressureValue = hPressureAUX.getPressureValue();

	interrupts();

	return fPressureValue;
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
// TEMPERATURE CONTROLLER ACCESS (TM4)

void initialize_TM4Controller(void)
{
	// Set PORTJ pin 5,6 direction (RE,DE : output) and Init in receive mode
	_init_RS485();

	#if USE_TM4_RS485_COMM_PROTOCOL
	_serial485.begin(TM4_BAUD_RATE, TM4_COMM_CONFIG);
	#else
	_serial485.begin(TM4_BAUD_RATE);	// default to SERIAL_8N1(8 sets data, Non parity bits, 1 stop bits)
	#endif

	hTM4Controller.setModbusNode(_serial485, TM4_MASTER_ADDR);

	hTM4Controller.setPreTransmission(control_preRS485Transmission);
	hTM4Controller.setPostTransmission(control_postRS485Trasmission);

	if (hTM4Controller.readBaudrate() == TM4_INVALID_COMM_STATUS) {

		if (setup_TM4ControllerCommunication() == false) {

			#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
			cout << F("[ERROR] TM4 Communication Setting Failed.") << endl;
			#endif

			display_TM4CommunicationSettingFail();

		}
		else {

			#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
			cout << F("[BOOT] TM4 Communication Setting Succeed.") << endl;
			#endif

			display_TM4CommunicationSettingPass();

		}

		wait_RebootSystemWithConsoleMsg();

	}

	hTM4Controller.clear();
}

void control_preRS485Transmission(void)
{
#if defined(_USE_PORT_ACCESS_INTRINSIC_)
      PORTJ = PORTJ & B10011111;
      PORTJ = PORTJ | B01100000;
#else
	_set_RS485RE(HIGH);
	_set_RS485DE(HIGH);
#endif //_USE_PORT_ACCESS_INTRINSIC_
}

void control_postRS485Trasmission(void)
{
#if defined(_USE_PORT_ACCESS_INTRINSIC_)
      PORTJ = PORTJ & B10011111;
      PORTJ = PORTJ | B00000000;
#else
	_set_RS485RE(LOW);
	_set_RS485DE(LOW);
#endif //_USE_PORT_ACCESS_INTRINSIC_
}

void set_ModbusNodeAddress(const unsigned char nSlaveAddr)
{
	hTM4Controller.setModbusNode(_serial485, nSlaveAddr);
}

bool setup_TM4ControllerCommunication(void)
{
	bool bSuccess = false;

	_serial485.begin(DEFAULT_TM4_BAUD_RATE, TM4_COMM_CONFIG);

	if (hTM4Controller.readBaudrate() == TM4_BAUDRATE_9600) {

		hTM4Controller.writeBaudrate(TM4_BAUD_RATE);

		_serial485.begin(TM4_BAUD_RATE, TM4_COMM_CONFIG);

		bSuccess = true;

	}
	else {

		unsigned int nBaudrate[] = {2400, 4800, 9600, 19200, 38400};

		for (int nIndex = TM4_BAUDRATE_2400; nIndex <= TM4_BAUDRATE_38400; nIndex++) {

			_serial485.begin(nBaudrate[nIndex], TM4_COMM_CONFIG);

			if (hTM4Controller.readBaudrate() == nIndex) {

				hTM4Controller.writeBaudrate(TM4_BAUD_RATE);

				_serial485.begin(TM4_BAUD_RATE, TM4_COMM_CONFIG);

				bSuccess = true;

				break;

			}

		}

	}

	return bSuccess;
}

bool reset_TM4ControllerToFactoryDefault(void)
{
	bool bResult = hTM4Controller.resetFactoryDefault();

	if (!bResult) {

		cout << F("[ERROR] TM4 Controller Init Failure.") << endl;

	}

	return bResult;
}

#if defined(_USE_APPLICATION_DEBUG_MSG_)
void verify_TM4ControllerFactoryDefault(void)
{
	cout << F("[TM4 FACTORY DEFAULT VERIFICATION]") << endl;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		unsigned char HeatingControlType = hTM4Controller.readHeatingControlType(eChannel);
		unsigned char MultiSetPointNum = hTM4Controller.readMultiSetPointNum(eChannel);

		unsigned char SensorInputType = hTM4Controller.readSensorInputType(eChannel);
		unsigned char SensorInputUnit = hTM4Controller.readSensorInputUnit(eChannel);
		float SensorInputBias = hTM4Controller.readSensorInputBias(eChannel);
		float SensorDigitalFilter = hTM4Controller.readSensorDigitalFilter(eChannel);
		float SVLowLimit = hTM4Controller.readSVLowLimit(eChannel);
		float SVHighLimit = hTM4Controller.readSVHighLimit(eChannel);
		unsigned char OperatingType = hTM4Controller.readOperatingType(eChannel);
		unsigned char ControlMethod = hTM4Controller.readControlMethod(eChannel);
		unsigned char AutotuningMode = hTM4Controller.readAutotuningMode(eChannel);
		float HeatingControlTime = hTM4Controller.readHeatingControlTime(eChannel);

		float ProportionalBand = hTM4Controller.readHeatingProportionalBand(eChannel);
		unsigned int IntegralTime = hTM4Controller.readHeatingIntegralTime(eChannel);
		unsigned int DerivationTime = hTM4Controller.readHeatingDerivationTime(eChannel);
		float MVLowLimit = hTM4Controller.readMVLowLimit(eChannel);
		float MVHighLimit = hTM4Controller.readMVHighLimit(eChannel);
		unsigned int RampUpRate = hTM4Controller.readRampUpRate(eChannel);
		unsigned int RampDownRate = hTM4Controller.readRampDownRate(eChannel);
		unsigned char RampTimeUnit = hTM4Controller.readRampTimeUnit(eChannel);

		unsigned char NumOfSetPoint = hTM4Controller.readNumOfSetPoint(eChannel);
		unsigned char InitialManualMV = hTM4Controller.readInitialManualMV(eChannel);
		float PresetManualMV = hTM4Controller.readPresetManualMV(eChannel);
		float SensorErrorMV = hTM4Controller.readSensorErrorMV(eChannel);
		unsigned char StopMV = hTM4Controller.readStopMV(eChannel);
		unsigned char StopAlarmOut = hTM4Controller.readStopAlarmOut(eChannel);

		cout << F("TM4 Channel[") << nChannel << F("]::") << endl;

		cout << F("\tHeating Control Type: ") << int(HeatingControlType) << endl;
		cout << F("\tMulti SetPoint Num: ") << int(MultiSetPointNum) << endl;

		cout << F("\tSensor Input Type: ") << int(SensorInputType) << endl;
		cout << F("\tSensor Input Unit: ") << int(SensorInputUnit) << endl;
		cout << F("\tSensor Input Bias: ") << SensorInputBias << endl;
		cout << F("\tSensor Digital Filter: ") << SensorDigitalFilter << endl;
		cout << F("\tSV Low Limit: ") << SVLowLimit << endl;
		cout << F("\tSV High Limit: ") << SVHighLimit << endl;
		cout << F("\tOperating Type: ") << int(OperatingType) << endl;
		cout << F("\tControl Method: ") << int(ControlMethod) << endl;
		cout << F("\tAutotuning Mode: ") << int(AutotuningMode) << endl;
		cout << F("\tHeating Control Time: ") << HeatingControlTime << endl;

		cout << F("\tProportional Band: ") << ProportionalBand << endl;
		cout << F("\tIntegral Time: ") << IntegralTime << endl;
		cout << F("\tDerivation Time: ") << DerivationTime << endl;
		cout << F("\tMV Low Limit: ") << MVLowLimit << endl;
		cout << F("\tMV High Limit: ") << MVHighLimit << endl;
		cout << F("\tRamp Up Rate: ") << RampUpRate << endl;
		cout << F("\tRamp Down Rate: ") << RampDownRate << endl;
		cout << F("\tRamp Time Unit: ") << int(RampTimeUnit) << endl;

		cout << F("\tNum Of SetPoint: ") << int(NumOfSetPoint) << endl;
		cout << F("\tInitial Manual MV: ") << int(InitialManualMV) << endl;
		cout << F("\tPreset Manual MV: ") << PresetManualMV << endl;
		cout << F("\tSensor Error MV: ") << SensorErrorMV << endl;
		cout << F("\tStop MV: ") << int(StopMV) << endl;
		cout << F("\tStop Alarm Out: ") << int(StopAlarmOut) << endl;

	}
}
#endif

bool isTM4TemperatureSensorAlarm(void)
{
	bool bIsErrorOccurred = false;

#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		bIsErrorOccurred |= hTM4Controller.isSensorAlarm(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

	}
#else
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel < TM4_BREWWATER_CHANNEL; nChannel++) {

		bIsErrorOccurred |= hTM4Controller.isSensorAlarm(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

	}
#endif

	return bIsErrorOccurred;
}

unsigned int verify_TM4TemperatureSensorAlarm(const unsigned int nErrorCode)
{
	unsigned char nErrorMask = 0x00;

#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if (hTM4Controller.isSensorAlarm(eChannel)) {

			unsigned char nChannelShift = eChannel - 1;
			unsigned char nTypeShift = abs(hTM4Controller.getSensorErrorCode(eChannel)) - 1;

			nErrorMask |= (0b00000001 << nChannelShift);
			nErrorMask |= (0b00010000 << nTypeShift);

#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
			// cout << F("[TM4_SENSOR_ERR] Ch: ") << int(eChannel) << F(", Type: ") << int(nTypeShift + 1) << endl;
#endif

		}

	}
#else
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel < TM4_BREWWATER_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if (hTM4Controller.isSensorAlarm(eChannel)) {

			unsigned char nChannelShift = eChannel - 1;
			unsigned char nTypeShift = abs(hTM4Controller.getSensorErrorCode(eChannel)) - 1;

			nErrorMask |= (0b00000001 << nChannelShift);
			nErrorMask |= (0b00010000 << nTypeShift);

#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
			// cout << F("[TM4_SENSOR_ERR] Ch: ") << int(eChannel) << F(", Type: ") << int(nTypeShift + 1) << endl;
#endif

		}

	}
#endif

	return nErrorCode | ((unsigned int)(nErrorMask) << 4);
}

void measure_TM4ControllerTemperatureSensorAll(void)
{
	hTM4Controller.readPresentTemperaturesAll();
}

void measure_TM4ControllerTemperatureSensorWithChannel(const enum CTM4Controller::TM4Channel nChannel)
{
	hTM4Controller.readPresentTemperature(nChannel);
}

void measure_TM4ControllerHeatingMVAll(void)
{
	hTM4Controller.readPresentHeatingMVAll();
}

void measure_TM4ControllerHeatingMVWithChannel(const enum CTM4Controller::TM4Channel nChannel)
{
	hTM4Controller.readPresentHeatingMV(nChannel);
}

// THREAD : Polling to TM4 Controller Register
void PollingTM4Temperature_Callback(void)
{
	hTM4Controller.readPresentTemperaturesAll();

// 	hTM4Controller.readPresentTemperature(TM4_PREHEAT_CHANNEL);
// 	hTM4Controller.readPresentTemperature(TM4_BREWING_CHANNEL);
// 	hTM4Controller.readPresentTemperature(TM4_GROUPHEAD_CHANNEL);
// #if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
// 	hTM4Controller.readPresentTemperature(TM4_BREWWATER_CHANNEL);
// #endif
}


//////////////////////////////////////////////////////////////////////////////////////////
// BREW PUSH-BUTTON ACCESS(READ) FOR USER INTERFACE

void initialize_BrewButton(void)
{
	hBrewButton.initButton();
	hBrewButton.setButtonLEDState(false);
}

void enable_BrewButton(void)
{
	hBrewButton.enableButton();
}

void disable_BrewButton(void)
{
	hBrewButton.disableButton();
	hBrewButton.clearButton();
}

void PollingBrewButton_Callback(void)
{
	hBrewButton.getButtonState();
}


//////////////////////////////////////////////////////////////////////////////////////////
// ROTARY ENCODER(WITH SWITCH) MODULE IJ-8 ACCESS(READ) FOR USER INTERFACE

void initialize_RotaryEncoder(void)
{
	hRotaryEncoder.init();
}

void enable_RotaryEncoderDirection(void)
{
	hRotaryEncoder.enableDirectionEvent();
}

void disable_RotaryEncoderDirection(void)
{
	hRotaryEncoder.disableDirectionEvent();
	hRotaryEncoder.clearDirection();
}

void enable_RotaryEncoder(void)
{
	hRotaryEncoder.enableRotaryEncoder();
}

void disable_RotaryEncoder(void)
{
	hRotaryEncoder.disableRotaryEncoder();
	hRotaryEncoder.clearRotaryEncoderAll();
}

// THREAD : Trigger of Rotary Encoder Count
void TriggerRotaryEncoderST_Callback(void)
{
	hRotaryEncoder.getRotaryEncoderDirectionState();
}

// THREAD : Polling to Rotary Encoder Switch
void PollingRotaryEncoderSW_Callback(void)
{
	hRotaryEncoder.getRotaryEncoderSwitchState();
}

void enable_RotaryEncoderInterrupt(void)
{
	enable_ExternalInterrupt(TriggerRotaryEncoderST_Callback, PIN_INTERRUPT_NUM_1, ROTARY_ENCODER_STEP_TRIGGER_TYPE);
}

void diable_RotaryEncoderInterrupt(void)
{
	disable_ExternalInterrupt(PIN_INTERRUPT_NUM_1);
}


//////////////////////////////////////////////////////////////////////////////////////////
// EXTERNAL SIGNAL(GPI, INTERRUPT) ACCESS FOR EXTERNAL DEVICE

#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
void initialize_ExternalSignal(void)
{
	hExternalSignal.clear();

#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
	hExternalSignal.setPreProcessing(preProcessing_ExternalSignal);
	hExternalSignal.setPostProcessing(postProcessing_ExternalSignal);
#endif
}

void reset_ExternalSignal(void)
{
	noInterrupts();

	hExternalSignal.clear();

	interrupts();
}

int get_ExternalSignalVAL(void)
{
	int nSignal;

	noInterrupts();

	nSignal = hExternalSignal.getSignal();

	interrupts();

	return nSignal;
}

unsigned int get_ExternalSignalCNT(void)
{
	unsigned int nSignalCount;

	noInterrupts();

	nSignalCount = hExternalSignal.getSignalCNT();

	interrupts();

	return nSignalCount;
}

#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
// Pre-Processing Function
void preProcessing_ExternalSignal(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[PRE_PROC] Aux. External Interrupt[#19]" << endl;
#endif
}

// Post-Processing Function
void postProcessing_ExternalSignal(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[POST_PROC] Aux. External Interrupt[#19]" << endl;
#endif
}
#endif

// THREAD : Trigger of External Interruptable Signal
void TriggerExternalSignal_Callback(void)
{
	hExternalSignal.pollingSignal();
}

void enable_ExternalSignalInterrupt(void)
{
	enable_ExternalInterrupt(TriggerExternalSignal_Callback, PIN_INTERRUPT_NUM_4, EXTERNAL_SIGNAL_TRIGGER_TYPE);
}

void disable_ExternalSignalInterrupt(void)
{
	disable_ExternalInterrupt(PIN_INTERRUPT_NUM_4);
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
// AUX. ANALOG SENSOR(VOLTAGE) ACCESS(READ) FOR USER INTERFACE

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void initialize_AnalogSensor(void)
{
	hAnalogInput.clear();
	hAnalogInput.setVoltageTuneValue(ANALOG_TUNE_FACTOR);
}

void reset_AnalogSensor(CAnalogSensor& rhAnalogInput)
{
	noInterrupts();

	rhAnalogInput.clear();

	interrupts();
}

float get_AnalogSensorValueInVoltage(void)
{
	float fVoltageValue;

	noInterrupts();

	fVoltageValue = hAnalogInput.getVoltageValue();

	interrupts();

	return fVoltageValue;
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
// SOLENOID VALVE ACCESS

void initialize_SolenoidValve(void)
{
	hBrewValve.init(SV_CLOSE);

#if defined(_USE_SOLENOID_VALVE_PROC_MODE_)
	hBrewValve.setPreProcessing(preProcessing_BrewSolenoidValve);
	hBrewValve.setPostProcessing(postProcessing_BrewSolenoidValve);
#endif
}

#if defined(_USE_SOLENOID_VALVE_PROC_MODE_)
// Pre-Processing Function
void preProcessing_BrewSolenoidValve(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[PRE_PROC] Brew Solenoid Valve" << endl;
#endif
}

// Post-Processing Function
void postProcessing_BrewSolenoidValve(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[POST_PROC] Brew Solenoid Valve" << endl;
#endif
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
// CONTROLLINO MAXI REAL-TIME CLOCK ACCESS FOR TIME DATE

void initialize_RTClock(void)
{
	// Controllino_RTC_init(RTC_CHIP_SELECT_ADDR);
	hRTClock.init(RTC_CHIP_SELECT_ADDR);
	hRTClock.clear();
}

void reset_TimeDateRTC(void)
{
	stTimeDate_t stInitTimeDate = {RTC_DEFAULT_DAY, 0, RTC_DEFAULT_MONTH, RTC_DEFAULT_YEAR, 0, 0, 0};

	noInterrupts();

	hRTClock.setTimeDate(stInitTimeDate);

	interrupts();
}

void get_TimeDateRTC(stTimeDate_t& stCurTimeDate)
{
	noInterrupts();

	hRTClock.getTimeDate(stCurTimeDate);

	interrupts();
}

void get_DayOfWeekRTC(stSetupDataSet_t& stSetupDateData)
{
	stTimeDate_t stSetTimeDate;

	stSetTimeDate.day = stSetupDateData.Day;
	stSetTimeDate.month = stSetupDateData.Month;
	stSetTimeDate.year = stSetupDateData.Year;

	stSetupDateData.Weekday = (unsigned char)(hRTClock.getWeekday(stSetTimeDate));
}

void set_TimeDateRTC(const stTimeDate_t& stSetTimeDate)
{
	noInterrupts();

	hRTClock.setTimeDate(stSetTimeDate);

	interrupts();
}

// THREAD : Polling to Real-Time Clock
void PollingRealTimeClock_Callback(void)
{
	hRTClock.readTimeDate();
}


//////////////////////////////////////////////////////////////////////////////////////////
// CONTROLLINO MAXI THERMAL OVERLOAD SENSOR ACCESS(/OVL)

void initialize_ThermalOVL(void)
{
#if defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
	/* When using Controllino MEGA or MAXI we have acess to /OVL pin */
	/* Direction for /OLV (PE7) its input (0) */
	(DDRE &= B01111111);
#endif
}

bool isThermalOverload(void)
{
#if defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
	return ((PINE >> 7) & 0x01) == 0;
#else
	return false;
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////
// BLDC MOTOR PUMP CONTROLLER ACCESS

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)

void initialize_MotorPump(void)
{
	hMotorPump.clear();

	hMotorPump.setDutyRange(MOTOR_LOWER_POWER, MOTOR_UPPER_POWER);

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
	hMotorPump.setPostAlarm(postAlarm_MotorPump);
#endif
	
	// hMotorPump.reset();	// just only 'power on reset', but do not reset (v1.2.2r-180220)
}

#else

// for OZBV-10A-D2M2
void initialize_MotorPump(void)
{
	_serial232.begin(OZBV10A_BAUD_RATE, OZBV10A_COMM_CONFIG);
	// _serial232.setTimeout(OZBV10A_RCV_WAIT_TIME);

	hMotorPump.setModbusNode(_serial232, OZBV_MASTER_ADDR);

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
	hMotorPump.setPostAlarm(postAlarm_MotorPump);
#endif
}

bool reset_MotorControllerToFactoryDefault(void)
{
	bool bResult = hMotorPump.init();

	if (!bResult) {

		cout << F("[ERROR] Motor Controller Init Failure.") << endl;

	}

	return bResult;
}

#if defined(_USE_APPLICATION_DEBUG_MSG_)
void verify_MotorControllerFactoryDefault(void)
{
	cout << F("[OZBV-10A-D2M2 FACTORY DEFAULT VERIFICATION]") << endl;

	unsigned int nCurrentPGain = hMotorPump.getParams(OZBV_CURRENT_P_GAIN_REG_ADDR);
	unsigned int nCurrentIGain = hMotorPump.getParams(OZBV_CURRENT_I_GAIN_REG_ADDR);
	unsigned int nSpeedPGain = hMotorPump.getParams(OZBV_SPEED_P_GAIN_REG_ADDR);
	unsigned int nSpeedIGain = hMotorPump.getParams(OZBV_SPEED_I_GAIN_REG_ADDR);

	unsigned int nACCTime = hMotorPump.getParams(OZBV_ACC_TIME_REG_ADDR);
	unsigned int nDECTime = hMotorPump.getParams(OZBV_DEC_TIME_REG_ADDR);

	unsigned int nMinSpeed = hMotorPump.getParams(OZBV_MIN_SPEED_REG_ADDR);
	unsigned int nMaxSpeed = hMotorPump.getParams(OZBV_MAX_SPEED_REG_ADDR);

	unsigned int nCurrentLimit = hMotorPump.getParams(OZBV_CURRENT_LIMIT_REG_ADDR);
	unsigned int nCurrentLimitTime = hMotorPump.getParams(OZBV_CURRENT_LIMIT_TIME_REG_ADDR);

	unsigned int nOverheatLimit = hMotorPump.getParams(OZBV_OVERHEAT_LIMIT_REG_ADDR);
	unsigned int nOverheatLimitTime = hMotorPump.getParams(OZBV_OVERHEAT_LIMIT_TIME_REG_ADDR);

	unsigned int nPowerDiffLimitTime = hMotorPump.getParams(OZBV_POWER_DIFF_LIMIT_TIME_REG_ADDR);

	unsigned char nStopMode = hMotorPump.getParams(OZBV_MOTOR_STOP_MODE_REG_ADDR);
	unsigned char nEMGStopMode = hMotorPump.getParams(OZBV_MOTOR_EMG_STOP_MODE_REG_ADDR);

	unsigned char nMonitoringMode = hMotorPump.getParams(OZBV_MONITORING_MODE_REG_ADDR);

	unsigned int nSpeedPermissibleRange = hMotorPump.getParams(OZBV_SPEED_PERMISSIBLE_RANGE_REG_ADDR);

	unsigned char nINPolarity = hMotorPump.getParams(OZBV_IN_POLARITY_SEL_REG_ADDR);
	unsigned char nOUTPolarity = hMotorPump.getParams(OZBV_OUT_POLARITY_SEL_REG_ADDR);

	unsigned char nRunMode = hMotorPump.getParams(OZBV_MOTOR_RUN_MODE_REG_ADDR);

	unsigned char nIN1Function = hMotorPump.getParams(OZBV_IN1_FUNCTION_REG_ADDR);
	unsigned char nIN2Function = hMotorPump.getParams(OZBV_IN2_FUNCTION_REG_ADDR);
	unsigned char nIN3Function = hMotorPump.getParams(OZBV_IN3_FUNCTION_REG_ADDR);
	unsigned char nIN4Function = hMotorPump.getParams(OZBV_IN4_FUNCTION_REG_ADDR);

	unsigned char nOUT1Function = hMotorPump.getParams(OZBV_OUT1_FUNCTION_REG_ADDR);
	unsigned char nOUT2Function = hMotorPump.getParams(OZBV_OUT2_FUNCTION_REG_ADDR);
	unsigned char nOUT3Function = hMotorPump.getParams(OZBV_OUT3_FUNCTION_REG_ADDR);

	unsigned char nAnalogINPolarity = hMotorPump.getParams(OZBV_ANALOG_INPUT_POLARITY_REG_ADDR);
	unsigned int nAnalogINMinimun = hMotorPump.getParams(OZBV_ANALOG_INPUT_MINIMUM_REG_ADDR);

	unsigned char nTorqueCheck = hMotorPump.getParams(OZBV_MOTOR_TORQUE_CHECK_REG_ADDR);
	unsigned int nTorque00RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE00_RPM_REG_ADDR);
	unsigned int nTorque00CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE00_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque00CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE00_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque01RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE01_RPM_REG_ADDR);
	unsigned int nTorque01CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE01_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque01CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE01_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque10RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE10_RPM_REG_ADDR);
	unsigned int nTorque10CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE10_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque10CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE10_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque11RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE11_RPM_REG_ADDR);
	unsigned int nTorque11CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE11_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque11CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE11_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque20RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE20_RPM_REG_ADDR);
	unsigned int nTorque20CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE20_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque20CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE20_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque21RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE21_RPM_REG_ADDR);
	unsigned int nTorque21CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE21_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque21CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE21_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque30RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE30_RPM_REG_ADDR);
	unsigned int nTorque30CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE30_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque30CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE30_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque31RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE31_RPM_REG_ADDR);
	unsigned int nTorque31CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE31_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque31CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE31_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque40RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE40_RPM_REG_ADDR);
	unsigned int nTorque40CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE40_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque40CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE40_CURRENT_LIMIT_REG_ADDR);
	unsigned int nTorque41RPM = hMotorPump.getParams(OZBV_MOTOR_TORQUE41_RPM_REG_ADDR);
	unsigned int nTorque41CurrentBase = hMotorPump.getParams(OZBV_MOTOR_TORQUE41_CURRENT_BASE_REG_ADDR);
	unsigned int nTorque41CurrentLimit = hMotorPump.getParams(OZBV_MOTOR_TORQUE41_CURRENT_LIMIT_REG_ADDR);
	unsigned char nSpeedOffset0 = hMotorPump.getParams(OZBV_SPEED_OFFSET0_REG_ADDR);
	unsigned char nSpeedOffset1 = hMotorPump.getParams(OZBV_SPEED_OFFSET1_REG_ADDR);
	unsigned char nSpeedOffset2 = hMotorPump.getParams(OZBV_SPEED_OFFSET2_REG_ADDR);
	unsigned int nTorqueCurrentLimitTime = hMotorPump.getParams(OZBV_TORQUE_CURRENT_LIMIT_TIME_REG_ADDR);

	unsigned char nRunDirection = hMotorPump.getParams(OZBV_MOTOR_RUN_DIRECTION_REG_ADDR);
	unsigned char nPolarityNum = hMotorPump.getParams(OZBV_MOTOR_POLARITY_NUMBER_REG_ADDR);

	unsigned int nCurrentFilter = hMotorPump.getParams(OZBV_CURRENT_FILTER_REG_ADDR);
	unsigned int nSpeedFilter = hMotorPump.getParams(OZBV_SPEED_FILTER_REG_ADDR);

	unsigned int nExternalVolumeFilter = hMotorPump.getParams(OZBV_EXTERNAL_VOLUME_FILTER_REG_ADDR);

	unsigned int nCurrentSensorCapacity = hMotorPump.getParams(OZBV_CURRENT_SENSOR_CAPACITY_REG_ADDR);

	int nVersion[2];

	hMotorPump.getVersion(nVersion);


	cout << F("\tH/W ver.") << nVersion[0] << endl;
	cout << F("\tS/W ver.") << nVersion[1] << endl;

	cout << F("\tCurrent P Gain: ") << nCurrentPGain << endl;
	cout << F("\tCurrent I Gain: ") << nCurrentIGain << endl;
	cout << F("\tSpeed P Gain: ") << nSpeedPGain << endl;
	cout << F("\tSpeed I Gain: ") << nSpeedIGain << endl;

	cout << F("\tACC Time: ") << nACCTime << endl;
	cout << F("\tDEC Time: ") << nDECTime << endl;

	cout << F("\tMinimum Speed: ") << nMinSpeed << endl;
	cout << F("\tMaximum Speed: ") << nMaxSpeed << endl;

	cout << F("\tCurrent Limit: ") << nCurrentLimit << endl;
	cout << F("\tCurrent Limit Time: ") << nCurrentLimitTime << endl;

	cout << F("\tOverheat Limit: ") << int(nOverheatLimit) << endl;
	cout << F("\tOverheat Limit Time: ") << nOverheatLimitTime << endl;

	cout << F("\tPower Diff. Limit Time: ") << nPowerDiffLimitTime << endl;

	cout << F("\tStop Mode: ") << int(nStopMode) << endl;
	cout << F("\tEMG Stop Mode: ") << int(nEMGStopMode) << endl;

	cout << F("\tMonitoring Mode: ") << int(nMonitoringMode) << endl;

	cout << F("\tPermissible Speed Range: ") << nSpeedPermissibleRange << endl;

	cout << F("\tIN Polarity: ") << int(nINPolarity) << endl;
	cout << F("\tOUT Polarity: ") << int(nOUTPolarity) << endl;

	cout << F("\tRun Mode: ") << int(nRunMode) << endl;

	cout << F("\tIN1 Fuction: ") << int(nIN1Function) << endl;
	cout << F("\tIN2 Fuction: ") << int(nIN2Function) << endl;
	cout << F("\tIN3 Fuction: ") << int(nIN3Function) << endl;
	cout << F("\tIN4 Fuction: ") << int(nIN4Function) << endl;

	cout << F("\tOUT1 Fuction: ") << int(nOUT1Function) << endl;
	cout << F("\tOUT2 Fuction: ") << int(nOUT2Function) << endl;
	cout << F("\tOUT3 Fuction: ") << int(nOUT3Function) << endl;

	cout << F("\tAnalog IN Polarity: ") << int(nAnalogINPolarity) << endl;
	cout << F("\tAnalog IN Minimum: ") << nAnalogINMinimun << endl;

	cout << F("\tTorque Check: ") << int(nTorqueCheck) << endl;
	cout << F("\tTorque00 RPM: ") << nTorque00RPM << endl;
	cout << F("\tTorque00 Current Base: ") << nTorque00CurrentBase << endl;
	cout << F("\tTorque00 Current Limit: ") << nTorque00CurrentLimit << endl;
	cout << F("\tTorque01 RPM: ") << nTorque01RPM << endl;
	cout << F("\tTorque01 Current Base: ") << nTorque01CurrentBase << endl;
	cout << F("\tTorque01 Current Limit: ") << nTorque01CurrentLimit << endl;
	cout << F("\tTorque10 RPM: ") << nTorque10RPM << endl;
	cout << F("\tTorque10 Current Base: ") << nTorque10CurrentBase << endl;
	cout << F("\tTorque10 Current Limit: ") << nTorque10CurrentLimit << endl;
	cout << F("\tTorque11 RPM: ") << nTorque11RPM << endl;
	cout << F("\tTorque11 Current Base: ") << nTorque11CurrentBase << endl;
	cout << F("\tTorque11 Current Limit: ") << nTorque11CurrentLimit << endl;
	cout << F("\tTorque20 RPM: ") << nTorque20RPM << endl;
	cout << F("\tTorque20 Current Base: ") << nTorque20CurrentBase << endl;
	cout << F("\tTorque20 Current Limit: ") << nTorque20CurrentLimit << endl;
	cout << F("\tTorque21 RPM: ") << nTorque21RPM << endl;
	cout << F("\tTorque21 Current Base: ") << nTorque21CurrentBase << endl;
	cout << F("\tTorque21 Current Limit: ") << nTorque21CurrentLimit << endl;
	cout << F("\tTorque30 RPM: ") << nTorque30RPM << endl;
	cout << F("\tTorque30 Current Base: ") << nTorque30CurrentBase << endl;
	cout << F("\tTorque30 Current Limit: ") << nTorque30CurrentLimit << endl;
	cout << F("\tTorque31 RPM: ") << nTorque31RPM << endl;
	cout << F("\tTorque31 Current Base: ") << nTorque31CurrentBase << endl;
	cout << F("\tTorque31 Current Limit: ") << nTorque31CurrentLimit << endl;
	cout << F("\tTorque40 RPM: ") << nTorque40RPM << endl;
	cout << F("\tTorque40 Current Base: ") << nTorque40CurrentBase << endl;
	cout << F("\tTorque40 Current Limit: ") << nTorque40CurrentLimit << endl;
	cout << F("\tTorque41 RPM: ") << nTorque41RPM << endl;
	cout << F("\tTorque41 Current Base: ") << nTorque41CurrentBase << endl;
	cout << F("\tTorque41 Current Limit: ") << nTorque41CurrentLimit << endl;
	cout << F("\tSpeed Offset0: ") << int(nSpeedOffset0) << endl;
	cout << F("\tSpeed Offset1: ") << int(nSpeedOffset1) << endl;
	cout << F("\tSpeed Offset2: ") << int(nSpeedOffset2) << endl;
	cout << F("\tTorque Current Limit Time: ") << nTorqueCurrentLimitTime << endl;

	cout << F("\tRun Direction: ") << int(nRunDirection) << endl;
	cout << F("\tPolarity Number: ") << int(nPolarityNum) << endl;

	cout << F("\tCurrent Filter: ") << nCurrentFilter << endl;
	cout << F("\tSpeed Filter: ") << nSpeedFilter << endl;

	cout << F("\tExt. Volume Filter: ") << nExternalVolumeFilter << endl;

	cout << F("\tCurrent Sensor Capacity: ") << nCurrentSensorCapacity << endl;
}
#endif

#endif

bool isMotorAlarm(void)
{
	bool bIsMotorAlarm;

	#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	noInterrupts();
	#endif

	bIsMotorAlarm = hMotorPump.isAlarm();

	#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	interrupts();
	#endif

	return bIsMotorAlarm;
}

#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
void postAlarm_MotorPump(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[POST_PROC] Motor Pump Alarm Received" << endl;
#endif
}
#endif

// THREAD : Polling to BLDC Motor Controller(ALARM)
void PollingMotorPumpAlarm_Callback(void)
{
	hMotorPump.pollingAlarm();
}


//////////////////////////////////////////////////////////////////////////////////////////
// OPTICAL FIBER AMPLIFIER ACCESS

#if AUTOMATION_SENSOR_ATTACHED

void initialize_OpticalFiberAmp(void)
{
	// hOpticalFiber.setActiveType(OPTICAL_FIBER_AMP_ACTIVE_TYPE);
	hOpticalFiber.setDuration(OPTICAL_FIBER_DARK_DURATION, OPTICAL_FIBER_LIGHT_HIGH_DURATION);

#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
	hOpticalFiber.setDarkProcessing(darkProcessing_OpticalFiberAmp);
	hOpticalFiber.setLightProcessing(lightProcessing_OpticalFiberAmp);
#endif

	hOpticalFiber.init();
}

void init_OpticalFiberAmp(void)
{
	noInterrupts();

	hOpticalFiber.init();

	interrupts();
}

void enable_OpticalFiberAmp(void)
{
	noInterrupts();

	hOpticalFiber.enable();

	interrupts();
}

void disable_OpticalFiberAmp(void)
{
	noInterrupts();

	hOpticalFiber.disable();

	interrupts();
}

int read_OpticalFiberSignal(void)
{
	int nSignal;

	noInterrupts();

	nSignal = hOpticalFiber.readSignal();

	interrupts();

	return nSignal;
}

int get_OpticalFiberSignal(void)
{
	int nSignal;

	noInterrupts();

	nSignal = hOpticalFiber.getSignal();

	interrupts();

	return nSignal;
}

bool get_OpticalFiberDiagnosis(void)
{
	bool bIsUnstable;

	noInterrupts();

	bIsUnstable = hOpticalFiber.isUnstable();

	interrupts();

	return bIsUnstable;
}

#if defined(_USE_OPTICAL_FIBER_PROC_MODE_)
// Dark-Processing Function
void darkProcessing_OpticalFiberAmp(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[PROC] Optical Fiber Amplifier (DARK)" << endl;
#endif
}

// Light-Processing Function
void lightProcessing_OpticalFiberAmp(void)
{
#if defined(_USE_PERIPHERAL_INTERFACE_DEBUG_MSG_)
	cout << "[PROC] Optical Fiber Amplifier (LIGHT)" << endl;
#endif
}
#endif

// THREAD : Polling to Optical Fiber AMP (Signal / Diagnosis)
void PollingOpticalFiberAmp_Callback(void)
{
	hOpticalFiber.pollingSignal();
}

#endif


//////////////////////////////////////////////////////////////////////////////////////////
// GROUPHEAD LAMP ACCESS

void initialize_GroupheadLamp(void)
{
	hLamp.init();
}

// THREAD : Polling to Timer for lazy turn-off
void PollingGroupheadLampTimer_Callback(void)
{
	hLamp.turnoffDELAYED();
}


//////////////////////////////////////////////////////////////////////////////////////////
// TEXT LCD (20x4) with MCP23008 I2C EXPANDER ACCESS

void initialize_TextLCD(void)
{
	hTextLCD.init(LCD_MAX_COL, LCD_MAX_LINE);
}

void print_Char(const unsigned char nPos, const unsigned char nLine, const unsigned char nVal)
{
	hTextLCD.writePos(nPos, nLine, nVal);
}

void print_String(const unsigned char nPos, const unsigned char nLine, const char* pStr)
{
	hTextLCD.printPos(nPos, nLine, pStr);
}

void print_Blank(const unsigned char nPos, const unsigned char nLine, const unsigned char nLen)
{
	hTextLCD.fillBlank(nPos, nLine, nLen);
}

void print_IntegerValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue)
{
	char strValue[MAX_INTEGER_VALUE_DIGIT+1];

	if (nValue >= MAX_INTEGER_VALUE_NUM)
		nValue = MAX_INTEGER_VALUE_NUM - 1;

	sprintf(strValue, "%-3d", nValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_LongValue(const unsigned char nPos, const unsigned char nLine, const unsigned long lValue)
{
	char strValue[MAX_LONG_VALUE_DIGIT+1];

	sprintf(strValue, "%07ld", lValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_LongValueUnsigned(const unsigned char nPos, const unsigned char nLine, const unsigned long lValue)
{
	char strValue[MAX_LONG_VALUE_DIGIT+1];

	sprintf(strValue, "%07lu", lValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_ExceptionCode(const unsigned char nPos, const unsigned char nLine, const unsigned int nExceptionCode)
{
	char strExceptionCode[MAX_EXCEPTION_CODE_DIGIT+1];

	sprintf(strExceptionCode, "%04X", nExceptionCode);

	hTextLCD.printPos(nPos, nLine, strExceptionCode);
}

void print_TimeDateValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue)
{
	char strValue[MAX_TIMEDATE_VALUE_DIGIT+1];

	if (nValue >= MAX_TIMEDATE_VALUE_NUM)
		nValue = MAX_TIMEDATE_VALUE_NUM - 1;

	sprintf(strValue, "%02d", nValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_IntegralDerivativeValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue)
{
	char strValue[MAX_INTEGRAL_DERIVATIVE_VALUE_DIGIT+1];

	if (nValue >= MAX_INTEGRAL_DERIVATIVE_VALUE_NUM)
		nValue = MAX_INTEGRAL_DERIVATIVE_VALUE_NUM - 1;

	sprintf(strValue, "%-4d", nValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_IntegerADCDigit(const unsigned char nPos, const unsigned char nLine, int nValue)
{
	char strValue[MAX_INTEGER_ADC_VALUE_DIGIT+1];

	if (nValue >= MAX_INTEGER_ADC_VALUE_NUM)
		nValue = MAX_INTEGER_ADC_VALUE_NUM - 1;

	sprintf(strValue, "%4d", nValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_IntegerMotorAccTime(const unsigned char nPos, const unsigned char nLine, unsigned int nValue)
{
	char strValue[MAX_INTEGER_MOTOR_ACC_VALUE_DIGIT+1];

	if (nValue >= MAX_INTEGER_MOTOR_ACC_VALUE_NUM)
		nValue = MAX_INTEGER_MOTOR_ACC_VALUE_NUM - 1;

	sprintf(strValue, "%-5u", nValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_FloatValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_VALUE_NUM)
		fValue = MAX_FLOAT_VALUE_NUM - 0.1F;

	dtostrf(fValue, -5, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_FloatFlowQuantityValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_FLOWQUANTITY_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_FLOWQUANTITY_VALUE_NUM)
		fValue = MAX_FLOAT_FLOWQUANTITY_VALUE_NUM - hFlowmeter.getTuningFactor();/*WATERFLOW_QUANTITY_FACTOR;*/

	dtostrf(fValue, -6, 1, strValue);

	// hTextLCD.fillBlank(nPos, nLine, MAX_FLOAT_FLOWQUANTITY_VALUE_DIGIT);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_FloatProgramShotTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_SHOTTIME_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_SHOTTIME_VALUE_NUM)
		fValue = MAX_FLOAT_SHOTTIME_VALUE_NUM - 0.1F;

	dtostrf(fValue, 6, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_FloatProgramFlowQuantityValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_FLOWQUANTITY_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_FLOWQUANTITY_VALUE_NUM)
		fValue = MAX_FLOAT_FLOWQUANTITY_VALUE_NUM - hFlowmeter.getTuningFactor();/*WATERFLOW_QUANTITY_FACTOR;*/

	dtostrf(fValue, 6, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

#if AUTOMATION_SENSOR_ATTACHED
void print_FloatAutoFlushingModeTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_RINSING_TIME_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_RINSING_TIME_VALUE_NUM)
		fValue = MAX_FLOAT_RINSING_TIME_VALUE_NUM - 0.1F;

	dtostrf(fValue, -4, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}
#else
void print_FloatFlushingDetectionTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_RINSING_TIME_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_RINSING_TIME_VALUE_NUM)
		fValue = MAX_FLOAT_RINSING_TIME_VALUE_NUM - 0.1F;

	dtostrf(fValue, -4, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}
#endif

void print_FloatPressureValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_PRESSURE_VALUE_DIGIT+1];

	if (fValue >= MAX_FLOAT_PRESSURE_VALUE_NUM)
		fValue = MAX_FLOAT_PRESSURE_VALUE_NUM - 0.1F;

	dtostrf(fValue, -4, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
}

void print_FloatInputBiasValue(const unsigned char nPos, const unsigned char nLine, float fValue)
{
	char strValue[MAX_FLOAT_INPUT_BIAS_VALUE_DIGIT+1];

	float fValueAbs = fabs(fValue);

	if (fValueAbs >= MAX_FLOAT_INPUT_BIAS_VALUE_NUM)
		fValueAbs = MAX_FLOAT_INPUT_BIAS_VALUE_NUM - 0.1F;

	dtostrf(fValueAbs, -4, 1, strValue);

	hTextLCD.writePos(nPos, nLine, (fValue > -0.1F ? LCD_PLUS_CHAR : LCD_MINUS_CHAR));
	hTextLCD.printPos(nPos+1, nLine, strValue);
}

void print_VersionCode(const unsigned char nPos, const unsigned char nLine, const unsigned char nVersion, const unsigned char nPatch, const unsigned char nTypedef)
{
	char strValue[MAX_VERSION_CODE_DIGIT+1];

	float fVersion = nVersion / 10.0F;

	if (fVersion >= MAX_VERSION_CODE_NUM)
		fVersion = MAX_VERSION_CODE_NUM - 0.1F;

	dtostrf(fVersion, -3, 1, strValue);

	hTextLCD.printPos(nPos, nLine, strValue);
	hTextLCD.writePos(nPos+MAX_VERSION_CODE_DIGIT, nLine, nTypedef);

	if (nLine == SOFTWARE_VERSION_VALUE_LINE) {

		hTextLCD.writePos(nPos+MAX_VERSION_CODE_DIGIT+2, nLine, '(');
		hTextLCD.writePos(nPos+MAX_VERSION_CODE_DIGIT+3, nLine, nPatch+0x30);
		hTextLCD.writePos(nPos+MAX_VERSION_CODE_DIGIT+4, nLine, ')');

	}
}


//////////////////////////////////////////////////////////////////////////////////////////
// RGB LED (16 Pixels) ACCESS

void initialize_PixelLED(void)
{
	// FastLED.addLeds<NEOPIXEL, NEOPIXEL_DIN>(hPixelLED._ledPalett, NUM_OF_PIXELS);

	hPixelLED.init();
}

void prepare_LEDPalettOnBrewStandby(const CRGB rgbFgColor, const bool bBrewSetupMode)
{
	CRGB rgbBgColor;
	enum CPixelLED::LedMode eLedMode;
	unsigned long ulShowInterval;

	if (!bBrewSetupMode) {

		rgbBgColor = RGB_BLANK;
		eLedMode = CPixelLED::STATIC_MODE;
		ulShowInterval = LEDPIXEL_STATIC_MODE_DRAWING_INTERVAL;

	}
	else {

		rgbBgColor = RGB_SETUP_ENTRY_BLINK_BG;
		eLedMode = CPixelLED::BLINK_MODE;
		ulShowInterval = LEDPIXEL_BLINK_MODE_DRAWING_INTERVAL;

	}

	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(rgbBgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(eLedMode);
	hPixelLED.setInterval(ulShowInterval);
}

void prepare_LEDPalettOnTimelapseHolding(const CRGB rgbFgColor, const bool bBrewSetupMode)
{
	CRGB rgbBgColor;
	enum CPixelLED::LedMode eLedMode;

	if (!bBrewSetupMode) {

		rgbBgColor = RGB_BLANK;
		eLedMode = CPixelLED::HOLDING_ON_STATIC_MODE;

	}
	else {

		rgbBgColor = RGB_SETUP_ENTRY_BLINK_BG;
		eLedMode = CPixelLED::HOLDING_ON_BLINK_MODE;

	}

	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(rgbBgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(eLedMode);
	hPixelLED.setInterval(LEDPIXEL_TIMELAPSE_MODE_HOLD_INTERVAL);

	hPixelLED.setLasttime(millis());
}

void prepare_LEDPalettOnBrewing(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_BREW_PROGRESS_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::VARIOUS_PROGRESS_MODE);
}

void prepare_LEDPalettOnSetupEntry(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_SETUP_ENTRY_BLINK_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::BLINK_MODE);
	hPixelLED.setInterval(LEDPIXEL_BLINK_MODE_DRAWING_INTERVAL);
}

void prepare_LEDPalettOnSetupExit(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_BLANK, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::STATIC_MODE);
	hPixelLED.setInterval(LEDPIXEL_STATIC_MODE_DRAWING_INTERVAL);
}

void prepare_LEDPalettOnSleep(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_BLANK, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::BREATHE_MODE);
	hPixelLED.setInterval(LEDPIXEL_BREATHE_MODE_DRAWING_INTERVAL);
}

void prepare_LEDPalettOnCleaning(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_CLEANING_PROGRESS_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::VARIOUS_PROGRESS_MODE);
}

void prepare_LEDPalettOnException(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_SYSTEM_ERROR_BLINK_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::BLINK_MODE);
	hPixelLED.setInterval(LEDPIXEL_BLINK_MODE_DRAWING_INTERVAL);
}

void prepare_LEDPalettOnAirVentilation(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_AIR_VENTILATION_STAGE_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::BLINK_MODE);
	hPixelLED.setInterval(LEDPIXEL_BLINK_MODE_DRAWING_INTERVAL);
}

void prepare_LEDPalettOnActivation(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_ACTIVATION_PROGRESS_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::LPUSHING_PROGRESS_MODE);	
}

void prepare_LEDPalettOnLongPushing(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_LONG_PUSH_PROGRESS_BG, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::LPUSHING_PROGRESS_MODE);
}

#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
void prepare_LEDPalettOnVLongPushing(const CRGB rgbFgColor)
{
	hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT);
	hPixelLED.setBackgroundColor(RGB_BLANK, NOT_UPDATE_LED_PALETT);
	hPixelLED.setPalettMode(CPixelLED::STATIC_MODE);
	hPixelLED.setInterval(LEDPIXEL_STATIC_MODE_DRAWING_INTERVAL);
}
#endif

