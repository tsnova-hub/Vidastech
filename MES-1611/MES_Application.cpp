

#include "MES_Application.h"


////////////////////////////////////////////////////////////////////////////////////////////
// 			Global Variables, Data Structures and Handles of Timers, Threads

stDataSet_t stDataSet;

stError_t stError;

stTimeDate_t stCurTimeDate;


int _nBrewState = BREW_STANDBY_STATE;
int _nSetupMenuState = LCD_DASHBOARD_ENTRY;

unsigned char _nMutuallyExclusiveActivated = ALL_USER_INTERFACE_ACTIVATED;

unsigned int _nPasswordCode = 0;

bool _bIgnoreRotaryEncoderEventForSaftyExit = false;
#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
bool _bResetTimerToPredictVLongPushingEvent = true;
bool _bPredictVLongPushingEvent = false;
#endif

bool _bJustGetStoppedBrewing = false;				 // to enhance performance of standing by brew boiler pressure
bool _bJustGetStoppedFlushing = false;

bool _bPowerSavingModeValid = true;
unsigned char _nPowerSavingState = PWS_NOT_WORKING;  // 0, if not working; 1, if now working; 2, if paused;

bool _bHeatingSuspendedAll = false;

unsigned char _nCleaningProgress = 0;

#if AUTOMATION_SENSOR_ATTACHED
bool _bIsAutoFlushing = false;
bool _bIsAutoFlushed = true;
bool _bPauseAutoFlushing = false;

bool _bIsAutoBrewing = false;
bool _bIsAutoBrewed = true;
bool _bPauseAutoBrewing = false;

bool _bIsAutoBrewingEnabled = true;
#endif


CEspresso hEspressoMachine = CEspresso();


CPressureSensorThread hPressureSensorThread = CPressureSensorThread();
CAnalogSensorThread hAnalogSensorThread = CAnalogSensorThread();

Thread hRealTimeClockThread = Thread();

ThreadController hAppThreadContainer = ThreadController();


StopWatch hSetupMenuTimer(StopWatch::SECONDS);
StopWatch hPowerSavingTimer(StopWatch::SECONDS);
StopWatch hUnderheatingObservationTimer(StopWatch::SECONDS);
StopWatch hAlarmMessageDisplayTimer(StopWatch::SECONDS);
StopWatch hBrewPressureStandbyDelayTimer(StopWatch::SECONDS);


////////////////////////////////////////////////////////////////////////////////////////////
// 			EEPROM Backup Memory Map (Tables for EEPROM Wear Leveling)

static unsigned char eeprom_backup_addr_pointer[MAX_BACKUP_PARAMETERS] = {
	0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0
};


//////////////////////////////////////////////////////////////////////////////////////////
// 			Definition of Setup-Menu handling Function Pointer

int (*procedure_SetupMenupage[MENUPAGE_NUM])(const int nSignal);



//////////////////////////////////////////////////////////////////////////////////////////
//			SYSTEM FRAMEWORK OF MOAI ESSPRESSO STATION

// System Init / Boot Sequences
void initialize_SystemParameters(void)
{
	if (hMotorPump.isRunning()) {

		hMotorPump.stop();

	}

	if (hBrewValve.isOpening()) {

		hBrewValve.close();

	}

	if (!suspend_BoilerHeater()) {

		// #if defined(_USE_APPLICATION_DEBUG_MSG_)
		cout << F("[BOOT] Failed to suspend Boiler Heater") << endl;
		// #endif

		wait_RebootSystemWithConsoleMsg();

	}

	config_FloatPrecisionConsole();

	initialize_StructureDataSet();

	if (isFactoryDefault() == false) {

		display_SystemFactoryReset();

		reset_StructureDataSet();

		reset_TM4ControllerToFactoryDefault();

		setup_BoilerTemperatureControllerBIAS();
		setup_BoilerTemperatureControllerSV();

		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
		reset_MotorControllerToFactoryDefault();
		#endif

		// #if defined(_USE_APPLICATION_DEBUG_MSG_)
		cout << F("\n[BOOT] Factory Default.\n") << endl;
		// #endif

	}
	else {

		restore_StructureDataSet();

		setup_PressureSensorAdjustment();

		verify_BoilerTemperatureConstrollerPID();
		verify_BoilerTemperatureControllerMV();
		verify_BoilerTemperatureControllerBIAS();
		verify_BoilerTemperatureControllerSV();

		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
		verify_MotorControllerACCTime();
		#endif

		// #if defined(_USE_APPLICATION_DEBUG_MSG_)
		cout << F("\n[BOOT] DataSet Restored.\n") << endl;
		// #endif

	}

#if defined(_USE_APPLICATION_DEBUG_MSG_)
	verify_TM4ControllerFactoryDefault();

	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	verify_MotorControllerFactoryDefault();
	#endif
#endif

	#if AUTOMATION_SENSOR_ATTACHED
	setup_AutomationMode();
	#else
	// verify_FlushingDetectionModeWithMotorPump();
	#endif


	clear_CurrentTimeDate();
	clear_SensorObservationAll();
	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	clear_MotorControllerStatusObservation();
	#endif


	initialize_TextLCDParameters();
	initialize_MenupageFunctionPointerTable();

	release_Authorization();

#if defined(_USE_APPLICATION_DEBUG_MSG_)
	// print_StructureDataSet();
	// print_EEPROMBackupAddressPointer();
#endif
	print_SystemErrorList(false);

	_console.print(F("\n\n\rMOAI EspressoStation Deactivated.\n\n\r"));
}

void initialize_StructureDataSet(void)
{
	stDataSet.ProgramNum = DEF_PROGRAM_NUM;

	stDataSet.stAcquisition.ShotTime = 0UL;

	stDataSet.stAcquisition.FlowCount = 0UL;
	stDataSet.stAcquisition.FlowQuantity = 0.0F;
	stDataSet.stAcquisition.FlowRate = 0.0F;

	stDataSet.stAcquisition.PreheatTemp = 0.0F;
	stDataSet.stAcquisition.BrewTemp = 0.0F;
	stDataSet.stAcquisition.GroupTemp = 0.0F;
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewWaterTemp = 0.0F;
#endif

	stDataSet.stAcquisition.PreheatMV = 0.0F;
	stDataSet.stAcquisition.BrewMV = 0.0F;
	stDataSet.stAcquisition.GroupMV = 0.0F;

	stDataSet.stAcquisition.PresentCurrent = 0L;
	stDataSet.stAcquisition.PresentSpeed = 0L;

	stDataSet.stAcquisition.BrewPress = 0.0F;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.PumpPress = 0.0F;
#endif
#if AUX_CURRENT_SENSOR_ATTACHED
	stDataSet.stAcquisition.AuxPress = 0.0F;
#endif

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	stDataSet.stAcquisition.AnalogInputVoltage = 0.0F;
#endif

	stDataSet.stAcquisition.AvgFlowRate = 0.0F;
	stDataSet.stAcquisition.AvgBrewPress = 0.0F;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.AvgPumpPress = 0.0F;
#endif
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	stDataSet.stAcquisition.AvgAnalogInputVoltage = 0.0F;
#endif

	stDataSet.stSetup.BrewSetupMode = DEF_BREW_SETUP_MODE;

	for (int nProgramNum = PROGRAM_SET_ONE; nProgramNum <= MANUAL_SET_ONE; nProgramNum++) {
		stDataSet.stSetup.ProgramMode[nProgramNum] = DEF_PROGRAM_MODE;
		stDataSet.stSetup.InjectionFlow[nProgramNum] = DEF_INJECTION_FLOW_COUNT;
		stDataSet.stSetup.ExtractionFlow[nProgramNum] = DEF_EXTRACTION_FLOW_COUNT;
		stDataSet.stSetup.InjectionTime[nProgramNum] = DEF_INJECTION_TIME;
		stDataSet.stSetup.ExtractionTime[nProgramNum] = DEF_EXTRACTION_TIME;
	}

	stDataSet.stSetup.InjectionMode = DEF_INJECTION_MODE;
	stDataSet.stSetup.ProgramSetNum = DEF_PROGRAM_SET_NUM;

	stDataSet.stSetup.InjectionPower = DEF_INJECTION_POWER;

	stDataSet.stSetup.SetHeaterChannel = TM4_PREHEAT_CHANNEL - 1;				// only for Setup-Menu
	stDataSet.stSetup.SetValueTemp[0] = DEF_SETPOINT_TEMPERATURE + DEF_CH1_SETPOINT_OFFSET;
	stDataSet.stSetup.SetValueTemp[1] = DEF_SETPOINT_TEMPERATURE + DEF_CH2_SETPOINT_OFFSET;
	stDataSet.stSetup.SetValueTemp[2] = DEF_SETPOINT_TEMPERATURE + DEF_CH3_SETPOINT_OFFSET;
	stDataSet.stSetup.SetValueTemp[3] = 0.0F;									// no heater, just measure
	stDataSet.stSetup.ProportionalBandTemp[0] = DEF_CH1_PBAND_TEMPERATURE;
	stDataSet.stSetup.ProportionalBandTemp[1] = DEF_CH2_PBAND_TEMPERATURE;
	stDataSet.stSetup.ProportionalBandTemp[2] = DEF_CH3_PBAND_TEMPERATURE;
	stDataSet.stSetup.ProportionalBandTemp[3] = DEF_CH4_PBAND_TEMPERATURE;
	stDataSet.stSetup.IntegralTime[0] = DEF_CH1_INTEGRAL_TIME;
	stDataSet.stSetup.IntegralTime[1] = DEF_CH2_INTEGRAL_TIME;
	stDataSet.stSetup.IntegralTime[2] = DEF_CH3_INTEGRAL_TIME;
	stDataSet.stSetup.IntegralTime[3] = DEF_CH4_INTEGRAL_TIME;
	stDataSet.stSetup.DerivativeTime[0] = DEF_CH1_DERIVATIVE_TIME;
	stDataSet.stSetup.DerivativeTime[1] = DEF_CH2_DERIVATIVE_TIME;
	stDataSet.stSetup.DerivativeTime[2] = DEF_CH3_DERIVATIVE_TIME;
	stDataSet.stSetup.DerivativeTime[3] = DEF_CH4_DERIVATIVE_TIME;
	stDataSet.stSetup.MVHighLimit[0] = DEF_CH1_MV_HIGH_LIMIT;
	stDataSet.stSetup.MVHighLimit[1] = DEF_CH2_MV_HIGH_LIMIT;
	stDataSet.stSetup.MVHighLimit[2] = DEF_CH3_MV_HIGH_LIMIT;
	stDataSet.stSetup.MVHighLimit[3] = DEF_CH4_MV_HIGH_LIMIT;

	stDataSet.stSetup.TempSensorChannel = TM4_PREHEAT_CHANNEL - 1;				// only for Setup-Menu
	for (int nSensor = 0; nSensor < 4; nSensor++) {
		stDataSet.stSetup.SensorInputBias[nSensor] = DEF_TM4_SENSOR_INPUT_BIAS;
	}

	stDataSet.stSetup.SetSensorChannel = 0;
	stDataSet.stSetup.SensorADCDigit = 0;

	for (int nSensor = 0; nSensor < 3; nSensor++) {
		stDataSet.stSetup.ZeroShift[nSensor] = float(DEF_ZERO_SHIFT);
		stDataSet.stSetup.SpanFactor[nSensor] = float(DEF_SPAN_FACTOR);
	}

	stDataSet.stSetup.MotorRun = DEF_MOTOR_RUN;
	stDataSet.stSetup.MotorStandbyMode = DEF_MOTOR_STANDBY_MODE;
	stDataSet.stSetup.MotorPower = DEF_MOTOR_POWER;

#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	stDataSet.stSetup.MotorAccTime = DEF_MOTOR_ACC_TIME;
#endif

	stDataSet.stSetup.LampMode = DEF_LAMP_MODE;
	stDataSet.stSetup.LampIntensity = DEF_LAMP_INTENSITY;

	stDataSet.stSetup.LedPixelMode[PROGRAM_SET_ONE] = DEF_LEDPIXEL_MODE_PROGRAM;
	stDataSet.stSetup.LedPixelMode[PROGRAM_SET_TWO] = DEF_LEDPIXEL_MODE_PROGRAM;
	stDataSet.stSetup.LedPixelMode[MANUAL_SET_ONE] = DEF_LEDPIXEL_MODE_MANUAL;

	stDataSet.stSetup.Year = 0x00;
	stDataSet.stSetup.Month = 0x00;
	stDataSet.stSetup.Day = 0x00;
	stDataSet.stSetup.Weekday = 0x01;
	stDataSet.stSetup.Hour = 0x00;
	stDataSet.stSetup.Minute = 0x00;
	stDataSet.stSetup.Second = 0x00;

	stDataSet.stSetup.PowerSaveMode = DEF_POWER_SAVE_MODE;
	stDataSet.stSetup.StartHour = DEF_POWER_SAVE_START_HOUR;
	stDataSet.stSetup.StartMinute = DEF_POWER_SAVE_START_MINUTE;
	// stDataSet.stSetup.StartSecond = DEF_POWER_SAVE_START_SECOND;
	stDataSet.stSetup.EndHour = DEF_POWER_SAVE_END_HOUR;
	stDataSet.stSetup.EndMinute = DEF_POWER_SAVE_END_MINUTE;
	// stDataSet.stSetup.EndSecond = DEF_POWER_SAVE_END_SECOND;

#if PUMP_PRESSURE_SENSOR_ATTACHED
	stDataSet.stSetup.LowInletAlarm = LOW_INLET_ALARM_OFF;
	stDataSet.stSetup.LowInletPress = DEF_LOW_INLET_ALARM_PRESSURE;
#endif

	stDataSet.stSetup.CleaningMode = DEF_CLEANING_MODE;

#if AUTOMATION_SENSOR_ATTACHED
	stDataSet.stSetup.AutomationMode = DEF_AUTOMATION_MODE;
	stDataSet.stSetup.AutoFlushingTime = DEF_AUTO_FLUSHING_TIME;
	stDataSet.stSetup.AutoBrewWaitTime = DEF_AUTO_BREW_WAIT_TIME;
#else
	stDataSet.stSetup.FlushingDetectionMode = DEF_FLUSHING_DETECTION_MODE;
	stDataSet.stSetup.FlushingDetectionTime = DEF_FLUSHING_DETECTION_TIME;
	stDataSet.stSetup.FlushingDetectionPress = DEF_FLUSHING_DETECTION_PRESS;
#endif

	stDataSet.stSetup.MaintenanceMode = DEF_MAINTENANCE_MODE;
	stDataSet.stSetup.DefaultSet = DEF_DEFAULT_SET;

	for (int nPos = 0; nPos < 4; nPos++) {
		stDataSet.stSetup.PasswordInput[nPos] = 0;		
	}

	stDataSet.stMaintenance.ErrorHistoryIndex = 0x01;
	stDataSet.stMaintenance.ErrorAddressIndex = 0x00;
	stDataSet.stMaintenance.ErrorCode = DEF_ERROR_NONE;
	stDataSet.stMaintenance.RecentErrorCode = DEF_ERROR_NONE;
	stDataSet.stMaintenance.NoticeCode = DEF_NOTICE_NONE;
	stDataSet.stMaintenance.RecentNoticeCode = DEF_NOTICE_NONE;
	stDataSet.stMaintenance.TotBrewCount = 0UL;
	stDataSet.stMaintenance.TotFlowQuantity = 0UL;
	stDataSet.stMaintenance.Overflowed = 0x00;

	stDataSet.stIndicator.nShotTimeLevel = BREW_PROFILE_NORMAL_LEVEL;
	stDataSet.stIndicator.nCurFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;
	stDataSet.stIndicator.nAvgFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;
	stDataSet.stIndicator.nAvgBrewPressLevel = BREW_PROFILE_NORMAL_LEVEL;

	// Errors / Alarms
	// Level : System Error
	stError.bIsSystemOverloaded = false;
	stError.bFailToStandbyBrewPressure = false;
	stError.bIsBrewPressureOver = false;
	stError.bIsBrewPressureUnder = false;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	stError.bIsLowInletPressure = false;
#endif
	stError.bIsGroupheadOverheated = false;
	stError.bIsBrewBoilerOverheated = false;
	stError.bIsPreheatBoilerOverheated = false;
	stError.bIsGroupheadUnderheated = false;
	stError.bIsBrewBoilerUnderheated = false;
	stError.bIsPreheatBoilerUnderheated = false;
	stError.bIsTemperatureSensorError = false;

	// Level : Notice Message
	stError.bIsFlowmeterFailed = false;
	stError.bIsMotorAlarmReceived = false;
	stError.bIsMaintenanceRequired = false;
	stError.bIsFlowrateExceeded = false;
#if AUTOMATION_SENSOR_ATTACHED
	stError.bIsOpticalFiberUnstable = false;
#endif
#ifdef _USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_
	stError.bIsEEPROMWorn = false;
#endif
}

void restore_StructureDataSet(void)
{
	load_EEPROMBackupAddressPointerTable();

	load_DefaultProgramNum(stDataSet);

	for (int nProgramNum = PROGRAM_SET_ONE; nProgramNum <= MANUAL_SET_ONE; nProgramNum++) {
		load_ProgramParam(nProgramNum, stDataSet.stSetup);
	}

	load_ManualProgramSetNumber(stDataSet.stSetup);
	load_InjectionSetupParam(stDataSet.stSetup);
	load_HeaterSetValueTemperature(stDataSet.stSetup);
	load_HeaterProportionalBandTemperature(stDataSet.stSetup);
	load_HeaterIntegralTime(stDataSet.stSetup);
	load_HeaterDerivativeTime(stDataSet.stSetup);
	load_HeaterMVHighLimit(stDataSet.stSetup);
	load_HeaterInputBias(stDataSet.stSetup);
	load_PressureSensorZeroShift(stDataSet.stSetup);
	load_PressureSensorSpanFactor(stDataSet.stSetup);
	load_MotorPumpParam(stDataSet.stSetup);
	load_MotorACCTimeParam(stDataSet.stSetup);
	load_GroupheadLampParam(stDataSet.stSetup);
	load_PowerSavingParam(stDataSet.stSetup);
#if AUTOMATION_SENSOR_ATTACHED
	load_AutomationParam(stDataSet.stSetup);
#else
	load_FlushingDetectionParam(stDataSet.stSetup);
#endif

	load_LEDPixelMode(stDataSet.stSetup);

	load_MaintenanceBrewUsage(stDataSet.stMaintenance);

	restore_SystemErrorIndex(stDataSet.stMaintenance);
	stDataSet.stMaintenance.RecentErrorCode = (get_SystemErrorCode(true, 0) & 0xFFFF);
}

void reset_StructureDataSet(void)
{
	save_EEPROMBackupAddressPointerTable();

	save_DefaultProgramNum(stDataSet.ProgramNum);

	for (int nProgramNum = PROGRAM_SET_ONE; nProgramNum <= MANUAL_SET_ONE; nProgramNum++) {
		save_ProgramParam(nProgramNum, stDataSet.stSetup);
	}

	save_ManualProgramSetNumber(stDataSet.stSetup.ProgramSetNum);
	save_InjectionSetupParam(stDataSet.stSetup);
	save_HeaterSetValueTemperature(stDataSet.stSetup);
	save_HeaterProportionalBandTemperature(stDataSet.stSetup);
	save_HeaterIntegralTime(stDataSet.stSetup);
	save_HeaterDerivativeTime(stDataSet.stSetup);
	save_HeaterMVHighLimit(stDataSet.stSetup);
	save_HeaterInputBias(stDataSet.stSetup);
	save_PressureSensorZeroShift(stDataSet.stSetup);
	save_PressureSensorSpanFactor(stDataSet.stSetup);
	save_MotorPumpParam(stDataSet.stSetup);
	save_MotorACCTimeParam(stDataSet.stSetup);
	save_GroupheadLampParam(stDataSet.stSetup);
	save_PowerSavingParam(stDataSet.stSetup);
#if AUTOMATION_SENSOR_ATTACHED
	save_AutomationParam(stDataSet.stSetup);
#else
	save_FlushingDetectionParam(stDataSet.stSetup);
#endif

	save_LEDPixelMode(stDataSet.stSetup);

	save_MaintenanceBrewUsage(stDataSet.stMaintenance);

	clear_SystemErrorHistory(stDataSet.stMaintenance);
}

void WaitForFirstSensorMeasurement(void)
{
	unsigned long ulStartPolling = millis();

	hPixelLED.drawPixels(RGB_INIT_BACKGROUND, RGB_INIT_BRIGHTNESS);

	while (millis() - ulStartPolling <= SYSTEM_WAIT_FOR_MEASURING_SENSOR_INITIALLY) {

		delay(TM4CONTROLLER_TEMP_MEASUREMENT_PERIOD);

		measure_PressureSensorAll();
		measure_TM4ControllerTemperatureSensorAll();

	}

#if BREW_PRESSURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue();
#endif

#if PUMP_PRESSURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.PumpPress = get_PumpPressureValue();
#endif

#if AUX_CURRENT_SENSOR_ATTACHED
	stDataSet.stAcquisition.AuxPress = get_AuxPressureValue();
#endif

#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.PreheatTemp = hTM4Controller.getPresentTemperatureValue(TM4_PREHEAT_CHANNEL);
#endif

#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewTemp = hTM4Controller.getPresentTemperatureValue(TM4_BREWING_CHANNEL);
#endif

#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.GroupTemp = hTM4Controller.getPresentTemperatureValue(TM4_GROUPHEAD_CHANNEL);
#endif

#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewWaterTemp = hTM4Controller.getPresentTemperatureValue(TM4_BREWWATER_CHANNEL);
#endif
}

void initialize_MenupageFunctionPointerTable(void)
{
	procedure_SetupMenupage[Menupage1] = procedure_ProgramOneSetup;
	procedure_SetupMenupage[Menupage2] = procedure_ProgramTwoSetup;
	procedure_SetupMenupage[Menupage3] = procedure_ManualOneSetup;
	procedure_SetupMenupage[Menupage4] = procedure_InjectionSetup;
	procedure_SetupMenupage[Menupage5] = procedure_GroupheadTempSetup;
	procedure_SetupMenupage[Menupage6] = procedure_BrewBoilerTempSetup;
	procedure_SetupMenupage[Menupage7] = procedure_PreheatBoilerTempSetup;
	procedure_SetupMenupage[Menupage8] = procedure_MotorPumpSetup;
	procedure_SetupMenupage[Menupage9] = procedure_LampSetup;
	procedure_SetupMenupage[Menupage10] = procedure_LEDPixelModeSetup;
	procedure_SetupMenupage[Menupage11] = procedure_TimeDateSetup;
	procedure_SetupMenupage[Menupage12] = procedure_PowerSavingSetup;
	procedure_SetupMenupage[Menupage13] = procedure_HeaterParameterSetup;
	procedure_SetupMenupage[Menupage14] = procedure_PressureSensorAdjustmentSetup;
	procedure_SetupMenupage[Menupage15] = procedure_MotorAccelerationSetup;
#if AUTOMATION_SENSOR_ATTACHED
	procedure_SetupMenupage[Menupage16] = procedure_AutomationSetup;
#else
	procedure_SetupMenupage[Menupage16] = procedure_FlushingDetectionModeSetup;
#endif
	procedure_SetupMenupage[Menupage17] = procedure_MaintenanceSetup;
	procedure_SetupMenupage[Menupage18] = procedure_DefaultResetSetup;
	procedure_SetupMenupage[Menupage19] = procedure_SystemInformationSetup;
	procedure_SetupMenupage[Menupage20] = procedure_PasswordAuthorizationSetup;
}

bool isFactoryDefault(void)
{
	unsigned char nFactoryResetFlag = load_FactoryResetFlag();

	if (nFactoryResetFlag != APPLICATION_RUN_CODE) {

		nFactoryResetFlag = FACTORY_DEFAULT_CODE;

		// reset_TimeDateRTC();

	}
	else {

		int nFactorySwitch = SW_PUSHED;

		if ((nFactorySwitch = digitalRead(ROTARY_ENCODER_SW)) == SW_PUSHED) {

			unsigned long ulStartPolling = millis();

			while (millis() - ulStartPolling <= SYSTEM_FACTORY_DEFAULT_KEY_HOLD_TIME) {

				nFactorySwitch |= digitalRead(ROTARY_ENCODER_SW);

			}

			if (nFactorySwitch == SW_PUSHED) {

				nFactoryResetFlag = FACTORY_DEFAULT_CODE;

			}

		}

	}

	if (nFactoryResetFlag == FACTORY_DEFAULT_CODE) {

		clear_ContentsEEPROM();

		save_FactoryResetFlag(APPLICATION_RUN_CODE);
		save_HardwareVersion(HARDWARE_VERSION);
		save_FirmwareVersion(FIRMWARE_VERSION);

		reset_TimeDateRTC();

		return false;

	}

	return true;
}

void activate_SystemStateMachine(void)
{
	if (WaitforSystemActivationKey()) {

		if (!resume_BoilerHeater()) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] Failed to resume Boiler Heater") << endl;
			// #endif

			wait_RebootSystemWithConsoleMsg();

		}


		_nBrewState = BREW_STANDBY_STATE;
		_nSetupMenuState = LCD_DASHBOARD_ENTRY;

		hEspressoMachine.init(&stDataSet);

		#if BREW_BOILER_PRESSURE_STABILIZATION
		hEspressoMachine.setIdleProcessing(display_TextLCDOnUserInterface);
		#endif

		hLamp.setMode(static_cast<CLamp::LampMode>(stDataSet.stSetup.LampMode));


		_console.print(F("\n\n\rMOAI EspressoStation Activated.\n\n\r"));

	}
	else {

		_console.print(F("\n\n\r[ERROR] System Activation Failed.\n\n\r"));

		wait_RebootSystemWithConsoleMsg();

	}
}

void reset_SystemParametersToFactoryDefault(void)
{
	unsigned char nProgramNum;
	float fZeroShift[3];
	float fSpanFactor[3];
	unsigned long ulTotFlowQuantity = 0UL;
	unsigned long ulTotBrewCount = 0UL;
	unsigned char nOverflowed = 0x00;

	nProgramNum = stDataSet.ProgramNum;

	for (int nSensor = 0; nSensor < 3; nSensor++) {
		fZeroShift[nSensor] = stDataSet.stSetup.ZeroShift[nSensor];
		fSpanFactor[nSensor] = stDataSet.stSetup.SpanFactor[nSensor];
	}

	ulTotFlowQuantity = stDataSet.stMaintenance.TotFlowQuantity;
	ulTotBrewCount = stDataSet.stMaintenance.TotBrewCount;
	nOverflowed = stDataSet.stMaintenance.Overflowed;

	initialize_StructureDataSet();

	stDataSet.ProgramNum = nProgramNum;

	for (int nSensor = 0; nSensor < 3; nSensor++) {
		stDataSet.stSetup.ZeroShift[nSensor] = fZeroShift[nSensor];
		stDataSet.stSetup.SpanFactor[nSensor] = fSpanFactor[nSensor];
	}

	stDataSet.stMaintenance.TotBrewCount = ulTotBrewCount;
	stDataSet.stMaintenance.TotFlowQuantity = ulTotFlowQuantity;
	stDataSet.stMaintenance.Overflowed = nOverflowed;

	reset_StructureDataSet();

	setup_BoilerTemperatureControllerPID();
	setup_BoilerTemperatureControllerMV();
	setup_BoilerTemperatureControllerBIAS();
	setup_BoilerTemperatureControllerSV();
}


bool WaitforSystemActivationKey(void)
{
	bool bIsActivated = false;
	bool bIsAirVentilating = false;

	unsigned long ulAirVentilationTime = 0UL;

	draw_LEDPalettOnAirVentilation();

	ulAirVentilationTime = millis();

	while (bIsAirVentilating || (millis() - ulAirVentilationTime <= SYSTEM_WAIT_FOR_AIR_VENTILATION_INITIALLY)) {

		int nCurrButton = BTN_PULLED;

		if ((nCurrButton = digitalRead(BREW_BUTTON_SWITCH)) == BTN_PUSHED) {

			unsigned long ulStartPolling = millis();

			while (millis() - ulStartPolling <= SYSTEM_AIR_VENTILATION_KEY_HOLD_TIME) {

				nCurrButton |= digitalRead(BREW_BUTTON_SWITCH);

				hPixelLED.drawPalettMode();

			}

		}

		if (nCurrButton == BTN_PUSHED) {

			if (!hBrewValve.isOpening()) {

				bIsAirVentilating = true;

				hBrewValve.open();

				while (digitalRead(BREW_BUTTON_SWITCH) == BTN_PUSHED) {

					hPixelLED.drawPalettMode();

					delay(5UL);

				}

			}
			else {

				bIsAirVentilating = false;

				hBrewValve.close();

				break;

			}

		}

		hPixelLED.drawPalettMode();

	}

	hPixelLED.drawPixels(RGB_ACTIVATION_PROGRESS_FG, RGB_INIT_BRIGHTNESS);

	while (!bIsActivated) {

		int nCurrButton = BTN_PULLED;

		if ((nCurrButton = digitalRead(BREW_BUTTON_SWITCH)) == BTN_PUSHED) {

			unsigned long ulStartPolling = millis();

			while (millis() - ulStartPolling <= SYSTEM_ACTIVATION_KEY_HOLD_TIME) {

				nCurrButton |= digitalRead(BREW_BUTTON_SWITCH);

			}

		}

		bIsActivated = nCurrButton == BTN_PUSHED ? true : false;

	}

	draw_LEDPalettOnActivation();
	hPixelLED.drawPalettMode();

	while (digitalRead(BREW_BUTTON_SWITCH) == BTN_PUSHED) {

		static bool _bIncrease = false;
		static unsigned char _nBrightness = MAX_DIM_BRIGHTNESS;

		if (_nBrightness >= MAX_DIM_BRIGHTNESS) {
			_bIncrease = false;
		}
		else if (_nBrightness <= MIN_DIM_BRIGHTNESS) {
			_bIncrease = true;
		}

		hPixelLED.setBrightness((_bIncrease ? _nBrightness += 1 : _nBrightness -= 1));

		delay(30UL);

	}

	hPixelLED.setBrightness(RGB_BRIGHTNESS);

	return bIsActivated;
}

void display_SystemActivation(void)
{
	display_SystemDashboard(true, stDataSet);

	draw_LEDPalettOnBrewStandby();

	if (hLamp.getMode() == CLamp::TURNON_ALWAYS) {

		hLamp.turnon();

	}
}

void activate_AllUserInterface(void)
{
	_nMutuallyExclusiveActivated = ALL_USER_INTERFACE_ACTIVATED;

	enable_BrewButton();
	enable_RotaryEncoder();
}

void deactivate_AllUserInterface(void)
{
	_nMutuallyExclusiveActivated = ALL_USER_INTERFACE_DEACTIVATED;

	disable_BrewButton();
	disable_RotaryEncoder();
}

void config_UserInterfaceActivation(const unsigned char nSetMask)
{
	unsigned char nSetMaskValue = nSetMask & 0b11;

	if (_nMutuallyExclusiveActivated != nSetMaskValue) {

		_nMutuallyExclusiveActivated = nSetMaskValue;

		switch (_nMutuallyExclusiveActivated) {

			case ALL_USER_INTERFACE_DEACTIVATED:
				disable_BrewButton();
				disable_RotaryEncoder();
			break;

			case BREW_BUTTON_MUTEX_ACTIVATED_MASK:
				enable_BrewButton();
				disable_RotaryEncoderDirection();
			break;

			case ROTARY_ENCODER_MUTEX_ACTIVATED_MASK:
				disable_BrewButton();
				enable_RotaryEncoder();
			break;

			case ALL_USER_INTERFACE_ACTIVATED:
			default:
				enable_BrewButton();
				enable_RotaryEncoder();
			break;

		}

		if (_nMutuallyExclusiveActivated & ROTARY_ENCODER_MUTEX_ACTIVATED_MASK) {

			enable_ExternalInterrupt(TriggerRotaryEncoderST_Callback, PIN_INTERRUPT_NUM_1, ROTARY_ENCODER_STEP_TRIGGER_TYPE);

		}
		else {

			if (_nMutuallyExclusiveActivated != ALL_USER_INTERFACE_DEACTIVATED) {

				disable_ExternalInterrupt(PIN_INTERRUPT_NUM_1);

			}

		}

	}
}

bool isMutuallyExclusiveActivated(const unsigned char nQueryMask)
{
	return (_nMutuallyExclusiveActivated & nQueryMask ? true : false);
}


// Application Threads Init / Callback
void initialize_AppThreads(void)
{
#if BREW_PRESSURE_SENSOR_ATTACHED
	hPressureSensorThread.add(&hPressureBrewBoiler);
#endif

#if PUMP_PRESSURE_SENSOR_ATTACHED
	hPressureSensorThread.add(&hPressureFeed);
#endif

#if AUX_CURRENT_SENSOR_ATTACHED
	hPressureSensorThread.add(&hPressureAUX);
#endif

#if (BREW_PRESSURE_SENSOR_ATTACHED || PUMP_PRESSURE_SENSOR_ATTACHED || AUX_CURRENT_SENSOR_ATTACHED)
	hPressureSensorThread.begin(PRESSURE_MEARSUREMENT_PERIOD);
	hAppThreadContainer.add(&hPressureSensorThread);
#endif

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	hAnalogSensorThread.add(&hAnalogInput);
#endif

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED	// || (ANY OTHER ANALOG INPUT HANDLE) || ...
	hAnalogSensorThread.begin(ANALOG_INPUT_MEASUREMENT_PERIOD);
	hAppThreadContainer.add(&hAnalogSensorThread);
#endif

	hRealTimeClockThread.onRun(PollingRealTimeClock_Callback);
	hRealTimeClockThread.setInterval(REALTIME_CLOCK_FETCH_PERIOD);
	hAppThreadContainer.add(&hRealTimeClockThread);
}

void AppThreadShouldRun_Callback(void)
{
	hAppThreadContainer.run();

	// if (hPressureSensorThread.shouldRun()) 

	// 	hPressureSensorThread.run();
}


// User Interface(Button, Rotary Encoder with Switch) Binding
int bind_SignalBrewUserInterface(void)
{
	unsigned char nButtonEvent;

	int nCurrState = _nBrewState;
	int nNextState = _nBrewState;

	if (nCurrState == SYSTEM_ERROR_STATE) {

		hBrewButton.clearButton();

		return nNextState;

	}

#if AUTOMATION_SENSOR_ATTACHED
	if (start_AutoBrewingMode()) {

		hBrewButton.clearButton();

		if (nCurrState == BREW_STANDBY_STATE) {

			nNextState = BREW_START_STATE;

		}

		return nNextState;

	}
#endif

	if (hBrewButton.isButtonEventOccured()) {

		switch (nButtonEvent = hBrewButton.getButton()) {

			case BUTTON_PUSHED_SHORT:

				if (nCurrState == BREW_MANUAL_STATE || nCurrState == BREW_SETUP_STATE) {

					nNextState = BREW_STAGE_CHANGE_STATE;

				}
				else {

					if (nCurrState != POWER_SAVING_STATE) {

						if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

#if AUTOMATION_SENSOR_ATTACHED
							if (nCurrState != AUTO_FLUSHING_START_STATE) {

								nNextState = nCurrState == BREW_STANDBY_STATE ? BREW_START_STATE : BREW_STOP_STATE;

							}
							else {

								nNextState = AUTO_FLUSHING_STOP_STATE;

							}
#else
							nNextState = nCurrState == BREW_STANDBY_STATE ? BREW_START_STATE : BREW_STOP_STATE;
#endif

						}
						else {

							nNextState = nCurrState != CLEANING_MACHINE_START_STATE ? CLEANING_MACHINE_START_STATE : CLEANING_MACHINE_STOP_STATE;

						}

					}

				}

			break;

			case BUTTON_PUSHED_DCLK:

				if (nCurrState != POWER_SAVING_STATE) {

					if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

						if (stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_OFF) {

#if AUTOMATION_SENSOR_ATTACHED
							if (nCurrState != AUTO_FLUSHING_START_STATE) {

								nNextState = nCurrState == BREW_STANDBY_STATE ? BREW_MODE_CHANGE_STATE : BREW_STOP_STATE;

							}
							else {

								nNextState = AUTO_FLUSHING_STOP_STATE;

							}
#else
							nNextState = nCurrState == BREW_STANDBY_STATE ? BREW_MODE_CHANGE_STATE : BREW_STOP_STATE;
#endif

						}
						else {

							nNextState = nCurrState == BREW_STANDBY_STATE ? BREW_SETUP_CANCEL_STATE : BREW_STOP_STATE;

						}

					}

				}

			break;

			case BUTTON_PUSHED_LONG:

				#if defined(_USE_APPLICATION_DEBUG_MSG_)
				// cout << F("BREW_BUTTON_EVENT::") << int(nButtonEvent) << endl;
				#endif

				if (nCurrState == BREW_STANDBY_STATE) {

					if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

						if (stDataSet.ProgramNum == MANUAL_SET_ONE) {

							nNextState = BREW_SETUP_EXIT_STATE;

						}
						else {

							nNextState = stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_OFF ? BREW_SETUP_ENTRY_STATE : BREW_SETUP_EXIT_STATE;

						}

					}

				}
				else {

					if (nCurrState == POWER_SAVING_STATE) {

						nNextState = POWER_SAVING_CONTINUE_STATE;

					}
					else {

						if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

#if AUTOMATION_SENSOR_ATTACHED
							if (nCurrState != AUTO_FLUSHING_START_STATE) {

								nNextState = BREW_STOP_STATE;

							}
							else {

								nNextState = AUTO_FLUSHING_STOP_STATE;

							}
#else
							nNextState = BREW_STOP_STATE;
#endif

						}

					}

				}

				#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
				_bPredictVLongPushingEvent = false;
				#endif

			break;

			case BUTTON_PUSHED_VLONG:

				#if defined(_USE_APPLICATION_DEBUG_MSG_)
				// cout << F("BREW_BUTTON_EVENT::") << int(nButtonEvent) << endl;
				#endif

				if (nCurrState == BREW_STANDBY_STATE) {

					if (_nPowerSavingState == PWS_PAUSED) {

						nNextState = stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_OFF ? POWER_SAVING_RESUME_STATE : BREW_SETUP_EXIT_STATE;

					}
					else {

						if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

							if (stDataSet.ProgramNum == MANUAL_SET_ONE) {

								nNextState = BREW_SETUP_EXIT_STATE;

							}
							else {

								nNextState = stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_OFF ? BREW_SETUP_ENTRY_STATE : BREW_SETUP_EXIT_STATE;

							}

						}

					}

				}
				else {

					if (nCurrState == POWER_SAVING_STATE) {

						nNextState = POWER_SAVING_PAUSE_STATE;

					}
					else {

						if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

#if AUTOMATION_SENSOR_ATTACHED
							if (nCurrState != AUTO_FLUSHING_START_STATE) {

								nNextState = BREW_STOP_STATE;

							}
							else {

								nNextState = AUTO_FLUSHING_STOP_STATE;

							}
#else
							nNextState = BREW_STOP_STATE;
#endif

						}

					}

				}

				#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
				_bPredictVLongPushingEvent = false;
				#endif

			break;

			case BUTTON_ON_LPUSHING:

				if (nCurrState == BREW_STANDBY_STATE || nCurrState == POWER_SAVING_STATE) {

					if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

						#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
						_bResetTimerToPredictVLongPushingEvent = true;
						_bPredictVLongPushingEvent = true;
						#endif

						draw_LEDPalettOnLongPushing();

					}

				}

			break;

		}

	}

	#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
	if (_bPredictVLongPushingEvent == true) { // keep on pushing

		draw_LEDPalettOnVLongPushing();

	}
	#endif

	#if defined(_USE_APPLICATION_DEBUG_MSG_)
	// cout << F("BREW_BUTTON_EVENT::") << nCurrState << F(">>") << nNextState << endl;
	#endif

	return nNextState;	
}


// System State Machine and Setup-Menu State Machine
bool app_BrewStateMachine(void)
{
	bool bMutuallyExclusiveActivated = true;

	int nSignal;

	switch (nSignal = bind_SignalBrewUserInterface()) {

		case BREW_STOP_STATE:

			if (hEspressoMachine.getBrewState() != CEspresso::BREW_STOP) {

				hEspressoMachine.stopBrewing();

			}

			if (hLamp.isTurnon()) {

				hLamp.turnoff();

			}

			_bJustGetStoppedBrewing = true;

			increase_BrewingCount();
			accumulate_BrewWaterQuantity();

			if (stDataSet.stAcquisition.ShotTime > FLOWMETER_FAILURE_TIME_CONDITION) {

				observe_FlowmeterFailure();		// [Notice] Don't change the position
				
			}

			queue_BrewObservations(false);

#if AUTOMATION_SENSOR_ATTACHED
			stop_AutoBrewingMode();
			enable_AutomationMode(false);
#endif

			draw_LEDPalettOnBrewStop();

			bMutuallyExclusiveActivated = false;

			_nBrewState = BREW_STANDBY_STATE;

		break;

		case BREW_START_STATE:

#if AUTOMATION_SENSOR_ATTACHED
			disable_AutomationMode();

			stabilize_BrewBoilerPressure();
#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_) || defined(_USE_BREW_PROFILE_ACQUISITION_)
			print_BrewingLogHeader();
			#endif

			if (stDataSet.stSetup.MotorStandbyMode >= MOTOR_STANDBY_PREP) {

				if (hMotorPump.isRunning()) {

					hMotorPump.stop();

				}

			}

			if (!hLamp.isTurnon()) {

				hLamp.turnon();

			}

			queue_BrewObservations(true);

			_bJustGetStoppedBrewing = false;

			hEspressoMachine.startBrewing();

			draw_LEDPalettOnBrewing();

			switch (hEspressoMachine.getBrewMode()) {

				case CEspresso::BREW_MODE:
					_nBrewState = BREW_PROGRAM_STATE;
				break;

				case CEspresso::SETUP_MODE:
					_nBrewState = BREW_SETUP_STATE;
				break;

				case CEspresso::FREE_MODE:
					_nBrewState = BREW_MANUAL_STATE;
				break;

			}

			bMutuallyExclusiveActivated = false;

		break;

		case BREW_STAGE_CHANGE_STATE:

			switch (hEspressoMachine.getBrewState()) {

				case CEspresso::BREW_INJECTION:
					hEspressoMachine.setBrewState(CEspresso::BREW_EXTRACTION);
				break;

				case CEspresso::BREW_EXTRACTION:
					hEspressoMachine.setBrewState(CEspresso::BREW_STOP);
				break;

			}

			bMutuallyExclusiveActivated = false;

			_nBrewState = hEspressoMachine.getBrewMode() == CEspresso::FREE_MODE ? BREW_MANUAL_STATE : BREW_SETUP_STATE;

		case BREW_MANUAL_STATE:
		case BREW_SETUP_STATE:
		case BREW_PROGRAM_STATE:

			if (hEspressoMachine.onBrewing() == CEspresso::BREW_STOP) { // hEspressoMachine.getBrewState() == CEspresso::BREW_STOP

				_nBrewState = BREW_STOP_STATE;

			}
			else {

				observe_SystemStatusOnBrewing();

				update_LEDPalettOnBrewing();

			}

			bMutuallyExclusiveActivated = false;

		break;

		case BREW_SETUP_ENTRY_STATE:

			stDataSet.stSetup.BrewSetupMode = BREW_SETUP_MODE_ON;

			hEspressoMachine.setBrewMode(CEspresso::SETUP_MODE);
			hEspressoMachine.clearProgramSet();

			draw_LEDPalettOnSetupEntry();

			_nBrewState = BREW_STANDBY_STATE;

		break;

		case BREW_SETUP_CANCEL_STATE:

			hEspressoMachine.clearProgramSet();

		case BREW_SETUP_EXIT_STATE:

			hEspressoMachine.setProgramNum(stDataSet.ProgramNum);

			if (hEspressoMachine.prepareToSave() == true) {

				stDataSet.stSetup.BrewSetupMode = BREW_SETUP_MODE_OFF;

				save_ProgramParam(stDataSet.ProgramNum, stDataSet.stSetup);

				if (stDataSet.ProgramNum == MANUAL_SET_ONE) {

					copy_ProgramParam(stDataSet.stSetup.ProgramSetNum, stDataSet.ProgramNum, stDataSet.stSetup);

					stDataSet.ProgramNum = stDataSet.stSetup.ProgramSetNum;
					hEspressoMachine.setProgramNum(stDataSet.stSetup.ProgramSetNum);

					load_ProgramParam(stDataSet.ProgramNum, stDataSet.stSetup);

				}

				hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);

			}
			else {

				stDataSet.stSetup.BrewSetupMode = BREW_SETUP_MODE_OFF;

				if (stDataSet.ProgramNum != MANUAL_SET_ONE) {

					hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);

				}

			}

			draw_LEDPalettOnSetupExit();

			bMutuallyExclusiveActivated = false;

			_nBrewState = BREW_STANDBY_STATE;

		break;

		case BREW_MODE_CHANGE_STATE:

			switch (stDataSet.ProgramNum) {

				case PROGRAM_SET_ONE:
					stDataSet.ProgramNum = PROGRAM_SET_TWO;
					hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);
				break;

				case PROGRAM_SET_TWO:
					stDataSet.ProgramNum = MANUAL_SET_ONE;
					hEspressoMachine.setBrewMode(CEspresso::FREE_MODE);
				break;

				case MANUAL_SET_ONE:
					stDataSet.ProgramNum = PROGRAM_SET_ONE;
					hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);
				break;

			}

			load_ProgramParam(stDataSet.ProgramNum, stDataSet.stSetup);

			update_LEDPalettOnBrewStandby();

			_nBrewState = BREW_STANDBY_STATE;

		// break;

		case BREW_STANDBY_STATE:
		// default:

			bMutuallyExclusiveActivated = false;

			standby_BrewBoilerPressure();

			observe_SystemStatusOnStandby();

#if AUTOMATION_SENSOR_ATTACHED
			if (start_AutoFlushingMode()) {

				_nBrewState = AUTO_FLUSHING_START_STATE;

				break;

			}
#endif

			if (sleep_BoilerHeater()) {

#if AUTOMATION_SENSOR_ATTACHED
				suspend_AutomationMode(); // disable_AutomationMode();
#endif

				if (hMotorPump.isRunning()) {

					hMotorPump.stop();

				}

				if (stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_ON) {

					stDataSet.stSetup.BrewSetupMode = BREW_SETUP_MODE_OFF;

					if (stDataSet.ProgramNum != MANUAL_SET_ONE) {

						hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);

					}

				}

				draw_LEDPalettOnSleep();

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[POWER SAVE] MOAI EspressoStation Slept.") << endl;
				// #endif

				_nBrewState = POWER_SAVING_STATE;

			}
			else {

				_nBrewState = BREW_STANDBY_STATE;

			}

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			// print_BrewingStandbyObservation();
			#endif

		break;

		case POWER_SAVING_PAUSE_STATE:
		// case POWER_SAVING_STOP_STATE:

			_nPowerSavingState = PWS_PAUSED;

			if (wakeup_BoilerHeater()) {

#if AUTOMATION_SENSOR_ATTACHED
				resume_AutomationMode(); // enable_AutomationMode(false);
#endif

				draw_LEDPalettOnWake();

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[POWER SAVE] MOAI EspressoStation Waked (PAUSED).") << endl;
				// #endif

				_nBrewState = BREW_STANDBY_STATE;

			}
			else {

				_nBrewState = POWER_SAVING_STATE;

			}

			break;

		case POWER_SAVING_RESUME_STATE:

			_nPowerSavingState = PWS_RESUMED;

			if (sleep_BoilerHeater()) {

#if AUTOMATION_SENSOR_ATTACHED
				suspend_AutomationMode(); // disable_AutomationMode();
#endif

				draw_LEDPalettOnSleep();

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[POWER SAVE] MOAI EspressoStation Slept (RESUMED).") << endl;
				// #endif

				_nBrewState = POWER_SAVING_STATE;

			}
			else {

				_nBrewState = BREW_STANDBY_STATE;

			}

			break;

		case POWER_SAVING_CONTINUE_STATE:

			if (_nPowerSavingState == PWS_NOW_WORKING) {

				draw_LEDPalettOnSleep();

				#if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[POWER SAVE] MOAI EspressoStation Slept (CONTINUED).") << endl;
				#endif

			}

		case POWER_SAVING_STATE:

			bMutuallyExclusiveActivated = false;

			observe_SystemStatusOnStandby();

			if (wakeup_BoilerHeater()) {

#if AUTOMATION_SENSOR_ATTACHED
				resume_AutomationMode(); // enable_AutomationMode(false);
#endif

				draw_LEDPalettOnWake();

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[POWER SAVE] MOAI EspressoStation Waked.") << endl;
				// #endif

				_nBrewState = BREW_STANDBY_STATE;

			}
			else {

				_nBrewState = POWER_SAVING_STATE;

			}

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			// print_BrewingStandbyObservation();
			#endif

		break;

		case CLEANING_MACHINE_START_STATE:
		case CLEANING_MACHINE_STOP_STATE:

			observe_SystemStatusOnStandby();

			if (procedure_CleaningMachine(nSignal)) {

				_nBrewState = CLEANING_MACHINE_START_STATE;

			}
			else {

#if AUTOMATION_SENSOR_ATTACHED
				enable_AutomationMode();
#endif

				accumulate_BrewWaterQuantity();
				
				eDashboard nCurDashboard = get_TextLCDDashboard();

				set_SelectedCursor(false);
				set_TextLCDDashboard(nCurDashboard);//DashboardA
				set_ChangingTextLCD(true);

				_nSetupMenuState = LCD_DASHBOARD_ENTRY;
				
				draw_LEDPalettOnBrewStandby();

				_nBrewState = BREW_STANDBY_STATE;

			}

		break;

#if AUTOMATION_SENSOR_ATTACHED
		case AUTO_FLUSHING_START_STATE:

			bMutuallyExclusiveActivated = false;

			if (procedure_AutoFlushingMode()) {

				observe_SystemStatusOnFlushing();

				_nBrewState = AUTO_FLUSHING_START_STATE;

			}
			else {

				_nBrewState = BREW_STANDBY_STATE;

			}

		break;

		case AUTO_FLUSHING_STOP_STATE:

			bMutuallyExclusiveActivated = false;

			stop_AutoFlushingMode();

			_nBrewState = BREW_STANDBY_STATE;

		break;
#endif

		case SYSTEM_ERROR_STATE:

			bMutuallyExclusiveActivated = false;

			observe_SystemStatusOnStandby();

			_nBrewState = SYSTEM_ERROR_STATE;

		break;

	}


	config_UserInterfaceActivation(bMutuallyExclusiveActivated ? BREW_BUTTON_MUTEX_ACTIVATED_MASK : ALL_USER_INTERFACE_ACTIVATED);


	return bMutuallyExclusiveActivated;
}

int bind_SignalSetupUserInterface(void)
{
	unsigned char nSwitchEvent;

	int nCurrState = _nSetupMenuState;
	int nNextState = _nSetupMenuState;

	eLCDView eCurrentView = get_TextLCDView();

	if (hRotaryEncoder.isSwitchEventOccured()) {

		switch (nSwitchEvent = hRotaryEncoder.getSwitch()) {

			case SWITCH_PUSHED_SHORT:

				switch (eCurrentView) {
					case DashboardView:
						stop_AlarmMessageDisplayTimer();	// alarm message check
						nNextState = LCD_DASHBOARD_CHANGE;
					break;
					case MenuboardView:
						nNextState = LCD_MENUPAGE_ENTRY;
					break;
					case MenupageView:
						nNextState = nCurrState == LCD_MENUPAGE_LIST_SELECT ? LCD_MENUPAGE_LIST_VIEW : LCD_MENUPAGE_LIST_SELECT;
					break;
					case AlarmView:
						set_ChangingTextLCD(true);
						display_EntryMessage(true);
						start_AlarmMessageDisplayTimer();
						nNextState = LCD_DASHBOARD_ENTRY;
					break;
					// case CleaningView:
					// break;
				}

			break;

			case SWITCH_PUSHED_DCLK:

				switch (eCurrentView) {
					case DashboardView:
						if (_nBrewState == BREW_STANDBY_STATE) {
#if AUTOMATION_SENSOR_ATTACHED
							disable_AutomationMode();
#endif
							set_ChangingTextLCD(true);
							display_CleaningEntryMessage(true);
							nNextState = LCD_CLEANING_MODE_ENTRY;
						}
					break;
					case MenuboardView:
						set_ChangingTextLCD(true);
						display_EntryMessage(true);
						stop_AlarmMessageDisplayTimer();	// alarm message check
						nNextState = LCD_DASHBOARD_ENTRY;
					break;
					case MenupageView:
						set_ChangingTextLCD(true);
						display_MenupageExitMessage(false);
						nNextState = LCD_MENUPAGE_CANCEL_EXIT;
					break;
					case AlarmView:
						set_ChangingTextLCD(true);
						display_EntryMessage(true);
						start_AlarmMessageDisplayTimer();
						nNextState = LCD_DASHBOARD_ENTRY;
					break;
					case CleaningView:
						if (_nBrewState == BREW_STANDBY_STATE) {
#if AUTOMATION_SENSOR_ATTACHED
							enable_AutomationMode();
#endif
							nNextState = LCD_CLEANING_MODE_EXIT;
						}
					break;
				}

			break;

			case SWITCH_PUSHED_LONG:
			case SWITCH_PUSHED_VLONG:

				if (isMutuallyExclusiveActivated(ROTARY_ENCODER_ACTIVATED)) {

					set_ChangingTextLCD(true);

					switch (eCurrentView) {
						case DashboardView:
							nNextState = LCD_MENUBOARD_ENTRY;
						break;
						case MenuboardView:
							stop_AlarmMessageDisplayTimer();	// alarm message check
							nNextState = LCD_DASHBOARD_ENTRY;
						break;
						case MenupageView:
							nNextState = LCD_MENUPAGE_SAVE_EXIT;
						break;
					}

				}

			break;

			case SWITCH_ON_LPUSHING:

				if (isMutuallyExclusiveActivated(ROTARY_ENCODER_ACTIVATED)) {

					nNextState = LCD_LONGPUSH_STATE;

				}

			break;

		}

		restart_SetupMenuTimer();

	}
	else {

		if (hRotaryEncoder.isDirectionEventOccured()) {

			if (!_bIgnoreRotaryEncoderEventForSaftyExit) {

				switch (eCurrentView) {
					case MenuboardView:
						nNextState = LCD_MENUBOARD_LIST_CHANGE;
					break;
					case MenupageView:
						nNextState = nCurrState == LCD_MENUPAGE_LIST_SELECT ? LCD_MENUPAGE_VALUE_CHANGE : LCD_MENUPAGE_LIST_CHANGE;
					break;
				}

			}

			restart_SetupMenuTimer();

			// hRotaryEncoder.clearSwitch();

		}

	}

	if (isSetupMenuTimedOver()) {

		if (get_TextLCDView() == MenuboardView) {

			set_ChangingTextLCD(true);

			display_EntryMessage(true);

			nNextState = LCD_DASHBOARD_ENTRY;

		}

	}

	#if defined(_USE_APPLICATION_DEBUG_MSG_)
	// cout << F("LCD_SWITCH_EVENT::") << nCurrState << F(">>") << nNextState << endl;
	#endif

	return nNextState;
}

bool app_SetupStateMachine(void)
{
	bool bMutuallyExclusiveActivated = true;

	int nSignal;

	switch (nSignal = bind_SignalSetupUserInterface()) {

		case LCD_DASHBOARD_CHANGE:

			switch (get_TextLCDDashboard()) {
				case DashboardA:
					set_TextLCDDashboard(DashboardB);
				break;
				case DashboardB:
					set_TextLCDDashboard(DashboardC);
				break;
				case DashboardC:
					set_TextLCDDashboard(DashboardA);
				break;
			}

		case LCD_DASHBOARD_ENTRY:

			if (get_ChangingTextLCD() == true) {

				release_Authorization();

				display_SystemDashboard(true, stDataSet);

				stop_SetupMenuTimer();

			}

			bMutuallyExclusiveActivated = false;

			_nSetupMenuState = LCD_DASHBOARD_ENTRY;

		break;

		case LCD_MENUBOARD_LIST_CHANGE:

			procedure_SetupMenuboard(get_TextLCDMenuboard());

		case LCD_MENUBOARD_ENTRY:

			if (get_ChangingTextLCD() == true) {

				_bIgnoreRotaryEncoderEventForSaftyExit = false;

				display_SystemMenuboard();

				start_SetupMenuTimer();

			}

			display_SelectCursor();

			bMutuallyExclusiveActivated = false;

			_nSetupMenuState = LCD_MENUBOARD_ENTRY;

		break;

		case LCD_MENUPAGE_ENTRY:

#if AUTOMATION_SENSOR_ATTACHED
			if (_nBrewState >= BREW_STAGE_CHANGE_STATE && _nBrewState <= BREW_PROGRAM_STATE || _nBrewState == AUTO_FLUSHING_START_STATE) {
				break;
			}
#else
			if (_nBrewState >= BREW_STAGE_CHANGE_STATE && _nBrewState <= BREW_PROGRAM_STATE) {
				break;
			}
#endif

			if (verify_AuthorizationToMenupageEntry(static_cast<eMenuPage>(get_SelectCursorLine() + LCD_MAX_LINE * get_TextLCDMenuboard()))) {

				set_TextLCDMenupage(static_cast<eMenuPage>(get_SelectCursorLine() + LCD_MAX_LINE * get_TextLCDMenuboard()));

				display_SystemMenupage(true, stDataSet, stCurTimeDate);

				display_SelectCursor();

				stop_SetupMenuTimer();

				hRotaryEncoder.startACC();

				_nSetupMenuState = LCD_MENUPAGE_LIST_VIEW;

			}
			else {

				_nSetupMenuState = LCD_MENUBOARD_ENTRY;

			}

		break;

		case LCD_MENUPAGE_LIST_SELECT:
		case LCD_MENUPAGE_LIST_VIEW:
		case LCD_MENUPAGE_LIST_CHANGE:
		case LCD_MENUPAGE_VALUE_CHANGE:
		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			_nSetupMenuState = (*procedure_SetupMenupage[get_TextLCDMenupage()])(nSignal);

			if (_nSetupMenuState == LCD_MENUBOARD_ENTRY) {

				hRotaryEncoder.stopACC();

				_bIgnoreRotaryEncoderEventForSaftyExit = true;

				set_LeavingMenupage(true);

			}
			else {

				if (nSignal > LCD_MENUPAGE_SAVE_EXIT) {

<<<<<<< HEAD
				hRotaryEncoder.stopACC();

				_bIgnoreRotaryEncoderEventForSaftyExit = true;
=======
					display_SelectCursor();
>>>>>>> feature/brew_profile_acquisition

				}

			}

		break;

		case LCD_CLEANING_MODE_ENTRY:

			if (get_ChangingTextLCD() == true) {
				
				stDataSet.stSetup.CleaningMode = CLEANING_MODE_ON;

				display_CleaningReadyMessage();

				draw_LEDPalettOnCleaning();

			}

			bMutuallyExclusiveActivated = false;

		break;

		case LCD_CLEANING_MODE_EXIT:
			
			stDataSet.stSetup.CleaningMode = CLEANING_MODE_OFF;
			
			draw_LEDPalettOnBrewStandby();

			set_SelectedCursor(false);
			set_TextLCDDashboard(get_TextLCDDashboard());//DashboardA
			set_ChangingTextLCD(true);

			_nSetupMenuState = LCD_DASHBOARD_ENTRY;

		break;

		case LCD_LONGPUSH_STATE:
		// default:

			switch (get_TextLCDView()) {
				case DashboardView:
					display_EntryMessage(false);
				break;
				case MenuboardView:
					display_EntryMessage(true);
				break;
				case MenupageView:
					display_MenupageExitMessage(true);
				break;
			}

			_nSetupMenuState = LCD_LONGPUSH_STATE;

		break;

	}


	config_UserInterfaceActivation(bMutuallyExclusiveActivated ? ROTARY_ENCODER_MUTEX_ACTIVATED_MASK : ALL_USER_INTERFACE_ACTIVATED);


	return bMutuallyExclusiveActivated;
}


// System Exception Handler (alarm message check)
int app_SystemExceptions(void)
{
#if AUTOMATION_SENSOR_ATTACHED
	static stError_t _stErrorOccurred = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
	static unsigned char _nPreviousMotorStandbyMode = 0;
#else
	static stError_t _stErrorOccurred = {false, false, false, false, false, false, false, false, false, false, false, false, false, false};
	static bool _bPreviousMotorStandbyMode = false;
#endif

	static bool _bSuspendHeating = false;

	static unsigned int _nPastErrorCode = DEF_ERROR_NONE;
	static unsigned int _nPastNoticeCode = DEF_NOTICE_NONE;
	unsigned int nPresentErrorCode = DEF_ERROR_NONE;
	unsigned int nPresentNoticeCode = DEF_NOTICE_NONE;

	unsigned char nNumOfErrorOccurred = 0;
	unsigned char nNumOfNoticeOccurred = 0;

	// System Error Message

	// Level : System Error - 0x0001
	if (stError.bIsSystemOverloaded) {

		if (!_stErrorOccurred.bIsSystemOverloaded) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] ECU Thermal Overload.") << endl;
			// #endif

			_stErrorOccurred.bIsSystemOverloaded = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_1;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_1;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsSystemOverloaded) {

			_stErrorOccurred.bIsSystemOverloaded = false;

		}

	}

	// Level : System Error - 0x0002
	if (stError.bFailToStandbyBrewPressure) {

		if (!_stErrorOccurred.bFailToStandbyBrewPressure) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Failed to standby Brew Pressure.") << endl;
			// #endif

			_stErrorOccurred.bFailToStandbyBrewPressure = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_2;
			save_SystemErrorCode(stDataSet.stMaintenance);


#if AUTOMATION_SENSOR_ATTACHED
			_nPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;
#else
			_bPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_PREP ? true : false;
#endif

			stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_OFF;
			save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

		}

		nPresentErrorCode = ERROR_MSG_2;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bFailToStandbyBrewPressure) {

			_stErrorOccurred.bFailToStandbyBrewPressure = false;

#if AUTOMATION_SENSOR_ATTACHED
			if (_nPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = _nPreviousMotorStandbyMode;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#else
			if (_bPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_PREP;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);
				
			}
#endif

		}

	}

	// Level : System Error - 0x0003
	if (stError.bIsBrewPressureOver) {

		if (!_stErrorOccurred.bIsBrewPressureOver) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Brew Boiler Over Pressure.") << endl;
			// #endif

			_stErrorOccurred.bIsBrewPressureOver = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_3;
			save_SystemErrorCode(stDataSet.stMaintenance);


#if AUTOMATION_SENSOR_ATTACHED
			_nPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;
#else
			_bPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_PREP ? true : false;
#endif

			stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_OFF;
			save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

		}

		nPresentErrorCode = ERROR_MSG_3;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsBrewPressureOver) {

			_stErrorOccurred.bIsBrewPressureOver = false;

#if AUTOMATION_SENSOR_ATTACHED
			if (_nPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = _nPreviousMotorStandbyMode;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#else
			if (_bPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_PREP;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#endif

		}

	}

	// Level : System Error - 0x0004
	if (stError.bIsBrewPressureUnder) {

		if (!_stErrorOccurred.bIsBrewPressureUnder) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Brew Boiler Under Pressure.") << endl;
			// #endif

			_stErrorOccurred.bIsBrewPressureUnder = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_4;
			save_SystemErrorCode(stDataSet.stMaintenance);


#if AUTOMATION_SENSOR_ATTACHED
			_nPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;
#else
			_bPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_PREP ? true : false;
#endif

			stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_OFF;
			save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

		}

		nPresentErrorCode = ERROR_MSG_4;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsBrewPressureUnder) {

			_stErrorOccurred.bIsBrewPressureUnder = false;

#if AUTOMATION_SENSOR_ATTACHED
			if (_nPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = _nPreviousMotorStandbyMode;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#else
			if (_bPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_PREP;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#endif

		}

	}

	// Level : System Error - 0x0005 (deprecated feature)
#if PUMP_PRESSURE_SENSOR_ATTACHED
	if (stError.bIsLowInletPressure) {

		if (!_stErrorOccurred.bIsLowInletPressure) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Low Inlet Pressure.") << endl;
			// #endif

			_stErrorOccurred.bIsLowInletPressure = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_5;
			save_SystemErrorCode(stDataSet.stMaintenance);


#if AUTOMATION_SENSOR_ATTACHED
			_nPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;
#else
			_bPreviousMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_PREP ? true : false;
#endif

			stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_OFF;
			save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

		}

		nPresentErrorCode = ERROR_MSG_5;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsLowInletPressure) {

			_stErrorOccurred.bIsLowInletPressure = false;

#if AUTOMATION_SENSOR_ATTACHED
			if (_nPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = _nPreviousMotorStandbyMode;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#else
			if (_bPreviousMotorStandbyMode) {

				stDataSet.stSetup.MotorStandbyMode = MOTOR_STANDBY_PREP;
				save_MotorStandbyMode(stDataSet.stSetup.MotorStandbyMode);

			}
#endif

		}

	}
#endif

	// Level : System Error - 0x0006
	if (stError.bIsGroupheadOverheated) {

		if (!_stErrorOccurred.bIsGroupheadOverheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Grouphead Overheated.") << endl;
			// #endif

			_stErrorOccurred.bIsGroupheadOverheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_6;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_6;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsGroupheadOverheated) {

			_stErrorOccurred.bIsGroupheadOverheated = false;

		}

	}

	// Level : System Error - 0x0007
	if (stError.bIsBrewBoilerOverheated) {

		if (!_stErrorOccurred.bIsBrewBoilerOverheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Brew Boiler Overheated.") << endl;
			// #endif

			_stErrorOccurred.bIsBrewBoilerOverheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_7;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_7;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsBrewBoilerOverheated) {

			_stErrorOccurred.bIsBrewBoilerOverheated = false;

		}

	}

	// Level : System Error - 0x0008
	if (stError.bIsPreheatBoilerOverheated) {

		if (!_stErrorOccurred.bIsPreheatBoilerOverheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Preheat Boiler Overheated.") << endl;
			// #endif

			_stErrorOccurred.bIsPreheatBoilerOverheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_8;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_8;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsPreheatBoilerOverheated) {

			_stErrorOccurred.bIsPreheatBoilerOverheated = false;

		}

	}

	// Level : System Error = 0x0009
	if (stError.bIsGroupheadUnderheated) {

		if (!_stErrorOccurred.bIsGroupheadUnderheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Grouphead Low Heat.") << endl;
			// #endif

			_stErrorOccurred.bIsGroupheadUnderheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_9;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_9;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsGroupheadUnderheated) {

			_stErrorOccurred.bIsGroupheadUnderheated = false;

		}
		
	}

	// Level : System Error = 0x000A
	if (stError.bIsBrewBoilerUnderheated) {

		if (!_stErrorOccurred.bIsBrewBoilerUnderheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Brew Boiler Low Heat.") << endl;
			// #endif

			_stErrorOccurred.bIsBrewBoilerUnderheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_10;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_10;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsBrewBoilerUnderheated) {

			_stErrorOccurred.bIsBrewBoilerUnderheated = false;

		}
		
	}

	// Level : System Error = 0x000B
	if (stError.bIsPreheatBoilerUnderheated) {

		if (!_stErrorOccurred.bIsPreheatBoilerUnderheated) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] Preheat Boiler Low Heat.") << endl;
			// #endif

			_stErrorOccurred.bIsPreheatBoilerUnderheated = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_11;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = ERROR_MSG_11;

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsPreheatBoilerUnderheated) {

			_stErrorOccurred.bIsPreheatBoilerUnderheated = false;

		}

	}

	// Level : System Error = 0x000C
	if (stError.bIsTemperatureSensorError) {

		if (!_stErrorOccurred.bIsTemperatureSensorError) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[SYSTEM.ERR] TM4 Temperature Sensor Error.") << endl;
			// #endif

			_stErrorOccurred.bIsTemperatureSensorError = true;

			stDataSet.stMaintenance.ErrorCode = ERROR_MSG_12;
			save_SystemErrorCode(stDataSet.stMaintenance);

		}

		nPresentErrorCode = verify_TM4TemperatureSensorAlarm(ERROR_MSG_12);

		nNumOfErrorOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsTemperatureSensorError) {

			_stErrorOccurred.bIsTemperatureSensorError = false;

		}

	}

	// System Notice Message

	// Level : System Notice - 0x1001
	if (stError.bIsFlowmeterFailed) {

		if (!_stErrorOccurred.bIsFlowmeterFailed) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] Flowmeter Failed.") << endl;
			// #endif

			_stErrorOccurred.bIsFlowmeterFailed = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_1;

		}

		nPresentNoticeCode = NOTICE_MSG_1;

		nNumOfNoticeOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsFlowmeterFailed) {

			_stErrorOccurred.bIsFlowmeterFailed = false;

		}

	}

	// Level : System Notice - 0x1002
	if (stError.bIsMotorAlarmReceived) {

		if (!_stErrorOccurred.bIsMotorAlarmReceived) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] Motor Alarm Received.") << endl;
			// #endif

			_stErrorOccurred.bIsMotorAlarmReceived = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_2;

		}

		nPresentNoticeCode = NOTICE_MSG_2;

		nNumOfNoticeOccurred++;

	}
	else {

		if (stError.bIsMotorAlarmReceived) {

			_stErrorOccurred.bIsMotorAlarmReceived = false;

		}

	}

	// Level : System Notice - 0x1003
	if (stError.bIsMaintenanceRequired) {

		if (!_stErrorOccurred.bIsMaintenanceRequired) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] System Maintenance Required.") << endl;
			// #endif

			_stErrorOccurred.bIsMaintenanceRequired = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_3;

		}

		nPresentNoticeCode = NOTICE_MSG_3;

		nNumOfNoticeOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsMaintenanceRequired) {

			_stErrorOccurred.bIsMaintenanceRequired = false;

		}

	}

	// Level : System Notice - 0x1004
	if (stError.bIsFlowrateExceeded) {

		if (!_stErrorOccurred.bIsFlowrateExceeded) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] Flow Rate Exceeded.") << endl;
			// #endif

			_stErrorOccurred.bIsFlowrateExceeded = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_4;

		}

		nPresentNoticeCode = NOTICE_MSG_4;

		nNumOfNoticeOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsFlowrateExceeded) {

			_stErrorOccurred.bIsFlowrateExceeded = false;

		}

	}

	// Level : System Notice - 0x1005
#if AUTOMATION_SENSOR_ATTACHED
	if (stError.bIsOpticalFiberUnstable) {

		if (!_stErrorOccurred.bIsOpticalFiberUnstable) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] Optical Fiber Amp Unstable.") << endl;
			// #endif

			_stErrorOccurred.bIsOpticalFiberUnstable = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_5;


			// hOpticalFiber.powerOnReset();
			disable_AutomationMode();
			init_AutomationMode();
			enable_AutomationMode(false);

		}

		nPresentNoticeCode = NOTICE_MSG_5;

		nNumOfNoticeOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsOpticalFiberUnstable) {

			_stErrorOccurred.bIsOpticalFiberUnstable = false;

		}

	}
#endif

	// Level : System Notice - 0x1006
#ifdef _USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_
	if (stError.bIsEEPROMWorn) {

		if (!_stErrorOccurred.bIsEEPROMWorn) {

			// #if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[NOTICE] EEPROM Worn out.") << endl;
			// #endif

			_stErrorOccurred.bIsEEPROMWorn = true;

			stDataSet.stMaintenance.NoticeCode = NOTICE_MSG_6;

		}

		nPresentNoticeCode = NOTICE_MSG_6;

		nNumOfNoticeOccurred++;

	}
	else {

		if (_stErrorOccurred.bIsEEPROMWorn) {

			_stErrorOccurred.bIsEEPROMWorn = false;

		}

	}
#endif

	// System Exception Handler

	// routine to suspend system
	if (nNumOfErrorOccurred) {

		if (nPresentErrorCode != DEF_ERROR_NONE) {

			_nPastErrorCode = stDataSet.stMaintenance.RecentErrorCode;

			stDataSet.stMaintenance.RecentErrorCode = nPresentErrorCode;

		}

		if (hEspressoMachine.getBrewState() != CEspresso::BREW_STOP) {

			hEspressoMachine.stopBrewing();

			if (hLamp.isTurnon()) {

				hLamp.turnoffEMG();

			}

			increase_BrewingCount();
			accumulate_BrewWaterQuantity();

			queue_BrewObservations(false);

		}

		if (stDataSet.stSetup.BrewSetupMode == BREW_SETUP_MODE_ON) {

			stDataSet.stSetup.BrewSetupMode = BREW_SETUP_MODE_OFF;

			if (stDataSet.ProgramNum != MANUAL_SET_ONE) {

				hEspressoMachine.setBrewMode(CEspresso::BREW_MODE);

			}

		}

		if (_nBrewState == CLEANING_MACHINE_START_STATE) {

			stDataSet.stSetup.CleaningMode = CLEANING_MODE_OFF;

		}

		if (_nBrewState == POWER_SAVING_STATE || _nPowerSavingState != PWS_NOT_WORKING) {

			_nPowerSavingState = PWS_NOT_WORKING;

			if (hPowerSavingTimer.isRunning()) {

				hPowerSavingTimer.stop();

			}

		}

		if (hMotorPump.isRunning()) {

			hMotorPump.stopEMG();

		}

		if (hBrewValve.isOpening()) {

			stop_Flowmeter();

			hBrewValve.close();

		}

		if (!_bSuspendHeating) {

			if (suspend_BoilerHeater()) {

				_bSuspendHeating = true;

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[NOTICE] Boiler Heater Suspended.") << endl;
				// #endif

			}
			else {

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[ERROR] Failed to suspend Boiler Heater") << endl;
				// #endif

			}

		}

		if (_nBrewState != SYSTEM_ERROR_STATE) {

			_nBrewState = SYSTEM_ERROR_STATE;

			draw_LEDPalettOnException();

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			print_SystemErrorList(true);
			#endif

		}

	}
	else {

		_nPastErrorCode = DEF_ERROR_NONE;

		stDataSet.stMaintenance.RecentErrorCode = DEF_ERROR_NONE;

		if (stDataSet.stMaintenance.ErrorCode != DEF_ERROR_NONE) {

			stDataSet.stMaintenance.ErrorCode = DEF_ERROR_NONE;

		}

		if (_bSuspendHeating) {

			if (resume_BoilerHeater()) {

				_bSuspendHeating = false;

				setup_BoilerTemperatureControllerSV();

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[NOTICE] Boiler Heater Resumed.") << endl;
				// #endif

			}
			else {

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[ERROR] Failed to resume Boiler Heater") << endl;
				// #endif

			}

		}

		if (_nBrewState == SYSTEM_ERROR_STATE) {

			_nBrewState = BREW_STANDBY_STATE;

			if (hLamp.getMode() == CLamp::TURNON_ALWAYS) {

				hLamp.turnon();

			}

			draw_LEDPalettOnBrewStandby();

		}

	}

	if (nNumOfNoticeOccurred) {

		if (nPresentNoticeCode != DEF_NOTICE_NONE) {

			_nPastNoticeCode = stDataSet.stMaintenance.RecentNoticeCode;

			stDataSet.stMaintenance.RecentNoticeCode = nPresentNoticeCode;

		}

	}
	else {

		_nPastNoticeCode = DEF_NOTICE_NONE;

		stDataSet.stMaintenance.RecentNoticeCode = DEF_NOTICE_NONE;

		if (stDataSet.stMaintenance.NoticeCode != DEF_NOTICE_NONE) {

			stDataSet.stMaintenance.NoticeCode = DEF_NOTICE_NONE;

		}

	}

	if (!nNumOfErrorOccurred && !nNumOfNoticeOccurred) {

		stop_AlarmMessageDisplayTimer();

	}

	// Display System Error/Notice Message

	if (get_TextLCDView() == DashboardView) {

		if (stDataSet.stMaintenance.RecentErrorCode != DEF_ERROR_NONE) {

			if (isAlarmMessageDisplay(SYSTEM_ERROR_MESSAGE_DISPLAY_PERIOD) || _nPastErrorCode != nPresentErrorCode) {

				display_ErrorMessage(stDataSet.stMaintenance);

				stop_AlarmMessageDisplayTimer();

			}

		}
		else {

			if (stDataSet.stMaintenance.RecentNoticeCode != DEF_NOTICE_NONE) {

				if (isAlarmMessageDisplay(SYSTEM_NOTICE_MESSAGE_DISPLAY_PERIOD) || _nPastNoticeCode != nPresentNoticeCode) {

					display_NoticeMessage(stDataSet.stMaintenance);

					stop_AlarmMessageDisplayTimer();

				}

			}

		}

	}
	// else {

	// 	stop_AlarmMessageDisplayTimer();

	// }
}


// Brew Pressure Stabilization
void wait_stabilizeBrewPressure(void)
{
	if (stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_OFF) {

		float fMotorPower = stDataSet.stSetup.MotorPower >= MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE ? stDataSet.stSetup.MotorPower : MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE;

		unsigned long ulStabilizationTime = millis();

		while (millis() - ulStabilizationTime < TIME_TO_STABILIZE_BREWPRES_ON_STOP_CLEANING) {

			if ((stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue()) >= UPPER_BREW_PRESSURE_UNDER_STANDBY1) {

				break;

			}

			hMotorPump.start(fMotorPower);

			// display_TextLCDOnUserInterface();

		}

		if (hMotorPump.isRunning()) {

			while (millis() - ulStabilizationTime < MIN_TIME_TO_STANDBY_BREW_PRESSURE);

			hMotorPump.stop();

		}

	}
}

void standby_BrewBoilerPressure(void)
{
	if ((stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_PREP || stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_SHOT) && stDataSet.stSetup.MaintenanceMode == MAINTENANCE_MODE_OFF && stDataSet.stSetup.MotorRun == MOTOR_RUN_OFF) {

		if (_bJustGetStoppedBrewing) {

			_bJustGetStoppedFlushing = true;

			if (hBrewPressureStandbyDelayTimer.isRunning()) {

				hBrewPressureStandbyDelayTimer.reset();

			}

		}

		if (_bJustGetStoppedFlushing) {

			if (!hBrewPressureStandbyDelayTimer.isRunning()) {

				hBrewPressureStandbyDelayTimer.reset();
				hBrewPressureStandbyDelayTimer.start();

				return;

			}
			else {

				if (hBrewPressureStandbyDelayTimer.elapsed() < DELAY_TIME_TO_STANDBY_BREW_PRESSURE) {

					return;

				}
				else {

					hBrewPressureStandbyDelayTimer.stop();

					_bJustGetStoppedFlushing = false;

				}

			}

		}

		static unsigned char _nStandbyBrewPressureFailedCount = 0;

		static unsigned long _lStartTime = 0UL;

		float fLowerBrewPressureOnStandby = _bJustGetStoppedBrewing ? LOWER_BREW_PRESSURE_UNDER_STANDBY2 : LOWER_BREW_PRESSURE_UNDER_STANDBY1;

		float fMotorPower = stDataSet.stSetup.MotorPower >= MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE ? stDataSet.stSetup.MotorPower : MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE;

		_bJustGetStoppedBrewing = false;

		if (!hMotorPump.isRunning()) {

			if (stDataSet.stAcquisition.BrewTemp >= MIN_TEMP_TO_ENABLE_STANDBY_BREW_PRESSURE || stDataSet.stAcquisition.PreheatTemp >= MIN_TEMP_TO_ENABLE_STANDBY_BREW_PRESSURE) {

				if (stDataSet.stAcquisition.BrewPress < fLowerBrewPressureOnStandby) {

					if (hBrewValve.isOpening()) {

						hBrewValve.close();

					}

					hMotorPump.start(fMotorPower);

					_lStartTime = millis();

				}

			}

		}
		else {

			if ((stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue()) >= UPPER_BREW_PRESSURE_UNDER_STANDBY1) {

				if (millis() - _lStartTime >= MIN_TIME_TO_STANDBY_BREW_PRESSURE) {

					hMotorPump.stop();

				}

				stError.bFailToStandbyBrewPressure = false;

			}
			else {

				if (millis() - _lStartTime >= MAX_TIME_TO_STANDBY_BREW_PRESSURE) {

					hMotorPump.stop();

					if (++_nStandbyBrewPressureFailedCount > MAX_COUNT_FAILED_TO_STANDBY_BREW_PRESSURE) {

						_nStandbyBrewPressureFailedCount = 0;

						stError.bFailToStandbyBrewPressure = true;

					}

				}

			}

		}

	}
	else {

		if (stDataSet.stSetup.MotorRun == MOTOR_RUN_OFF) {

			if (hMotorPump.isRunning()) {

				hMotorPump.stop();

			}

		}

		if (hBrewPressureStandbyDelayTimer.isRunning()) {

			hBrewPressureStandbyDelayTimer.stop();

			_bJustGetStoppedFlushing = false;

		}

	}
}

#if AUTOMATION_SENSOR_ATTACHED
void stabilize_BrewBoilerPressure(void)
{
	if (stDataSet.stSetup.MotorStandbyMode == MOTOR_STANDBY_SHOT) {

		if ((stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue()) < LOWER_BREW_PRESSURE_UNDER_STANDBY3) {

			float fMotorPower = stDataSet.stSetup.MotorPower >= MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE ? stDataSet.stSetup.MotorPower : MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE;

			unsigned long ulStabilizationTime = millis();

			while (millis() - ulStabilizationTime < TIME_TO_STABILIZE_BREWPRES_TO_AUTO_BREWING) {

				hLamp.turnon();

				hMotorPump.start(fMotorPower);

				if ((stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue()) >= UPPER_BREW_PRESSURE_UNDER_STANDBY2) {

					#if defined(_USE_APPLICATION_DEBUG_MSG_)
					cout << F("[NOTICE] Brew Boiler Pressure : reach to 9.0 [bar]") << endl;
					#endif

					invalidate_TextLCD(stDataSet, stCurTimeDate);

					break;

				}

				invalidate_TextLCD(stDataSet, stCurTimeDate);

			}

			if (hMotorPump.isRunning()) {

				while (millis() - ulStabilizationTime < MIN_TIME_TO_STANDBY_BREW_PRESSURE);

				// hMotorPump.stop();

			}

		}

	}
}
#endif


// Brewing Water Qauntity and Brew Count
void increase_BrewingCount(void)
{
	if (hEspressoMachine.isFlushingDetected() == false) {

		unsigned long ulBrewCount = stDataSet.stMaintenance.TotBrewCount;//load_TotalBrewCount();

		if (ulBrewCount < MAXIMUM_BREW_COUNT_DIGIT) {

			ulBrewCount += 1;

		}
		else {

			ulBrewCount = 0UL;

			if ((stDataSet.stMaintenance.Overflowed & 0xF0) == 0x00) {

				stDataSet.stMaintenance.Overflowed |= 0x10;

				save_OverflowedUsage(stDataSet.stMaintenance.Overflowed);

			}

		}

		stDataSet.stMaintenance.TotBrewCount = ulBrewCount;

		save_TotalBrewCount(ulBrewCount);

	}
}

void accumulate_BrewWaterQuantity(void)
{
	static float _fAccWaterFlowInMillis = 0.0f;

	_fAccWaterFlowInMillis += get_WaterFlowQuantity(CFlowmeter::UNIT_MILLIS);

	#if defined(_USE_APPLICATION_DEBUG_MSG_)
	// cout << F("Water Flow [ml] : ") << _fAccWaterFlowInMillis << endl;
	#endif

	if (_fAccWaterFlowInMillis >= 1000.0f) {

		_fAccWaterFlowInMillis -= 1000.0f;

		unsigned long ulWaterFlowInLiters = stDataSet.stMaintenance.TotFlowQuantity;//load_TatalFlowQuantity();

		if (ulWaterFlowInLiters < MAXIMUM_BREW_WATER_QUANTITY_DIGIT) {

			ulWaterFlowInLiters += 1UL;

		}
		else {

			ulWaterFlowInLiters = 0UL;

			if ((stDataSet.stMaintenance.Overflowed & 0x0F) == 0x00) {

				stDataSet.stMaintenance.Overflowed |= 0x01;

				save_OverflowedUsage(stDataSet.stMaintenance.Overflowed);

			}

		}

		stDataSet.stMaintenance.TotFlowQuantity = ulWaterFlowInLiters;

		save_TotalFlowQuantity(ulWaterFlowInLiters);

	}
}

void reset_BrewingCount(void)
{
	stDataSet.stMaintenance.TotBrewCount = 0UL;
	stDataSet.stMaintenance.Overflowed &= 0x0F;

	save_TotalBrewCount(0ul);
	save_OverflowedUsage(stDataSet.stMaintenance.Overflowed);
}

void reset_BrewWaterQuantity(void)
{
	stDataSet.stMaintenance.TotFlowQuantity = 0UL;
	stDataSet.stMaintenance.Overflowed &= 0xF0;

	save_TotalFlowQuantity(0UL);
	save_OverflowedUsage(stDataSet.stMaintenance.Overflowed);
}


// Brew Observation(Profile) Confidence
void observe_BrewProfileConfidence(const stAcqDataSet_t& stAcqQueue)
{
	static bool _bIsFirst = true;

	float fShotTimeConfidence = 1.0F;
	float fCurFlowrateConfidence = 1.0F;
	float fAvgFlowrateConfidence = 1.0F;
	float fAvgBrewPressConfidence = 1.0F;

	if (_bIsFirst) {

		_bIsFirst = false;

	}
	else {

		fShotTimeConfidence = (stDataSet.stAcquisition.ShotTime/1000.0F) / (stAcqQueue.ShotTime/1000.0F + 0.001F);
		fCurFlowrateConfidence = (stDataSet.stAcquisition.FlowRate / stAcqQueue.FlowRate + 0.001F);
		fAvgFlowrateConfidence = (stDataSet.stAcquisition.AvgFlowRate / stAcqQueue.AvgFlowRate + 0.001F);
		fAvgBrewPressConfidence = (stDataSet.stAcquisition.AvgBrewPress / stAcqQueue.AvgBrewPress + 0.001F);

		if (fShotTimeConfidence > 1.0F) {

			fShotTimeConfidence = 1.0F / fShotTimeConfidence;

		}

		if (fCurFlowrateConfidence > 1.0F) {

			fCurFlowrateConfidence = 1.0F / fCurFlowrateConfidence;

		}

		if (fAvgFlowrateConfidence > 1.0F) {

			fAvgFlowrateConfidence = 1.0F / fAvgFlowrateConfidence;

		}

		if (fAvgBrewPressConfidence > 1.0F) {

			fAvgBrewPressConfidence = 1.0F / fAvgBrewPressConfidence;

		}			

		if (fShotTimeConfidence <= BREW_PROFILE_NOTICE_CONDITION) {

			stDataSet.stIndicator.nShotTimeLevel = fShotTimeConfidence <= BREW_PROFILE_ALARM_CONDITION ? BREW_PROFILE_ALARM_LEVEL : BREW_PROFILE_NOTICE_LEVEL;

		}
		else {

			stDataSet.stIndicator.nShotTimeLevel = BREW_PROFILE_NORMAL_LEVEL;

		}

		if (fCurFlowrateConfidence <= BREW_PROFILE_NOTICE_CONDITION) {

			stDataSet.stIndicator.nCurFlowrateLevel = fCurFlowrateConfidence <= BREW_PROFILE_ALARM_CONDITION ? BREW_PROFILE_ALARM_LEVEL : BREW_PROFILE_NOTICE_LEVEL;

		}
		else {

			stDataSet.stIndicator.nCurFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;

		}

		if (fAvgFlowrateConfidence <= BREW_PROFILE_NOTICE_CONDITION) {

			stDataSet.stIndicator.nAvgFlowrateLevel = fAvgFlowrateConfidence <= BREW_PROFILE_ALARM_CONDITION ? BREW_PROFILE_ALARM_LEVEL : BREW_PROFILE_NOTICE_LEVEL;

		}
		else {

			stDataSet.stIndicator.nAvgFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;
		}

		if (fAvgBrewPressConfidence <= BREW_PROFILE_NOTICE_CONDITION) {

			stDataSet.stIndicator.nAvgBrewPressLevel = fAvgBrewPressConfidence <= BREW_PROFILE_ALARM_CONDITION ? BREW_PROFILE_ALARM_LEVEL : BREW_PROFILE_NOTICE_LEVEL;

		}
		else {

			stDataSet.stIndicator.nAvgBrewPressLevel = BREW_PROFILE_NORMAL_LEVEL;

		}

	}

	#if defined(_USE_APPLICATION_DEBUG_MSG_)
	cout << F("[BREW PROFILE CONFIDENCE]") << endl;
	cout << F("Shot Time : ") << fShotTimeConfidence << endl;
	cout << F("Flow Rate : ") << fCurFlowrateConfidence << endl;
	cout << F("Flow Rate (avg) : ") << fAvgFlowrateConfidence << endl;
	cout << F("Brew Pres (avg) : ") << fAvgBrewPressConfidence << endl;
	cout << F("\n") << endl;
	#endif
}

void queue_BrewObservations(const bool bStartBrewing)
{
	static stAcqDataSet_t _stAcqQueue;
	static stSetupDataSet_t _stSetupQueue;

	static stIndicator_t _stIndicatorQueue;

	unsigned char nProgramNum = stDataSet.ProgramNum;//hEspressoMachine.getProgramNum();

	if (bStartBrewing == true) {

		// Queue
		// _stAcqQueue = stDataSet.stAcquisition;
		_stAcqQueue.ShotTime = stDataSet.stAcquisition.ShotTime;

		_stAcqQueue.FlowCount = stDataSet.stAcquisition.FlowCount;
		_stAcqQueue.FlowQuantity = stDataSet.stAcquisition.FlowQuantity;
		_stAcqQueue.FlowRate = stDataSet.stAcquisition.FlowRate;

		_stAcqQueue.AvgFlowRate = stDataSet.stAcquisition.AvgFlowRate;
		_stAcqQueue.AvgBrewPress = stDataSet.stAcquisition.AvgBrewPress;
#if PUMP_PRESSURE_SENSOR_ATTACHED
		_stAcqQueue.AvgPumpPress = stDataSet.stAcquisition.AvgPumpPress;
#endif

		// _stSetupQueue = stDataSet.stSetup;
		_stSetupQueue.InjectionFlow[nProgramNum] = stDataSet.stSetup.InjectionFlow[nProgramNum];
		_stSetupQueue.ExtractionFlow[nProgramNum] = stDataSet.stSetup.ExtractionFlow[nProgramNum];
		_stSetupQueue.InjectionTime[nProgramNum] = stDataSet.stSetup.InjectionTime[nProgramNum];
		_stSetupQueue.ExtractionTime[nProgramNum] = stDataSet.stSetup.ExtractionTime[nProgramNum];

		_stIndicatorQueue = stDataSet.stIndicator;
		stDataSet.stIndicator.nShotTimeLevel = BREW_PROFILE_NORMAL_LEVEL;
		stDataSet.stIndicator.nCurFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;
		stDataSet.stIndicator.nAvgFlowrateLevel = BREW_PROFILE_NORMAL_LEVEL;
		stDataSet.stIndicator.nAvgBrewPressLevel = BREW_PROFILE_NORMAL_LEVEL;

	}
	else {

		if (hEspressoMachine.isFlushingDetected() == true) {

			// Dequeue
			// stDataSet.stAcquisition = _stAcqQueue;
			stDataSet.stAcquisition.ShotTime = _stAcqQueue.ShotTime;

			stDataSet.stAcquisition.FlowCount = _stAcqQueue.FlowCount;
			stDataSet.stAcquisition.FlowQuantity = _stAcqQueue.FlowQuantity;
			stDataSet.stAcquisition.FlowRate = _stAcqQueue.FlowRate;

			stDataSet.stAcquisition.AvgFlowRate = _stAcqQueue.AvgFlowRate;
			stDataSet.stAcquisition.AvgBrewPress = _stAcqQueue.AvgBrewPress;
#if PUMP_PRESSURE_SENSOR_ATTACHED
			stDataSet.stAcquisition.AvgPumpPress = _stAcqQueue.AvgPumpPress;
#endif

			// stDataSet.stSetup = _stSetupQueue;
			stDataSet.stSetup.InjectionFlow[nProgramNum] = _stSetupQueue.InjectionFlow[nProgramNum];
			stDataSet.stSetup.ExtractionFlow[nProgramNum] = _stSetupQueue.ExtractionFlow[nProgramNum];
			stDataSet.stSetup.InjectionTime[nProgramNum] = _stSetupQueue.InjectionTime[nProgramNum];
			stDataSet.stSetup.ExtractionTime[nProgramNum] = _stSetupQueue.ExtractionTime[nProgramNum];

			stDataSet.stIndicator = _stIndicatorQueue;

		}
		else {

			observe_BrewProfileConfidence(_stAcqQueue);

		}

	}
}


// Cleaing Mode
bool procedure_CleaningMachine(const int nSignal)
{
	static int _nIteration = 0;
	static int _nCleaningStage = CLEANING_STAGE_READY;
	static int _nCleaningSubstage = CLEANING_SUBSTAGE_READY;

	static unsigned long _ulStageStartTime = 0UL;
	static unsigned long _ulStageEndTime = 0UL;

	if (nSignal == CLEANING_MACHINE_START_STATE) {

		if (_nCleaningStage == CLEANING_STAGE_BEGIN) {

#if AUTOMATION_SENSOR_ATTACHED
			disable_AutomationMode();
#endif

			if (hMotorPump.isRunning()) {

				hMotorPump.stop();

			}

			if (hBrewValve.isOpening()) {

				hBrewValve.close();

			}

			_nIteration = 0;
			_ulStageStartTime = 0UL;
			_ulStageEndTime = 0UL;

			_nCleaningStage = CLEANING_STAGE_FIRST;
			_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

			_nCleaningProgress = 0;

			start_Flowmeter();

			display_CleaningModeMessage();

			// draw_LEDPalettOnCleaning();

			hPixelLED.makeFixedProgressPalett(NUM_OF_PIXELS, NUM_OF_PIXELS);

		}

		if (_nCleaningStage == CLEANING_STAGE_FIRST) {

			if (millis() - _ulStageStartTime >= _ulStageEndTime) {

				switch (_nCleaningSubstage) {

					case CLEANING_SUBSTAGE_READY:

						if (hMotorPump.isRunning()) {

							_nCleaningProgress++;//0 1 2

							hMotorPump.stop();

						}

					case CLEANING_SUBSTAGE_FIRST:

						if (!hBrewValve.isOpening()) {

							hBrewValve.open();

						}

						_nIteration++;
						_ulStageStartTime = millis();
						_ulStageEndTime = 5000UL;

						_nCleaningSubstage = CLEANING_SUBSTAGE_SECOND;

					break;

					case CLEANING_SUBSTAGE_SECOND:

						hBrewValve.close();

						_ulStageStartTime = millis();
						_ulStageEndTime = 5000UL;

						_nCleaningSubstage = CLEANING_SUBSTAGE_THIRD;

					break;

					case CLEANING_SUBSTAGE_THIRD:

						hMotorPump.start(MOTOR_POWER_FOR_CLEANING_MODE);

						hBrewValve.open();

						_ulStageStartTime = millis();
						_ulStageEndTime = 5000UL;

						if (_nIteration < CLEANING_SUBSTAGE_FiRST_ITERATION) {

							_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

						}
						else {

							_nCleaningSubstage = CLEANING_SUBSTAGE_FORTH;

						}

					break;

					case CLEANING_SUBSTAGE_FORTH:

						hMotorPump.stop();

						hBrewValve.close();

						_nIteration = 0;
						_ulStageStartTime = millis();
						_ulStageEndTime = 30000UL;//10000UL;

						_nCleaningStage = CLEANING_STAGE_SECOND;
						_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

					break;

				}

			}

		}

		if (_nCleaningStage == CLEANING_STAGE_SECOND) {

			if (millis() - _ulStageStartTime >= _ulStageEndTime) {

				switch (_nCleaningSubstage) {

					case CLEANING_SUBSTAGE_READY:

						_nCleaningProgress++;//3

					case CLEANING_SUBSTAGE_FIRST:

						hBrewValve.open();

						_ulStageStartTime = millis();
						_ulStageEndTime = 10000UL;//5000UL;

						_nCleaningSubstage = CLEANING_SUBSTAGE_SECOND;

					break;

					case CLEANING_SUBSTAGE_SECOND:

						hBrewValve.close();

						_nIteration = 0;
						_ulStageStartTime = millis();
						_ulStageEndTime = 60000UL;

						_nCleaningStage = CLEANING_STAGE_THIRD;
						_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

					break;

				}

			}

		}

		if (_nCleaningStage == CLEANING_STAGE_THIRD) {

			if (millis() - _ulStageStartTime >= _ulStageEndTime) {

				switch (_nCleaningSubstage) {

					case CLEANING_SUBSTAGE_READY:

						_nCleaningProgress++;//4 5 6

					case CLEANING_SUBSTAGE_FIRST:

						hMotorPump.start(MOTOR_POWER_FOR_CLEANING_MODE);

						hBrewValve.open();

						_nIteration++;
						_ulStageStartTime = millis();
						_ulStageEndTime = 5000UL;

						_nCleaningSubstage = CLEANING_SUBSTAGE_SECOND;

					break;

					case CLEANING_SUBSTAGE_SECOND:

						hMotorPump.stop();

						hBrewValve.close();

						_ulStageStartTime = millis();

						if (_nIteration < CLEANING_SUBSTAGE_THIRD_ITERATION) {

							_ulStageEndTime = 10000UL;

						}
						else {

							_nIteration = 0;
							_ulStageEndTime = 30000UL;

							_nCleaningStage = CLEANING_STAGE_FORTH;

						}

						_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

					break;

				}

			}

		}

		if (_nCleaningStage == CLEANING_STAGE_FORTH) {

			if (millis() - _ulStageStartTime >= _ulStageEndTime) {

				switch (_nCleaningSubstage) {

					case CLEANING_SUBSTAGE_READY:

						_nCleaningProgress++;//7 8 9 10 11 12 13 14 15

						hMotorPump.start(MOTOR_POWER_FOR_CLEANING_MODE);

					case CLEANING_SUBSTAGE_FIRST:

						hBrewValve.open();

						_nIteration++;
						_ulStageStartTime = millis();
						_ulStageEndTime = 5000UL;

						_nCleaningSubstage = CLEANING_SUBSTAGE_SECOND;

					break;

					case CLEANING_SUBSTAGE_SECOND:

						hBrewValve.close();

						_ulStageStartTime = millis();
						_ulStageEndTime = 3000UL;

						if (_nIteration >= CLEANING_SUBSTAGE_FORTH_ITERATION) {

							_nIteration = 0;

							_nCleaningStage = CLEANING_STAGE_END;

						}

						_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

					break;

				}
			}

		}		

		if (_nCleaningStage >= CLEANING_STAGE_END) {

			if (millis() - _ulStageStartTime >= _ulStageEndTime) {

				hMotorPump.stop();

				stop_Flowmeter();

				_nCleaningStage = CLEANING_STAGE_READY;
				_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

				_nCleaningProgress = 0;

				stDataSet.stSetup.CleaningMode = CLEANING_MODE_OFF;

				return false;

			}

		}

	}
	else {

		stop_Flowmeter();

		if (hBrewValve.isOpening()) {

			hBrewValve.close();

		}

		wait_stabilizeBrewPressure();

		_nIteration = 0;
		_ulStageStartTime = 0UL;
		_ulStageEndTime = 0UL;

		_nCleaningStage = CLEANING_STAGE_READY;
		_nCleaningSubstage = CLEANING_SUBSTAGE_READY;

		_nCleaningProgress = 0;

		stDataSet.stSetup.CleaningMode = CLEANING_MODE_OFF;

		return false;

	}

	return true;
}


// Optical Fiber Amplifier (BF4RP)
#if AUTOMATION_SENSOR_ATTACHED

void setup_AutomationMode(const bool bSuspendByForce)
{
	// in this function, do not use Peripheral Interfaces (because of noInterrupts()/interrupts() function)

	if (!bSuspendByForce && stDataSet.stSetup.AutomationMode == AUTOMATION_MODE_ON) {

		hOpticalFiber.turnon();

		hOpticalFiber.enable();

		// wait to stabilize optical fiber amp
		unsigned long ulStabilizeTime = millis();

		while(millis() - ulStabilizeTime < OPTICAL_FIBER_AMP_STABILIZE_DURATION);

	}
	else {

		hOpticalFiber.turnoff();

		hOpticalFiber.disable();

	}

	// init. automation mode (equal to init_AutomationMode function)
	hOpticalFiber.init();

	_bIsAutoFlushing = false;
	_bIsAutoFlushed = true;

	_bPauseAutoFlushing = false;

	_bIsAutoBrewing = false;
	_bIsAutoBrewed = false;

	_bIsAutoBrewingEnabled = stDataSet.stSetup.AutoBrewWaitTime == 0.0F ? false : true;

	_bPauseAutoBrewing = hOpticalFiber.getSignal() == hOpticalFiber.Dark();
}

void suspend_AutomationMode(void)
{
	setup_AutomationMode(AUTOMATION_MODE_SUSPEND);
}

void resume_AutomationMode(void)
{
	load_AutomationParam(stDataSet.stSetup);

	setup_AutomationMode(AUTOMATION_MODE_RESUME);
}

void init_AutomationMode(void)
{
	init_OpticalFiberAmp();

	_bIsAutoFlushing = false;
	_bIsAutoFlushed = true;

	_bPauseAutoFlushing = false;

	_bIsAutoBrewing = false;
	_bIsAutoBrewed = false;

	_bIsAutoBrewingEnabled = stDataSet.stSetup.AutoBrewWaitTime == 0.0F ? false : true;

	_bPauseAutoBrewing = get_OpticalFiberSignal() == hOpticalFiber.Dark();
}

void enable_AutomationMode(const bool bSkipAutoFlushingByForce)
{
	if (hOpticalFiber.isRunning()) {

		enable_OpticalFiberAmp();

		// skip auto-flushing if porta filter removed while auto-brewing
		_bPauseAutoFlushing = (read_OpticalFiberSignal() == hOpticalFiber.Dark()) || bSkipAutoFlushingByForce;

	}
}

void disable_AutomationMode(void)
{
	if (hOpticalFiber.isRunning()) {

		disable_OpticalFiberAmp();

	}
}

void start_AutoFlushing(void)
{
	float fMotorPower = stDataSet.stSetup.MotorPower >= MIN_MOTOR_POWER_TO_AUTO_FLUSHING ? stDataSet.stSetup.MotorPower : MIN_MOTOR_POWER_TO_AUTO_FLUSHING;

	if (hMotorPump.isRunning()) {

		hMotorPump.stop();

	}

	if (!hLamp.isTurnon()) {

		hLamp.turnon();

	}

	start_Flowmeter();

	hBrewValve.open();

	hMotorPump.start(fMotorPower);

	disable_OpticalFiberAmp();
}

void stop_AutoFlushing(void)
{
	stop_Flowmeter();

	hBrewValve.close();

	if (hMotorPump.isRunning()) {

		hMotorPump.stop();

	}

	if (hLamp.isTurnon()) {

		hLamp.turnoff();

	}

	stDataSet.stAcquisition.FlowCount = get_FlowmeterCNT();

	_bJustGetStoppedFlushing = true;

	enable_OpticalFiberAmp();
}

bool start_AutoFlushingMode(void)
{
	if (hOpticalFiber.isRunning() == false || hOpticalFiber.isEnabled() == false) {

		return false;

	}

	if (get_OpticalFiberSignal() == hOpticalFiber.Dark()) {	// if PORTA_FILTER_REMOVED (hOpticalFiber.Dark())

		if (_bIsAutoFlushing == false && _bIsAutoFlushed == false) {

			if (_bPauseAutoFlushing == true) {

				_bPauseAutoFlushing = false;

				_bIsAutoFlushing = false;
				_bIsAutoFlushed = true;

			}
			else {

				if (_nSetupMenuState <= LCD_MENUBOARD_LIST_CHANGE) {

					return true;

				}
				else {

					_bIsAutoFlushed = true;

				}

			}

		}

	}
	else {													// if PORTA_FILTER_INSERTED (hOpticalFiber.Light())

		// double check if porta filter inserted
		if (read_OpticalFiberSignal() == hOpticalFiber.Light()) {

			_bIsAutoFlushing = false;
			_bIsAutoFlushed = false;

		}

	}

	return false;
}

void stop_AutoFlushingMode(void)
{
	if (_bIsAutoFlushing == true) {

		_bIsAutoFlushing = false;
		_bIsAutoFlushed = true;

		stop_AutoFlushing();

		accumulate_BrewWaterQuantity();

		observe_FlowmeterFailure();	// [Notice] Don't change the position

	}

	_bPauseAutoBrewing = get_OpticalFiberSignal() == hOpticalFiber.Dark();
}

bool procedure_AutoFlushingMode(void)
{
	static unsigned long _ulFlushingStartTime = 0UL;

	if (_bIsAutoFlushing == false) {

		_bIsAutoFlushing = true;
		_bIsAutoFlushed = false;

		start_AutoFlushing();

		_ulFlushingStartTime = millis();

	}
	else {

		unsigned long ulAutoFlushingTime = stDataSet.stSetup.AutoFlushingTime * 1000UL;

		if (millis() - _ulFlushingStartTime >= ulAutoFlushingTime) {

			_bIsAutoFlushing = false;
			_bIsAutoFlushed = true;

			stop_AutoFlushing();

			accumulate_BrewWaterQuantity();

			observe_FlowmeterFailure();	// [Notice] Don't change the position

			// skip auto-brewing if porta filter inserted while auto-flushing
			_bPauseAutoBrewing = (read_OpticalFiberSignal() == hOpticalFiber.Light()) && _bIsAutoBrewingEnabled;

		}

	}

	return _bIsAutoFlushing;
}

bool start_AutoBrewingMode(void)
{
	if (hOpticalFiber.isRunning() == false || hOpticalFiber.isEnabled() == false || _bIsAutoBrewingEnabled == false) {

		_bPauseAutoBrewing = false;

		_bIsAutoBrewing = false;
		_bIsAutoBrewed = true;

		return false;

	}

	if (get_OpticalFiberSignal() == hOpticalFiber.Light()) {	// if PORTA_FILTER_INSERTED (hOpticalFiber.Light())

		if (_bIsAutoBrewing == false && _bIsAutoBrewed == false) {

			static bool _bIsBrewingReady = false;
			static unsigned long _ulBrewingReadyTime = 0UL;

			if (_bIsBrewingReady == false) {

				_bIsBrewingReady = true;

				_ulBrewingReadyTime = millis();

			}
			else {

				unsigned long ulAutoBrewWaitTime = ((stDataSet.stSetup.AutoBrewWaitTime * 1000UL) - OPTICAL_FIBER_LIGHT_HIGH_DURATION);

				if (millis() - _ulBrewingReadyTime >= ulAutoBrewWaitTime) {

					_bIsBrewingReady = false;

					if (hEspressoMachine.getBrewMode() != CEspresso::BREW_MODE || _bPauseAutoBrewing == true) {

						_bPauseAutoBrewing = false;

						_bIsAutoBrewing = false;
						_bIsAutoBrewed = true;

					}
					else {

						if (_nSetupMenuState <= LCD_MENUBOARD_LIST_CHANGE) {

							_bIsAutoBrewing = true;

						}
						else {

							_bIsAutoBrewed = true;

						}

					}

					return _bIsAutoBrewing;

				}

			}

		}

	}
	else {														// if PORTA_FILTER_REMOVED (hOpticalFiber.Dark())

		// double check if porta filter removed
		if (read_OpticalFiberSignal() == hOpticalFiber.Dark()) {

			_bIsAutoBrewing = false;
			_bIsAutoBrewed = false;

		}

	}

	return false;
}

void stop_AutoBrewingMode(void)
{
	if (_bIsAutoBrewingEnabled == true) {

		if (_bIsAutoBrewing == true) {

			_bIsAutoBrewing = false;
			_bIsAutoBrewed = true;

		}
		else {

			_bPauseAutoBrewing = get_OpticalFiberSignal() == hOpticalFiber.Dark();

		}

	}
}

#else

// Flushing Detection Mode
void verify_FlushingDetectionMode(const float fMotorPower)
{
	if (fMotorPower < MIN_MOTOR_POWER_FOR_FLUSHING_DETECTION) {

		stDataSet.stSetup.FlushingDetectionMode = FLUSHING_DETECTION_MODE_OFF;

	}
	else {

		if (stDataSet.stSetup.FlushingDetectionMode == FLUSHING_DETECTION_MODE_OFF) {

			stDataSet.stSetup.FlushingDetectionMode = load_FlushingDetectionMode();

		}

	}
}

void verify_FlushingDetectionModeWithMotorPump(void)
{
	if (stDataSet.stSetup.InjectionPower < MIN_MOTOR_POWER_FOR_FLUSHING_DETECTION || stDataSet.stSetup.MotorPower < MIN_MOTOR_POWER_FOR_FLUSHING_DETECTION) {

		stDataSet.stSetup.FlushingDetectionMode = FLUSHING_DETECTION_MODE_OFF;

	}
}

#endif


// Maintenance Mode
bool enable_SystemMaintenanceMode(void)
{
	if (hBrewValve.isOpening()) {

		hBrewValve.close();

	}

	if (hMotorPump.isRunning()) {

		hMotorPump.stop();

	}

	if (suspend_BoilerHeater()) {

		start_Flowmeter();

		hBrewValve.open();

		return true;

	}
	else {

		// #if defined(_USE_APPLICATION_DEBUG_MSG_)
		cout << F("[ERROR] Failed to suspend Boiler Heater") << endl;
		// #endif

	}

	return false;
}

bool disable_SystemMaintenanceMode(void)
{
	if (hBrewValve.isOpening()) {

		stop_Flowmeter();

		hBrewValve.close();

		accumulate_BrewWaterQuantity();

	}

	if (!resume_BoilerHeater()) {

		// #if defined(_USE_APPLICATION_DEBUG_MSG_)
		cout << F("[ERROR] Failed to resume Boiler Heater") << endl;
		// #endif

		return false;

	}

	return true;
}


// Pressure Sensor Adjustment
void setup_PressureSensorAdjustment(void)
{
	set_PressureSensorZeroShiftWithValue(hPressureBrewBoiler, stDataSet.stSetup.ZeroShift[BREW_BOILER_PRESSURE_SENSOR_CH]);
	set_PressureSensorSpanFactorWithValue(hPressureBrewBoiler, stDataSet.stSetup.SpanFactor[BREW_BOILER_PRESSURE_SENSOR_CH]);

	#if PUMP_PRESSURE_SENSOR_ATTACHED
	set_PressureSensorZeroShiftWithValue(hPressureFeed, stDataSet.stSetup.ZeroShift[PUMP_PRESSURE_SENSOR_CH]);
	set_PressureSensorSpanFactorWithValue(hPressureFeed, stDataSet.stSetup.SpanFactor[PUMP_PRESSURE_SENSOR_CH]);
	#endif

	#if AUX_CURRENT_SENSOR_ATTACHED
	set_PressureSensorZeroShiftWithValue(hPressureAUX, stDataSet.stSetup.ZeroShift[AUX_PRESSURE_SENSOR_CH]);
	set_PressureSensorSpanFactorWithValue(hPressureAUX, stDataSet.stSetup.SpanFactor[AUX_PRESSURE_SENSOR_CH]);
	#endif
}

void config_PressureSensorAdjustmentToFactoryDefault(void)
{
	unsigned long ulConfigStartTime = millis();

	if (stDataSet.stSetup.SetSensorChannel == BREW_BOILER_PRESSURE_SENSOR_CH) {

		stDataSet.stSetup.ZeroShift[BREW_BOILER_PRESSURE_SENSOR_CH] = float(DEF_ZERO_SHIFT);
		stDataSet.stSetup.SpanFactor[BREW_BOILER_PRESSURE_SENSOR_CH] = float(DEF_SPAN_FACTOR);

		#if BREW_PRESSURE_SENSOR_ATTACHED
		reset_PressureSensorAdjustment(hPressureBrewBoiler);
		#endif

	}
	#if AUX_CURRENT_SENSOR_ATTACHED
	else if (stDataSet.stSetup.SetSensorChannel == AUX_PRESSURE_SENSOR_CH) {

		stDataSet.stSetup.ZeroShift[AUX_PRESSURE_SENSOR_CH] = float(DEF_ZERO_SHIFT);
		stDataSet.stSetup.SpanFactor[AUX_PRESSURE_SENSOR_CH] = float(DEF_SPAN_FACTOR);

		reset_PressureSensorAdjustment(hPressureAUX);

	}
	#endif
	else {	// == PUMP_PRESSURE_SENSOR_CH

		stDataSet.stSetup.ZeroShift[PUMP_PRESSURE_SENSOR_CH] = float(DEF_ZERO_SHIFT);
		stDataSet.stSetup.SpanFactor[PUMP_PRESSURE_SENSOR_CH] = float(DEF_SPAN_FACTOR);

		#if PUMP_PRESSURE_SENSOR_ATTACHED
		reset_PressureSensorAdjustment(hPressureFeed);
		#endif

	}

	disable_RotaryEncoder();

	while (millis() - ulConfigStartTime < TIME_TO_HOLD_PRESSURE_SENSOR_DEFAULT_RESET) {

		update_PressureSensorAnalogDigit(false);
		invalidate_TextLCD(stDataSet, stCurTimeDate);

		delay(1UL);
	}

	enable_RotaryEncoder();
}

void config_PressureSensorZeroShift(void)
{
	if (hMotorPump.isRunning()) {

		hMotorPump.stop();
		
	}

	if (!hBrewValve.isOpening()) {

		hBrewValve.open();

	}

	enable_RotaryEncoder();

	unsigned long ulConfigStartTime = millis();

	while (millis() - ulConfigStartTime < TIME_TO_PREPARE_PRESSURE_SENSOR_ZERO_SHIFT) {

		update_PressureSensorAnalogDigit(false);
		invalidate_TextLCD(stDataSet, stCurTimeDate);

		delay(1UL);
	}

	disable_RotaryEncoder();

	if (stDataSet.stSetup.SetSensorChannel == BREW_BOILER_PRESSURE_SENSOR_CH) {

		#if BREW_PRESSURE_SENSOR_ATTACHED
		set_PressureSensorZeroShift(hPressureBrewBoiler);

		stDataSet.stSetup.ZeroShift[BREW_BOILER_PRESSURE_SENSOR_CH] = hPressureBrewBoiler.getZeroShiftValue();
		#endif

	}
	#if AUX_CURRENT_SENSOR_ATTACHED
	else if (stDataSet.stSetup.SetSensorChannel == AUX_PRESSURE_SENSOR_CH) {

		set_PressureSensorZeroShift(hPressureAUX);

		stDataSet.stSetup.ZeroShift[AUX_PRESSURE_SENSOR_CH] = hPressureAUX.getZeroShiftValue();

	}
	#endif
	else {	// == PUMP_PRESSURE_SENSOR_CH

		#if PUMP_PRESSURE_SENSOR_ATTACHED
		set_PressureSensorZeroShift(hPressureFeed);

		stDataSet.stSetup.ZeroShift[PUMP_PRESSURE_SENSOR_CH] = hPressureFeed.getZeroShiftValue();
		#endif

	}

	hBrewValve.close();
}

void config_PressureSensorSpanFactor(void)
{
	if (hBrewValve.isOpening()) {

		hBrewValve.close();

	}

	float fMotorPower = stDataSet.stSetup.MotorPower;

	if (fMotorPower < MIN_MOTOR_POWER_TO_ADJUST_PRESSURE_SENSOR) {

		fMotorPower = MIN_MOTOR_POWER_TO_ADJUST_PRESSURE_SENSOR;

	}

	hMotorPump.start(fMotorPower);

	enable_RotaryEncoder();

	unsigned long ulConfigStartTime = millis();

	while (millis() - ulConfigStartTime < TIME_TO_PREPARE_PRESSURE_SENSOR_SPAN_TIME) {

		update_PressureSensorAnalogDigit(false);
		invalidate_TextLCD(stDataSet, stCurTimeDate);

		delay(1UL);
		
	}

	disable_RotaryEncoder();

	if (stDataSet.stSetup.SetSensorChannel == BREW_BOILER_PRESSURE_SENSOR_CH) {

		#if BREW_PRESSURE_SENSOR_ATTACHED
		set_PressureSensorSpanFactor(hPressureBrewBoiler);

		stDataSet.stSetup.SpanFactor[BREW_BOILER_PRESSURE_SENSOR_CH] = hPressureBrewBoiler.getSpanFactorValue();
		#endif

	}
	#if AUX_CURRENT_SENSOR_ATTACHED
	if (stDataSet.stSetup.SetSensorChannel == AUX_PRESSURE_SENSOR_CH) {

		set_PressureSensorSpanFactor(hPressureAUX);

		stDataSet.stSetup.SpanFactor[AUX_PRESSURE_SENSOR_CH] = hPressureAUX.getSpanFactorValue();

	}
	#endif
	else {	// == PUMP_PRESSURE_SENSOR_CH

		#if PUMP_PRESSURE_SENSOR_ATTACHED
		set_PressureSensorSpanFactor(hPressureFeed);

		stDataSet.stSetup.SpanFactor[PUMP_PRESSURE_SENSOR_CH] = hPressureFeed.getSpanFactorValue();
		#endif

	}

	if (hMotorPump.isRunning()) {

		hMotorPump.stop();

	}
}

// Authorization Utilities
void add_AuthorizationCode(const unsigned char nNumberPosition)
{
	_nPasswordCode += stDataSet.stSetup.PasswordInput[nNumberPosition] * (1000 / pow(10, nNumberPosition));
}

bool request_Authorization(void)
{
	bool bIsAuthorized;

	if (_nPasswordCode == AUTHORIZATION_CODE) {

		bIsAuthorized = true;

		cout << F("Authorization Success.") << endl;

	}
	else {

		bIsAuthorized = false;

		cout << F("Authorization Failed.") << endl;

	}

	_nPasswordCode = 0;

	stDataSet.stSetup.PasswordInput[0] = 0;
	stDataSet.stSetup.PasswordInput[1] = 0;
	stDataSet.stSetup.PasswordInput[2] = 0;
	stDataSet.stSetup.PasswordInput[3] = 0;

	set_AuthorizedString(bIsAuthorized);

	return bIsAuthorized;
}

void release_Authorization(void)
{
	_nPasswordCode = 0;

	stDataSet.stSetup.PasswordInput[0] = 0;
	stDataSet.stSetup.PasswordInput[1] = 0;
	stDataSet.stSetup.PasswordInput[2] = 0;
	stDataSet.stSetup.PasswordInput[3] = 0;

	set_AuthorizedString(false);
}

bool IsAuthorized(void)
{
#if defined(_SYSTEM_ENGINEERING_MODE_)
	return true;
#else
	return get_AuthorizedString();
#endif
}

bool verify_AuthorizationToMenupageEntry(const eMenuPage nMenupage)
{
#if defined(_SYSTEM_ENGINEERING_MODE_)
	set_AuthorizedString(true);

	return true;
#else
	if (nMenupage >= Menupage13 && nMenupage <= Menupage15) {

		return IsAuthorized();

	}
	else {

		if (nMenupage == Menupage20) {

			return (!IsAuthorized());

		}

	}

	return true;
#endif
}

// Setup Menu Timer Utilities
void start_SetupMenuTimer(void)
{
	hSetupMenuTimer.reset();
	hSetupMenuTimer.start();
}

void stop_SetupMenuTimer(void)
{
	if (hSetupMenuTimer.isRunning()) {

		hSetupMenuTimer.stop();

	}
}

void restart_SetupMenuTimer(void)
{
	if (hSetupMenuTimer.isRunning()) {

		hSetupMenuTimer.reset();
		hSetupMenuTimer.start();

	}
}

bool isSetupMenuTimedOver(void)
{
	if (hSetupMenuTimer.isRunning()) {

		if (hSetupMenuTimer.elapsed() >= TIME_TO_SAVE_AND_EXIT_MENU) {

			hSetupMenuTimer.stop();

			return true;

		}

	}

	return false;
}

void start_AlarmMessageDisplayTimer(void)
{
	hAlarmMessageDisplayTimer.reset();
	hAlarmMessageDisplayTimer.start();
}

void stop_AlarmMessageDisplayTimer(void)
{
	if (hAlarmMessageDisplayTimer.isRunning()) {

		hAlarmMessageDisplayTimer.stop();

	}
}

bool isAlarmMessageDisplay(const unsigned long ulDisplayTimePeriod)
{
	bool bDisplayMessage = false;

	if (!hAlarmMessageDisplayTimer.isRunning()) {

		bDisplayMessage = true;

	}
	else {

		if (hAlarmMessageDisplayTimer.elapsed() >= ulDisplayTimePeriod) {

			bDisplayMessage = true;

		}

	}

	return bDisplayMessage;
}


// Setup Menu Procedures
int procedure_SetupMenuboard(const int nCurrentMenuboard)
{
	int nNextMenuboard = nCurrentMenuboard;

	int nLine = get_SelectCursorLine();

	nLine += hRotaryEncoder.getDirection();

	if (nCurrentMenuboard == Menuboard1 && nLine < LCD_SETUPMENU_PROGRAM1_LINE) {

		nNextMenuboard = Menuboard5;
		nLine = LCD_SETUPMENU_PASSWORD_LINE;

	}
	else if (nCurrentMenuboard == Menuboard5 && nLine > LCD_SETUPMENU_PASSWORD_LINE) {

		nNextMenuboard = Menuboard1;
		nLine = LCD_SETUPMENU_PROGRAM1_LINE;

	}
	else {

		if (nLine < 0) {

			nLine = LCD_MAX_LINE-1;
			nNextMenuboard = nCurrentMenuboard - 1;

		}
		else if (nLine > LCD_MAX_LINE-1) {

			nLine = 0;
			nNextMenuboard = nCurrentMenuboard + 1;

		}

	}

	if (nNextMenuboard != nCurrentMenuboard) {

		set_TextLCDMenuboard(static_cast<eMenuBoard>(nNextMenuboard));

	}

	set_SelectCursorLine(nLine);

	return LCD_MENUBOARD_ENTRY;
}

int procedure_ProgramOneSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < PROGRAM_MODE_VALUE_LINE) {
				nLine = EXTRACTION_VALUE_LINE;
			}
			else if (nLine > EXTRACTION_VALUE_LINE) {
				nLine = PROGRAM_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == PROGRAM_MODE_VALUE_LINE) {

				unsigned char nProgramMode = stDataSet.stSetup.ProgramMode[PROGRAM_SET_ONE];

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.ProgramMode[PROGRAM_SET_ONE] = nProgramMode == PROGRAM_MODE_FLOW ? PROGRAM_MODE_TIME : PROGRAM_MODE_FLOW;

			}
			else if (nLine == INJECTION_VALUE_LINE) {

				if (stDataSet.stSetup.ProgramMode[PROGRAM_SET_ONE] == PROGRAM_MODE_FLOW) {

					int nInjectionFlow = stDataSet.stSetup.InjectionFlow[PROGRAM_SET_ONE];

					nInjectionFlow += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nInjectionFlow < MIN_INJECTION_FLOWCNT) {
						nInjectionFlow = MAX_INJECTION_FLOWCNT;
					}
					else if (nInjectionFlow > MAX_INJECTION_FLOWCNT) {
						nInjectionFlow = MIN_INJECTION_FLOWCNT;
					}

					stDataSet.stSetup.InjectionFlow[PROGRAM_SET_ONE] = nInjectionFlow;

				}
				else {

					float fInjectionTime = stDataSet.stSetup.InjectionTime[PROGRAM_SET_ONE];

					fInjectionTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);

					if (fInjectionTime < MIN_INJECTION_TIME) {
						fInjectionTime = (MAX_INJECTION_TIME/1000UL);
					}
					else if (fInjectionTime > (MAX_INJECTION_TIME/1000UL)) {
						fInjectionTime = MIN_INJECTION_TIME;
					}

					stDataSet.stSetup.InjectionTime[PROGRAM_SET_ONE] = fInjectionTime;

				}

			}
			else { // nLine == EXTRACTION_VALUE_LINE

				if (stDataSet.stSetup.ProgramMode[PROGRAM_SET_ONE] == PROGRAM_MODE_FLOW) {

					int nExtractionFlow = stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_ONE];

					nExtractionFlow += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nExtractionFlow < MIN_EXTRACTION_FLOWCNT) {
						nExtractionFlow = MAX_EXTRACTION_FLOWCNT;
					}
					else if (nExtractionFlow > MAX_INJECTION_FLOWCNT) {
						nExtractionFlow = MIN_EXTRACTION_FLOWCNT;
					}

					stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_ONE] = nExtractionFlow;

				}
				else {

					float fExtractionTime = stDataSet.stSetup.ExtractionTime[PROGRAM_SET_ONE];

					fExtractionTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);

					if (fExtractionTime < (MIN_EXTRACTION_TIME/1000UL)) {
						fExtractionTime = (MAX_EXTRACTION_TIME/1000UL);
					}
					else if (fExtractionTime > (MAX_EXTRACTION_TIME/1000UL)) {
						fExtractionTime = (MIN_EXTRACTION_TIME/1000UL);
					}

					stDataSet.stSetup.ExtractionTime[PROGRAM_SET_ONE] = fExtractionTime;

				}

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				unsigned int nInjectionFlow = stDataSet.stSetup.InjectionFlow[PROGRAM_SET_ONE];
				unsigned int nExtractionFlow = stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_ONE];

				float fInjectionTime = stDataSet.stSetup.InjectionTime[PROGRAM_SET_ONE];
				float fExtractionTime = stDataSet.stSetup.ExtractionTime[PROGRAM_SET_ONE];

				if (hEspressoMachine.invalidateCondition(nInjectionFlow, nExtractionFlow, fInjectionTime, fExtractionTime)) {

					save_ProgramParam(PROGRAM_SET_ONE, stDataSet.stSetup);

				}

			}
			else {

				load_ProgramParam(PROGRAM_SET_ONE, stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard1);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_PROGRAM1_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_ProgramTwoSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < PROGRAM_MODE_VALUE_LINE) {
				nLine = EXTRACTION_VALUE_LINE;
			}
			else if (nLine > EXTRACTION_VALUE_LINE) {
				nLine = PROGRAM_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == PROGRAM_MODE_VALUE_LINE) {

				unsigned char nProgramMode = stDataSet.stSetup.ProgramMode[PROGRAM_SET_TWO];

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.ProgramMode[PROGRAM_SET_TWO] = nProgramMode == PROGRAM_MODE_FLOW ? PROGRAM_MODE_TIME : PROGRAM_MODE_FLOW;

			}
			else if (nLine == INJECTION_VALUE_LINE) {

				if (stDataSet.stSetup.ProgramMode[PROGRAM_SET_TWO] == PROGRAM_MODE_FLOW) {

					int nInjectionFlow = stDataSet.stSetup.InjectionFlow[PROGRAM_SET_TWO];

					nInjectionFlow += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nInjectionFlow < MIN_INJECTION_FLOWCNT) {
						nInjectionFlow = MAX_INJECTION_FLOWCNT;
					}
					else if (nInjectionFlow > MAX_INJECTION_FLOWCNT) {
						nInjectionFlow = MIN_INJECTION_FLOWCNT;
					}

					stDataSet.stSetup.InjectionFlow[PROGRAM_SET_TWO] = nInjectionFlow;

				}
				else {

					float fInjectionTime = stDataSet.stSetup.InjectionTime[PROGRAM_SET_TWO];

					fInjectionTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);

					if (fInjectionTime < MIN_INJECTION_TIME) {
						fInjectionTime = (MAX_INJECTION_TIME/1000UL);
					}
					else if (fInjectionTime > (MAX_INJECTION_TIME/1000UL)) {
						fInjectionTime = MIN_INJECTION_TIME;
					}

					stDataSet.stSetup.InjectionTime[PROGRAM_SET_TWO] = fInjectionTime;

				}

			}
			else { // nLine == EXTRACTION_VALUE_LINE

				if (stDataSet.stSetup.ProgramMode[PROGRAM_SET_TWO] == PROGRAM_MODE_FLOW) {

					int nExtractionFlow = stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_TWO];

					nExtractionFlow += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nExtractionFlow < MIN_EXTRACTION_FLOWCNT) {
						nExtractionFlow = MAX_INJECTION_FLOWCNT;
					}
					else if (nExtractionFlow > MAX_INJECTION_FLOWCNT) {
						nExtractionFlow = MIN_EXTRACTION_FLOWCNT;
					}

					stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_TWO] = nExtractionFlow;

				}
				else {

					float fExtractionime = stDataSet.stSetup.ExtractionTime[PROGRAM_SET_TWO];

					fExtractionime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);

					if (fExtractionime < (MIN_EXTRACTION_TIME/1000UL)) {
						fExtractionime = (MAX_EXTRACTION_TIME/1000UL);
					}
					else if (fExtractionime > (MAX_EXTRACTION_TIME/1000UL)) {
						fExtractionime = (MIN_EXTRACTION_TIME/1000UL);
					}

					stDataSet.stSetup.ExtractionTime[PROGRAM_SET_TWO] = fExtractionime;

				}

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				unsigned int nInjectionFlow = stDataSet.stSetup.InjectionFlow[PROGRAM_SET_TWO];
				unsigned int nExtractionFlow = stDataSet.stSetup.ExtractionFlow[PROGRAM_SET_TWO];

				float fInjectionTime = stDataSet.stSetup.InjectionTime[PROGRAM_SET_TWO];
				float fExtractionTime = stDataSet.stSetup.ExtractionTime[PROGRAM_SET_TWO];

				if (hEspressoMachine.invalidateCondition(nInjectionFlow, nExtractionFlow, fInjectionTime, fExtractionTime)) {

					save_ProgramParam(PROGRAM_SET_TWO, stDataSet.stSetup);

				}

			}
			else {

				load_ProgramParam(PROGRAM_SET_TWO, stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard1);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_PROGRAM2_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_ManualOneSetup(const int nSignal)
{
	int nState = nSignal;
	unsigned char nProgramSetNum;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(INJECTION_POWER_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nProgramSetNum = stDataSet.stSetup.ProgramSetNum;

			// hRotaryEncoder.getDirection();

			stDataSet.stSetup.ProgramSetNum = nProgramSetNum == PROGRAM_SET_ONE ? PROGRAM_SET_TWO : PROGRAM_SET_ONE;

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_ManualProgramSetNumber(stDataSet.stSetup.ProgramSetNum);

			}
			else {

				load_ManualProgramSetNumber(stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard1);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_MANUAL_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_InjectionSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < INJECTION_MODE_VALUE_LINE) {
				nLine = INJECTION_POWER_VALUE_LINE;
			}
			else if (nLine > INJECTION_POWER_VALUE_LINE) {
				nLine = INJECTION_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == INJECTION_MODE_VALUE_LINE) {

				unsigned char nInjectionMode = stDataSet.stSetup.InjectionMode;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.InjectionMode = nInjectionMode == INJECTION_MODE_OFF ? INJECTION_MODE_ON : INJECTION_MODE_OFF;

			}
			else { // nLine == INJECTION_POWER_VALUE_LINE

				float fInjectionPower = stDataSet.stSetup.InjectionPower;

				fInjectionPower += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1) * 0.01f);

				if (fInjectionPower < 0.00F) {
					fInjectionPower = 1.00F;
				}
				else if (fInjectionPower > 1.00F) {
					fInjectionPower = 0.00F;
				}

				stDataSet.stSetup.InjectionPower = fInjectionPower;

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_InjectionSetupParam(stDataSet.stSetup);

			}
			else {

				load_InjectionSetupParam(stDataSet.stSetup);

			}

#if AUTOMATION_SENSOR_ATTACHED == false
			// verify_FlushingDetectionMode(stDataSet.stSetup.InjectionPower);
#endif

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard1);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_INJECTION_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_GroupheadTempSetup(const int nSignal)
{
	int nState = nSignal;
	float fSetPointTemperature;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(SET_TEMP_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			fSetPointTemperature = stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1];

<<<<<<< HEAD
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);
=======
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1) * 0.1f);
>>>>>>> feature/brew_profile_acquisition

			if (fSetPointTemperature < MIN_BOILER_TEMPERATURE) {
				fSetPointTemperature = MAX_BOILER_TEMPERATURE;
			}
			else if (fSetPointTemperature > MAX_BOILER_TEMPERATURE) {
				fSetPointTemperature = MIN_BOILER_TEMPERATURE;
			}

			stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1] = fSetPointTemperature;

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (_nBrewState != POWER_SAVING_STATE) {

					config_GroupheadHeaterSetPointTemperature(stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1]);

				}
				else {

					save_GroupheadSetValueTemperature(stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1]);

				}

			}
			else {

				stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1] = load_GroupheadSetValueTemperature();

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard2);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_GROUPHEAD_B_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_BrewBoilerTempSetup(const int nSignal)
{
	int nState = nSignal;
	float fSetPointTemperature;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(SET_TEMP_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			fSetPointTemperature = stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1];

<<<<<<< HEAD
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);
=======
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1) * 0.1f);
>>>>>>> feature/brew_profile_acquisition

			if (fSetPointTemperature < MIN_BOILER_TEMPERATURE) {
				fSetPointTemperature = MAX_BOILER_TEMPERATURE;
			}
			else if (fSetPointTemperature > MAX_BOILER_TEMPERATURE) {
				fSetPointTemperature = MIN_BOILER_TEMPERATURE;
			}

			stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1] = fSetPointTemperature;

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (_nBrewState != POWER_SAVING_STATE) {

					config_BrewBoilerSetPointTemperature(stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1]);

				}
				else {

					save_BrewBoilerSetValueTemperature(stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1]);

				}

			}
			else {

				stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1] = load_BrewBoilerSetValueTemperature();

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard2);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_BREW_B_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_PreheatBoilerTempSetup(const int nSignal)
{
	int nState = nSignal;
	float fSetPointTemperature;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(SET_TEMP_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			fSetPointTemperature = stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1];

<<<<<<< HEAD
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);
=======
			fSetPointTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1) * 0.1f);
>>>>>>> feature/brew_profile_acquisition

			if (fSetPointTemperature < MIN_BOILER_TEMPERATURE) {
				fSetPointTemperature = MAX_BOILER_TEMPERATURE;
			}
			else if (fSetPointTemperature > MAX_BOILER_TEMPERATURE) {
				fSetPointTemperature = MIN_BOILER_TEMPERATURE;
			}

			stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1] = fSetPointTemperature;

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (_nBrewState != POWER_SAVING_STATE) {

					config_PreheatBoilerSetPointTemperature(stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1]);

				}
				else {

					save_PreheatBoilerSetValueTemperature(stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1]);

				}

			}
			else {

				stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1] = load_PreheatBoilerSetValueTemperature();

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard2);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_PREHEAT_B_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_MotorPumpSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < MOTOR_POWER_VALUE_LINE) {
				nLine = MOTOR_RUN_VALUE_LINE;
			}
			else if (nLine > MOTOR_RUN_VALUE_LINE) {
				nLine = MOTOR_POWER_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == MOTOR_POWER_VALUE_LINE) {

				float fMotorPower = stDataSet.stSetup.MotorPower;

				fMotorPower += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1) * 0.01f);

				if (fMotorPower < 0.01F) {
					fMotorPower = 1.00F;
				}
				else if (fMotorPower > 1.00F) {
					fMotorPower = 0.01F;
				}

				stDataSet.stSetup.MotorPower = fMotorPower;

			}
			else if (nLine == STANDBY_MODE_VALUE_LINE) {

#if AUTOMATION_SENSOR_ATTACHED
				int nMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;

				nMotorStandbyMode += hRotaryEncoder.getDirection();

				if (nMotorStandbyMode < MOTOR_STANDBY_OFF) {
					nMotorStandbyMode = MOTOR_STANDBY_SHOT;
				}
				else if (nMotorStandbyMode > MOTOR_STANDBY_SHOT) {
					nMotorStandbyMode = MOTOR_STANDBY_OFF;
				}

				stDataSet.stSetup.MotorStandbyMode = nMotorStandbyMode;
#else
				unsigned char nMotorStandbyMode = stDataSet.stSetup.MotorStandbyMode;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.MotorStandbyMode = nMotorStandbyMode == MOTOR_STANDBY_OFF ? MOTOR_STANDBY_PREP : MOTOR_STANDBY_OFF;
#endif

			}
			else { // nLine == MOTOR_RUN_VALUE_LINE

				unsigned char nMotorRun = stDataSet.stSetup.MotorRun;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.MotorRun = nMotorRun == MOTOR_RUN_OFF ? MOTOR_RUN_ON : MOTOR_RUN_OFF;

				if (stDataSet.stSetup.MotorRun == MOTOR_RUN_ON) {

					hMotorPump.start(stDataSet.stSetup.MotorPower);

				}
				else {

					hMotorPump.stop();

				}

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (hMotorPump.isRunning()) {

				hMotorPump.stop();
				
			}

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_MotorPumpParam(stDataSet.stSetup);

			}
			else {

				load_MotorPumpParam(stDataSet.stSetup);

			}

			stDataSet.stSetup.MotorRun = MOTOR_RUN_OFF;

			#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			stDataSet.stSetup.MotorAccTimeEST = hMotorPump.convACCTimeWithRPM(stDataSet.stSetup.MotorAccTime, stDataSet.stSetup.MotorPower);
			#endif

#if AUTOMATION_SENSOR_ATTACHED == false
			// verify_FlushingDetectionMode(stDataSet.stSetup.MotorPower);
#endif

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard2);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_MOTORPUMP_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_LampSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < LAMP_MODE_VALUE_LINE) {
				nLine = BRIGHTNESS_VALUE_LINE;
			}
			else if (nLine > BRIGHTNESS_VALUE_LINE) {
				nLine = LAMP_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == LAMP_MODE_VALUE_LINE) {

				int nLampMode = stDataSet.stSetup.LampMode;

				nLampMode += hRotaryEncoder.getDirection();

				if (nLampMode < LAMP_TURNOFF_ALWAYS) {
					nLampMode = LAMP_TURNON_ALWAYS;
				}
				else if (nLampMode > LAMP_TURNON_ALWAYS) {
					nLampMode = LAMP_TURNOFF_ALWAYS;
				}

				stDataSet.stSetup.LampMode = nLampMode;

				hLamp.setMode(static_cast<CLamp::LampMode>(nLampMode));

			}
			else { // nLine == BRIGHTNESS_VALUE_LINE

				unsigned char nLampIntensity = stDataSet.stSetup.LampIntensity;

				// hRotaryEncoder.getDirection();

				nLampIntensity = nLampIntensity == LAMP_LOW_INTENSITY ? LAMP_HIGH_INTENSITY : LAMP_LOW_INTENSITY;

				stDataSet.stSetup.LampIntensity = nLampIntensity;

				hLamp.setIntensity(static_cast<CLamp::LampIntensity>(nLampIntensity));

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_GroupheadLampParam(stDataSet.stSetup);

			}
			else {

				load_GroupheadLampParam(stDataSet.stSetup);

			}

			hLamp.setMode(static_cast<CLamp::LampMode>(stDataSet.stSetup.LampMode));
			hLamp.setIntensity(static_cast<CLamp::LampIntensity>(stDataSet.stSetup.LampIntensity));

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard3);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_LAMP_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_LEDPixelModeSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	int nLedPixelMode, nProgramNum;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < PROGRAM1_INDICATOR_VALUE_LINE) {
				nLine = MANUAL1_INDICATOR_VALUE_LINE;
			}
			else if (nLine > MANUAL1_INDICATOR_VALUE_LINE) {
				nLine = PROGRAM1_INDICATOR_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			switch (nLine) {
				case PROGRAM1_INDICATOR_VALUE_LINE:
					nProgramNum = PROGRAM_SET_ONE;
				break;
				case PROGRAM2_INDICATOR_VALUE_LINE:
					nProgramNum = PROGRAM_SET_TWO;
				break;
				case MANUAL1_INDICATOR_VALUE_LINE:
					nProgramNum = MANUAL_SET_ONE;
				break;
			}

			nLedPixelMode = stDataSet.stSetup.LedPixelMode[nProgramNum];

			nLedPixelMode += hRotaryEncoder.getDirection();

			if (nProgramNum != MANUAL_SET_ONE) {
				#if PUMP_PRESSURE_SENSOR_ATTACHED
				if (nLedPixelMode < LEDPIXEL_PROGRESS_MODE) {
					nLedPixelMode = LEDPIXEL_PUMPPRESS_MODE;
				}
				else if (nLedPixelMode > LEDPIXEL_PUMPPRESS_MODE) {
					nLedPixelMode = LEDPIXEL_PROGRESS_MODE;
				}
				#else
				if (nLedPixelMode < LEDPIXEL_PROGRESS_MODE) {
					nLedPixelMode = LEDPIXEL_BREWPRESS_MODE;
				}
				else if (nLedPixelMode > LEDPIXEL_BREWPRESS_MODE) {
					nLedPixelMode = LEDPIXEL_PROGRESS_MODE;
				}
				#endif
			}
			else {
				#if PUMP_PRESSURE_SENSOR_ATTACHED
				if (nLedPixelMode < LEDPIXEL_TIMELAPSE_MODE) {
					nLedPixelMode = LEDPIXEL_PUMPPRESS_MODE;
				}
				else if (nLedPixelMode > LEDPIXEL_PUMPPRESS_MODE) {
					nLedPixelMode = LEDPIXEL_TIMELAPSE_MODE;
				}
				#else
				if (nLedPixelMode < LEDPIXEL_TIMELAPSE_MODE) {
					nLedPixelMode = LEDPIXEL_BREWPRESS_MODE;
				}
				else if (nLedPixelMode > LEDPIXEL_BREWPRESS_MODE) {
					nLedPixelMode = LEDPIXEL_TIMELAPSE_MODE;
				}
				#endif
			}

			stDataSet.stSetup.LedPixelMode[nProgramNum] = nLedPixelMode;

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_LEDPixelMode(stDataSet.stSetup);

			}
			else {

				load_LEDPixelMode(stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard3);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_LEDPIXEL_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_TimeDateSetup(const int nSignal)
{
	int nState = nSignal;
	int nPos, nLine;

	static int _nScratched = 0;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nPos = get_SelectCursorPos();
			nLine = get_SelectCursorLine();

			#if defined(_SETUP_WEEK_DAY_)
			if (nLine == WDAY_VALUE_LINE) {

				nLine += hRotaryEncoder.getDirection();

				if (nLine < WDAY_VALUE_LINE) {
					nLine = TIME_VALUE_LINE;
					nPos = TIME_SECOND_SELECTION_POS;
				}
				else if (nLine > WDAY_VALUE_LINE) {
					nLine = DATE_VALUE_LINE;
					nPos = DATE_YEAR_SELECTION_POS;
				}

			}
			else {

				if (nLine == DATE_VALUE_LINE) {

					nPos += (hRotaryEncoder.getDirection() * 5);

					if (nPos < DATE_YEAR_SELECTION_POS) {
						nLine = WDAY_VALUE_LINE;
						nPos = WEEKDAY_SELECTION_POS;
					}
					else if (nPos > DATE_DAY_SELECTION_POS) {
						nLine = TIME_VALUE_LINE;
						nPos = TIME_HOUR_SELECTION_POS;
					}

				}
				else { // nLine = TIME_VALUE_LINE

					nPos += (hRotaryEncoder.getDirection() * 5);

					if (nPos < TIME_HOUR_SELECTION_POS) {
						nLine = DATE_VALUE_LINE;
						nPos = DATE_DAY_SELECTION_POS;
					}
					else if (nPos > TIME_SECOND_SELECTION_POS) {
						nLine = WDAY_VALUE_LINE;
						nPos = WEEKDAY_SELECTION_POS;
					}

				}

			}
			#else
			if (nLine == DATE_VALUE_LINE) {

				nPos += (hRotaryEncoder.getDirection() * 5);

				if (nPos < DATE_YEAR_SELECTION_POS) {
					nLine = TIME_VALUE_LINE;
					nPos = TIME_SECOND_SELECTION_POS;
				}
				else if (nPos > DATE_DAY_SELECTION_POS) {
					nLine = TIME_VALUE_LINE;
					nPos = TIME_HOUR_SELECTION_POS;
				}

			}
			else { // nLine = TIME_VALUE_LINE

				nPos += (hRotaryEncoder.getDirection() * 5);

				if (nPos < TIME_HOUR_SELECTION_POS) {
					nLine = DATE_VALUE_LINE;
					nPos = DATE_DAY_SELECTION_POS;
				}
				else if (nPos > TIME_SECOND_SELECTION_POS) {
					nLine = DATE_VALUE_LINE;
					nPos = DATE_YEAR_SELECTION_POS;
				}

			}
			#endif //_SETUP_WEEK_DAY_

			set_SelectCursorLine(nLine);
			set_SelectCursorPos(nPos);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			_bPowerSavingModeValid = false;

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			if (_nScratched == 0) {

				_nScratched = 1;

				copy_TimeDate2SetupParams();
				
				select_SetupTimeDateContents(true);

			}

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == WDAY_VALUE_LINE) {

				int nWeekday = stDataSet.stSetup.Weekday;

				nWeekday += hRotaryEncoder.getDirection();

				if (nWeekday < RTC_SUN) {
					nWeekday = RTC_SAT;
				}
				else if (nWeekday > RTC_SAT) {
					nWeekday = RTC_SUN;
				}

				stDataSet.stSetup.Weekday = nWeekday;

			}
			else if (nLine == DATE_VALUE_LINE) {

				nPos = get_SelectCursorPos();

				if (nPos == DATE_YEAR_SELECTION_POS) {

					int nYear = stDataSet.stSetup.Year;

					nYear += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nYear < 0) {
						nYear = MAX_YEAR;
					}
					else if (nYear > MAX_YEAR) {
						nYear = 0;
					}

					stDataSet.stSetup.Year = nYear;

				}
				else if (nPos == DATE_MONTH_SELECTION_POS) {

					int nMonth = stDataSet.stSetup.Month;

					nMonth += hRotaryEncoder.getDirection();

					if (nMonth < 1) {
						nMonth = MAX_MONTH;
					}
					else if (nMonth > MAX_MONTH) {
						nMonth = 1;
					}

					stDataSet.stSetup.Month = nMonth;

				}
				else { // nPos == DATE_DAY_SELECTION_POS

					int nDay = stDataSet.stSetup.Day;

					nDay += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nDay < 1) {
						nDay = MAX_DAY;
					}
					else if (nDay > MAX_DAY) {
						nDay = 1;
					}

					stDataSet.stSetup.Day = nDay;

				}

				#if !defined(_SETUP_WEEK_DAY_)
				get_DayOfWeekRTC(stDataSet.stSetup);
				#endif

			}
			else { // nLine == TIME_VALUE_LINE

				nPos = get_SelectCursorPos();

				if (nPos == TIME_HOUR_SELECTION_POS) {

					int nHour = stDataSet.stSetup.Hour;

					nHour += hRotaryEncoder.getDirection();

					if (nHour < 0) {
						nHour = MAX_HOUR-1;
					}
					else if (nHour >= MAX_HOUR) {
						nHour = 0;
					}

					stDataSet.stSetup.Hour = nHour;

				}
				else if (nPos == TIME_MINUTE_SELECTION_POS) {

					int nMinute = stDataSet.stSetup.Minute;

					nMinute += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nMinute < 0) {
						nMinute = MAX_MINUTE-1;
					}
					else if (nMinute >= MAX_MINUTE) {
						nMinute = 0;
					}

					stDataSet.stSetup.Minute = nMinute;

				}
				else { // nPos == TIME_SECOND_SELECTION_POS

					int nSecond = stDataSet.stSetup.Second;

					nSecond += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));

					if (nSecond < 0) {
						nSecond = MAX_SECOND-1;
					}
					else if (nSecond >= MAX_SECOND) {
						nSecond = 0;
					}

					stDataSet.stSetup.Second = nSecond;

				}

			}

			if (_nScratched < 2) {

				_nScratched = 2;

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (_nScratched >= 2) {

					stTimeDate_t stSetupTimeDate;

					stSetupTimeDate.year = stDataSet.stSetup.Year;
					stSetupTimeDate.month = stDataSet.stSetup.Month;
					stSetupTimeDate.day = stDataSet.stSetup.Day;
					stSetupTimeDate.weekday = stDataSet.stSetup.Weekday;
					stSetupTimeDate.hour = stDataSet.stSetup.Hour;
					stSetupTimeDate.minute = stDataSet.stSetup.Minute;
					stSetupTimeDate.second = stDataSet.stSetup.Second;

					set_TimeDateRTC(stSetupTimeDate);

				}

			}

			_nScratched = 0;

			_bPowerSavingModeValid = true;

			select_SetupTimeDateContents(false);

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard3);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_TIMEDATE_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_PowerSavingSetup(const int nSignal)
{
	int nState = nSignal;
	int nPos;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nPos = get_SelectCursorPos();
			nLine = get_SelectCursorLine();

			if (nLine == POWER_SAVING_MODE_VALUE_LINE) {

				nLine += hRotaryEncoder.getDirection();

				if (nLine < POWER_SAVING_MODE_VALUE_LINE) {
					nLine = END_TIME_VALUE_LINE;
					nPos = MINUTE_SELECTION_POS;
				}
				else if (nLine > POWER_SAVING_MODE_VALUE_LINE) {
					nLine = BEGIN_TIME_VALUE_LINE;
					nPos = HOUR_SELECTION_POS;
				}

			}
			else {

				if (nLine == BEGIN_TIME_VALUE_LINE) {

					nPos += (hRotaryEncoder.getDirection() * 5);

					if (nPos < HOUR_SELECTION_POS) {
						nLine = POWER_SAVING_MODE_VALUE_LINE;
						nPos = ACTIVATION_SELECTION_POS;
					}
					else if (nPos > MINUTE_SELECTION_POS) {
						nLine = END_TIME_VALUE_LINE;
						nPos = HOUR_SELECTION_POS;
					}

				}
				else { // nLine == END_TIME_VALUE_LINE

					nPos += (hRotaryEncoder.getDirection() * 5);

					if (nPos < HOUR_SELECTION_POS) {
						nLine = BEGIN_TIME_VALUE_LINE;
						nPos = MINUTE_SELECTION_POS;
					}
					else if (nPos > MINUTE_SELECTION_POS) {
						nLine = POWER_SAVING_MODE_VALUE_LINE;
						nPos = ACTIVATION_SELECTION_POS;
					}

				}

			}

			set_SelectCursorLine(nLine);
			set_SelectCursorPos(nPos);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			_bPowerSavingModeValid = false;

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			nLine = get_SelectCursorLine();

			if (nLine > POWER_SAVING_MODE_VALUE_LINE) {

				if (_nBrewState == POWER_SAVING_STATE || _nPowerSavingState == PWS_PAUSED) {

					nState = LCD_MENUPAGE_LIST_VIEW;

					break;

				}

			}

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == POWER_SAVING_MODE_VALUE_LINE) {

				int nPowerSavingMode = stDataSet.stSetup.PowerSaveMode;

				// hRotaryEncoder.getDirection();

				if (nPowerSavingMode == POWER_SAVE_MODE_OFF) {

					_bPowerSavingModeValid = false;
					stDataSet.stSetup.PowerSaveMode = POWER_SAVE_MODE_ON;

				}
				else {

					_bPowerSavingModeValid = true;
					stDataSet.stSetup.PowerSaveMode = POWER_SAVE_MODE_OFF;

				}

			}
			else if (nLine == BEGIN_TIME_VALUE_LINE) {

				nPos = get_SelectCursorPos();

				if (nPos == HOUR_SELECTION_POS) {

					int nStartHour = stDataSet.stSetup.StartHour;

					nStartHour += hRotaryEncoder.getDirection();

					if (nStartHour < 0) {
						nStartHour = MAX_HOUR-1;
					}
					else if (nStartHour >= MAX_HOUR) {
						nStartHour = 0;
					}

					stDataSet.stSetup.StartHour = nStartHour;

				}
				else { // if (nPos == MINUTE_SELECTION_POS)

					int nStartMinute = stDataSet.stSetup.StartMinute;

					nStartMinute += (hRotaryEncoder.getDirection() * 10);

					if (nStartMinute < 0) {
						nStartMinute = MAX_MINUTE-10;
					}
					else if (nStartMinute >= MAX_MINUTE) {
						nStartMinute = 0;
					}

					stDataSet.stSetup.StartMinute = nStartMinute;

				}
				// else { // nPos == SECOND_SELECTION_POS

				// 	int nStartSecond = stDataSet.stSetup.StartSecond;

				// 	nStartSecond += (hRotaryEncoder.getDirection() * 10);

				// 	if (nStartSecond < 0) {
				// 		nStartSecond = MAX_SECOND-10;
				// 	}
				// 	else if (nStartSecond >= MAX_SECOND) {
				// 		nStartSecond = 0;
				// 	}

				// 	stDataSet.stSetup.StartSecond = nStartSecond;

				// }

			}
			else { // nLine == END_TIME_VALUE_LINE

				nPos = get_SelectCursorPos();

				if (nPos == HOUR_SELECTION_POS) {

					int nEndHour = stDataSet.stSetup.EndHour;

<<<<<<< HEAD
					nBeginMinute += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));
=======
					nEndHour += hRotaryEncoder.getDirection();
>>>>>>> feature/brew_profile_acquisition

					if (nEndHour < 0) {
						nEndHour = MAX_HOUR-1;
					}
					else if (nEndHour >= MAX_HOUR) {
						nEndHour = 0;
					}

					stDataSet.stSetup.EndHour = nEndHour;

				}
				else { // if (nPos == MINUTE_SELECTION_POS)

					int nEndMinute = stDataSet.stSetup.EndMinute;

<<<<<<< HEAD
					nBeginSecond += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(1));
=======
					nEndMinute += (hRotaryEncoder.getDirection() * 10);
>>>>>>> feature/brew_profile_acquisition

					if (nEndMinute < 0) {
						nEndMinute = MAX_MINUTE-10;
					}
					else if (nEndMinute >= MAX_MINUTE) {
						nEndMinute = 0;
					}

					stDataSet.stSetup.EndMinute = nEndMinute;

				}
				// else { // nPos == SECOND_SELECTION_POS

				// 	int nEndSecond = stDataSet.stSetup.EndSecond;

				// 	nEndSecond += (hRotaryEncoder.getDirection() * 10);

				// 	if (nEndSecond < 0) {
				// 		nEndSecond = MAX_SECOND-10;
				// 	}
				// 	else if (nEndSecond >= MAX_SECOND) {
				// 		nEndSecond = 0;
				// 	}

				// 	stDataSet.stSetup.EndSecond = nEndSecond;

				// }

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				unsigned int nHours, nMins;

				verify_PowerSavingDuration(stDataSet.stSetup.StartMinute, nHours, nMins);

				if (stDataSet.stSetup.PowerSaveMode == POWER_SAVE_MODE_ON) {

					save_PowerSavingParam(stDataSet.stSetup);

					if (nHours == MAX_HOUR) {

						// : 24 hours (continue to save power)

						hTextLCD.printPos(0, SAVE_MESSAGE_LINE+1, LCD_PWS_CONTINUE_STRING);

					}
					else {

						// : < 24 hours (10 minutes ~ 23 hours 50 minutes)

						char strDuration[14];

						sprintf(strDuration, "%02d HRS %02d MINS", nHours, nMins);

						hTextLCD.printPos(3, SAVE_MESSAGE_LINE+1, strDuration);

					}

				}
				else {

					save_PowerSavingMode(stDataSet.stSetup.PowerSaveMode);

					load_PowerSavingTime(stDataSet.stSetup);

					hTextLCD.printPos(0, SAVE_MESSAGE_LINE+1, LCD_PWS_INACTIVE_STRING);

				}

				delay(TEXT_LCD_INVALIDATE_PERIOD3);

			}
			else {

				load_PowerSavingParam(stDataSet.stSetup);

			}

			_bPowerSavingModeValid = true;

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard3);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_POWERSAVE_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_HeaterParameterSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (stDataSet.stSetup.SetHeaterChannel < TM4_INPUT_BIAS_SETUP_CHANNEL-1) {

				if (nLine < HEATER_SET_VALUE_LINE) {
					nLine = D_TIME_VALUE_LINE;
				}
				else if (nLine > D_TIME_VALUE_LINE) {
					nLine = HEATER_SET_VALUE_LINE;
				}

			}
			else {

				if (nLine < HEATER_SET_VALUE_LINE) {
					nLine = I_TIME_VALUE_LINE;
				}
				else if (nLine > I_TIME_VALUE_LINE) {
					nLine = HEATER_SET_VALUE_LINE;
				}

			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			if (get_SelectCursorLine() > HEATER_SET_VALUE_LINE) {

				if (!IsAuthorized()) {

					nState = LCD_MENUPAGE_LIST_VIEW;

					break;

				}

			}

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == HEATER_SET_VALUE_LINE) {

				int nPreviousChannel = stDataSet.stSetup.SetHeaterChannel;
				int nCurrentChannel = stDataSet.stSetup.SetHeaterChannel;

				nCurrentChannel += hRotaryEncoder.getDirection();

				if (nCurrentChannel < TM4_PREHEAT_CHANNEL - 1) {
					nCurrentChannel = TM4_INPUT_BIAS_SETUP_CHANNEL - 1;
				}
				else if (nCurrentChannel > TM4_INPUT_BIAS_SETUP_CHANNEL - 1) {
					nCurrentChannel = TM4_PREHEAT_CHANNEL - 1;
				}

				stDataSet.stSetup.SetHeaterChannel = nCurrentChannel;

				if ((nPreviousChannel == TM4_MVHIGH_SETUP_CHANNEL-1 || nCurrentChannel == TM4_MVHIGH_SETUP_CHANNEL-1) ||
					(nPreviousChannel == TM4_INPUT_BIAS_SETUP_CHANNEL-1 || nCurrentChannel == TM4_INPUT_BIAS_SETUP_CHANNEL-1)) {

					change_HeaterSetupMenupage(true, stDataSet);

				}

			}
			else {

				if (stDataSet.stSetup.SetHeaterChannel <= TM4_GROUPHEAD_CHANNEL-1) {

					if (nLine == P_BAND_VALUE_LINE) {

						float fPBandTemperature = stDataSet.stSetup.ProportionalBandTemp[stDataSet.stSetup.SetHeaterChannel];

<<<<<<< HEAD
						int nAccFactorPower = pow(hRotaryEncoder.getAccFactor(10), 2);

						fPBandTemperature += (hRotaryEncoder.getDirection() * nAccFactorPower * 0.1f);
=======
						fPBandTemperature += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(10) * 0.1f);
>>>>>>> feature/brew_profile_acquisition

						if (fPBandTemperature < 0.1F) {
							fPBandTemperature = 999.9F;
						}
						else if (fPBandTemperature > 999.9F) {
							fPBandTemperature = 0.1F;
						}

						stDataSet.stSetup.ProportionalBandTemp[stDataSet.stSetup.SetHeaterChannel] = fPBandTemperature;

					}
					else if (nLine == I_TIME_VALUE_LINE) {

						int nIntegralTime = stDataSet.stSetup.IntegralTime[stDataSet.stSetup.SetHeaterChannel];

<<<<<<< HEAD
						int nAccFactorPower = pow(hRotaryEncoder.getAccFactor(10), 2);

						nIntegralTime += (hRotaryEncoder.getDirection() * nAccFactorPower);
=======
						nIntegralTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(10));
>>>>>>> feature/brew_profile_acquisition

						if (nIntegralTime < 0) {
							nIntegralTime = 9999UL;
						}
						else if (nIntegralTime > 9999UL) {
							nIntegralTime = 0;
						}

						stDataSet.stSetup.IntegralTime[stDataSet.stSetup.SetHeaterChannel] = nIntegralTime;

					}
					else { // nLine == D_TIME_VALUE_LINE

						int nDerivativeTime = stDataSet.stSetup.DerivativeTime[stDataSet.stSetup.SetHeaterChannel];

<<<<<<< HEAD
						int nAccFactorPower = pow(hRotaryEncoder.getAccFactor(10), 2);

						nDerivativeTime += (hRotaryEncoder.getDirection() * nAccFactorPower);
=======
						nDerivativeTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(10));
>>>>>>> feature/brew_profile_acquisition

						if (nDerivativeTime < 0) {
							nDerivativeTime = 9999UL;
						}
						else if (nDerivativeTime > 9999UL) {
							nDerivativeTime = 0;
						}

						stDataSet.stSetup.DerivativeTime[stDataSet.stSetup.SetHeaterChannel] = nDerivativeTime;

					}

				}
				else {

					if (stDataSet.stSetup.SetHeaterChannel == TM4_MVHIGH_SETUP_CHANNEL-1) {

						float fMVHighLimit = stDataSet.stSetup.MVHighLimit[nLine-1];

						fMVHighLimit += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(2) * 0.1f);

						if (fMVHighLimit < 0.1F) {
							fMVHighLimit = 100.0F;
						}
						else if (fMVHighLimit > 100.0F) {
							fMVHighLimit = 0.1F;
						}

						stDataSet.stSetup.MVHighLimit[nLine-1] = fMVHighLimit;

					}
					else { // stDataSet.stSetup.SetHeaterChannel == TM4_INPUT_BIAS_SETUP_CHANNEL-1

						if (nLine == INPUT_SENSOR_CH_LINE) {

							int nCurrentChannel = stDataSet.stSetup.TempSensorChannel;

							nCurrentChannel += hRotaryEncoder.getDirection();

							if (nCurrentChannel < TM4_PREHEAT_CHANNEL - 1) {
								nCurrentChannel = TM4_BREWWATER_CHANNEL - 1;
							}
							else if (nCurrentChannel > TM4_BREWWATER_CHANNEL - 1) {
								nCurrentChannel = TM4_PREHEAT_CHANNEL - 1;
							}

							stDataSet.stSetup.TempSensorChannel = nCurrentChannel;

						}
						else if (nLine == INPUT_BIAS_VALUE_LINE) {

							float fSensorInputBias = stDataSet.stSetup.SensorInputBias[stDataSet.stSetup.TempSensorChannel];

							fSensorInputBias += (hRotaryEncoder.getDirection() * 0.1f);

							if (fSensorInputBias <= MIN_SENSOR_INPUT_BIAS-0.1F) {
								fSensorInputBias = MAX_SENSOR_INPUT_BIAS;
							}
							else if (fSensorInputBias >= MAX_SENSOR_INPUT_BIAS+0.1F) {
								fSensorInputBias = MIN_SENSOR_INPUT_BIAS;
							}

							stDataSet.stSetup.SensorInputBias[stDataSet.stSetup.TempSensorChannel] = fSensorInputBias;

						}

					}

				}

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (IsAuthorized()) {

				if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

					config_BoilerTemperatureControllerPID();
					config_BoilerTemperatureControllerMV();
					config_BoilerTemperatureControllerBIAS();

				}
				else {

					load_HeaterProportionalBandTemperature(stDataSet.stSetup);
					load_HeaterIntegralTime(stDataSet.stSetup);
					load_HeaterDerivativeTime(stDataSet.stSetup);
					load_HeaterMVHighLimit(stDataSet.stSetup);
					load_HeaterInputBias(stDataSet.stSetup);

				}

			}

			stDataSet.stSetup.SetHeaterChannel = 0;
			stDataSet.stSetup.TempSensorChannel = 0;

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard4);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_HEATER_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_PressureSensorAdjustmentSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	static bool _bBoilerHeaterSupspended = false;
	static bool _bZeroShiftConfigured = false;
	static bool _bSpanConfigured = false;

	if (IsAuthorized()) {

		if (!_bBoilerHeaterSupspended) {

			if (!(_bBoilerHeaterSupspended = suspend_BoilerHeater())) {

				// #if defined(_USE_APPLICATION_DEBUG_MSG_)
				cout << F("[ERROR] Failed to suspend Boiler Heater") << endl;
				// #endif

			}

		}

	}

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < SENSOR_CHANNEL_VALUE_LINE) {
				nLine = SENSOR_DEFAULT_VALUE_LINE;
			}
			else if (nLine > SENSOR_DEFAULT_VALUE_LINE) {
				nLine = SENSOR_CHANNEL_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;

			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			nLine = get_SelectCursorLine();

			if (nLine > SENSOR_CHANNEL_VALUE_LINE) {

				if (!IsAuthorized()) {

					nState = LCD_MENUPAGE_LIST_VIEW;

					break;

				}

				set_SelectedCursor(true);
				display_SelectCursor();

				if (nLine == SENSOR_DEFAULT_VALUE_LINE) {

					config_PressureSensorAdjustmentToFactoryDefault();

					display_SensorInitNoticeMessage(stDataSet, stCurTimeDate);

					_bZeroShiftConfigured = false;
					_bSpanConfigured = false;

				}
				else if (nLine == SENSOR_ZEROSHIFT_VALUE_LINE) {

					if (_bBoilerHeaterSupspended) {

						config_PressureSensorZeroShift();

						display_SensorZeroShiftNoticeMessage(stDataSet, stCurTimeDate);

						_bZeroShiftConfigured = true;

					}

				}
				else { // nLine == SENSOR_SPAN_VALUE_LINE

					if (_bBoilerHeaterSupspended) {

						if (_bZeroShiftConfigured) {

							config_PressureSensorSpanFactor();

							display_SensorSpanNoticeMessage(stDataSet, stCurTimeDate);

							_bSpanConfigured = true;

						}
						else {

							display_SensorAdjFailNoticeMessage(stDataSet, stCurTimeDate);

							// #if defined(_USE_APPLICATION_DEBUG_MSG_)
							cout << F("[NOTICE] Adjust Zero Shift first") << endl;
							// #endif

						}

					}

				}

				nState = LCD_MENUPAGE_LIST_VIEW;

				break;

			}

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == SENSOR_CHANNEL_VALUE_LINE) {

				int nSensorChannel = stDataSet.stSetup.SetSensorChannel;

				nSensorChannel += hRotaryEncoder.getDirection();

				if (nSensorChannel < BREW_BOILER_PRESSURE_SENSOR_CH) {
					nSensorChannel = PUMP_PRESSURE_SENSOR_CH;
				}
				else if (nSensorChannel > PUMP_PRESSURE_SENSOR_CH) {
					nSensorChannel = BREW_BOILER_PRESSURE_SENSOR_CH;
				}

				stDataSet.stSetup.SetSensorChannel = nSensorChannel;

				update_PressureSensorAnalogDigit(true);

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (IsAuthorized()) {

				if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

					save_PressureSensorZeroShift(stDataSet.stSetup);
					save_PressureSensorSpanFactor(stDataSet.stSetup);

					if (_bSpanConfigured) {

						_bZeroShiftConfigured = false;
						_bSpanConfigured = false;

					}

				}
				else {

					load_PressureSensorZeroShift(stDataSet.stSetup);
					load_PressureSensorSpanFactor(stDataSet.stSetup);

					setup_PressureSensorAdjustment();

					_bZeroShiftConfigured = false;
					_bSpanConfigured = false;

				}

			}

			if (hBrewValve.isOpening()) {

				hBrewValve.close();

			}

			if (hMotorPump.isRunning()) {

				hMotorPump.stop();

			}

			if (_bBoilerHeaterSupspended) {

				if (resume_BoilerHeater()) {

					_bBoilerHeaterSupspended = false;

				}
				else {

					// #if defined(_USE_APPLICATION_DEBUG_MSG_)
					cout << F("[ERROR] Failed to resume Boiler Heater") << endl;
					// #endif

				}

			}

			stDataSet.stSetup.SetSensorChannel = 0;

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard4);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_SENSOR_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	update_PressureSensorAnalogDigit(false);

	return nState;
}

int procedure_MotorAccelerationSetup(const int nSignal)
{
	int nState = nSignal;
	int nMotorAccTime;
<<<<<<< HEAD

	// int nStepTime = 1UL;
=======
>>>>>>> feature/brew_profile_acquisition

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(MOTOR_ACC_TIME_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nMotorAccTime = stDataSet.stSetup.MotorAccTime;

			nMotorAccTime += (hRotaryEncoder.getDirection() * hRotaryEncoder.getAccFactor(10));

			if (nMotorAccTime < _MIN_ACC_TIME_) {
				nMotorAccTime = _MAX_ACC_TIME_;
			}
			else if (nMotorAccTime > _MAX_ACC_TIME_) {
				nMotorAccTime = _MIN_ACC_TIME_;
			}

			stDataSet.stSetup.MotorAccTime = nMotorAccTime;
			#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			stDataSet.stSetup.MotorAccTimeEST = hMotorPump.convACCTimeWithRPM(nMotorAccTime, stDataSet.stSetup.MotorPower);
			#endif

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
				hMotorPump.setACC(stDataSet.stSetup.MotorAccTime);
				#endif

				save_MotorACCTime(stDataSet.stSetup.MotorAccTime);

			}
			else {

				load_MotorACCTimeParam(stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard4);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_MOTOR_ACC_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

#if AUTOMATION_SENSOR_ATTACHED
int procedure_AutomationSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < AUTOMATION_MODE_VALUE_LINE) {
				nLine = AUTO_BREW_WAIT_TIME_VALUE_LINE;
			}
			else if (nLine > AUTO_BREW_WAIT_TIME_VALUE_LINE) {
				nLine = AUTOMATION_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == AUTOMATION_MODE_VALUE_LINE) {

				int nAutomationMode = stDataSet.stSetup.AutomationMode;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.AutomationMode = nAutomationMode == AUTOMATION_MODE_OFF ? AUTOMATION_MODE_ON : AUTOMATION_MODE_OFF;

			}
			else if (nLine == AUTO_FLUSHING_TIME_VALUE_LINE) {

				float fAutoFlushingTime = stDataSet.stSetup.AutoFlushingTime;

				fAutoFlushingTime += (hRotaryEncoder.getDirection() * 0.5f);

				if (fAutoFlushingTime < MIN_TIME_TO_AUTO_FLUSHING) {
					fAutoFlushingTime = MAX_TIME_TO_AUTO_FLUSHING;
				}
				else if (fAutoFlushingTime > MAX_TIME_TO_AUTO_FLUSHING) {
					fAutoFlushingTime = MIN_TIME_TO_AUTO_FLUSHING;
				}

				stDataSet.stSetup.AutoFlushingTime = fAutoFlushingTime;

			}
			else { // nLine == AUTO_BREW_WAIT_TIME_VALUE_LINE

				float fAutoBrewWaitTime = stDataSet.stSetup.AutoBrewWaitTime;

				fAutoBrewWaitTime += (hRotaryEncoder.getDirection() * 0.5f);

				if (fAutoBrewWaitTime == -0.5F) {
					fAutoBrewWaitTime = MIN_WAIT_TIME_TO_AUTO_BREW;
				}
				else if (fAutoBrewWaitTime == -1.5F) {
					fAutoBrewWaitTime = MAX_WAIT_TIME_TO_AUTO_BREW;
				}
				else if (fAutoBrewWaitTime < MIN_WAIT_TIME_TO_AUTO_BREW) {
					fAutoBrewWaitTime = -1.0F;
				}
				else if (fAutoBrewWaitTime > MAX_WAIT_TIME_TO_AUTO_BREW) {
					fAutoBrewWaitTime = -1.0F;
				}

				stDataSet.stSetup.AutoBrewWaitTime = fAutoBrewWaitTime;

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (load_AutomationMode() != stDataSet.stSetup.AutomationMode) {

					setup_AutomationMode((_nBrewState == POWER_SAVING_STATE));

				}

				save_AutomationParam(stDataSet.stSetup);

			}
			else {

				load_AutomationParam(stDataSet.stSetup);

			}

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard4);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_CLEANING_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

#else

int procedure_FlushingDetectionModeSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			nLine = get_SelectCursorLine();

			nLine += hRotaryEncoder.getDirection();

			if (nLine < FLUSHING_DETECTION_MODE_VALUE_LINE) {
				nLine = FLUSHING_DETECTION_PRES_VALUE_LINE;
			}
			else if (nLine > FLUSHING_DETECTION_PRES_VALUE_LINE) {
				nLine = FLUSHING_DETECTION_MODE_VALUE_LINE;
			}

			set_SelectCursorLine(nLine);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == FLUSHING_DETECTION_MODE_VALUE_LINE) {

				int nFlushingDetectionMode = stDataSet.stSetup.FlushingDetectionMode;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.FlushingDetectionMode = nFlushingDetectionMode == FLUSHING_DETECTION_MODE_OFF ? FLUSHING_DETECTION_MODE_ON : FLUSHING_DETECTION_MODE_OFF;

			}
			else if (nLine == FLUSHING_DETECTION_TIME_VALUE_LINE) {

				float fFlushingDetectionTime = stDataSet.stSetup.FlushingDetectionTime;

				fFlushingDetectionTime += (hRotaryEncoder.getDirection() * 0.1f);

				if (fFlushingDetectionTime < MIN_TIME_FOR_FLUSHING_DETECTION) {
					fFlushingDetectionTime = MAX_TIME_FOR_FLUSHING_DETECTION;
				}
				else if (fFlushingDetectionTime > MAX_TIME_FOR_FLUSHING_DETECTION) {
					fFlushingDetectionTime = MIN_TIME_FOR_FLUSHING_DETECTION;
				}

				stDataSet.stSetup.FlushingDetectionTime = fFlushingDetectionTime;

			}
			else { // nLine == FLUSHING_DETECTION_PRES_VALUE_LINE

				float fFlushingDetectionPress = stDataSet.stSetup.FlushingDetectionPress;

				fFlushingDetectionPress += (hRotaryEncoder.getDirection() * 0.1f);

				if (fFlushingDetectionPress < MIN_BREW_PRESSURE_FOR_FLUSHING_DETECTION) {
					fFlushingDetectionPress = MAX_BREW_PRESSURE_FOR_FLUSHING_DETECTION;
				}
				else if (fFlushingDetectionPress > MAX_BREW_PRESSURE_FOR_FLUSHING_DETECTION) {
					fFlushingDetectionPress = MIN_BREW_PRESSURE_FOR_FLUSHING_DETECTION;
				}

				stDataSet.stSetup.FlushingDetectionPress = fFlushingDetectionPress;

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				save_FlushingDetectionParam(stDataSet.stSetup);

			}
			else {

				load_FlushingDetectionParam(stDataSet.stSetup);

			}

			// verify_FlushingDetectionModeWithMotorPump();


			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard4);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_CLEANING_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}
#endif

int procedure_MaintenanceSetup(const int nSignal)
{
	int nState = nSignal;
	int nLine;

	static bool _bFirstEntry = true;
	static bool _bTotFlowQtyCleared = false;
	static unsigned long _ulPrevTotFlowQuantity = 0UL;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

			// if (IsAuthorized()) {

				nLine = get_SelectCursorLine();

				nLine += hRotaryEncoder.getDirection();

				if (nLine < MAINTENANCE_MODE_VALUE_LINE) {
					nLine = TOTAL_FLOW_VALUE_LINE;
				}
				else if (nLine > TOTAL_FLOW_VALUE_LINE) {
					nLine = MAINTENANCE_MODE_VALUE_LINE;
				}

				set_SelectCursorLine(nLine);

			// }

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_VIEW:

			if (_bFirstEntry) {

				_bFirstEntry = false;

				_ulPrevTotFlowQuantity = stDataSet.stMaintenance.TotFlowQuantity;

			}

			set_SelectedCursor(false);

			nState = LCD_MENUPAGE_LIST_VIEW;

		break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nLine = get_SelectCursorLine();

			if (nLine == MAINTENANCE_MODE_VALUE_LINE) {

				unsigned char nMaintenanceMode = stDataSet.stSetup.MaintenanceMode;

				// hRotaryEncoder.getDirection();

				stDataSet.stSetup.MaintenanceMode = nMaintenanceMode == MAINTENANCE_MODE_OFF ? MAINTENANCE_MODE_ON : MAINTENANCE_MODE_OFF;

				if (stDataSet.stSetup.MaintenanceMode == MAINTENANCE_MODE_ON) {

					enable_SystemMaintenanceMode();

				}
				else {

					disable_SystemMaintenanceMode();

				}

			}
			else { // nLine == TOTAL_FLOW_VALUE_LINE

				unsigned long ulTotFlowQuantity = stDataSet.stMaintenance.TotFlowQuantity;

				// hRotaryEncoder.getDirection();

				if (ulTotFlowQuantity == _ulPrevTotFlowQuantity) {

					_bTotFlowQtyCleared = true;

					stDataSet.stMaintenance.TotFlowQuantity = 0UL;

				}
				else {

					_bTotFlowQtyCleared = false;

					stDataSet.stMaintenance.TotFlowQuantity = _ulPrevTotFlowQuantity;

				}

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (IsAuthorized()) {

					save_TotalFlowQuantity(stDataSet.stMaintenance.TotFlowQuantity);

					if (_bTotFlowQtyCleared == true) {

						stDataSet.stMaintenance.Overflowed &= 0xF0;

						save_OverflowedUsage(stDataSet.stMaintenance.Overflowed);

					}

				}

			}
			else {

				load_MaintenanceBrewUsage(stDataSet.stMaintenance);

			}

			_bFirstEntry = true;
			_bTotFlowQtyCleared = false;
			_ulPrevTotFlowQuantity = 0UL;

			stDataSet.stSetup.MaintenanceMode = MAINTENANCE_MODE_OFF;

			disable_SystemMaintenanceMode();

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard5);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_MAINTENANCE_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_DefaultResetSetup(const int nSignal)
{
	int nState = nSignal;

	unsigned char nDefaultSet;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	hRotaryEncoder.getDirection();

		// 	set_SelectCursorLine(DEFAUT_SET_VALUE_LINE);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			nDefaultSet = stDataSet.stSetup.DefaultSet;

			// hRotaryEncoder.getDirection();

			stDataSet.stSetup.DefaultSet = nDefaultSet == DEFAULT_RESET_OFF ? DEFAULT_RESET_ON : DEFAULT_RESET_OFF;

			set_DefaultResetString((stDataSet.stSetup.DefaultSet == DEFAULT_RESET_ON ? true : false));

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				if (stDataSet.stSetup.DefaultSet == DEFAULT_RESET_ON) {

					reset_SystemParametersToFactoryDefault();

				}

			}

			stDataSet.stSetup.DefaultSet = DEFAULT_RESET_OFF;

			set_DefaultResetString(false);

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard5);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_RESET_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_SystemInformationSetup(const int nSignal)
{
	int nState = nSignal;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		case LCD_MENUPAGE_LIST_SELECT:

		case LCD_MENUPAGE_VALUE_CHANGE:

		case LCD_MENUPAGE_LIST_VIEW:

			nState = LCD_MENUPAGE_LIST_VIEW;
			
		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			set_NoSelectionPage(false);

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard5);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_INFORMATION_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}

int procedure_PasswordAuthorizationSetup(const int nSignal)
{
	int nState = nSignal;
	int nPos;

	static bool _bFirstEntry = true;
	static bool _bLastEntry = false;
	static int _nPasswordSelection = 0;

	switch (nSignal) {

		case LCD_MENUPAGE_LIST_CHANGE:

		// 	nPos = get_SelectCursorPos();

		// 	nPos += (hRotaryEncoder.getDirection() * 3);

		// 	if (nPos < PASSWORD0_SELECTION_POS) {
		// 		nPos = PASSWORD3_SELECTION_POS;
		// 	}
		// 	else if (nPos > PASSWORD3_SELECTION_POS) {
		// 		nPos = PASSWORD0_SELECTION_POS;
		// 	}

		// 	set_SelectCursorPos(nPos);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;

		// break;

		case LCD_MENUPAGE_LIST_VIEW:

			if (_bFirstEntry) {

				_bFirstEntry = false;

				nPos = get_SelectCursorPos();

			}
			else {

				nPos = get_SelectCursorPos() + 3;

				if (nPos < PASSWORD0_SELECTION_POS) {
					nPos = PASSWORD3_SELECTION_POS;
				}
				else if (nPos > PASSWORD3_SELECTION_POS) {
					nPos = PASSWORD0_SELECTION_POS;
				}

				set_SelectCursorPos(nPos);

				if (!_bLastEntry) {

					add_AuthorizationCode(_nPasswordSelection);

					if (_nPasswordSelection == 3) {

						_bLastEntry = true;

					}

				}

				stDataSet.stSetup.PasswordInput[_nPasswordSelection] = 10;

			}

			switch (nPos) {
				case PASSWORD0_SELECTION_POS:
					_nPasswordSelection = 0;
				break;
				case PASSWORD1_SELECTION_POS:
					_nPasswordSelection = 1;
				break;
				case PASSWORD2_SELECTION_POS:
					_nPasswordSelection = 2;
				break;
				case PASSWORD3_SELECTION_POS:
					_nPasswordSelection = 3;
				break;
			}

			if (!_bLastEntry) {

				stDataSet.stSetup.PasswordInput[_nPasswordSelection] = 0;

			}
			else {

				_bFirstEntry = true;
				_bLastEntry = false;
				_nPasswordSelection = 0;

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE, LCD_PW_VERIFY_STRING);

				if (request_Authorization()) {

					hTextLCD.printPos(0, SAVE_MESSAGE_LINE+1, LCD_PW_PASS_STRING);

					delay(TEXT_LCD_INVALIDATE_PERIOD2);

					set_SelectedCursor(false);
					set_TextLCDMenuboard(Menuboard5);
					set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_PASSWORD_LINE);

					nState = LCD_MENUBOARD_ENTRY;

				}
				else {

					hTextLCD.printPos(0, SAVE_MESSAGE_LINE+1, LCD_PW_FAIL_STRING);

					delay(TEXT_LCD_INVALIDATE_PERIOD2);

					hTextLCD.printPos(6, SAVE_MESSAGE_LINE,   "        ");
					hTextLCD.printPos(8, SAVE_MESSAGE_LINE+1, "    ");

					nState = LCD_MENUPAGE_LIST_VIEW;

				}

				break;

			}

		// 	set_SelectedCursor(false);

		// 	nState = LCD_MENUPAGE_LIST_VIEW;
			
		//  break;

		case LCD_MENUPAGE_LIST_SELECT:

			set_SelectedCursor(true);

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_VALUE_CHANGE:

			if (!_bLastEntry) {

				int nPasswordNumber = stDataSet.stSetup.PasswordInput[_nPasswordSelection];

				nPasswordNumber += hRotaryEncoder.getDirection();

				if (nPasswordNumber < 0) {
					nPasswordNumber = 9;
				}
				else if (nPasswordNumber > 9) {
					nPasswordNumber = 0;
				}

				stDataSet.stSetup.PasswordInput[_nPasswordSelection] = nPasswordNumber;

			}

			nState = LCD_MENUPAGE_LIST_SELECT;

		break;

		case LCD_MENUPAGE_SAVE_EXIT:
		case LCD_MENUPAGE_CANCEL_EXIT:

			if (nSignal == LCD_MENUPAGE_SAVE_EXIT) {

				bool bPassed = request_Authorization();

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE+1, (bPassed ? LCD_PW_PASS_STRING : LCD_PW_FAIL_STRING));

				delay(TEXT_LCD_INVALIDATE_PERIOD2);

			}
			else {

				release_Authorization();

			}

			_bFirstEntry = true;
			_bLastEntry = false;
			_nPasswordSelection = 0;

			set_SelectedCursor(false);
			set_TextLCDMenuboard(Menuboard5);
			set_SelectCursorPosition(HOME_CURSOR_POS, LCD_SETUPMENU_PASSWORD_LINE);

			nState = LCD_MENUBOARD_ENTRY;

		break;

	}

	return nState;
}


// Motor Pump Controller Utilities
#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
void read_MotorControllerObservation(void)
{
	static unsigned long _lPollingOZBVTime = 0UL;

	if (millis() - _lPollingOZBVTime >= MOTOR_ALARM_POLLING_PERIOD) {

		_lPollingOZBVTime = millis();

		PollingMotorPumpAlarm_Callback();

	}

#if defined(_USE_APPLICATION_DEBUG_MSG_)

	#if defined(_OBSERVE_MOTOR_CURRENT_VALUE_) || defined(_OBSERVE_MOTOR_SPEED_VALUE_)
	if (hMotorPump.isRunning() == true) {

		static unsigned long _lPollingOZBVEStatus = 0UL;

		if (millis() - _lPollingOZBVEStatus >= MOTOR_PRESENT_STATUS_POLLING_PERIOD) {

			_lPollingOZBVEStatus = millis();

			#if defined(_OBSERVE_MOTOR_SPEED_VALUE_)
			stDataSet.stAcquisition.PresentSpeed = hMotorPump.getSpeed();
			#endif

			#if defined(_OBSERVE_MOTOR_CURRENT_VALUE_)
			stDataSet.stAcquisition.PresentCurrent = hMotorPump.getCurrent();
			#endif

		}

	}
	#endif

#else

	#if defined(_OBSERVE_MOTOR_CURRENT_VALUE_) || defined(_OBSERVE_MOTOR_SPEED_VALUE_)
	if (hMotorPump.isRunning() == true) {

		static unsigned long _lPollingOZBVEStatus = 0UL;

		if (millis() - _lPollingOZBVEStatus >= MOTOR_PRESENT_STATUS_POLLING_PERIOD) {

			_lPollingOZBVEStatus = millis();

			#if defined(_OBSERVE_MOTOR_SPEED_VALUE_)
			stDataSet.stAcquisition.PresentSpeed = hMotorPump.getSpeed();
			#endif

			#if defined(_OBSERVE_MOTOR_CURRENT_VALUE_)
			stDataSet.stAcquisition.PresentCurrent = hMotorPump.getCurrent();
			#endif

		}

	}
	#endif

#endif
}

void clear_MotorControllerStatusObservation(void)
{
	stDataSet.stAcquisition.PresentCurrent = 0L;
	stDataSet.stAcquisition.PresentSpeed = 0L;
}

void verify_MotorControllerACCTime(void)
{
	if (stDataSet.stSetup.MotorAccTime != hMotorPump.verifyACCTime()) {

		hMotorPump.setACC(stDataSet.stSetup.MotorAccTime);

	}
}
#endif


// Boiler/Heater Controller (TM4)
void read_BoilerTemperatureControllerPID(void)
{
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		stDataSet.stSetup.ProportionalBandTemp[nChannel-1] = hTM4Controller.readHeatingProportionalBand(eChannel);
		stDataSet.stSetup.IntegralTime[nChannel-1] = hTM4Controller.readHeatingIntegralTime(eChannel);
		stDataSet.stSetup.DerivativeTime[nChannel-1] = hTM4Controller.readHeatingDerivationTime(eChannel);

	}
}

bool verify_BoilerTemperatureConstrollerPID(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if (stDataSet.stSetup.ProportionalBandTemp[nChannel-1] != hTM4Controller.readHeatingProportionalBand(eChannel)) {

			bSuccess &= hTM4Controller.writeHeatingProportionalBand(eChannel, stDataSet.stSetup.ProportionalBandTemp[nChannel-1]) ? false : true;

		}

		if (stDataSet.stSetup.IntegralTime[nChannel-1] != hTM4Controller.readHeatingIntegralTime(eChannel)) {

			bSuccess &= hTM4Controller.writeHeatingIntegralTime(eChannel, stDataSet.stSetup.IntegralTime[nChannel-1]) ? false : true;

		}

		if (stDataSet.stSetup.DerivativeTime[nChannel-1] != hTM4Controller.readHeatingDerivationTime(eChannel)) {

			bSuccess &= hTM4Controller.writeHeatingDerivationTime(eChannel, stDataSet.stSetup.DerivativeTime[nChannel-1]) ? false : true;

		}

	}

	return bSuccess;
}

bool setup_BoilerTemperatureControllerPID(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		bSuccess &= hTM4Controller.writeHeatingProportionalBand(eChannel, stDataSet.stSetup.ProportionalBandTemp[nChannel-1]) ? false : true;
		bSuccess &= hTM4Controller.writeHeatingIntegralTime(eChannel, stDataSet.stSetup.IntegralTime[nChannel-1]) ? false : true;
		bSuccess &= hTM4Controller.writeHeatingDerivationTime(eChannel, stDataSet.stSetup.DerivativeTime[nChannel-1]) ? false : true;

	}

	return bSuccess;
}

bool config_BoilerTemperatureControllerPID(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if ((bSuccess &= hTM4Controller.writeHeatingProportionalBand(eChannel, stDataSet.stSetup.ProportionalBandTemp[nChannel-1]) ? false : true)) {

			float fPBandTemperature = stDataSet.stSetup.ProportionalBandTemp[nChannel-1];
			save_HeaterProportionalBandTemperatureWithChannel(nChannel-1, fPBandTemperature);

		}

		if ((bSuccess &= hTM4Controller.writeHeatingIntegralTime(eChannel, stDataSet.stSetup.IntegralTime[nChannel-1]) ? false : true)) {

			unsigned int nIntegralTime = stDataSet.stSetup.IntegralTime[nChannel-1];
			save_HeaterIntegralTimeWithChannel(nChannel-1, nIntegralTime);

		}

		if ((bSuccess &= hTM4Controller.writeHeatingDerivationTime(eChannel, stDataSet.stSetup.DerivativeTime[nChannel-1]) ? false : true)) {

			unsigned int nDerivativeTime = stDataSet.stSetup.DerivativeTime[nChannel-1];
			save_HeaterDerivativeTimeWithChannel(nChannel-1, nDerivativeTime);

		}

	}

	return bSuccess;
}

void read_BoilerTemperatureControllerMV(void)
{
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		stDataSet.stSetup.MVHighLimit[nChannel-1] = hTM4Controller.readMVHighLimit(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

	}
}

bool verify_BoilerTemperatureControllerMV(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if (stDataSet.stSetup.MVHighLimit[nChannel-1] != hTM4Controller.readMVHighLimit(eChannel)) {

			bSuccess &= hTM4Controller.writeMVHighLimit(eChannel, stDataSet.stSetup.MVHighLimit[nChannel-1]) ? false : true;

		}

	}

	return bSuccess;
}

bool setup_BoilerTemperatureControllerMV(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		bSuccess &= hTM4Controller.writeMVHighLimit(static_cast<enum CTM4Controller::TM4Channel>(nChannel), stDataSet.stSetup.MVHighLimit[nChannel-1]) ? false : true;

	}

	return bSuccess;
}

bool config_BoilerTemperatureControllerMV(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		if ((bSuccess &= hTM4Controller.writeMVHighLimit(eChannel, stDataSet.stSetup.MVHighLimit[nChannel-1]) ? false : true)) {

			save_HeaterMVHighLimitWithChannel(nChannel-1, stDataSet.stSetup.MVHighLimit[nChannel-1]);

		}

	}

	return bSuccess;
}

bool verify_BoilerTemperatureControllerBIAS(void)
{
	bool bSuccess = true;

	float fTM4SensorInputBias[TM4_TOTAL_NO_OF_CH] = {DEFAULT_CH1_SENSOR_INPUT_BIAS, DEFAULT_CH2_SENSOR_INPUT_BIAS, DEFAULT_CH3_SENSOR_INPUT_BIAS, DEFAULT_CH4_SENSOR_INPUT_BIAS};

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		enum CTM4Controller::TM4Channel eChannel = static_cast<enum CTM4Controller::TM4Channel>(nChannel);

		fTM4SensorInputBias[nChannel-1] += stDataSet.stSetup.SensorInputBias[nChannel-1];

		if (fTM4SensorInputBias[nChannel-1] != hTM4Controller.readSensorInputBias(eChannel)) {

			bSuccess &= hTM4Controller.writeSensorInputBias(eChannel, fTM4SensorInputBias[nChannel-1]) ? false : true;

		}

	}

	return bSuccess;
}

bool setup_BoilerTemperatureControllerBIAS(void)
{
	bool bSuccess = true;

	float fTM4SensorInputBias[TM4_TOTAL_NO_OF_CH] = {DEFAULT_CH1_SENSOR_INPUT_BIAS, DEFAULT_CH2_SENSOR_INPUT_BIAS, DEFAULT_CH3_SENSOR_INPUT_BIAS, DEFAULT_CH4_SENSOR_INPUT_BIAS};

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		fTM4SensorInputBias[nChannel-1] += stDataSet.stSetup.SensorInputBias[nChannel-1];

		bSuccess &= hTM4Controller.writeSensorInputBias(static_cast<enum CTM4Controller::TM4Channel>(nChannel), fTM4SensorInputBias[nChannel-1]) ? false : true;

	}

	return bSuccess;
}

bool config_BoilerTemperatureControllerBIAS(void)
{
	bool bSuccess = true;

	float fTM4SensorInputBias[TM4_TOTAL_NO_OF_CH] = {DEFAULT_CH1_SENSOR_INPUT_BIAS, DEFAULT_CH2_SENSOR_INPUT_BIAS, DEFAULT_CH3_SENSOR_INPUT_BIAS, DEFAULT_CH4_SENSOR_INPUT_BIAS};

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_BREWWATER_CHANNEL; nChannel++) {

		fTM4SensorInputBias[nChannel-1] += stDataSet.stSetup.SensorInputBias[nChannel-1];

		bSuccess &= hTM4Controller.writeSensorInputBias(static_cast<enum CTM4Controller::TM4Channel>(nChannel), fTM4SensorInputBias[nChannel-1]) ? false : true;

	}

	if (bSuccess) {

		save_HeaterInputBias(stDataSet.stSetup);

	}

	return bSuccess;
}

bool read_BoilerTemperatureControllerSV(void)
{
	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		float fSetPointValue = hTM4Controller.readSetTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel));

		if (fSetPointValue != TM4_INVALID_SETPOINT_READ) {

			stDataSet.stSetup.SetValueTemp[nChannel-1] = fSetPointValue;

		}
		else {

			return false;

		}

	}

	return true;
}

bool verify_BoilerTemperatureControllerSV(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		if (hTM4Controller.readSetTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel)) == TM4_INVALID_SETPOINT_READ) {

			continue;

		}

		float fSetPointTemperature = constrain(stDataSet.stSetup.SetValueTemp[nChannel-1], MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

		if (fSetPointTemperature != hTM4Controller.getSetTemperatureValue(static_cast<enum CTM4Controller::TM4Channel>(nChannel))) {

			bSuccess &= hTM4Controller.writeSetTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel), fSetPointTemperature) ? false : true;

		}

	}

	return bSuccess;
}

bool setup_BoilerTemperatureControllerSV(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		float fSetPointTemperature = constrain(stDataSet.stSetup.SetValueTemp[nChannel-1], MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

		bSuccess &= hTM4Controller.writeSetTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel), fSetPointTemperature) ? false : true;

	}

	return bSuccess;
}

// to setup power-saving mode
bool setup_BoilerTemperatureControllerSetPointWithValue(const float fSetPoint)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		float fOffset = (nChannel == TM4_GROUPHEAD_CHANNEL) ? -20.0F : 0.0F;
		float fSetTemp = constrain(fSetPoint, MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

		bSuccess &= hTM4Controller.writeSetTemperature(static_cast<enum CTM4Controller::TM4Channel>(nChannel), fSetTemp + fOffset) ? false : true;

	}

	return bSuccess;
}

bool config_BoilerTemperatureControllerSetPointWithChannel(const enum CTM4Controller::TM4Channel eChannel, const float fSetPoint)
{
	float fSetPointTemperature = constrain(fSetPoint, MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);
	
	if (!(hTM4Controller.writeSetTemperature(eChannel, fSetPointTemperature))) {

		if (fSetPointTemperature != stDataSet.stSetup.SetValueTemp[eChannel-1]) {

			stDataSet.stSetup.SetValueTemp[eChannel-1] = fSetPointTemperature;

		}

		save_HeaterSetValueTemperatureWithChannel(eChannel-1, fSetPointTemperature);

	}
	else {

		return false;

	}

	return true;
}

bool config_PreheatBoilerSetPointTemperature(const float fSetPoint)
{
	float fSetPointTemperature = constrain(fSetPoint, MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

	if (!(hTM4Controller.writeSetTemperature(TM4_PREHEAT_CHANNEL, fSetPointTemperature))) {

		// if fSetPoint parameter is passed by the element of stDataSet, its comparison can be skipped.
		if (fSetPointTemperature != stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1]) {

			stDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1] = fSetPointTemperature;

		}

		save_PreheatBoilerSetValueTemperature(fSetPointTemperature);

	}
	else {

		return false;

	}

	return true;
}

bool config_BrewBoilerSetPointTemperature(const float fSetPoint)
{
	float fSetPointTemperature = constrain(fSetPoint, MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

	if (!(hTM4Controller.writeSetTemperature(TM4_BREWING_CHANNEL, fSetPointTemperature))) {

		// if fSetPoint parameter is passed by the element of stDataSet, its comparison can be skipped.
		if (fSetPointTemperature != stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1]) {

			stDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1] = fSetPointTemperature;

		}

		save_BrewBoilerSetValueTemperature(fSetPointTemperature);

	}
	else {

		return false;

	}

	return true;
}

bool config_GroupheadHeaterSetPointTemperature(const float fSetPoint)
{
	float fSetPointTemperature = constrain(fSetPoint, MIN_BOILER_TEMPERATURE, MAX_BOILER_TEMPERATURE);

	if (!(hTM4Controller.writeSetTemperature(TM4_GROUPHEAD_CHANNEL, fSetPointTemperature))) {

		// if fSetPoint parameter is passed by the element of stDataSet, its comparison can be skipped.
		if (fSetPointTemperature != stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1]) {

			stDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1] = fSetPointTemperature;

		}

		save_GroupheadSetValueTemperature(fSetPointTemperature);

	}
	else {

		return false;

	}

	return true;
}

bool sleep_BoilerHeater(void)
{
	if (_bPowerSavingModeValid) {

		if (stDataSet.stSetup.PowerSaveMode == POWER_SAVE_MODE_ON) {

			if (isPowerSavingTime(stCurTimeDate) == true) {

				if (setup_BoilerTemperatureControllerSetPointWithValue(BOILER_TEMPERATURE_UNDER_POWER_SAVING)) {

					return true;

				}
				else {

					setup_BoilerTemperatureControllerSV();

				}

			}

		}
		else {

			_nPowerSavingState = PWS_NOT_WORKING;

			if (hPowerSavingTimer.isRunning()) {

				hPowerSavingTimer.stop();

			}

		}

	}

	return false;
}

bool wakeup_BoilerHeater(void)
{
	bool bWakeup = false;

	if (_bPowerSavingModeValid == true) {

		if (stDataSet.stSetup.PowerSaveMode == POWER_SAVE_MODE_ON) {

			if (isPowerSavingTime(stCurTimeDate) == false) {

				bWakeup = setup_BoilerTemperatureControllerSV();

			}

		}
		else {

			_nPowerSavingState = PWS_NOT_WORKING;

			if (hPowerSavingTimer.isRunning()) {

				hPowerSavingTimer.stop();

			}
			
			bWakeup = setup_BoilerTemperatureControllerSV();

		}

	}

	return bWakeup;
}

bool isPowerSavingTime(const stTimeDate_t& stTimeDate)
{
	static unsigned long _ulPowerSavingDuration = 0UL;		// in seconds

	bool bIsPowerSavingTime;

	switch (_nBrewState) {

		case BREW_STANDBY_STATE:

			bIsPowerSavingTime = false;

			if (_nPowerSavingState == PWS_NOT_WORKING) {

				if (stTimeDate.hour == stDataSet.stSetup.StartHour) {

					if (stTimeDate.minute >= stDataSet.stSetup.StartMinute && stTimeDate.minute <= (stDataSet.stSetup.StartMinute + 9)) {

						unsigned int nHours, nMins;

						verify_PowerSavingDuration(stTimeDate.minute/*stDataSet.stSetup.StartMinute*/, nHours, nMins);

						_ulPowerSavingDuration = (nHours * MAX_MINUTE + nMins) * MAX_SECOND;

						#if defined(_USE_APPLICATION_DEBUG_MSG_)
						cout << F("[NOTICE] Power Saving Duration : ") << nHours << F("h ") << nMins << F("m (") << _ulPowerSavingDuration << F(" secs)") << endl;
						#endif

						bIsPowerSavingTime = true;

						hPowerSavingTimer.reset();
						hPowerSavingTimer.start();

						_nPowerSavingState = PWS_NOW_WORKING;

					}

				}

			}
			else {

				bIsPowerSavingTime = true;

				if (hPowerSavingTimer.isRunning()) {

					if (hPowerSavingTimer.elapsed() >= _ulPowerSavingDuration) {

						hPowerSavingTimer.stop();

						bIsPowerSavingTime = false;

					}

				}
				else {

					bIsPowerSavingTime = false;

				}

				if (bIsPowerSavingTime == false) {

					_ulPowerSavingDuration = 0UL;

					_nPowerSavingState = PWS_NOT_WORKING;

				}
				else {

					switch (_nPowerSavingState) {

						case PWS_RESUMED:
							_nPowerSavingState = PWS_NOW_WORKING;
						break;

						case PWS_PAUSED:
							bIsPowerSavingTime = false;
						break;

					}

				}

			}

		break;

		case POWER_SAVING_STATE:
		default:

			bIsPowerSavingTime = false;

			if (hPowerSavingTimer.isRunning()) {

				if (hPowerSavingTimer.elapsed() >= _ulPowerSavingDuration) {

					hPowerSavingTimer.stop();

				}
				else {

					bIsPowerSavingTime = true;

				}

			}

			if (bIsPowerSavingTime == false) {

				_ulPowerSavingDuration = 0UL;

				_nPowerSavingState = PWS_NOT_WORKING;

			}
			else {

				if (_nPowerSavingState == PWS_PAUSED) {

					bIsPowerSavingTime = false;

				}

			}

		break;

	}

	return bIsPowerSavingTime;
}

void verify_PowerSavingDuration(const unsigned char nCurrentMinute, unsigned int& nHours, unsigned int& nMins)
{
	unsigned int nDurationTime;
	unsigned long ulStartSavingTime, ulEndSavingTime;

	ulStartSavingTime = stDataSet.stSetup.StartHour * 100 + nCurrentMinute;

	if (stDataSet.stSetup.StartHour < stDataSet.stSetup.EndHour) {

		ulEndSavingTime = stDataSet.stSetup.EndHour * 100 + stDataSet.stSetup.EndMinute;

	}
	else if (stDataSet.stSetup.StartHour == stDataSet.stSetup.EndHour) {

		if (stDataSet.stSetup.StartMinute < stDataSet.stSetup.EndMinute) {

			ulEndSavingTime = stDataSet.stSetup.EndHour * 100 + stDataSet.stSetup.EndMinute;
			
		}
		else {

			ulEndSavingTime = (stDataSet.stSetup.EndHour + MAX_HOUR) * 100 + stDataSet.stSetup.EndMinute;

		}

	}
	else { // if (stDataSet.stSetup.StartHour > stDataSet.stSetup.EndHour)

		ulEndSavingTime = (stDataSet.stSetup.EndHour + MAX_HOUR) * 100 + stDataSet.stSetup.EndMinute;

	}

	nDurationTime = ulEndSavingTime - ulStartSavingTime;
	nHours = int(nDurationTime / 100);
	nMins = nDurationTime - (nHours * 100);

	if ((nCurrentMinute > 40 && stDataSet.stSetup.EndMinute == 0) || nMins >= MAX_MINUTE) {

		nMins -= 40;

	}
}

bool suspend_BoilerHeater(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		bSuccess &= hTM4Controller.disableOutputControl(static_cast<enum CTM4Controller::TM4Channel>(nChannel)) ? false : true;

	}

	_bHeatingSuspendedAll = true;

	stop_UnderheatingObservationTimer();

	return bSuccess;
}

bool resume_BoilerHeater(void)
{
	bool bSuccess = true;

	for (int nChannel = TM4_PREHEAT_CHANNEL; nChannel <= TM4_GROUPHEAD_CHANNEL; nChannel++) {

		bSuccess &= hTM4Controller.enableOutputControl(static_cast<enum CTM4Controller::TM4Channel>(nChannel)) ? false : true;

	}

	_bHeatingSuspendedAll = false;

	start_UnderheatingObservationTimer();

	return bSuccess;
}


// Sensor Measurement (Observation)
void read_TemperatureSensorAll(void)
{
#if !defined(_AUTONICS_DATA_ACQ_CLIENT_CONNECTED_)
	static unsigned long _lPollingTM4Time = 0UL;

	if (millis() - _lPollingTM4Time >= TM4CONTROLLER_TEMP_MEASUREMENT_PERIOD) {

		_lPollingTM4Time = millis();

		PollingTM4Temperature_Callback();

	}
#endif
}

void read_TM4ControllerObservation(void)
{
#if !defined(_AUTONICS_DATA_ACQ_CLIENT_CONNECTED_)
	static unsigned long _lPollingTM4Temp = 0UL;

	if (millis() - _lPollingTM4Temp >= TM4CONTROLLER_TEMP_MEASUREMENT_PERIOD) {

		_lPollingTM4Temp = millis();

		PollingTM4Temperature_Callback();

	}

	if (get_TextLCDView() == MenupageView) {

		eMenuPage eCurrentMenupage = get_TextLCDMenupage();

		if (eCurrentMenupage >= Menupage5 && eCurrentMenupage <= Menupage7) {

			static unsigned long _lPollingTM4Heat = 0UL;

			if (millis() - _lPollingTM4Heat >= TM4CONTROLLER_MV_MEASUREMENT_PERIOD) {

				CTM4Controller::TM4Channel eChannel;

				switch (eCurrentMenupage) {
					case Menupage5:
						eChannel = CTM4Controller::TM4CH3;
						break;
					case Menupage6:
						eChannel = CTM4Controller::TM4CH2;
						break;
					case Menupage7:
					default:
						eChannel = CTM4Controller::TM4CH1;
						break;
				}

				_lPollingTM4Heat = millis();

				measure_TM4ControllerHeatingMVWithChannel(eChannel);

			}		

		}

	}
#endif
}

void measure_PressureSensorAll(void)
{
#if BREW_PRESSURE_SENSOR_ATTACHED
	hPressureBrewBoiler.measurePressure();
#endif

#if PUMP_PRESSURE_SENSOR_ATTACHED
	hPressureFeed.measurePressure();
#endif

#if AUX_CURRENT_SENSOR_ATTACHED
	hPressureAUX.measurePressure();
#endif
}

void update_SensorObservationAll(void)
{
	update_PressureSensorObservation();
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	update_AnalogInputObservation();
#endif
	update_TemperatureSensorObservation();
	update_HeatingMVObservation();
}

void clear_SensorObservationAll(void)
{
	clear_PressureSensorObservation();
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	clear_AnalogInputObservation();
#endif
	clear_TemperatureSensorObservation();
}

void clear_PressureSensorObservation(void)
{
	stDataSet.stAcquisition.BrewPress = 0.0f;
	stDataSet.stAcquisition.PumpPress = 0.0f;
#if AUX_CURRENT_SENSOR_ATTACHED
	stDataSet.stAcquisition.AuxPress = 0.0f;
#endif
}

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void init_AnalogInputAveragedObservation(void)
{
	stDataSet.stAcquisition.AvgAnalogInputVoltage = calculate_AveragedAnalogInputVoltage(true);
}

void clear_AnalogInputObservation(void)
{
	stDataSet.stAcquisition.AnalogInputVoltage = 0.0f;

	stDataSet.stAcquisition.AvgAnalogInputVoltage = calculate_AveragedAnalogInputVoltage(true);
}
#endif

void clear_TemperatureSensorObservation(void)
{
	stDataSet.stAcquisition.PreheatTemp = 0.0f;
	stDataSet.stAcquisition.BrewTemp = 0.0f;
	stDataSet.stAcquisition.GroupTemp = 0.0f;
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewWaterTemp = 0.0f;
#endif
}

void update_PressureSensorObservation(void)
{
	if (hEspressoMachine.getBrewState() == CEspresso::BREW_STOP) {

		if (hPressureSensorThread.isTriggered() == true) {

			#if BREW_PRESSURE_SENSOR_ATTACHED
			stDataSet.stAcquisition.BrewPress = get_BrewBoilerPressureValue();
			#endif

			#if PUMP_PRESSURE_SENSOR_ATTACHED
			stDataSet.stAcquisition.PumpPress = get_PumpPressureValue();
			#endif

			#if AUX_CURRENT_SENSOR_ATTACHED
			stDataSet.stAcquisition.AuxPress = get_AuxPressureValue();
			#endif

		}

	}
}

void update_PressureSensorAnalogDigit(const bool bImmediately)
{
	static unsigned long _ulADCDigitGetTime = 0UL;

	bool bEnableToGet = false;

	if (bImmediately) {

		bEnableToGet = true;

	}
	else {

		if (millis() - _ulADCDigitGetTime >= 500UL) {

			bEnableToGet = true;

		}

	}

	if (bEnableToGet) {

		#if PUMP_PRESSURE_SENSOR_ATTACHED
		stDataSet.stSetup.SensorADCDigit = stDataSet.stSetup.SetSensorChannel ? hPressureFeed.getAnalogRawValue() : hPressureBrewBoiler.getAnalogRawValue();
		#else
		stDataSet.stSetup.SensorADCDigit = stDataSet.stSetup.SetSensorChannel ? 0 : hPressureBrewBoiler.getAnalogRawValue();
		#endif

		_ulADCDigitGetTime = millis();

	}
}

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void update_AnalogInputObservation(void)
{
	if (hAnalogSensorThread.isTriggered() == true) {

		stDataSet.stAcquisition.AnalogInputVoltage = get_AnalogSensorValueInVoltage();
		stDataSet.stAcquisition.AvgAnalogInputVoltage = calculate_AveragedAnalogInputVoltage(false);

	}
}
#endif

void update_TemperatureSensorObservation(void)
{
#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.PreheatTemp = hTM4Controller.getPresentTemperatureValue(TM4_PREHEAT_CHANNEL);
#endif
#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewTemp = hTM4Controller.getPresentTemperatureValue(TM4_BREWING_CHANNEL);
#endif
#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.GroupTemp = hTM4Controller.getPresentTemperatureValue(TM4_GROUPHEAD_CHANNEL);
#endif
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	stDataSet.stAcquisition.BrewWaterTemp = hTM4Controller.getPresentTemperatureValue(TM4_BREWWATER_CHANNEL);
#endif
}

void update_HeatingMVObservation(void)
{
	stSetupDataSet_t stSetupDataSet = stDataSet.stSetup;

	stDataSet.stAcquisition.PreheatMV = hTM4Controller.getPresentHeatingMV(TM4_PREHEAT_CHANNEL);
	stDataSet.stAcquisition.BrewMV = hTM4Controller.getPresentHeatingMV(TM4_BREWING_CHANNEL);
	stDataSet.stAcquisition.GroupMV = hTM4Controller.getPresentHeatingMV(TM4_GROUPHEAD_CHANNEL);

	// stDataSet.stAcquisition.PreheatMV = (stDataSet.stAcquisition.PreheatMV / stSetupDataSet.MVHighLimit[TM4_PREHEAT_CHANNEL-1]) * 100;
	// stDataSet.stAcquisition.BrewMV = (stDataSet.stAcquisition.BrewMV / stSetupDataSet.MVHighLimit[TM4_BREWING_CHANNEL-1]) * 100;
	// stDataSet.stAcquisition.GroupMV = (stDataSet.stAcquisition.GroupMV / stSetupDataSet.MVHighLimit[TM4_GROUPHEAD_CHANNEL-1]) * 100;
}

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
float calculate_AveragedAnalogInputVoltage(const bool bReset)
{
	static unsigned long _nNum_of_samples = 0UL;
	static float _fSum_of_samples = 0.0F;
	static float _fAvgAnalogInputVoltage = 0.0F;

	if (bReset) {

		_nNum_of_samples = 0UL;
		_fSum_of_samples = 0.0F;
		_fAvgAnalogInputVoltage = stDataSet.stAcquisition.AnalogInputVoltage;

	}
	else {

		_fSum_of_samples += stDataSet.stAcquisition.AnalogInputVoltage;

		_fAvgAnalogInputVoltage = _fSum_of_samples / ++_nNum_of_samples;

	}

	return _fAvgAnalogInputVoltage;
}
#endif

void observe_SystemStatusOnStandby(void)
{
	observe_SystemControllerOVL(); // included codes to disable interrupt inside the function

#if PUMP_PRESSURE_SENSOR_ATTACHED
	observe_WaterPumpPressure();
#endif

	observe_BrewBoilerPressureStatus();

	observe_TemperatureSensorStatus();

	observe_BoilerHeatingStatus();

	observe_MotorPumpStatus(); // included codes to disable interrupt inside the function

#if AUTOMATION_SENSOR_ATTACHED
	observe_OpticalFiberAmp();
#endif

	observe_TotalBrewWaterQuantity();

	#if !defined(_USE_APPLICATION_DEBUG_MSG_) && defined(_USE_BREW_PROFILE_ACQUISITION_)
	print_SensorObservation();
	#endif
}

void observe_SystemStatusOnBrewing(void)
{
	observe_SystemControllerOVL(); // included codes to disable interrupt inside the function

#if PUMP_PRESSURE_SENSOR_ATTACHED
	observe_WaterPumpPressure();
#endif

	observe_BrewBoilerPressureStatus();

	observe_TemperatureSensorStatus();

	observe_BoilerHeatingStatus();

	observe_MotorPumpStatus(); // included codes to disable interrupt inside the function

	observe_FlowrateExceeded();
}

#if AUTOMATION_SENSOR_ATTACHED
void observe_SystemStatusOnFlushing(void)
{
	observe_SystemControllerOVL(); // included codes to disable interrupt inside the function

#if PUMP_PRESSURE_SENSOR_ATTACHED
	observe_WaterPumpPressure();
#endif

	observe_BrewBoilerPressureStatus();

	observe_TemperatureSensorStatus();

	observe_BoilerHeatingStatus();

	observe_MotorPumpStatus(); // included codes to disable interrupt inside the function

	// observe_OpticalFiberAmp();
}
#endif

void observe_SystemControllerOVL(void)
{
	stError.bIsSystemOverloaded = isThermalOverload();
}

#if PUMP_PRESSURE_SENSOR_ATTACHED
void observe_WaterPumpPressure(void)
{
	static bool _bLowInletPressureDetected = false;
	static unsigned long _ulTimeOnLowInletPressure = 0UL;

	if (stDataSet.stSetup.LowInletAlarm == LOW_INLET_ALARM_ON && stDataSet.stSetup.MaintenanceMode == MAINTENANCE_MODE_OFF) {

		if (stDataSet.stAcquisition.PumpPress <= stDataSet.stSetup.LowInletPress) {

			if (!_bLowInletPressureDetected) {

				_bLowInletPressureDetected = true;

				_ulTimeOnLowInletPressure = millis();

			}

			if (!stError.bIsLowInletPressure) {

				if (millis() - _ulTimeOnLowInletPressure >= TIME_TO_DECIDE_LOW_INLET_PRESSURE) {

					stError.bIsLowInletPressure = true;

				}

			}

			return;

		}

	}

	_bLowInletPressureDetected = false;

	stError.bIsLowInletPressure = false;
}
#endif

void observe_BrewBoilerPressureStatus(void)
{
	static bool _bOverBrewPressureDetected = false;
	static bool _bLowBrewPressureDetected = false;

	static unsigned long _ulTimeOnOverBrewPressure = 0UL;
	static unsigned long _ulTimeOnLowBrewPressure = 0UL;

	if (stDataSet.stSetup.MaintenanceMode == MAINTENANCE_MODE_OFF && stDataSet.stSetup.MotorRun == MOTOR_RUN_OFF) {

		// Check if the Pressure of the Brew-Boiler is Over
		if (stDataSet.stAcquisition.BrewPress > MAX_BREW_PRESSURE_UNDER_STANDBY) {

			if (!_bOverBrewPressureDetected) {

				_bOverBrewPressureDetected = true;

				_ulTimeOnOverBrewPressure = millis();

			}

			if (!stError.bIsBrewPressureOver) {

				if (millis() - _ulTimeOnOverBrewPressure >= TIME_TO_DECIDE_OVER_BREW_PRESSURE) {

					stError.bIsBrewPressureOver = true;

				}

			}

			return;

		}
		else {

			_bOverBrewPressureDetected = false;

			stError.bIsBrewPressureOver = false;

		}

		// Check if the Pressure of the Brew-Boiler is Under
		if (stDataSet.stSetup.CleaningMode == CLEANING_MODE_OFF) {

			if (stDataSet.stAcquisition.BrewPress < MIN_BREW_PRESSURE_UNDER_STANDBY) {

				if (!_bLowBrewPressureDetected) {

					_bLowBrewPressureDetected = true;

					_ulTimeOnLowBrewPressure = millis();

				}

				if (!stError.bIsBrewPressureUnder) {

					if (millis() - _ulTimeOnLowBrewPressure >= TIME_TO_DECIDE_LOW_BREW_PRESSURE) {

						stError.bIsBrewPressureUnder = true;

					}

				}

				return;

			}

		}

	}

	_bOverBrewPressureDetected = false;
	_bLowBrewPressureDetected = false;

	stError.bIsBrewPressureOver = false;
	stError.bIsBrewPressureUnder = false;
}

void observe_BoilerHeatingStatus(void)
{
	stError.bIsGroupheadOverheated = stDataSet.stAcquisition.GroupTemp >= BOILER_OVERHEATING_TEMPERATURE ? true : false;
	stError.bIsBrewBoilerOverheated = stDataSet.stAcquisition.BrewTemp >= BOILER_OVERHEATING_TEMPERATURE ? true : false;
	stError.bIsPreheatBoilerOverheated = stDataSet.stAcquisition.PreheatTemp >= BOILER_OVERHEATING_TEMPERATURE ? true : false;

	if (isUnderheatingObservationEnabled()) {

		stError.bIsGroupheadUnderheated = stDataSet.stAcquisition.GroupTemp < BOILER_UNDERHEATING_TEMPERATURE ? true : false;
		stError.bIsBrewBoilerUnderheated = stDataSet.stAcquisition.BrewTemp < BOILER_UNDERHEATING_TEMPERATURE ? true : false;
		stError.bIsPreheatBoilerUnderheated = stDataSet.stAcquisition.PreheatTemp < BOILER_UNDERHEATING_TEMPERATURE ? true : false;

	}
}

void observe_TemperatureSensorStatus(void)
{
	if (isTM4TemperatureSensorAlarm()) {

		stError.bIsTemperatureSensorError = true;

	}
	else {

		stError.bIsTemperatureSensorError = false;

	}
}

void observe_MotorPumpStatus(void)
{
	if (isMotorAlarm()) {

		stError.bIsMotorAlarmReceived = true;

	}
	else {

		stError.bIsMotorAlarmReceived = false;

	}
}

void observe_FlowmeterFailure(void)
{
	if (stDataSet.stAcquisition.FlowCount == FLOWMETER_FAILURE_COUNT_CONDITION) {

		stError.bIsFlowmeterFailed = true;

	} 
	else {

		stError.bIsFlowmeterFailed = false;

	}

	stError.bIsFlowrateExceeded = false;
}

void observe_FlowrateExceeded(void)
{
	if (stDataSet.stAcquisition.FlowRate > FLOWRATE_EXCEED_CONDITION) {

		stError.bIsFlowrateExceeded = true;

	}
	else {

		stError.bIsFlowrateExceeded = false;

	}
}

#if AUTOMATION_SENSOR_ATTACHED
void observe_OpticalFiberAmp(void)
{
	static bool _bFirstUnstabilityObserved = false;
	static unsigned long _ulTimeToUnstability = 0UL;

	if (get_OpticalFiberDiagnosis()/*hOpticalFiber.isUnstable()*/) {

		if (stError.bIsOpticalFiberUnstable == false) {

			if (_bFirstUnstabilityObserved == false) {

				_bFirstUnstabilityObserved = true;

				_ulTimeToUnstability = millis();

			}
			else {

				if (millis() - _ulTimeToUnstability >= MAX_TIME_TO_DETECT_OPTICAL_SIGNAL_UNSTABLE) {

					stError.bIsOpticalFiberUnstable = true;

				}

			}

		}

	}
	else {

		_bFirstUnstabilityObserved = false;

		stError.bIsOpticalFiberUnstable = false;

	}
}
#endif

void observe_TotalBrewWaterQuantity(void)
{
	bool isCountOverflowed = (stDataSet.stMaintenance.Overflowed & 0x0F) ? true : false;

	unsigned long ulRangeOfMaintenanceNotice = stDataSet.stMaintenance.TotFlowQuantity % BREW_WATER_QTY_REQUIRED_MAINTENANCE;

	if (ulRangeOfMaintenanceNotice < BREW_WATER_QTY_MAINTENANCE_NOTICE_RANGE) {

		if (stDataSet.stMaintenance.TotFlowQuantity >= BREW_WATER_QTY_MAINTENANCE_NOTICE_RANGE) {

			stError.bIsMaintenanceRequired = true;

		}
		else {

			if (isCountOverflowed == true) {

				stError.bIsMaintenanceRequired = true;

			}

		}

	}
	else {

		stError.bIsMaintenanceRequired = false;

	}
}

void start_UnderheatingObservationTimer(void)
{
	hUnderheatingObservationTimer.reset();
	hUnderheatingObservationTimer.start();
}

void stop_UnderheatingObservationTimer(void)
{
	if (hUnderheatingObservationTimer.isRunning()) {

		hUnderheatingObservationTimer.stop();

	}
}

bool isUnderheatingObservationEnabled(void)
{
	if (hUnderheatingObservationTimer.isRunning()) {

		if (hUnderheatingObservationTimer.elapsed() >= BOILER_UNDERHEATING_OBSERVATION_START_TIME) {

			hUnderheatingObservationTimer.stop();

		}
		else {

			return false;

		}

	}

	return (!_bHeatingSuspendedAll);
}


// Real Time Clock Utilities
void clear_CurrentTimeDate(void)
{
	memset(&stCurTimeDate, 0, sizeof(stTimeDate_t));
}

void copy_TimeDate2SetupParams(void)
{
	// noInterrupts();

	stDataSet.stSetup.Year = stCurTimeDate.year;
	stDataSet.stSetup.Month = stCurTimeDate.month;
	stDataSet.stSetup.Day = stCurTimeDate.day;
	stDataSet.stSetup.Weekday = stCurTimeDate.weekday;
	stDataSet.stSetup.Hour = stCurTimeDate.hour;
	stDataSet.stSetup.Minute = stCurTimeDate.minute;
	stDataSet.stSetup.Second = stCurTimeDate.second;

	// interrupts();
}

void UpdatingCurrentTimeDate_Callback(void)
{
	hRTClock.getTimeDate(stCurTimeDate);
}


// Text LCD Display
void display_TextLCDOnUserInterface(void)
{
	invalidate_TextLCD(stDataSet, stCurTimeDate);
}


// RGB LED Visualization
void draw_LEDPalettOnBrewStandby(void)
{
	CRGB rgbFgColor;

	switch (stDataSet.ProgramNum) {
		case PROGRAM_SET_ONE:
			rgbFgColor = RGB_PROGRAM_ONE;
		break;

		case PROGRAM_SET_TWO:
			rgbFgColor = RGB_PROGRAM_TWO;
		break;

		case MANUAL_SET_ONE:
			rgbFgColor = RGB_MANUAL_ONE;
		break;
	}

	prepare_LEDPalettOnBrewStandby(rgbFgColor, stDataSet.stSetup.BrewSetupMode);
}

void draw_LEDPalettOnBrewStop(void)
{
	CRGB rgbFgColor;

	switch (stDataSet.ProgramNum) {
		case PROGRAM_SET_ONE:
			rgbFgColor = RGB_PROGRAM_ONE;
		break;

		case PROGRAM_SET_TWO:
			rgbFgColor = RGB_PROGRAM_TWO;
		break;

		case MANUAL_SET_ONE:
			rgbFgColor = RGB_MANUAL_ONE;
		break;
	}

	bool bIsTimelapseMode = stDataSet.stSetup.LedPixelMode[stDataSet.ProgramNum] == LEDPIXEL_TIMELAPSE_MODE;
	bool bIsProgressModeException = stDataSet.stSetup.LedPixelMode[stDataSet.ProgramNum] == LEDPIXEL_PROGRESS_MODE && 
									hEspressoMachine.getBrewMode() == CEspresso::SETUP_MODE;

	if (bIsTimelapseMode || bIsProgressModeException) {

		prepare_LEDPalettOnTimelapseHolding(rgbFgColor, stDataSet.stSetup.BrewSetupMode);

	}
	else {

		prepare_LEDPalettOnBrewStandby(rgbFgColor, stDataSet.stSetup.BrewSetupMode);

	}
}

void draw_LEDPalettOnBrewing(void)
{
	CRGB rgbFgColor = RGB_BREW_INJECTION;

	if (hEspressoMachine.getBrewState() == CEspresso::BREW_EXTRACTION) {

		rgbFgColor = RGB_BREW_EXTRACTION;

	}

	prepare_LEDPalettOnBrewing(rgbFgColor);
}

void draw_LEDPalettOnSetupEntry(void)
{
	CRGB rgbFgColor = stDataSet.ProgramNum == PROGRAM_SET_ONE ? RGB_PROGRAM_ONE : RGB_PROGRAM_TWO;

	prepare_LEDPalettOnSetupEntry(rgbFgColor);
}

void draw_LEDPalettOnSetupExit(void)
{
	CRGB rgbFgColor;

	switch (stDataSet.ProgramNum) {
		case PROGRAM_SET_ONE:
			rgbFgColor = RGB_PROGRAM_ONE;
		break;

		case PROGRAM_SET_TWO:
			rgbFgColor = RGB_PROGRAM_TWO;
		break;

		case MANUAL_SET_ONE:
			rgbFgColor = RGB_MANUAL_ONE;
		break;
	}

	prepare_LEDPalettOnSetupExit(rgbFgColor);
}

void draw_LEDPalettOnSleep(void)
{
	CRGB rgbFgColor = RGB_BREATHING_SLEEP;

	prepare_LEDPalettOnSleep(rgbFgColor);
}

void draw_LEDPalettOnWake(void)
{
	// hPixelLED.setBrightness(RGB_BRIGHTNESS);

	// delay(500UL);

	draw_LEDPalettOnBrewStandby();
}

void draw_LEDPalettOnCleaning(void)
{
	CRGB rgbFgColor = RGB_CLEANING_MACHINE;

	prepare_LEDPalettOnCleaning(rgbFgColor);
}

void draw_LEDPalettOnException(void)
{
	CRGB rgbFgColor = RGB_SYSTEM_ERROR_BLINK_FG;

	prepare_LEDPalettOnException(rgbFgColor);
}

void draw_LEDPalettOnLongPushing(void)
{
	CRGB rgbFgColor = RGB_LONG_PUSH_PROGRESS_FG;

	prepare_LEDPalettOnLongPushing(rgbFgColor);
}

#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
void draw_LEDPalettOnVLongPushing(void)
{
	static unsigned long _ulLongPushingTime = 0UL;

	if (_bResetTimerToPredictVLongPushingEvent == true) {

		_bResetTimerToPredictVLongPushingEvent = false;

		_ulLongPushingTime = millis();

	}
	else {

		if (millis() - _ulLongPushingTime >= TIME_TO_DETECT_VLONG_PUSHING_EVENT) {

			_bPredictVLongPushingEvent = false;

			prepare_LEDPalettOnVLongPushing(RGB_VLONG_PUSH_FG);

		}

	}
}
#endif

void draw_LEDPalettOnAirVentilation(void)
{
	CRGB rgbFgColor = RGB_AIR_VENTILATION_STAGE_FG;

	prepare_LEDPalettOnAirVentilation(rgbFgColor);
}

void draw_LEDPalettOnActivation(void)
{
	CRGB rgbFgColor = RGB_ACTIVATION_PROGRESS_FG;

	prepare_LEDPalettOnLongPushing(rgbFgColor);
}

void draw_LEDPixelOnUserInterface(void)
{
	if (hEspressoMachine.getBrewState() != CEspresso::BREW_STOP) {

		update_LEDPalettOnBrewingProgress();

	}
	else {

		if (_nBrewState == CLEANING_MACHINE_START_STATE) {

			update_LEDPalettOnCleaningProgress();

		}
		// else {

			// TODO: Specific Progress

		// }

	}

	hPixelLED.drawPalettMode();
}

void update_LEDPalettOnBrewStandby(void)
{
	CRGB rgbFgColor;

	switch (stDataSet.ProgramNum) {
		case PROGRAM_SET_ONE:
			rgbFgColor = RGB_PROGRAM_ONE;
		break;

		case PROGRAM_SET_TWO:
			rgbFgColor = RGB_PROGRAM_TWO;
		break;

		case MANUAL_SET_ONE:
			rgbFgColor = RGB_MANUAL_ONE;
		break;
	}

	hPixelLED.setForegroundColor(rgbFgColor, UPDATE_LED_PALETT);
}

void update_LEDPalettOnBrewing(void)
{
	if (hEspressoMachine.isChanged() == true) {

		CRGB rgbFgColor = hEspressoMachine.getBrewState() == CEspresso::BREW_INJECTION ? RGB_BREW_INJECTION : RGB_BREW_EXTRACTION;

		hPixelLED.setForegroundColor(rgbFgColor, NOT_UPDATE_LED_PALETT/*UPDATE_LED_PALETT*/);

		// hPixelLED.setProgressColorChanged(true);

	}
}

void update_LEDPalettOnBrewingProgress(void)
{
	unsigned int nCurrProgressData, nMaxProgressData;
	unsigned int nFirstDigitData, nSecondDigitData;

	stAcqDataSet_t* pstAcquisition = &(stDataSet.stAcquisition);

	if (stDataSet.stSetup.LedPixelMode[stDataSet.ProgramNum] == LEDPIXEL_PROGRESS_MODE) {
		
		// 1) DEFAULT PROGRESS MODE : Water Flow Count or Brew Time
		if (hEspressoMachine.getBrewMode() == CEspresso::BREW_MODE) {

			if (hEspressoMachine.getBrewBase() == CEspresso::BREW_FLOWCNT) {

				nCurrProgressData = pstAcquisition->FlowCount;
				nMaxProgressData = hEspressoMachine.getMaxBrewingSet(CEspresso::BREW_FLOWCNT);

			}
			else {

				nCurrProgressData = static_cast<unsigned int>(pstAcquisition->ShotTime / 1000.0f);
				nMaxProgressData = hEspressoMachine.getMaxBrewingSet(CEspresso::BREW_FLOWCNT); 		// Notice : not BREW_TIMECNT

			}

			hPixelLED.makeFixedProgressPalett(nCurrProgressData, nMaxProgressData);

		}
		// 2) PROGRESS MODE : Brew Time (60 sec / Max 360 sec)
		else {

			nFirstDigitData = static_cast<unsigned int>(floorf(pstAcquisition->ShotTime / 1000.0f));
			nSecondDigitData = static_cast<unsigned int>(floorf(nFirstDigitData / 10.0f));

			hPixelLED.makeTimelapsePalett(nFirstDigitData % LEDPIXEL_TIMELAPSE_FIRST_DIGIT_NUM, nSecondDigitData % LEDPIXEL_TIMELAPSE_SECOND_DIGIT_NUM);

		}

	}
	// 2) PROGRESS MODE : Brew Time (60 sec / Max 360 sec)
	else if (stDataSet.stSetup.LedPixelMode[stDataSet.ProgramNum] == LEDPIXEL_TIMELAPSE_MODE) {

		nFirstDigitData = static_cast<unsigned int>(floorf(pstAcquisition->ShotTime / 1000.0f));
		nSecondDigitData = static_cast<unsigned int>(floorf(nFirstDigitData / 10.0f));

		hPixelLED.makeTimelapsePalett(nFirstDigitData % LEDPIXEL_TIMELAPSE_FIRST_DIGIT_NUM, nSecondDigitData % LEDPIXEL_TIMELAPSE_SECOND_DIGIT_NUM);

	}
	else {

		switch (stDataSet.stSetup.LedPixelMode[stDataSet.ProgramNum]) {

			// 3) PROGRESS MODE : Water Flow Rate
			case LEDPIXEL_FLOWRATE_MODE:
				// nCurrProgressData = static_cast<unsigned int>(floorf(pstAcquisition->FlowRate));
				nCurrProgressData = static_cast<unsigned int>(roundf(pstAcquisition->FlowRate));
				nMaxProgressData = FLOWRATE_VALUE_LIMIT;
			break;

			// 4) PROGRESS MODE : Brew Pressure
			case LEDPIXEL_BREWPRESS_MODE:
				// nCurrProgressData = static_cast<unsigned int>(floorf(pstAcquisition->BrewPress));
				nCurrProgressData = static_cast<unsigned int>(roundf(pstAcquisition->BrewPress));
				nMaxProgressData = PRESSURE_VALUE_LIMIT;
			break;

			// 5) PROGRESS MODE : Pump Pressure
			#if PUMP_PRESSURE_SENSOR_ATTACHED
			case LEDPIXEL_PUMPPRESS_MODE:
				// nCurrProgressData = static_cast<unsigned int>(floorf(pstAcquisition->PumpPress));
				nCurrProgressData = static_cast<unsigned int>(roundf(pstAcquisition->PumpPress));
				nMaxProgressData = PRESSURE_VALUE_LIMIT;
			break;
			#endif

		}

		nCurrProgressData = constrain(nCurrProgressData, 0, nMaxProgressData);

		hPixelLED.makeVariousProgressPalett(nCurrProgressData, nMaxProgressData);

	}
}

void update_LEDPalettOnCleaningProgress(void)
{
	unsigned int nCurrProgressData = _nCleaningProgress;
	unsigned int nMaxProgressData = NUM_OF_PIXELS;

	hPixelLED.makeFixedProgressPalett(nCurrProgressData, nMaxProgressData);
}


// System EEPROM Utilities
void save_FactoryResetFlag(const unsigned char nFactoryResetFlag)
{
	write_Byte8ParameterEEPROM(FACTORY_RESET_FLAG_ADDR, nFactoryResetFlag);
}

unsigned char load_FactoryResetFlag(void)
{
	return read_Byte8ParameterEEPROM(FACTORY_RESET_FLAG_ADDR);
}

void save_EEPROMBackupAddressPointerTable(void)
{
	for (int nIndex = 0; nIndex < MAX_BACKUP_PARAMETERS; nIndex++) {

		write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);

	}
}

void load_EEPROMBackupAddressPointerTable(void)
{
	for (int nIndex = 0; nIndex < MAX_BACKUP_PARAMETERS; nIndex++) {

		eeprom_backup_addr_pointer[nIndex] = read_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex));

	}	
}

void save_HardwareVersion(const unsigned char nVersion)
{
	write_Byte8ParameterEEPROM(HARDWARE_VERSION_ADDR, nVersion);
}

unsigned char load_HardwareVersion(void)
{
	return read_Byte8ParameterEEPROM(HARDWARE_VERSION_ADDR);
}

void save_FirmwareVersion(const unsigned char nVersion)
{
	write_Byte8ParameterEEPROM(FIRMWARE_VERSION_ADDR, nVersion);
}

unsigned char load_FirmwareVersion(void)
{
	return read_Byte8ParameterEEPROM(FIRMWARE_VERSION_ADDR);
}

void save_DefaultProgramNum(const unsigned char nProgramNum)
{
	write_Byte8ParameterEEPROM(DEFAULT_PROGRAM_NUM_ADDR, nProgramNum);
}

void load_DefaultProgramNum(stDataSet_t& stDataSet)
{
	stDataSet.ProgramNum = read_Byte8ParameterEEPROM(DEFAULT_PROGRAM_NUM_ADDR);
}

unsigned char load_DefaultProgramNum(void)
{
	return read_Byte8ParameterEEPROM(DEFAULT_PROGRAM_NUM_ADDR);
}

void clear_SystemErrorHistory(stMaintenanceDataSet_t& stMaintenance)
{
	stMaintenance.ErrorHistoryIndex = 0x01;
	stMaintenance.ErrorAddressIndex = 0x00;
	stMaintenance.ErrorCode = DEF_ERROR_NONE;

	write_Byte8ParameterEEPROM(ERROR_NEXT_INDEX_ADDR,	0x01);
	write_Byte8ParameterEEPROM(ERROR_NEXT_POINTER_ADDR, 0x00);

	for (int nErrAddrIdx = 0; nErrAddrIdx < ERROR_CODE_HISTORY_SIZE; nErrAddrIdx += 0x03) {

		write_Byte8ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrAddrIdx,		  0x00);
		write_Word16ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrAddrIdx+0x01, DEF_ERROR_NONE);

	}

	stMaintenance.RecentErrorCode = (get_SystemErrorCode(true, 0) & 0xFFFF);
}

void restore_SystemErrorIndex(stMaintenanceDataSet_t& stMaintenance)
{
	stMaintenance.ErrorHistoryIndex = read_Byte8ParameterEEPROM(ERROR_NEXT_INDEX_ADDR);
	stMaintenance.ErrorAddressIndex = read_Byte8ParameterEEPROM(ERROR_NEXT_POINTER_ADDR);
}

unsigned long get_SystemErrorCode(const bool bReload, const unsigned char nHistIndex)
{
	static unsigned long _ulErrorHistory[ERROR_HISTORY_SIZE];

	unsigned char nHistoryIndex;

	if (bReload) {

		memset(_ulErrorHistory, DEF_ERROR_NONE, sizeof(_ulErrorHistory));

		load_SystemErrorHistory(_ulErrorHistory);

	}

	nHistoryIndex = constrain(nHistIndex, 0, ERROR_HISTORY_SIZE-1);

	return _ulErrorHistory[nHistoryIndex];
}

int compare_ErrorHistoryIndex(const void *firstElem, const void *secondElem)
{
	int nFirst, nSecond, nResult;

	nFirst = (*(unsigned long*)firstElem) >> 16;
	nSecond = (*(unsigned long*)secondElem) >> 16;

	if (nFirst > nSecond)

		nResult = -1;			// -1 : Descending Order, 1 : Ascending Order

	else if (nFirst < nSecond)

		nResult = 1;			// 1 : Descending Order, -1 : Ascending Order

	else

		nResult = 0;

	return nResult;
}

void save_SystemErrorCode(stMaintenanceDataSet_t& stMaintenance)
{
	unsigned char nErrorHistoryIndex = stMaintenance.ErrorHistoryIndex;
	unsigned char nErrorAddressIndex = stMaintenance.ErrorAddressIndex;
	unsigned int nErrorCode = stMaintenance.ErrorCode;

	write_Byte8ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrorAddressIndex,		 nErrorHistoryIndex);
	write_Word16ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrorAddressIndex+0x01, nErrorCode);

	nErrorHistoryIndex += 1;
	nErrorAddressIndex += 3;

	if (nErrorAddressIndex >= ERROR_CODE_HISTORY_SIZE) {

		nErrorAddressIndex = 0x00;

	}

	if (nErrorHistoryIndex >= MAX_ERROR_HISTORY_INDEX) {

		nErrorHistoryIndex = 0x01;

	}

	stMaintenance.ErrorHistoryIndex = nErrorHistoryIndex;
	stMaintenance.ErrorAddressIndex = nErrorAddressIndex;

	write_Byte8ParameterEEPROM(ERROR_NEXT_INDEX_ADDR,	nErrorHistoryIndex);
	write_Byte8ParameterEEPROM(ERROR_NEXT_POINTER_ADDR, nErrorAddressIndex);
}

void load_SystemErrorHistory(unsigned long ulErrorHistory[])
{
	int nHistIndex = 0;

	for (int nErrAddrIdx = 0; nErrAddrIdx < ERROR_CODE_HISTORY_SIZE; nErrAddrIdx += 0x03) {

		unsigned int nLowWord, nHighWord; 

		nHighWord = read_Byte8ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrAddrIdx) & 0x00FF;
		nLowWord = read_Word16ParameterEEPROM(ERROR_CODE_HISTORY_ADDR+nErrAddrIdx+0x01);

		ulErrorHistory[nHistIndex++] = _long32(nHighWord, nLowWord);

	}

	qsort(ulErrorHistory, ERROR_HISTORY_SIZE, sizeof(unsigned long), compare_ErrorHistoryIndex);
}

void save_MaintenanceBrewUsage(const stMaintenanceDataSet_t stMaintenance)
{
	save_TotalBrewCount(stMaintenance.TotBrewCount);
	save_TotalFlowQuantity(stMaintenance.TotFlowQuantity);
	save_OverflowedUsage(stMaintenance.Overflowed);
}

void load_MaintenanceBrewUsage(stMaintenanceDataSet_t& stMaintenance)
{
	stMaintenance.TotBrewCount = load_TotalBrewCount();
	stMaintenance.TotFlowQuantity = load_TotalFlowQuantity();
	stMaintenance.Overflowed = load_OverflowedUsage();
}

void save_TotalBrewCount(const unsigned long ulTotBrewCount)
{
	unsigned int nIndex = TOT_BREW_COUNT_PTR_INDEX;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (TOTAL_BREW_COUNT_BACKUP_BASE_ADDR + 0x04 * nPointer);

	write_Word32ParameterEEPROM(nAddress, ulTotBrewCount);

	while (ulTotBrewCount != read_Word32ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (TOTAL_BREW_COUNT_BACKUP_BASE_ADDR + 0x04 * nPointer);

			write_Word32ParameterEEPROM(nAddress, ulTotBrewCount);
			write_Byte8ParameterEEPROM(TOTAL_BREW_COUNT_BACKUP_POINTER, eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[EEPROM] Memory Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

unsigned long load_TotalBrewCount(void)
{
	unsigned int nPointer = eeprom_backup_addr_pointer[TOT_BREW_COUNT_PTR_INDEX];
	unsigned int nAddress = (TOTAL_BREW_COUNT_BACKUP_BASE_ADDR + 0x04 * nPointer);

	return read_Word32ParameterEEPROM(nAddress);
}

void save_TotalFlowQuantity(const unsigned long ulTotFlowQuantity)
{
	unsigned int nIndex = TOT_FLOW_QUANTITY_PTR_INDEX;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (TOTAL_FLOW_QTY_BACKUP_BASE_ADDR + 0x04 * nPointer);

	write_Word32ParameterEEPROM(nAddress, ulTotFlowQuantity);

	while (ulTotFlowQuantity != read_Word32ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (TOTAL_FLOW_QTY_BACKUP_BASE_ADDR + 0x04 * nPointer);

			write_Word32ParameterEEPROM(nAddress, ulTotFlowQuantity);
			write_Byte8ParameterEEPROM(TOTAL_FLOW_QTY_BACKUP_POINTER, eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

unsigned long load_TotalFlowQuantity(void)
{
	unsigned int nPointer = eeprom_backup_addr_pointer[TOT_FLOW_QUANTITY_PTR_INDEX];
	unsigned int nAddress = (TOTAL_FLOW_QTY_BACKUP_BASE_ADDR + 0x04 * nPointer);

	return read_Word32ParameterEEPROM(nAddress);
}

void save_OverflowedUsage(const unsigned char nOverflowed)
{
	unsigned int nIndex = CNT_OVERFLOW_PTR_INDEX;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (COUNT_OVERFLOW_BACKUP_BASE_ADDR + nPointer);

	write_Byte8ParameterEEPROM(nAddress, nOverflowed);

	while (nOverflowed != read_Byte8ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (COUNT_OVERFLOW_BACKUP_BASE_ADDR + nPointer);

			write_Byte8ParameterEEPROM(nAddress, nOverflowed);
			write_Byte8ParameterEEPROM(COUNT_OVERFLOW_BACKUP_POINTER, eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

unsigned char load_OverflowedUsage(void)
{
	unsigned int nPointer = eeprom_backup_addr_pointer[CNT_OVERFLOW_PTR_INDEX];
	unsigned int nAddress = (COUNT_OVERFLOW_BACKUP_BASE_ADDR + nPointer);

	return read_Byte8ParameterEEPROM(nAddress);
}

void save_ProgramParam(const int nProgramNum, const stSetupDataSet_t stSetupDataSet)
{
	save_ProgramMode(nProgramNum, stSetupDataSet.ProgramMode[nProgramNum]);
	save_ProgramInjectionFlow(nProgramNum, stSetupDataSet);
	save_ProgramExtractionFlow(nProgramNum, stSetupDataSet);
	save_ProgramInjectionTime(nProgramNum, stSetupDataSet);
	save_ProgramExtractionTime(nProgramNum, stSetupDataSet);
}

void load_ProgramParam(const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.ProgramMode[nProgramNum] = load_ProgramMode(nProgramNum);
	stSetupDataSet.InjectionFlow[nProgramNum] = load_ProgramInjectionFlow(nProgramNum);
	stSetupDataSet.ExtractionFlow[nProgramNum] = load_ProgramExtractionFlow(nProgramNum);
	stSetupDataSet.InjectionTime[nProgramNum] = load_ProgramInjectionTime(nProgramNum);
	stSetupDataSet.ExtractionTime[nProgramNum] = load_ProgramExtractionTime(nProgramNum);
}

void copy_ProgramParam(const int nProgramSetNum, const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.InjectionFlow[nProgramSetNum] = stSetupDataSet.InjectionFlow[nProgramNum];
	stSetupDataSet.ExtractionFlow[nProgramSetNum] = stSetupDataSet.ExtractionFlow[nProgramNum];
	stSetupDataSet.InjectionTime[nProgramSetNum] = stSetupDataSet.InjectionTime[nProgramNum];
	stSetupDataSet.ExtractionTime[nProgramSetNum] = stSetupDataSet.ExtractionTime[nProgramNum];

	save_ProgramParam(nProgramSetNum, stSetupDataSet);
}

void save_ProgramMode(const int nProgramNum, const unsigned char nProgramMode)
{
	unsigned int nIndex = FIRST_PROGRAM_MODE_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_PROGRAM_MODE_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + nPointer;

	write_Byte8ParameterEEPROM(nAddress, nProgramMode);

	while (nProgramMode != read_Byte8ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (FIRST_PROGRAM_MODE_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + nPointer;

			write_Byte8ParameterEEPROM(nAddress, nProgramMode);
			write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

unsigned char load_ProgramMode(const int nProgramNum)
{
	unsigned int nIndex = FIRST_PROGRAM_MODE_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_PROGRAM_MODE_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + nPointer;

	return read_Byte8ParameterEEPROM(nAddress);
}

void save_ProgramInjectionFlow(const int nProgramNum, const stSetupDataSet_t stSetupDataSet)
{
	unsigned int nIndex = FIRST_INJECTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	write_Word16ParameterEEPROM(nAddress, stSetupDataSet.InjectionFlow[nProgramNum]);

	while (stSetupDataSet.InjectionFlow[nProgramNum] != read_Word16ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

			write_Word16ParameterEEPROM(nAddress, stSetupDataSet.InjectionFlow[nProgramNum]);
			write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

void save_ProgramExtractionFlow(const int nProgramNum, const stSetupDataSet_t stSetupDataSet)
{
	unsigned int nIndex = FIRST_EXTRACTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	write_Word16ParameterEEPROM(nAddress, stSetupDataSet.ExtractionFlow[nProgramNum]);

	while (stSetupDataSet.ExtractionFlow[nProgramNum] != read_Word16ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

			write_Word16ParameterEEPROM(nAddress, stSetupDataSet.ExtractionFlow[nProgramNum]);
			write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

void load_ProgramInjectionFlow(const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nIndex = FIRST_INJECTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	stSetupDataSet.InjectionFlow[nProgramNum] = read_Word16ParameterEEPROM(nAddress);
}

void load_ProgramExtractionFlow(const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nIndex = FIRST_EXTRACTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	stSetupDataSet.ExtractionFlow[nProgramNum] = read_Word16ParameterEEPROM(nAddress);
}

unsigned int load_ProgramInjectionFlow(const int nProgramNum)
{
	unsigned int nIndex = FIRST_INJECTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	return read_Word16ParameterEEPROM(nAddress);
}

unsigned int load_ProgramExtractionFlow(const int nProgramNum)
{
	unsigned int nIndex = FIRST_EXTRACTION_FLOWCNT_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	return read_Word16ParameterEEPROM(nAddress);
}

void save_ProgramInjectionTime(const int nProgramNum, const stSetupDataSet_t stSetupDataSet)
{
	unsigned int nIndex = FIRST_INJECTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	unsigned int nInjectionTime = _convert_float_to_decimal(stSetupDataSet.InjectionTime[nProgramNum], 10ul);

	write_Word16ParameterEEPROM(nAddress, nInjectionTime);

	while (nInjectionTime != read_Word16ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (FIRST_INJECTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

			write_Word16ParameterEEPROM(nAddress, nInjectionTime);
			write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);;

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

void save_ProgramExtractionTime(const int nProgramNum, const stSetupDataSet_t stSetupDataSet)
{
	unsigned int nIndex = FIRST_EXTRACTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	unsigned int nExtractionTime = _convert_float_to_decimal(stSetupDataSet.ExtractionTime[nProgramNum], 10ul);

	write_Word16ParameterEEPROM(nAddress, nExtractionTime);

	while (nExtractionTime != read_Word16ParameterEEPROM(nAddress)) {

		if (nPointer < MAX_BACKUP_LOCATIONS-1) {

			nPointer++;
			eeprom_backup_addr_pointer[nIndex] = nPointer;
			nAddress = (FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

			write_Word16ParameterEEPROM(nAddress, nExtractionTime);
			write_Byte8ParameterEEPROM((TABLE_FOR_BACKUP_POINTER_BASE_ADDR + nIndex), eeprom_backup_addr_pointer[nIndex]);

		}
		else {

			#if defined(_USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_)
			stError.bIsEEPROMWorn = true;
			#endif

			#if defined(_USE_APPLICATION_DEBUG_MSG_)
			cout << F("[ERROR] EEPROM Write Failed.\n") << endl;
			#endif

			break;

		}

	}
}

void load_ProgramInjectionTime(const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nIndex = FIRST_INJECTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	stSetupDataSet.InjectionTime[nProgramNum] = read_Word16ParameterEEPROM(nAddress) / 10.0f;
}

void load_ProgramExtractionTime(const int nProgramNum, stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nIndex = FIRST_EXTRACTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	stSetupDataSet.ExtractionTime[nProgramNum] = read_Word16ParameterEEPROM(nAddress) / 10.0f;
}

float load_ProgramInjectionTime(const int nProgramNum)
{
	unsigned int nIndex = FIRST_INJECTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_INJECTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	return read_Word16ParameterEEPROM(nAddress) / 10.0f;
}

float load_ProgramExtractionTime(const int nProgramNum)
{
	unsigned int nIndex = FIRST_EXTRACTION_TIME_PTR_INDEX + 0x05 * nProgramNum;

	unsigned int nPointer = eeprom_backup_addr_pointer[nIndex];
	unsigned int nAddress = (FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR + (0x09 * MAX_BACKUP_LOCATIONS) * nProgramNum) + 0x02 * nPointer;

	return read_Word16ParameterEEPROM(nAddress) / 10.0f;
}

void save_ManualProgramSetNumber(const unsigned char nProgramSetNum)
{
	write_Byte8ParameterEEPROM(PROGRAM_SET_NUMBER_ADDR, nProgramSetNum);
}

void load_ManualProgramSetNumber(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.ProgramSetNum = read_Byte8ParameterEEPROM(PROGRAM_SET_NUMBER_ADDR);
}

unsigned char load_ManualProgramSetNumber(void)
{
	return read_Byte8ParameterEEPROM(PROGRAM_SET_NUMBER_ADDR);
}

void save_InjectionSetupParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(INJECTION_SETUP_BASE_ADDR,		stSetupDataSet.InjectionMode);
	write_Word16ParameterEEPROM(INJECTION_SETUP_BASE_ADDR+0x01, _convert_float_to_decimal(stSetupDataSet.InjectionPower, 100ul));
}

void load_InjectionSetupParam(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(INJECTION_SETUP_BASE_ADDR+0x01);

	stSetupDataSet.InjectionPower = constrain(float(int(nRawData) / 100.0f), 0.0f, 1.0f);

	stSetupDataSet.InjectionMode = read_Byte8ParameterEEPROM(INJECTION_SETUP_BASE_ADDR);
}

void save_InjectionMode(const unsigned char nInjectionMode)
{
	write_Byte8ParameterEEPROM(INJECTION_MODE_ADDR, nInjectionMode);
}

void load_InjectionMode(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.InjectionMode = read_Byte8ParameterEEPROM(INJECTION_MODE_ADDR);
}

unsigned char load_InjectionMode(void)
{
	return read_Byte8ParameterEEPROM(INJECTION_MODE_ADDR);
}

void save_InjectionPower(const float fInjectionPower)
{
	write_Word16ParameterEEPROM(INJECTION_POWER_ADDR, _convert_float_to_decimal(fInjectionPower, 100ul));
}

void load_InjectionPower(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(INJECTION_POWER_ADDR);

	stSetupDataSet.InjectionPower = constrain(float(int(nRawData) / 100.0f), 0.0f, 1.0f);
}

float load_InjectionPower(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(INJECTION_POWER_ADDR);

	float fInjectionPower = constrain(float(int(nRawData) / 100.0f), 0.0f, 1.0f);

	return fInjectionPower;
}

void save_HeaterSetValueTemperature(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR,		 _convert_float_to_decimal(stSetupDataSet.SetValueTemp[0], 10ul));
	write_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR+0x02, _convert_float_to_decimal(stSetupDataSet.SetValueTemp[1], 10ul));
	write_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR+0x04, _convert_float_to_decimal(stSetupDataSet.SetValueTemp[2], 10ul));
}

void load_HeaterSetValueTemperature(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR);
	stSetupDataSet.SetValueTemp[0] = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	nRawData = read_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR+0x02);
	stSetupDataSet.SetValueTemp[1] = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	nRawData = read_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR+0x04);
	stSetupDataSet.SetValueTemp[2] = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);
}

void save_HeaterSetValueTemperatureWithChannel(const int nChannel, const float fSetValueTemp)
{
	unsigned int nAddress = HEATER_SETUP_BASE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, _convert_float_to_decimal(fSetValueTemp, 10ul));
}

float load_HeaterSetValueTemperatureWithChannel(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(HEATER_SETUP_BASE_ADDR + (0x02 * nChannel));

	float fSetValueTemp = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	return fSetValueTemp;
}

void save_PreheatBoilerSetValueTemperature(const float fSetValueTemp)
{
	write_Word16ParameterEEPROM(PREHEAT_SET_VALUE_ADDR, _convert_float_to_decimal(fSetValueTemp, 10ul));
}

float load_PreheatBoilerSetValueTemperature(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(PREHEAT_SET_VALUE_ADDR);

	float fSetValueTemp = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	return fSetValueTemp;
}

void save_BrewBoilerSetValueTemperature(const float fSetValueTemp)
{
	write_Word16ParameterEEPROM(BREW_SET_VALUE_ADDR, _convert_float_to_decimal(fSetValueTemp, 10ul));
}

float load_BrewBoilerSetValueTemperature(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(BREW_SET_VALUE_ADDR);

	float fSetValueTemp = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	return fSetValueTemp;
}

void save_GroupheadSetValueTemperature(const float fSetValueTemp)
{
	write_Word16ParameterEEPROM(GROUPHEAD_SET_VALUE_ADDR, _convert_float_to_decimal(fSetValueTemp, 10ul));
}

float load_GroupheadSetValueTemperature(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(GROUPHEAD_SET_VALUE_ADDR);

	float fSetValueTemp = constrain(float(int(nRawData) / 10.0f), 0.0f, 100.0f);

	return fSetValueTemp;
}

void save_HeaterProportionalBandTemperature(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR,	  _convert_float_to_decimal(stSetupDataSet.ProportionalBandTemp[0], 10ul));
	write_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR+0x02, _convert_float_to_decimal(stSetupDataSet.ProportionalBandTemp[1], 10ul));
	write_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR+0x04, _convert_float_to_decimal(stSetupDataSet.ProportionalBandTemp[2], 10ul));
}

void load_HeaterProportionalBandTemperature(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR);
	stSetupDataSet.ProportionalBandTemp[0] = constrain(float(int(nRawData) / 10.0f), 0.1f, 999.9f);

	nRawData = read_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR+0x02);
	stSetupDataSet.ProportionalBandTemp[1] = constrain(float(int(nRawData) / 10.0f), 0.1f, 999.9f);

	nRawData = read_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR+0x04);
	stSetupDataSet.ProportionalBandTemp[2] = constrain(float(int(nRawData) / 10.0f), 0.1f, 999.9f);
}

void save_HeaterProportionalBandTemperatureWithChannel(const int nChannel, const float fPBandTemperature)
{
	unsigned int nAddress = HEATER_PBAND_VALUE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, _convert_float_to_decimal(fPBandTemperature, 10ul));
}

float load_HeaterProportionalBandTemperatureWithChannel(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(HEATER_PBAND_VALUE_ADDR + (0x02 * nChannel));

	float fPBandTemperature = constrain(float(int(nRawData) / 10.0f), 0.1f, 999.9f);

	return fPBandTemperature;
}

void save_HeaterIntegralTime(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR,	  stSetupDataSet.IntegralTime[0]);
	write_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR+0x02, stSetupDataSet.IntegralTime[1]);
	write_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR+0x04, stSetupDataSet.IntegralTime[2]);
}

void load_HeaterIntegralTime(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR);
	stSetupDataSet.IntegralTime[0] = constrain(nRawData, 0ul, 9999ul);

	nRawData = read_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR+0x02);
	stSetupDataSet.IntegralTime[1] = constrain(nRawData, 0ul, 9999ul);

	nRawData = read_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR+0x04);
	stSetupDataSet.IntegralTime[2] = constrain(nRawData, 0ul, 9999ul);
}

void save_HeaterIntegralTimeWithChannel(const int nChannel, const unsigned int nIntegralTime)
{
	unsigned int nAddress = HEATER_ITIME_VALUE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, nIntegralTime);
}

unsigned int load_HeaterIntegralTimeWithChannel(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(HEATER_ITIME_VALUE_ADDR + (0x02 * nChannel));

	unsigned int nIntegralTime = constrain(nRawData, 0ul, 9999ul);

	return nIntegralTime;
}

void save_HeaterDerivativeTime(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR,	  stSetupDataSet.DerivativeTime[0]);
	write_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR+0x02, stSetupDataSet.DerivativeTime[1]);
	write_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR+0x04, stSetupDataSet.DerivativeTime[2]);
}

void load_HeaterDerivativeTime(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR);
	stSetupDataSet.DerivativeTime[0] = constrain(nRawData, 0ul, 9999ul);

	nRawData = read_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR+0x02);
	stSetupDataSet.DerivativeTime[1] = constrain(nRawData, 0ul, 9999ul);

	nRawData = read_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR+0x04);
	stSetupDataSet.DerivativeTime[2] = constrain(nRawData, 0ul, 9999ul);
}

void save_HeaterDerivativeTimeWithChannel(const int nChannel, const unsigned int nDerivativeTime)
{
	unsigned int nAddress = HEATER_DTIME_VALUE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, nDerivativeTime);
}

unsigned int load_HeaterDerivativeTimeWithChannel(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(HEATER_DTIME_VALUE_ADDR + (0x02 * nChannel));

	unsigned int nDerivativeTime = constrain(nRawData, 0ul, 9999ul);

	return nDerivativeTime;
}

void save_HeaterMVHighLimit(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR,		_convert_float_to_decimal(stSetupDataSet.MVHighLimit[0], 10ul));
	write_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR+0x02,	_convert_float_to_decimal(stSetupDataSet.MVHighLimit[1], 10ul));
	write_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR+0x04,	_convert_float_to_decimal(stSetupDataSet.MVHighLimit[2], 10ul));
}

void load_HeaterMVHighLimit(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR);
	stSetupDataSet.MVHighLimit[0] = constrain(float(int(nRawData) / 10.0f), 0.1f, 100.0f);

	nRawData = read_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR+0x02);
	stSetupDataSet.MVHighLimit[1] = constrain(float(int(nRawData) / 10.0f), 0.1f, 100.0f);

	nRawData = read_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR+0x04);
	stSetupDataSet.MVHighLimit[2] = constrain(float(int(nRawData) / 10.0f), 0.1f, 100.0f);
}

void save_HeaterMVHighLimitWithChannel(const int nChannel, const float fMVHighLimit)
{
	unsigned int nAddress = HEATER_MV_SETUP_BASE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, _convert_float_to_decimal(fMVHighLimit, 10ul));
}

float load_HeaterMVHighLimitWithChannel(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(HEATER_MV_SETUP_BASE_ADDR + (0x02 * nChannel));

	float fMVHighLimit = constrain(float(int(nRawData) / 10.0f), 0.1f, 100.0f);

	return fMVHighLimit;
}

void save_HeaterInputBias(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR,     _convert_float_to_decimal(stSetupDataSet.SensorInputBias[0], 10ul));
	write_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x2, _convert_float_to_decimal(stSetupDataSet.SensorInputBias[1], 10ul));
	write_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x4, _convert_float_to_decimal(stSetupDataSet.SensorInputBias[2], 10ul));
	write_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x6, _convert_float_to_decimal(stSetupDataSet.SensorInputBias[3], 10ul));
}

void load_HeaterInputBias(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR);
	stSetupDataSet.SensorInputBias[0] = constrain(float(int(nRawData) / 10.0f), MIN_SENSOR_INPUT_BIAS, MAX_SENSOR_INPUT_BIAS);

	nRawData = read_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x02);
	stSetupDataSet.SensorInputBias[1] = constrain(float(int(nRawData) / 10.0f), MIN_SENSOR_INPUT_BIAS, MAX_SENSOR_INPUT_BIAS);

	nRawData = read_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x04);
	stSetupDataSet.SensorInputBias[2] = constrain(float(int(nRawData) / 10.0f), MIN_SENSOR_INPUT_BIAS, MAX_SENSOR_INPUT_BIAS);

	nRawData = read_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR+0x06);
	stSetupDataSet.SensorInputBias[3] = constrain(float(int(nRawData) / 10.0f), MIN_SENSOR_INPUT_BIAS, MAX_SENSOR_INPUT_BIAS);
}

void save_HeaterInputBias(const int nChannel, const float fSensorInputBias)
{
	unsigned int nAddress = TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR + (0x02 * nChannel);

	write_Word16ParameterEEPROM(nAddress, _convert_float_to_decimal(fSensorInputBias, 10ul));
}

float load_HeaterInputBias(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR + (0x02 * nChannel));

	float fSensorInputBias = constrain(float(int(nRawData) / 10.0f), MIN_SENSOR_INPUT_BIAS, MAX_SENSOR_INPUT_BIAS);

	return fSensorInputBias;
}

void save_PressureSensorZeroShift(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR,	   _convert_float_to_decimal(stSetupDataSet.ZeroShift[0], 100ul));
	write_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR+0x02, _convert_float_to_decimal(stSetupDataSet.ZeroShift[1], 100ul));
	write_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR+0x04, _convert_float_to_decimal(stSetupDataSet.ZeroShift[2], 100ul));
}

void load_PressureSensorZeroShift(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR);
	stSetupDataSet.ZeroShift[0] = constrain(float(int(nRawData) / 100.0f), -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR+0x02);
	stSetupDataSet.ZeroShift[1] = constrain(float(int(nRawData) / 100.0f), -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR+0x04);
	stSetupDataSet.ZeroShift[2] = constrain(float(int(nRawData) / 100.0f), -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);
}

float load_PressureSensorZeroShift(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR + (0x02 * nChannel));

	float fZeroShift = constrain(float(int(nRawData) / 100.0f), -MIN_PRESSURE_ANALOG, MIN_PRESSURE_ANALOG);

	return fZeroShift;
}

void save_PressureSensorSpanFactor(const stSetupDataSet_t stSetupDataSet)
{
	write_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR,	    _convert_float_to_decimal(stSetupDataSet.SpanFactor[0], 10000ul));
	write_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR+0x02, _convert_float_to_decimal(stSetupDataSet.SpanFactor[1], 10000ul));
	write_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR+0x04, _convert_float_to_decimal(stSetupDataSet.SpanFactor[2], 10000ul));
}

void load_PressureSensorSpanFactor(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR);
	stSetupDataSet.SpanFactor[0] = constrain(float(int(nRawData) / 10000.0f), 0.001f, 2.000f);

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR+0x02);
	stSetupDataSet.SpanFactor[1] = constrain(float(int(nRawData) / 10000.0f), 0.001f, 2.000f);

	nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR+0x04);
	stSetupDataSet.SpanFactor[2] = constrain(float(int(nRawData) / 10000.0f), 0.001f, 2.000f);
}

float load_PressureSensorSpanFactor(const int nChannel)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR + (0x02 * nChannel));

	float fSpanFactor = constrain(float(int(nRawData) / 10000.0f), 0.001f, 2.000f);

	return fSpanFactor;
}

void save_MotorPumpParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR,		stSetupDataSet.MotorRun);
	write_Byte8ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR+0x01,	stSetupDataSet.MotorStandbyMode);
	write_Word16ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR+0x02, _convert_float_to_decimal(stSetupDataSet.MotorPower, 100ul));
}

void load_MotorPumpParam(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR+0x02);
	stSetupDataSet.MotorPower = constrain(float(int(nRawData) / 100.0f), 0.0f, 1.0f);

	stSetupDataSet.MotorRun = read_Byte8ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR);
	stSetupDataSet.MotorStandbyMode = read_Byte8ParameterEEPROM(MOTORPUMP_SETUP_BASE_ADDR+0x01);
}

void save_MotorRunFlag(const unsigned char nMotorRunFlag)
{
	write_Byte8ParameterEEPROM(MOTOR_RUN_ADDR, nMotorRunFlag);
}

unsigned char load_MotorRunFlag(void)
{
	return read_Byte8ParameterEEPROM(MOTOR_RUN_ADDR);
}

void save_MotorStandbyMode(const unsigned char nMotorStandbyMode)
{
	write_Byte8ParameterEEPROM(MOTOR_STANDBY_MODE_ADDR, nMotorStandbyMode);
}

unsigned char load_MotorStandbyMode(void)
{
	return read_Byte8ParameterEEPROM(MOTOR_STANDBY_MODE_ADDR);
}

void save_MotorPower(const float fMotorPower)
{
	write_Word16ParameterEEPROM(MOTOR_POWER_ADDR, _convert_float_to_decimal(fMotorPower, 100ul));
}

float load_MotorPower(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(MOTOR_POWER_ADDR);

	float fMotorPower = constrain(float(int(nRawData) / 100.0f), 0.0f, 1.0f);

	return fMotorPower;
}

void save_GroupheadLampParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(LAMP_SETUP_BASE_ADDR,	  stSetupDataSet.LampMode);
	write_Byte8ParameterEEPROM(LAMP_SETUP_BASE_ADDR+0x01, stSetupDataSet.LampIntensity);
}

void load_GroupheadLampParam(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.LampMode = read_Byte8ParameterEEPROM(LAMP_SETUP_BASE_ADDR);
	stSetupDataSet.LampIntensity = read_Byte8ParameterEEPROM(LAMP_SETUP_BASE_ADDR+0x01);
}

void save_GroupheadLampMode(const unsigned char nLampMode)
{
	write_Byte8ParameterEEPROM(LAMP_MODE_ADDR, nLampMode);
}

unsigned char load_GroupheadLampMode(void)
{
	return read_Byte8ParameterEEPROM(LAMP_MODE_ADDR);
}

void save_GroupheadLampIntensity(const unsigned char nLampIntensity)
{
	write_Byte8ParameterEEPROM(LAMP_INTENSITY_ADDR, nLampIntensity);
}

unsigned char load_GroupheadLampIntensity(void)
{
	return read_Byte8ParameterEEPROM(LAMP_INTENSITY_ADDR);
}

void save_LEDPixelMode(const stSetupDataSet_t& stSetupDataSet)
{
	write_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR, 	   stSetupDataSet.LedPixelMode[PROGRAM_SET_ONE]);
	write_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR+0x01, stSetupDataSet.LedPixelMode[PROGRAM_SET_TWO]);
	write_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR+0x02, stSetupDataSet.LedPixelMode[MANUAL_SET_ONE]);
}

void load_LEDPixelMode(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.LedPixelMode[PROGRAM_SET_ONE] = read_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR);
	stSetupDataSet.LedPixelMode[PROGRAM_SET_TWO] = read_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR+0x01);
	stSetupDataSet.LedPixelMode[MANUAL_SET_ONE] = read_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR+0x02);
}

unsigned char load_LEDPixelMode(const int nProgramNum)
{
	return read_Byte8ParameterEEPROM(LEDPIXEL_MODE_SETUP_BASE_ADDR + nProgramNum);
}

void save_MotorACCTimeParam(stSetupDataSet_t& stSetupDataSet)
{
	write_Word16ParameterEEPROM(MOTOR_ACC_TIME_SETUP_BASE_ADDR+0x01, stSetupDataSet.MotorAccTime);
	
#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	stSetupDataSet.MotorAccTimeEST = hMotorPump.convACCTimeWithRPM(stSetupDataSet.MotorAccTime, stSetupDataSet.MotorPower);
#endif
}

void load_MotorACCTimeParam(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(MOTOR_ACC_TIME_SETUP_BASE_ADDR+0x01);

	stSetupDataSet.MotorAccTime = constrain(nRawData, _MIN_ACC_TIME_, _MAX_ACC_TIME_);

#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	stSetupDataSet.MotorAccTimeEST = hMotorPump.convACCTimeWithRPM(stSetupDataSet.MotorAccTime, stSetupDataSet.MotorPower);
#endif
}

void save_MotorACCTime(const unsigned int nMotorAccTime)
{
	write_Word16ParameterEEPROM(MOTOR_ACC_TIME_MSEC_ADDR, nMotorAccTime);
}

unsigned int load_MotorACCTime(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(MOTOR_ACC_TIME_MSEC_ADDR);

	return constrain(nRawData, _MIN_ACC_TIME_, _MAX_ACC_TIME_);
}

#if AUTOMATION_SENSOR_ATTACHED

void save_AutomationParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR,       stSetupDataSet.AutomationMode);
	write_Word16ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR+0x01, _convert_float_to_decimal(stSetupDataSet.AutoFlushingTime, 10ul));
	write_Word16ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR+0x03, _convert_float_to_decimal(stSetupDataSet.AutoBrewWaitTime, 10ul));
}

void load_AutomationParam(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR+0x01);
	stSetupDataSet.AutoFlushingTime = constrain(float(int(nRawData) / 10.0f), MIN_TIME_TO_AUTO_FLUSHING, MAX_TIME_TO_AUTO_FLUSHING);

	nRawData = read_Word16ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR+0x03);
	stSetupDataSet.AutoBrewWaitTime = constrain(float(int(nRawData) / 10.0f), 0.0F/*MIN_WAIT_TIME_TO_AUTO_BREW*/, MAX_WAIT_TIME_TO_AUTO_BREW);

	stSetupDataSet.AutomationMode = read_Byte8ParameterEEPROM(AUTOMATION_SETUP_BASE_ADDR);
}

void save_AutomationMode(const unsigned char nAutomationMode)
{
	write_Byte8ParameterEEPROM(AUTOMATION_MODE_ADDR, nAutomationMode);
}

unsigned char load_AutomationMode(void)
{
	return read_Byte8ParameterEEPROM(AUTOMATION_MODE_ADDR);
}

void save_AutoFlushingTime(const float fAutoFlushingTime)
{
	write_Word16ParameterEEPROM(AUTO_FLUSHING_TIME_ADDR, _convert_float_to_decimal(fAutoFlushingTime, 10ul));
}

float load_AutoFlushingTime(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(AUTO_FLUSHING_TIME_ADDR);

	float fAutoFlushingTime = constrain(float(int(nRawData) / 10.0f), MIN_TIME_TO_AUTO_FLUSHING, MAX_TIME_TO_AUTO_FLUSHING);

	return fAutoFlushingTime;
}

void save_AutoBrewWaitTime(const float fAutoBrewWaitTime)
{
	write_Word16ParameterEEPROM(AUTO_BREW_WAIT_TIME_ADDR, _convert_float_to_decimal(fAutoBrewWaitTime, 10ul));
}

float load_AutoBrewWaitTime(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(AUTO_BREW_WAIT_TIME_ADDR);

	float fAutoBrewWaitTime = constrain(float(int(nRawData) / 10.0f), 0.0F/*MIN_WAIT_TIME_TO_AUTO_BREW*/, MAX_WAIT_TIME_TO_AUTO_BREW);

	return fAutoBrewWaitTime;
}

#else

void save_FlushingDetectionParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR,       stSetupDataSet.FlushingDetectionMode);
	write_Word16ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR+0x01, _convert_float_to_decimal(stSetupDataSet.FlushingDetectionTime, 10ul));
	write_Word16ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR+0x03, _convert_float_to_decimal(stSetupDataSet.FlushingDetectionPress, 10ul));
}

void load_FlushingDetectionParam(stSetupDataSet_t& stSetupDataSet)
{
	unsigned int nRawData;

	nRawData = read_Word16ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR+0x01);
	stSetupDataSet.FlushingDetectionTime = constrain(float(int(nRawData) / 10.0f), MIN_TIME_FOR_FLUSHING_DETECTION, MAX_TIME_FOR_FLUSHING_DETECTION);

	nRawData = read_Word16ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR+0x03);
	stSetupDataSet.FlushingDetectionPress = constrain(float(int(nRawData) / 10.0f), MIN_BREW_PRESSURE_FOR_FLUSHING_DETECTION, MAX_BREW_PRESSURE_FOR_FLUSHING_DETECTION);

	stSetupDataSet.FlushingDetectionMode = read_Byte8ParameterEEPROM(FLUSHING_DETECTION_SETUP_BASE_ADDR);
}

void save_FlushingDetectionMode(const unsigned char nFlushingDetectionMode)
{
	write_Byte8ParameterEEPROM(FLUSHING_DETECTION_MODE_ADDR, nFlushingDetectionMode);
}

unsigned char load_FlushingDetectionMode(void)
{
	return read_Byte8ParameterEEPROM(FLUSHING_DETECTION_MODE_ADDR);
}

void save_FlushingDetectionTime(const float fFlushingDetectionTime)
{
	write_Word16ParameterEEPROM(FLUSHING_DETECTION_TIME_ADDR, _convert_float_to_decimal(fFlushingDetectionTime, 10ul));
}

float load_FlushingDetectionTime(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(FLUSHING_DETECTION_TIME_ADDR);

	float fFlushingDetectionTime = constrain(float(int(nRawData) / 10.0f), MIN_TIME_FOR_FLUSHING_DETECTION, MAX_TIME_FOR_FLUSHING_DETECTION);

	return fFlushingDetectionTime;
}

void save_FlushingDetectionPressure(const float fFlushingDetectionPress)
{
	write_Word16ParameterEEPROM(FLUSHING_DETECTION_PRES_ADDR, _convert_float_to_decimal(fFlushingDetectionPress, 10ul));
}

float load_FlushingDetectionPressure(void)
{
	unsigned int nRawData = read_Word16ParameterEEPROM(FLUSHING_DETECTION_PRES_ADDR);

	float fFlushingDetectionPress = constrain(float(int(nRawData) / 10.0f), MIN_BREW_PRESSURE_FOR_FLUSHING_DETECTION, MAX_BREW_PRESSURE_FOR_FLUSHING_DETECTION);

	return fFlushingDetectionPress;
}

#endif

void save_PowerSavingParam(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR, stSetupDataSet.PowerSaveMode);

	write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x01,	stSetupDataSet.StartHour);
	write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x02, stSetupDataSet.StartMinute);
	// write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x03, stSetupDataSet.StartSecond);
	write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x04, stSetupDataSet.EndHour);
	write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x05, stSetupDataSet.EndMinute);
	// write_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x06, stSetupDataSet.EndSecond);
}

void load_PowerSavingParam(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.PowerSaveMode = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR);

	stSetupDataSet.StartHour = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x01);
	stSetupDataSet.StartMinute = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x02);
	// stSetupDataSet.StartSecond = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x03);
	stSetupDataSet.EndHour = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x04);
	stSetupDataSet.EndMinute = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x05);
	// stSetupDataSet.EndSecond = read_Byte8ParameterEEPROM(POWER_SAVE_SETUP_BASE_ADDR+0x06);
}

void save_PowerSavingMode(const unsigned char nPowerSavingMode)
{
	write_Byte8ParameterEEPROM(POWER_SAVE_MODE_ADDR, nPowerSavingMode);
}

unsigned char load_PowerSavingMode(void)
{
	return read_Byte8ParameterEEPROM(POWER_SAVE_MODE_ADDR);
}

void save_PowerSavingTime(const stSetupDataSet_t stSetupDataSet)
{
	write_Byte8ParameterEEPROM(START_HOUR_ADDR,		stSetupDataSet.StartHour);
	write_Byte8ParameterEEPROM(START_HOUR_ADDR+0x01, stSetupDataSet.StartMinute);
	// write_Byte8ParameterEEPROM(START_HOUR_ADDR+0x02, stSetupDataSet.StartSecond);
	write_Byte8ParameterEEPROM(START_HOUR_ADDR+0x03, stSetupDataSet.EndHour);
	write_Byte8ParameterEEPROM(START_HOUR_ADDR+0x04, stSetupDataSet.EndMinute);
	// write_Byte8ParameterEEPROM(START_HOUR_ADDR+0x05, stSetupDataSet.EndSecond);
}

void load_PowerSavingTime(stSetupDataSet_t& stSetupDataSet)
{
	stSetupDataSet.StartHour = read_Byte8ParameterEEPROM(START_HOUR_ADDR);
	stSetupDataSet.StartMinute = read_Byte8ParameterEEPROM(START_HOUR_ADDR+0x01);
	// stSetupDataSet.StartSecond = read_Byte8ParameterEEPROM(START_HOUR_ADDR+0x02);
	stSetupDataSet.EndHour = read_Byte8ParameterEEPROM(START_HOUR_ADDR+0x03);
	stSetupDataSet.EndMinute = read_Byte8ParameterEEPROM(START_HOUR_ADDR+0x04);
	// stSetupDataSet.EndSecond = read_Byte8ParameterEEPROM(START_HOUR_ADDR+0x05);
}


// Print Data-log and System Informations
void print_SystemInformation(void)
{
	unsigned char cSWMajor = (unsigned char)(FIRMWARE_VERSION / 10);
	unsigned char cSWMinor = FIRMWARE_VERSION - (cSWMajor * 10);
	unsigned char cSWPatch = FIRMWARE_PATCH;

	unsigned char cHWMajor = (unsigned char)(HARDWARE_VERSION / 10);
	unsigned char cHWMinor = HARDWARE_VERSION - (cHWMajor * 10);

	_console.print(F("\n\nMOAI EspressoStation by Vidastech Inc.\n\n\r"));

	_console.println(F("\nMACHINE_INFORMATION::"));

	_console.print(F("\tMODEL No: "));
	_console.println(HARDWARE_MODELNAME);

	_console.print(F("\tH/W Ver : "));
	_console.print(cHWMajor);
	_console.print(F("."));
	_console.print(cHWMinor);
	_console.println(HARDWARE_TYPEDEF);

	_console.print(F("\tS/W Ver : "));
	_console.print(cSWMajor);
	_console.print(F("."));
	_console.print(cSWMinor);
	_console.print(F("."));
	_console.print(cSWPatch);
	_console.println(FIRMWARE_TYPEDEF);

	_console.print(F("\n\r"));
}

void print_SystemErrorList(const bool bReload)
{
	cout << F("\n[ERROR_CODE_LIST]") << endl;

	int nNumOfErrors = 0;

	if (bReload) {

		get_SystemErrorCode(bReload, 0);

	}

	for (int nIndex = 0; nIndex < ERROR_HISTORY_SIZE; nIndex++) {

		unsigned long ulErrorData = get_SystemErrorCode(false, nIndex);

		if (ulErrorData) {

			nNumOfErrors++;

			cout << F("\t[") << int(ulErrorData >> 16) << F("] ");

			_console.println(((ulErrorData & 0xFFFF) + 0xE000), HEX);

		}

	}

	cout << (nNumOfErrors ? F("\n\r") : F("\tNONE\n\r")) << endl;
}

#if defined(_USE_APPLICATION_DEBUG_MSG_)
void print_EEPROMBackupAddressPointer(void)
{
	_console.println(F("[EEPROM BACKUP Address Pointer]"));

	for (int index = TOT_BREW_COUNT_PTR_INDEX; index < MAX_BACKUP_PARAMETERS; index++) {

		_console.print(F("\t"));
		_console.print(index);
		_console.print(F("::\t"));
		_console.println(eeprom_backup_addr_pointer[index]);

	}

	_console.println(F("\n\r"));
}

void print_StructureDataSet(void)
{
	cout << F("[SYSTEM PARAMETERS]") << endl;

	cout << F("ST_DATA_SET::") << endl;
	cout << F("\tProgramNum::") << int(stDataSet.ProgramNum) << endl;

	cout << F("ST_ACQ::") << endl;
	cout << F("\tShotTime::") << stDataSet.stAcquisition.ShotTime << endl;
	cout << F("\tFlowCount::") << stDataSet.stAcquisition.FlowCount << endl;
	cout << F("\tFlowQuantity::") << stDataSet.stAcquisition.FlowQuantity << endl;
	cout << F("\tFlowRate::") << stDataSet.stAcquisition.FlowRate << endl;
	cout << F("\tPreheatTemp::") << stDataSet.stAcquisition.PreheatTemp << endl;
	cout << F("\tBrewTemp::") << stDataSet.stAcquisition.BrewTemp << endl;
	cout << F("\tGroupTemp::") << stDataSet.stAcquisition.GroupTemp << endl;
#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	cout << F("\tBrewWaterTemp::") << stDataSet.stAcquisition.BrewWaterTemp << endl;
#endif
	cout << F("\tPreheatMV::") << stDataSet.stAcquisition.PreheatMV << endl;
	cout << F("\tBrewMV::") << stDataSet.stAcquisition.BrewMV << endl;
	cout << F("\tGroupMV::") << stDataSet.stAcquisition.GroupMV << endl;
#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	cout << F("\tPresentCurrent::") << stDataSet.stAcquisition.PresentCurrent << endl;
	cout << F("\tPresentSpeed::") << stDataSet.stAcquisition.PresentSpeed << endl;
#endif
	cout << F("\tBrewPress::") << stDataSet.stAcquisition.BrewPress << endl;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	cout << F("\tPumpPress::") << stDataSet.stAcquisition.PumpPress << endl;
#endif
#if AUX_CURRENT_SENSOR_ATTACHED
	cout << F("\tAuxPress::") << stDataSet.stAcquisition.AuxPress << endl;
#endif
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	cout << F("\tAnalogInput::") << stDataSet.stAcquisition.AnalogInputVoltage << endl;
#endif
	cout << F("\tAvgFlowRate::") << stDataSet.stAcquisition.AvgFlowRate << endl;
	cout << F("\tAvgBrewPress::") << stDataSet.stAcquisition.AvgBrewPress << endl;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	cout << F("\tAvgPumpPress::") << stDataSet.stAcquisition.AvgPumpPress << endl;
#endif
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	cout << F("\tAvgAnalogInputVoltage::") << stDataSet.stAcquisition.AvgAnalogInputVoltage << endl;
#endif

	cout << F("ST_SETUP::") << endl;
	for (int nProgramNum = PROGRAM_SET_ONE; nProgramNum <= PROGRAM_SET_TWO; nProgramNum++) {
		cout << F("\tProgram[") << nProgramNum << F("]") << endl;
		cout << F("\tProgramMode::") << int(stDataSet.stSetup.ProgramMode[nProgramNum]) << endl;
		cout << F("\tInjectionFlow::") << stDataSet.stSetup.InjectionFlow[nProgramNum] << endl;
		cout << F("\tExtractionFlow::") << stDataSet.stSetup.ExtractionFlow[nProgramNum] << endl;
		cout << F("\tInjectionTime::") << stDataSet.stSetup.InjectionTime[nProgramNum] << endl;
		cout << F("\tExtractionTime::") << stDataSet.stSetup.ExtractionTime[nProgramNum] << endl;
	}

	cout << F("\tInjectiomMode::") << int(stDataSet.stSetup.InjectionMode) << endl;
	cout << F("\tProgramSetNum::") << int(stDataSet.stSetup.ProgramSetNum) << endl;
	cout << F("\tInjectionPower::") << stDataSet.stSetup.InjectionPower << endl;

	for (int nHeater = 0; nHeater < 4; nHeater++) {
		cout << F("\tSetValueTemp[") << nHeater << F("]::") << stDataSet.stSetup.SetValueTemp[nHeater] << endl;
		cout << F("\tP_Band[") << nHeater << F("]::") << stDataSet.stSetup.ProportionalBandTemp[nHeater] << endl;
		cout << F("\tI_Time[") << nHeater << F("]::") << stDataSet.stSetup.IntegralTime[nHeater] << endl;
		cout << F("\tD_Time[") << nHeater << F("]::") << stDataSet.stSetup.DerivativeTime[nHeater] << endl;
		cout << F("\tMV_High[") << nHeater << F("]::") << stDataSet.stSetup.MVHighLimit[nHeater] << endl;
	}

	for (int nSensor = 0; nSensor < 4; nSensor++) {
		cout << F("\tInputBiasAdded[") << nSensor+1 << F("]::") << stDataSet.stSetup.SensorInputBias[nSensor] << endl;
	}

	for (int nSensor = 0; nSensor < 3; nSensor++) {
		cout << F("\tPressureSensor[") << nSensor << F("]::ZERO=") << stDataSet.stSetup.ZeroShift[nSensor] << endl;
		cout << F("\tPressureSensor[") << nSensor << F("]::SPAN=") << stDataSet.stSetup.SpanFactor[nSensor] << endl;
	}

	cout << F("\tMotorRun::") << int(stDataSet.stSetup.MotorRun) << endl;
	cout << F("\tMotorStandbyMode::") << int(stDataSet.stSetup.MotorStandbyMode) << endl;
	cout << F("\tMotorPower::") << stDataSet.stSetup.MotorPower << endl;
#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	cout << F("\tMotorAccTime::") << stDataSet.stSetup.MotorAccTime << endl;
#endif
	cout << F("\tLampMode::") << int(stDataSet.stSetup.LampMode) << endl;
	cout << F("\tLampIntensity::") << int(stDataSet.stSetup.LampIntensity) << endl;
	for (int nProgramNum = PROGRAM_SET_ONE; nProgramNum <= MANUAL_SET_ONE; nProgramNum++) {
		cout << F("\tLedPixelMode[") << nProgramNum << F("]::") << int(stDataSet.stSetup.LedPixelMode[nProgramNum]) << endl;
	}
	cout << F("\tTime::") << int(stDataSet.stSetup.Year) << F("/") << int(stDataSet.stSetup.Month) << F("/") << int(stDataSet.stSetup.Day) << F(", ") << int(stDataSet.stSetup.Weekday) << endl;
	cout << F("\tDate::") << int(stDataSet.stSetup.Hour) << F(":") << int(stDataSet.stSetup.Minute) << F(":") << int(stDataSet.stSetup.Second) << endl;
	cout << F("\tPowerSaveMode::") << int(stDataSet.stSetup.PowerSaveMode) << endl;
	cout << F("\tStartTime::") << int(stDataSet.stSetup.StartHour) << F(":") << int(stDataSet.stSetup.StartMinute) /*<< F(":") << int(stDataSet.stSetup.StartSecond)*/ << endl;
	cout << F("\tEndTime::") << int(stDataSet.stSetup.EndHour) << F(":") << int(stDataSet.stSetup.EndMinute) /*<< F(":") << int(stDataSet.stSetup.EndSecond)*/ << endl;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	cout << F("\tLowInletAlarm::") << int(stDataSet.stSetup.LowInletAlarm) << endl;
	cout << F("\tLowInletAlarmPress::") << stDataSet.stSetup.LowInletPress << endl;
#endif
	cout << F("\tCleaningMode::") << int(stDataSet.stSetup.CleaningMode) << endl;
#if AUTOMATION_SENSOR_ATTACHED
	cout << F("\tAutomationMode::") << int(stDataSet.stSetup.AutomationMode) << endl;
	cout << F("\tAutoFlushingTime::") << stDataSet.stSetup.AutoFlushingTime << endl;
	cout << F("\tAutoBrewWaitTime::") << stDataSet.stSetup.AutoBrewWaitTime << endl;
#else
	cout << F("\tFlushingDetectionMode::") << int(stDataSet.stSetup.FlushingDetectionMode) << endl;
	cout << F("\tFlushingDetectionTime::") << stDataSet.stSetup.FlushingDetectionTime << endl;
	cout << F("\tFlushingDetectionPress::") << stDataSet.stSetup.FlushingDetectionPress << endl;
#endif
	cout << F("\tDefaultSet::") << int(stDataSet.stSetup.DefaultSet) << endl;

	cout << F("ST_MAINTENANCE::") << endl;
	cout << F("\tErrorHistoryIndex::") << int(stDataSet.stMaintenance.ErrorHistoryIndex) << endl;
	cout << F("\tErrorAddressIndex::") << int(stDataSet.stMaintenance.ErrorAddressIndex) << endl;
	cout << F("\tErrorCode::") << stDataSet.stMaintenance.ErrorCode << endl;
	cout << F("\tRecentErrorCode::") << stDataSet.stMaintenance.RecentErrorCode << endl;
	cout << F("\tTotFlowQuantity::") << stDataSet.stMaintenance.TotFlowQuantity << endl;
	cout << F("\tTotBrewCount::") << stDataSet.stMaintenance.TotBrewCount << endl;
	cout << F("\tOverflowed::") << int(stDataSet.stMaintenance.Overflowed) << endl;

	cout << F("ST_ERROR::") << endl;
	cout << F("\tFailToStandbyBrewPressure::") << stError.bFailToStandbyBrewPressure << endl;
	cout << F("\tBrewPressureOver::") << stError.bIsBrewPressureOver << endl;
	cout << F("\tBrewPressureUnder::") << stError.bIsBrewPressureUnder << endl;
	cout << F("\tSystemOverload::") << stError.bIsSystemOverloaded << endl;
#if PUMP_PRESSURE_SENSOR_ATTACHED
	cout << F("\tLowInletPressure::") << stError.bIsLowInletPressure << endl;
#endif
	cout << F("\tGroupheadOverheated::") << stError.bIsGroupheadOverheated << endl;
	cout << F("\tBrewBoilerOverheated::") << stError.bIsBrewBoilerOverheated << endl;
	cout << F("\tPreheatBoilerOverheated::") << stError.bIsPreheatBoilerOverheated << endl;
	cout << F("\tGroupheadUnderheated::") << stError.bIsGroupheadUnderheated << endl;
	cout << F("\tBrewBoilerUnderheated::") << stError.bIsBrewBoilerUnderheated << endl;
	cout << F("\tPreheatBoilerUnderheated::") << stError.bIsPreheatBoilerUnderheated << endl;
	cout << F("\tTemperatureSensorError::") << stError.bIsTemperatureSensorError << endl;

	cout << F("\tFlowmeterFailed::") << stError.bIsFlowmeterFailed << endl;
	cout << F("\tMotorAlarmReceived::") << stError.bIsMotorAlarmReceived << endl;
	cout << F("\tMaintenanceRequired::") << stError.bIsMaintenanceRequired << endl;
	cout << F("\tFlowrateExceeded::") << stError.bIsFlowrateExceeded << endl;
#if AUTOMATION_SENSOR_ATTACHED
	cout << F("\tOpticalFiberAmpUnstable::") << stError.bIsOpticalFiberUnstable << endl;
#endif
#ifdef _USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_
	cout << F("\tEEPROMWorn::") << stError.bIsEEPROMWorn << endl;
#endif

	// hLamp.printStatus();
}

void print_BrewingLogHeader(void)
{
	cout << F("\n\rBREW IDX : [") << stDataSet.stMaintenance.TotBrewCount + 1 << F("]") << endl;
	cout << F("BREW DATE: ") << int(stCurTimeDate.year) << F("/") << int(stCurTimeDate.month) << F("/") << int(stCurTimeDate.day) << F(" ")
							 << int(stCurTimeDate.hour) << F(":") << int(stCurTimeDate.minute)<< F(":") << int(stCurTimeDate.second) << endl;

	cout << F("LOG DATA : ")
		 << F("[SHOT TIME] ") << F("[FLOW CNT] ") << F("[FLOW RATE] ") << F("[FLOWRATE AVG] ")
	// #if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
		 << F("[BOUT TEMP] ")
	// #endif
	#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << F("[BREW TEMP] ")
	#endif
	#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
		 << F("[GRPH TEMP] ")
	#endif
	#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << F("[PREH TEMP] ")
	#endif
	#if BREW_PRESSURE_SENSOR_ATTACHED
		 << F("[BREW PRES] ")
		 << F("[BREW PRES AVG] ")
	#endif
	#if PUMP_PRESSURE_SENSOR_ATTACHED
		 << F("[PUMP PRES] ")
		 << F("[PUMP PRES AVG] ")
	#endif
	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
		 << F("[MOTOR SPD] ")
		 << F("[MOTOR CURRENT] ")
	#endif
		 << F("\n\r") << endl;
}

void print_BrewingStandbyObservation(void)
{
	static unsigned long _ulLogPrintTime = 0UL;

	if (millis() - _ulLogPrintTime >= 5000UL) {

		_ulLogPrintTime = millis();

		#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
		cout << F("PREH TEMP: ") << stDataSet.stAcquisition.PreheatTemp << endl;
		#endif
		#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
		cout << F("BREW TEMP: ") << stDataSet.stAcquisition.BrewTemp << endl;
		#endif
		#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
		cout << F("GRPH TEMP: ") << stDataSet.stAcquisition.GroupTemp << endl;
		#endif
		#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
		cout << F("BOUT TEMP: ") << stDataSet.stAcquisition.BrewWaterTemp << endl;
		#endif
		#if BREW_PRESSURE_SENSOR_ATTACHED
		cout << F("BREW PRES: ") << stDataSet.stAcquisition.BrewPress << endl;
		#endif
		#if PUMP_PRESSURE_SENSOR_ATTACHED
		cout << F("PUMP PRES: ") << stDataSet.stAcquisition.PumpPress << endl;
		#endif
		#if AUX_CURRENT_SENSOR_ATTACHED
		cout << F("AUX  PRES: ") << stDataSet.stAcquisition.AuxPress << endl;
		#endif
		cout << F("-------------------STANDBY-------------------") << endl;

	}	
}
#else

#if defined(_USE_BREW_PROFILE_ACQUISITION_)
void print_BrewingLogHeader(void)
{
	cout << F("ENT::") << F("start") << endl;

	cout << F("IDX::") << stDataSet.stMaintenance.TotBrewCount + 1 << endl;

	cout << F("DATE::")
		 << int(stCurTimeDate.year) << F("-") << int(stCurTimeDate.month) << F("-") << int(stCurTimeDate.day) << F("::")
		 << int(stCurTimeDate.hour) << F(":") << int(stCurTimeDate.minute)<< F(":") << int(stCurTimeDate.second)
	<< endl;
}

void print_SensorObservation(void)
{
	static unsigned long _ulLogPrintTime = 0UL;

	if (millis() - _ulLogPrintTime >= PERIOD_TO_SEND_SENSOR_OBSERVATION) {

		_ulLogPrintTime = millis();

		cout << F("SEN::")
			 << stDataSet.stAcquisition.BrewTemp 				<< F("::")
			 << stDataSet.stAcquisition.GroupTemp 				<< F("::")
			 << stDataSet.stAcquisition.PreheatTemp 			<< F("::")
			 << stDataSet.stAcquisition.BrewPress
		<< endl;

	}
}
#endif

#endif

