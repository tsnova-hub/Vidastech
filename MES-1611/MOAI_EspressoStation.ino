//////////////////////////////////////////////////////////////////////////////////////////
//
//	Source Code for Controller of MOAI Espresso Station (MES1611-SD V2.0)
//
//	Ver. : 2.0.0 (released)
//	H/W Ver. : 2.0 (released)
//	Board: Optimized for CONTROLLINO MAXI (COMPATIBLE WITH ARDUINO MEGA)
//
//	- Company : Vidastech Inc.
//	--------------------------------------------------------------------------------------
//	- Written by WabaRyu to Develop MOAI Espresso Station
//	- Last fixed : 2019 / 06 / 03
//	- Last Build : 190603
// 
//	  "board_id": "sketchbook.controllino.avr.controllino_maxi" 
//	  "ide_path": "Arduino Compiler/Arduino 1.8.5"
//	  "lib_paths":
//	      "libraries/StandardCplusplus", 
//	      "libraries/Timer3", 
//	      "libraries/StopWatch-0.1.5", 
//	      "libraries/ArduinoThread-2.1.1", 
//	      "libraries/FastLED-3.1.8", 
//	      "libraries/LiquidTWI-1.5.1", 
//	      "libraries/ModbusMaster-2.0.0", 
//	      "libraries/Controllino-1.1.2", 
//	      "hardware/controllino/avr/libraries/Wire", 
//	      "hardware/controllino/avr/libraries/SPI", 
//	      "hardware/controllino/avr/libraries/EEPROM"
//	  "shield": "MES-1611-SD-InterfaceBD" (>= v1.0.0)
//	  "rgbled": "RGBLEDStick 16x5050 V1"
//
//	- Change Log :
//		_USE_APPLICATION_DEBUG_MSG_ Disabled
//		modify (v1.2.1) - SW Version Informatin (ex: "v1.2.1" --> "1.2 r (1)")
//		modify (v1.2.1) - Factory Default Set with Rotary Encoder Switch
//		modify (v1.2.1) - First Factory Default Set with RTC reset
//		feature(v1.2.1) - System Notice (EEPROM Worn out)
//		bugfix (v1.2.1) - Reset Menu ('Program Num' mismatched, Reset Brew Profile Confidence Value)
//		Hotfix (v1.2.1) - EEPROM Wear Leveling (feature-added: eeprom verify/write)
//		Hotfix (v1.2.2) - MotorPump Controller reset problem (system freezing issue)
//		feature(v1.2.3) - System Exception Handling Enhancement (Handler, Error/Notice Message Indication)
//		modify (v1.3.0) - Tune Brightness dedicated to RGBLEDStick 16x5050 v1.0 (RGBLEDStick16x5050-V1)
//		bugfix (v1.3.0) - Injection Power and Motor Power (floating number fault issue)
//		modify (v1.3.0) - Flushing Detection Menu (Time, Pressure Range : 1 ~ 8 [sec], 1 ~ 8 [bar])
//		modify (v1.3.0) - LED Color (RGB_BREW_INJECTION : "Brown" --> "Sienna")
//		modify (v1.3.0) - Moving Averaged Temperature Calculation (Window : 10 samples, measure period : 250 msec)
//		modify (v1.3.0) - Moving Averaged Pressure (ADC Digit Level), Pressure Sensor Calibration Enhanced
//		modify (v1.3.0) - Pressure Sensor Setup Menu Renewal (Notice/Fail Message Added)
//		modify (v1.3.0) - Pressure Sensor Measuring Period : 25msec, Moving Average Window : 20samples
//		modify (v1.3.0) - Total Water Quantity Reset Function Release to User (can clear Maintenance Required Notice)
//		modify (v1.3.0) - Flushing Detection Params : 5.0 [sec], 2.0 [bar] (default)
//		modify (v1.3.0) - Brew Boiler Pressure standby Operation Enhanced
//		modify (v1.3.1) - Arduino AVR, Controllino Boards Package, External open-source libraries Upgrade
//		hotfix (v1.3.1) - Low Flowrate Configuration (Cleaning Mode, Standby Brew Boiler Pressure, Time to check Low Brew Boiler Pressure)
//		modify (v1.3.2) - Brew Boiler Pressure standby Operation Enhanced (upper brew boiler pressure, min motor running time to standby)
//		modify (v1.3.2) - Brew Boiler Pressure standby Operation Enhanced (add temperature condition : brew or preheat boiler >= 63 deg.C)
//		feature(v1.3.2) - Decaying RPM of Flowmeter, Tunning the Time to detect 'Under Brew Pressure' error (10 --> 30 [sec])
//		feature(v1.3.3) - LED Pixel Mode : add 'Shot Time' Mode (70 [sec] timelapse)
//		modify (v1.3.3) - Flushing Detection Params : Time (1.0~30.0 [sec]), Pressure (0.0~10.0 [bar])
//		modify (v1.3.3) - Min Effective Brew Time for Flushing Detection (8.0 --> 10.5 [sec]) : CEspresso Class
//		modify (v1.3.3) - Remove Constraint : Min Effective Motor Power for Flushing Detection Mode (31.0 [%], 1.0 [%] for OZBV-10A-D2M2)
//		modify (v1.3.4) - LED Pixel Mode : hold timelapse palett on static/blink mode for 1.5 [sec] (when brew stop)
//		modify (v1.3.4) - Lamp Setup : apply changes immediately on Lamp Setup Menu
//		bugfix (v1.3.4) - Lamp bugfix : add turnoff EMG. to CLamp class and apply turnoff EMG. to System Exception routine
//		feature(v1.4.0) - Build Messages : MES Configurations (use Preprocessor directive 'pragma message')
//		modify (v1.4.0) - Refactoring CEspresso, CMotorPump, Variables (Flushing Detection), etc.
//		modify (v1.4.0) - Refactoring global and local static variables, etc.
//		modify (v1.4.0) - Display Day of week(calculated in CRTClock) on Time Date Setup Menu
//		feature(v1.4.0) - add TM4 Communication Setting and enhance Modebus RTU in TM4 class (must reboot to init TM4)
//		modify (v1.4.0) - init text-lcd and rgb led before TM4 initialization (to display notice message for TM4 Comm. Setting)
//		modify (v1.4.0) - update the procedure that console out Modbus RTU Exception Message (CTM4Controller, CMotorPumpBLDC)
//		feature(v1.4.0) - add TM4 Sensor Error (Level: System Error)
//		feature(v1.4.0) - add Flowmeter tuning factor (CFlowmeter class : Flow count vs. volume)
//		modify (v1.4.0) - change MV High Limit to 80 [%] (Pre-heat, Brew), 90 [%] (Grouphead)
//		modify (v1.4.0) - change Console Error Message to 'EEPROM Write Failed.'
//		feature(v1.4.0) - add Brew-Water temperature sensor setup to CTM4Controller class for temperature-test (K.L type sensor)
//		bugfix (v1.4.0) - Quick Sorting Problems
//		modify (v1.4.0) - Error History List : Descending ordered Index
//		bugfix (v1.4.0) - LED Pixel color update fail in first 30 [sec] (in static mode)
//		feature(v1.4.0) - add Rotary Encoder Acceleration (CRotaryEncoder class)
// 		modify (v1.4.0) - tune sensibility of Rotary Encoder Acceleration (in Program 1/2-time, PID, MV setup)
//		feature(v2.0.0) - Motor Pump Revision (OZBV-10A-D2M2 + Procon Motor Pump)
//		bugfix (v2.0.0) - Upper Limit Value Problem and Signed Integer Display Problem (CMotorPump - ACC/DEC Time, LCD APIs)
//		feature(v2.0.0) - Delayed Lamp-Turnoff (to turn off in 1 [sec] when brew stop, add capacitor(>1000 [uF], 16 [v]) at PWM Output terminal)
//		bugfix (v2.0.0) - Injection / Extraction Time Setup (eeprom access : floating number fault issue)
//		feature(v2.0.0) - add Optical Fiber Amplifier (BF4RP) for Auto-Flushing
//		feature(v2.0.0) - add Auto-Brewing function in program mode('Red'/'Blue') with Optical Fiber Amplifier (BF4RP)
//		feature(v2.0.0) - add Automation Setup Menu integrated with Auto-Flushing and Auto-Brewing features
//		modify (v2.0.0) - optimize Automation Mode (Auto-Flushing/Auto-Brewing feature with COpticalFiber class)
//		modify (v2.0.0) - change default heating setpoint (Preheat Boiler) to 85 [deg. C]
//		feature(v2.0.0) - add Brew Boiler Pressure Stabilization before Brewing, so add Motor Standby 'SHOT' Mode (OFF, PREP, SHOT)
//		modify (v2.0.0) - enhance how to schedule Power Saving Mode with start/end-time (hour, minute, max 24hours) and modify Power Saving Menu
//		modify (v2.0.0) - enhance System Exception Handling under the environment of Power Saving Mode
//		bugfix (v2.0.0) - fix problems with Power Saving Pause/Resume and Power Saving Mode ON/OFF
// 		modify (v2.0.0) - tune sensibility of Rotary Encoder Acceleration in Grouphead/Brew Boiler/Preheat Boiler setup
// 		modify (v2.0.0) - turn off and disable Optical Fiber Amplifier while Power Saving
// 		modify (v2.0.0) - enhance PixelLED Operations while long/vlong-pushing (Brew-Button on Head)
// 		bugfix (v2.0.0) - optimize for dynamic memory size to be under 85.0 [%] (Problem : out of dynamic memory)
//		modify (v2.0.0) - Moving Averaged Temperature Calculation (Window : 5 samples, measure period : 250 msec)
//		modify (v2.0.0) - bugfix : flowmeter failure after auto-flushing and when it stops Brewing manually as soon as auto-brewing
//		feature(v2.0.0) - add brew profile acquisition (serial comm. with data acquisition app.)
//		modify (v2.0.0) - enhance Motor Stand-by Mode (re-define function of MOTOR_STANDBY_PREP and MOTOR_STANDBY_SHOT)
//		bugfix (v2.0.0) - problem : out of index (in CRTClock::getWeekday method)
//		modify (v2.0.0) - change to reset RTC when it set the system in factory default
//		modify (v2.0.0) - add lazy standing-by brew pressure to prevent motor pump noise and stress, change pressure conditions in Standby 'SHOT' mode
//
//	- Build Information :
//		Sketch uses 103,946 bytes (40.9%) of program storage space. Maximum is 253,952 bytes.
//		Global variables use 6,734 bytes (82.2%) of dynamic memory, leaving 1,458 bytes for local variables. Maximum is 8,192 bytes.
//
//////////////////////////////////////////////////////////////////////////////////////////


#include "MES_Arduino.h"
#include "MES_Peripheral_Interface.h"
#include "MES_Application.h"

#include "MES_PragmaMessages.h"


void setup()
{
	// TODO : place system initialization code in here.

	initialize_ArduinoPlatform();

	initialize_PeripheralDevice();


	print_SystemInformation();


	deactivate_AllUserInterface();

	display_SystemInitialization();

	initialize_SystemParameters();

	initialize_SystemThreads();

	initialize_AppThreads();
	

	display_SystemInformation();


	activate_SystemStateMachine();

	display_SystemActivation();

	activate_AllUserInterface();
}


void loop()
{
	// TODO : implement procedures to run repeatedly/or periodically.

	app_BrewStateMachine();

	app_SetupStateMachine();

	app_SystemExceptions();

	draw_LEDPixelOnUserInterface();

	display_TextLCDOnUserInterface();

	read_TM4ControllerObservation();

	update_SensorObservationAll();

	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	read_MotorControllerObservation();
	#endif

	#if defined(_SERIAL_COMMUNICATION_WITH_CLIENT_ENABLED_)
	serialResponse();
	#endif

	// delay(10UL);
}


//////////////////////////////////////////////////////////////////////////////////////////
//			SERIAL INTERRUPT HANDLER FOR SERIAL COMMUNICATION (WITH HOST / PERIPHERALs)

#if defined(_SERIAL_COMMUNICATION_WITH_CLIENT_ENABLED_)
// serialEvent, serialEvent1, serialEvent2, serialEvent3
void serialEvent()
{

}

void serialEvent1()
{

}

void serialEvent2()
{

}

void serialEvent3()
{

}

void serialResponse(void)
{

}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
//			INITIALIZATION OF ARDUINO PLATFORM (BOARD INIT. / CONSOLE INIT.)

void initialize_ArduinoPlatform(void)
{
	delay(SYSTEM_WAIT_FOR_PWR_ON_RESET);

	analogReference(ANALOG_READ_REF_TYPE);

	_console.begin(CONSOLE_BAUD_RATE, CONSOLE_COMM_CONFIG);
	_console.flush();
}


//////////////////////////////////////////////////////////////////////////////////////////
//			INITIALIZATION OF EXTERNAL PERIPHERALS AND CONTROLLINO'S INTERNAL PERIPHERALS

void initialize_PeripheralDevice(void)
{
	initialize_RTClock();

	initialize_ThermalOVL();

	initialize_PixelLED();

	initialize_TextLCD();

	initialize_TM4Controller();

	initialize_SolenoidValve();

	initialize_PressureSensor();

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	initialize_AnalogSensor();
#endif

#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
	initialize_ExternalSignal();
#endif

	initialize_MotorPump();

	initialize_Flowmeter();

#if AUTOMATION_SENSOR_ATTACHED
	initialize_OpticalFiberAmp();
#endif

	initialize_GroupheadLamp();

	initialize_BrewButton();

	initialize_RotaryEncoder();
}


//////////////////////////////////////////////////////////////////////////////////////////
//			INTERRUPT HANDLER INITIALIZATION (INTENAL TIMERS, EXTERNAL INTERRUPTS)

// Attach/Enable all Handlers(Resources) for Internal(Timer)/External(Peripherals) Interrupt Signals
void initialize_SystemThreads(void) 
{
	WaitForFirstSensorMeasurement();

	noInterrupts();
	
	// Set Timer3 Interrupt Handler
	attach_Timer3Interrupt(Timer3InterruptHandler, (unsigned long)TIMER3_INTERRUPT_PERIOD);

	// Set External Signal Interrupt Handler [INT 0] (not used, but polling periodically)
	// enable_ExternalInterrupt(TriggerBrewButton_Callback, PIN_INTERRUPT_NUM_0, BREW_BUTTON_TRIGGER_TYPE);

	// Set External Signal Interrupt Handler [INT 1]
	enable_ExternalInterrupt(TriggerRotaryEncoderST_Callback, PIN_INTERRUPT_NUM_1, ROTARY_ENCODER_STEP_TRIGGER_TYPE);
	
	// Set External Signal Interrupt Handler [INT 5] (enable when it's need only)
	// enable_ExternalInterrupt(TriggerWaterFlowmeter_Callback, PIN_INTERRUPT_NUM_5, FLOWMETER_TRIGGER_TYPE);

	// Set External Signal Interrupt Handler [INT 4]
	#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
	enable_ExternalInterrupt(TriggerExternalSignal_Callback, PIN_INTERRUPT_NUM_4, EXTERNAL_SIGNAL_TRIGGER_TYPE);
	#endif

	interrupts();
}


// Timer3 Interrupt Handler
void Timer3InterruptHandler(void)
{
	static unsigned int _nTimerIntCount = 0UL;

	_nTimerIntCount += TIMER3_INTERRUPT_PERIOD;

	if (!(_nTimerIntCount % BREW_BUTTON_POLLING_PERIOD)) {
		PollingBrewButton_Callback();
	}

	if (!(_nTimerIntCount % ROTARY_ENCODER_SW_POLLING_PERIOD)) {	
		PollingRotaryEncoderSW_Callback();
	}

	if (!(_nTimerIntCount % APP_THREAD_SHOULD_RUN_PERIOD)) {
		AppThreadShouldRun_Callback();
	}

	if (!(_nTimerIntCount % TIME_DATE_UPDATE_PERIOD)) {
		UpdatingCurrentTimeDate_Callback();
	}

	if (!(_nTimerIntCount % WATER_FLOW_STATUS_POLLING_PERIOD)) {
		PollingWaterFlowStatus_Callback();
	}

#if AUTOMATION_SENSOR_ATTACHED
	if (!(_nTimerIntCount % OPTICAL_FIBER_AMP_POLLING_PERIOD)) {
		PollingOpticalFiberAmp_Callback();
	}
#endif

#if defined(_LAMP_DELAYED_OFF_)
	if (!(_nTimerIntCount % LAMP_DELAYED_TURNOFF_POLLING_PERIOD)) {
		PollingGroupheadLampTimer_Callback();
	}
#endif
	
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	if (!(_nTimerIntCount % MOTOR_ALARM_POLLING_PERIOD)) {
		PollingMotorPumpAlarm_Callback();
	}
#endif

	_nTimerIntCount = _nTimerIntCount % MAX_TIEMR_INTERRUPT_COUNT;
}

