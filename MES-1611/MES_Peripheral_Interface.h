#ifndef _MES_PERIPHERAL_INTERFACE_H_
#define _MES_PERIPHERAL_INTERFACE_H_


#include "MES_Arduino.h"
#include "MES_DataSet_Definition.h"
#include "MES_PixelLED_Color.h"
#include "MES_SetupMenu_Template.h"


#include "RTC/CRTClock.h"

#include "Flowmeter/CFlowmeter.h"
#include "PressureSensor/CPressureSensor.h"
#include "TM4/CTM4Controller.h"
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
#include "MotorPump/CMotorPump.h"
#else
#include "MotorPump/CMotorPump_ozbv.h"
#endif
#include "RotaryEncoder/CRotaryEncoder.h"
#include "BrewButton/CBrewButton.h"
#include "AnalogSensor/CAnalogSensor.h"
#include "ExternalSignal/CExternalSignal.h"
#include "SolenoidValve/CSolenoidValve.h"
#if AUTOMATION_SENSOR_ATTACHED
#include "OpticalFiber/COpticalFiber.h"
#endif
#include "Lamp/CLamp.h"
#include "LCD/CTextLCD.h"
#include "PixelLed/CPixelLED.h"


#if defined(_USE_APPLICATION_DEBUG_MSG_)
#define _USE_PERIPHERAL_INTERFACE_DEBUG_MSG_
#endif


////////////////////////////////////////////////////////////
// Definition of Rotary Encoder (ST)

#define ROTARY_ENCODER_STEP_TRIGGER_TYPE		(RISING)


////////////////////////////////////////////////////////////
// Definition of Brew Push-Button

#define BREW_BUTTON_TRIGGER_TYPE				(FALLING)


////////////////////////////////////////////////////////////
// Definition of Water Flowmeter

#define FLOWMETER_TRIGGER_TYPE					(CHANGE)
#define FLOWRATE_VALUE_LIMIT					(12.0F)									// (16.0F)
#define FLOWMETER_NOISE_REDUCTION_TIME			(20UL)									// 20 msec


////////////////////////////////////////////////////////////
// Definition of External Interruptable Signal 

#define EXTERNAL_SIGNAL_TRIGGER_TYPE			(FALLING)
#define EXTERNAL_SIGNAL_ACTIVE_TYPE				(CExternalSignal::ACTIVE_LOW)


////////////////////////////////////////////////////////////
// Definition of Optical Fiber Amplifier

#define OPTICAL_FIBER_AMP_ACTIVE_TYPE			(COpticalFiber::LIGHT_ON)
#define OPTICAL_FIBER_DARK_DURATION				(1000UL)								// > 1000 msec
#define OPTICAL_FIBER_LIGHT_HIGH_DURATION		(1000UL)								// > 1000 msec

#define OPTICAL_FIBER_AMP_STABILIZE_DURATION	(OPTICAL_FIBER_LIGHT_HIGH_DURATION)


////////////////////////////////////////////////////////////
// Definition of Tune Factor of Analog Pressure Sensor 

#define BREW_BOILER_PRESSURE_SENSOR_CH			(0)
#define PUMP_PRESSURE_SENSOR_CH					(1)
#define AUX_PRESSURE_SENSOR_CH					(2)

#define PRESSURE_TUNE_FACTOR					(0.0F)
#define PRESSURE_VALUE_LIMIT					(12.0F)									// 16.0F Tested

#define PRESSURE_SENSOR_ZERO_SHIFT				(DEFAULT_ZERO_SHIFT)
#define PRESSURE_SENSOR_SPAN_FACTOR				(DEFAULT_SPAN_FACTOR)


////////////////////////////////////////////////////////////
// Definition of Tune Factor of Analog Sensor 

#define ANALOG_TUNE_FACTOR						(0.0F)
#define ANALOG_SIGNAL_CONDITION					(VOLTAGE_DIVIDE_FACTOR24)


////////////////////////////////////////////////////////////
// Definition of Solenoid Valve

#define SOLENOID_VALVE_TYPE						(CSolenoidValve::NORMAL_CLOSE)


////////////////////////////////////////////////////////////
// Autonics TM4 Channel Mapping for MOAI Espresso Station (Modbus RTU)

#define TM4_ALL_HEATER_CHANNEL					(CTM4Controller::TM4ALL)
#define TM4_PREHEAT_CHANNEL						(CTM4Controller::TM4CH1)
#define TM4_BREWING_CHANNEL						(CTM4Controller::TM4CH2)
#define TM4_GROUPHEAD_CHANNEL					(CTM4Controller::TM4CH3)
#define TM4_BREWWATER_CHANNEL					(CTM4Controller::TM4CH4)
#define TM4_MVHIGH_SETUP_CHANNEL				(TM4_BREWWATER_CHANNEL)
#define TM4_INPUT_BIAS_SETUP_CHANNEL			(TM4_BREWWATER_CHANNEL+1)

#define TM4_TEMPERATURE_SENSOR_OPENED			(TM4_SENSOR_ERROR_OPENED)
#define TM4_TEMPERATURE_SENSOR_UPPER_RANGE		(TM4_SENSOR_ERROR_UPPER_RANGE)
#define TM4_TEMPERATURE_SENSOR_LOWER_RANGE		(TM4_SENSOR_ERROR_LOWER_RANGE)

#define TM4_INVALID_SETPOINT_READ				(TM4_INVALID_TEMPERATURE)
#define TM4_INVALID_COILSTATUS_READ				(TM4_INVALID_COIL_STATUS)

#define MIN_BOILER_TEMPERATURE					(DEFAULT_HEATING_MIN_TEMPERATURE)
#define MAX_BOILER_TEMPERATURE					(DEFAULT_HEATING_MAX_TEMPERATURE)

#define MIN_SENSOR_INPUT_BIAS					(DEFAULT_MIN_SENSOR_INPUT_BIAS)
#define MAX_SENSOR_INPUT_BIAS					(DEFAULT_MAX_SENSOR_INPUT_BIAS)

#define DEFAULT_BOILER_TEMPERATURE				(DEFAULT_HEATING_SV_TEMPERATURE)
#define DEFAULT_TEMPERATURE_SENSOR_INPUT_BIAS	(DEFAULT_SENSOR_INPUT_BIAS)


#define PREHEAT_HEATER_P_BAND					(DEFAULT_CH1_HEATING_PROPORTIONAL_BAND)		// 0.1 ~ 999.9
#define PREHEAT_HEATER_I_TIME					(DEFAULT_CH1_HEATING_INTEGRAL_TIME)			// 0 ~ 9999
#define PREHEAT_HEATER_D_TIME					(DEFAULT_CH1_HEATING_DERIVATION_TIME)		// 0 ~ 9999
#define PREHEAT_BOILER_HEATING_POWER			(DEFAULT_CH1_HEATING_MV_HIGH_LIMIT)			// 0.1 ~ 100.0

#define BREW_HEATER_P_BAND						(DEFAULT_CH2_HEATING_PROPORTIONAL_BAND)		// 0.1 ~ 999.9
#define BREW_HEATER_I_TIME						(DEFAULT_CH2_HEATING_INTEGRAL_TIME)			// 0 ~ 9999
#define BREW_HEATER_D_TIME						(DEFAULT_CH2_HEATING_DERIVATION_TIME)		// 0 ~ 9999
#define BREW_BOILER_HEATING_POWER				(DEFAULT_CH2_HEATING_MV_HIGH_LIMIT)			// 0.1 ~ 100.0

#define GROUPHEAD_HEATER_P_BAND					(DEFAULT_CH3_HEATING_PROPORTIONAL_BAND)		// 0.1 ~ 999.9
#define GROUPHEAD_HEATER_I_TIME					(DEFAULT_CH3_HEATING_INTEGRAL_TIME)			// 0 ~ 9999
#define GROUPHEAD_HEATER_D_TIME					(DEFAULT_CH3_HEATING_DERIVATION_TIME)		// 0 ~ 9999
#define GROUPHEAD_HEATING_POWER					(DEFAULT_CH3_HEATING_MV_HIGH_LIMIT)			// 0.1 ~ 100.0


////////////////////////////////////////////////////////////
// Definition for Text LCD Display Unit (I2C)

#define LCD_I2C_ADDR							(0x00)
#define LCD_MAX_COL 							(NUM_COLUMNS)
#define LCD_MAX_LINE 							(NUM_LINES)
#define LCD_BACKLIGHT_COLOR						(CTextLCD::GREEN)

#define MAX_INTEGER_VALUE_DIGIT					(3)
#define MAX_INTEGER_VALUE_NUM					(1000UL)

#define MAX_INTEGRAL_DERIVATIVE_VALUE_DIGIT		(4)
#define MAX_INTEGRAL_DERIVATIVE_VALUE_NUM		(10000UL)

#define MAX_INTEGER_ADC_VALUE_DIGIT				(4)
#define MAX_INTEGER_ADC_VALUE_NUM				(1024UL)

#define MAX_INTEGER_MOTOR_ACC_VALUE_DIGIT		(5)
#define MAX_INTEGER_MOTOR_ACC_VALUE_NUM			(100000UL)

#define MAX_FLOAT_VALUE_DIGIT					(5)
#define MAX_FLOAT_VALUE_NUM						(1000.0F)

#define MAX_FLOAT_SHOTTIME_VALUE_DIGIT			(6)
#define MAX_FLOAT_SHOTTIME_VALUE_NUM			(1000.0F)

#define MAX_FLOAT_FLOWQUANTITY_VALUE_DIGIT		(6)
#define MAX_FLOAT_FLOWQUANTITY_VALUE_NUM		(10000.0F)

#define MAX_FLOAT_RINSING_TIME_VALUE_DIGIT		(4)
#define MAX_FLOAT_RINSING_TIME_VALUE_NUM		(100.0F)

#define MAX_FLOAT_PRESSURE_VALUE_DIGIT			(4)
#define MAX_FLOAT_PRESSURE_VALUE_NUM			(100.0F)

#define MAX_FLOAT_INPUT_BIAS_VALUE_DIGIT		(4)
#define MAX_FLOAT_INPUT_BIAS_VALUE_NUM			(100.0F)

#define MAX_LONG_VALUE_DIGIT					(7)
#define MAX_LONG_VALUE_NUM						(10000000UL)

#define MAX_TIMEDATE_VALUE_DIGIT				(2)
#define MAX_TIMEDATE_VALUE_NUM					(100UL)

#define MAX_EXCEPTION_CODE_DIGIT				(7)
#define MAX_EXCEPTION_CODE_NUM 					(0xFFFF)

#define MAX_VERSION_CODE_DIGIT					(4)
#define MAX_VERSION_CODE_NUM					(100.0F)


////////////////////////////////////////////////////////////
// Definition for Controllino MAXI RTC (SPI)

#define RTC_CHIP_SELECT_ADDR					(0x00)
#define RTC_DEFAULT_DAY							(RTC_FACTORY_DEFAULT_DAY)
#define RTC_DEFAULT_MONTH						(RTC_FACTORY_DEFAULT_MONTH)
#define RTC_DEFAULT_YEAR						(RTC_FACTORY_DEFAULT_YEAR)

#define RTC_SUN									(CRTClock::SUN)
#define RTC_MON 								(CRTClock::MON)
#define RTC_TUE 								(CRTClock::TUE)
#define RTC_WED									(CRTClock::WED)
#define RTC_THR 								(CRTClock::THU)
#define RTC_FRI									(CRTClock::FRI)
#define RTC_SAT 								(CRTClock::SAT)


////////////////////////////////////////////////////////////
// Definition of BLDC Motor Controller (PWM)

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)

#define MOTOR_LOWER_POWER						(_MIN_DUTY_RATE_)	// rate=0%
#define MOTOR_UPPER_POWER						(_MAX_DUTY_RATE_)	// rate=100%

#else

#define MOTOR_LOWER_POWER						(_ZERO_RPM_RATE_)	// rate=0%		(for OZBV-10A-D2M2)
#define MOTOR_UPPER_POWER						(_MAX_RPM_RATE_)	// rate=100%	(for OZBV-10A-D2M2)

#define MOTOR_ACC_TIME							(_DEFAULT_ACC_TIME_)//				(for OZBV-10A-D2M2)

#endif


////////////////////////////////////////////////////////////
// Definition of LED Pixel (Serial Dout)

#define LEDPIXEL_STATIC_MODE_DRAWING_INTERVAL	(ON_STATIC_DRAW_INTERVAL)
#define LEDPIXEL_BLINK_MODE_DRAWING_INTERVAL 	(ON_BLINK_DRAW_INTERVAL)
#define LEDPIXEL_BREATHE_MODE_DRAWING_INTERVAL	(ON_BREATH_INC_INTERVAL)
#define LEDPIXEL_PUSHING_MODE_DRAWING_INTERVAL	(ON_PUSHING_DRAW_INTERVAL)

#define LEDPIXEL_PUSHING_MODE_HOLD_INTERVAL		(ON_PUSHING_HOLD_INTERVAL)
#define LEDPIXEL_TIMELAPSE_MODE_HOLD_INTERVAL 	(ON_TIMELAPSE_HOLD_INTERVAL)

#define LEDPIXEL_TIMELAPSE_FIRST_DIGIT_NUM		(LED_TIMELAPSE_FIRST_DIGIT_NUM)
#define LEDPIXEL_TIMELAPSE_SECOND_DIGIT_NUM		(LED_TIMELAPSE_SECOND_DIGIT_NUM)



////////////////////////////////////////////////////////////
// Interface(Class Instances) to External Peripheral Devices

// Handle/Instance of Controllino MAXI RTC
extern CRTClock hRTClock;

// Handle/Instance of Controllino MAXI /OVL
// extern CThermalOVL hThermalOVL;

// Handle/Instance of Flowmeter Sensor
extern CFlowmeter hFlowmeter;

// Handle/Instance of Brew-Boiler Pressure Sensor
extern CPressureSensor hPressureBrewBoiler;
#if PUMP_PRESSURE_SENSOR_ATTACHED
extern CPressureSensor hPressureFeed;
#endif

// Handle/Instance of Analog Input Sensor
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
extern CAnalogSensor hAnalogInput;
#endif

// Handle/Instance of External Input Signal
#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
extern CExternalSignal hExternalSignal;
#endif

// Handle/Instance of TM4 Controller
extern CTM4Controller hTM4Controller;

// Handle/Instance of BLDC Motor Controller
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
extern CMotorPump hMotorPump;
#else
extern CMotorPumpBLDC hMotorPump;
#endif

// Handle/Instance of Optical Fiber Amp
#if AUTOMATION_SENSOR_ATTACHED
extern COpticalFiber hOpticalFiber;
#endif

// Handle/Instance of BREW Solenoid Valve
extern CSolenoidValve hBrewValve;

// Handle/Instance of Brew Push-Button
extern CBrewButton hBrewButton;

// Handle/Instance of Rotary Encoder IJR
extern CRotaryEncoder hRotaryEncoder;

// Handle/Instance of Grouphead Lamp
extern CLamp hLamp;

// Handle/Instance of Text LCD Module
extern CTextLCD hTextLCD;

// Handle/Instance of RGB LED
extern CPixelLED hPixelLED;


////////////////////////////////////////////////////////////
// External Peripheral Interfaces

// Flowmeter
void initialize_Flowmeter(void);
void tune_Flowmeter(void);
void start_Flowmeter(void);
void stop_Flowmeter(void);
void reset_Flowmeter(void);
unsigned int get_FlowmeterCNT(void);
float get_WaterFlowQuantity(const enum CFlowmeter::WaterUnit nUnit);
float get_AveragedFlowmeterQTY(enum CFlowmeter::FlowUnit nUnit = CFlowmeter::UNIT_RPM);
float get_FlowmeterFactor(void);
#if defined(_USE_FLOWMETER_PROC_MODE_)
void postProcessing_Flowmeter(void);
#endif

void PollingWaterFlowStatus_Callback(void);
void TriggerWaterFlowmeter_Callback(void);
void enable_FlowmeterInterrupt(void);
void disable_FlowmeterInterrupt(void);

// Pressure Sensor
void initialize_PressureSensor(void);
void set_PressureSensorZeroShift(CPressureSensor& rhPressureSensor);
void set_PressureSensorSpanFactor(CPressureSensor& rhPressureSensor);
void reset_PressureSensorAdjustment(CPressureSensor& rhPressureSensor);
void set_PressureSensorZeroShiftWithValue(CPressureSensor& rhPressureSensor, const float fZeroShift);
void set_PressureSensorSpanFactorWithValue(CPressureSensor& rhPressureSensor, const float fSpanFactor);
void reset_PressureSensor(CPressureSensor& rhPressureSensor);
void reset_PressureSensor(void);
float get_PressureSensorValue(CPressureSensor& rhPressureSensor);
float get_BrewBoilerPressureValue(void);
#if PUMP_PRESSURE_SENSOR_ATTACHED
float get_PumpPressureValue(void);
#endif
#if AUX_CURRENT_SENSOR_ATTACHED
float get_AuxPressureValue(void);
#endif

// Temperature Controller (TM4)
void initialize_TM4Controller(void);
void control_preRS485Transmission(void);
void control_postRS485Trasmission(void);
void set_ModbusNodeAddress(const unsigned char nSlaveAddr);
bool setup_TM4ControllerCommunication(void);
bool reset_TM4ControllerToFactoryDefault(void);
#if defined(_USE_APPLICATION_DEBUG_MSG_)
void verify_TM4ControllerFactoryDefault(void);
#endif
bool isTM4TemperatureSensorAlarm(void);
unsigned int verify_TM4TemperatureSensorAlarm(const unsigned int nErrorCode);
void measure_TM4ControllerTemperatureSensorAll(void);
void measure_TM4ControllerTemperatureSensorWithChannel(const enum CTM4Controller::TM4Channel nChannel);
void measure_TM4ControllerHeatingMVAll(void);
void measure_TM4ControllerHeatingMVWithChannel(const enum CTM4Controller::TM4Channel nChannel);

void PollingTM4Temperature_Callback(void);

// Brew Button
void initialize_BrewButton(void);
void enable_BrewButton(void);
void disable_BrewButton(void);

void PollingBrewButton_Callback(void);

// Rotary Encoder with Switch
void initialize_RotaryEncoder(void);
void enable_RotaryEncoderDirection(void);
void disable_RotaryEncoderDirection(void);
void enable_RotaryEncoder(void);
void disable_RotaryEncoder(void);

void TriggerRotaryEncoderST_Callback(void);
void PollingRotaryEncoderSW_Callback(void);
void enable_RotaryEncoderInterrupt(void);
void disable_RotaryEncoderInterrupt(void);

// External Input Signal (General-Purpose Input or Interrupt)
#if EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED
void initialize_ExternalSignal(void);
void reset_ExternalSignal(void);
int get_ExternalSignalVAL(void);
unsigned int get_ExternalSignalCNT(void);
#if defined(_USE_EXTERNAL_SIGNAL_PROC_MODE_)
void preProcessing_ExternalSignal(void);
void postProcessing_ExternalSignal(void);
#endif
#endif

void TriggerExternalSignal_Callback(void);
void enable_ExternalSignalInterrupt(void);
void disable_ExternalSignalInterrupt(void);

// External Analog Input (Voltage)
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void initialize_AnalogSensor(void);
void reset_AnalogSensor(CAnalogSensor& rhAnalogInput);
float get_AnalogSensorValueInVoltage(void);
#endif

// Solenoid Valve
void initialize_SolenoidValve(void);
#if defined(_USE_SOLENOID_VALVE_PROC_MODE_)
void preProcessing_BrewSolenoidValve(void);
void postProcessing_BrewSolenoidValve(void);
#endif

// Controllino RTC
void initialize_RTClock(void);
void reset_TimeDateRTC(void);
void get_TimeDateRTC(stTimeDate_t& stCurTimeDate);
void get_DayOfWeekRTC(stSetupDataSet_t& stSetupDateData);
void set_TimeDateRTC(const stTimeDate_t& stSetTimeDate);

void PollingRealTimeClock_Callback(void);

// Controllino OVL
void initialize_ThermalOVL(void);
bool isThermalOverload(void);

// BLDC Motor Pump Controller
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
void initialize_MotorPump(void);
#else
// for OZBV-10A-D2M2
void initialize_MotorPump(void);
bool reset_MotorControllerToFactoryDefault(void);
#if defined(_USE_APPLICATION_DEBUG_MSG_)
void verify_MotorControllerFactoryDefault(void);
#endif
#endif
bool isMotorAlarm(void);
#if defined(_USE_MOTOR_PUMP_ALARM_PROC_MODE_)
void postAlarm_MotorPump(void);
#endif

void PollingMotorPumpAlarm_Callback(void);

// Optical Fiber Amplifier
#if AUTOMATION_SENSOR_ATTACHED
void initialize_OpticalFiberAmp(void);
void init_OpticalFiberAmp(void);
void enable_OpticalFiberAmp(void);
void disable_OpticalFiberAmp(void);
int get_OpticalFiberSignal(void);
int read_OpticalFiberSignal(void);
bool get_OpticalFiberDiagnosis(void);
void darkProcessing_OpticalFiberAmp(void);
void lightProcessing_OpticalFiberAmp(void);

void PollingOpticalFiberAmp_Callback(void);
#endif

// Grouphead Lamp
void initialize_GroupheadLamp(void);
void PollingGroupheadLampTimer_Callback(void);

// Text LCD
void initialize_TextLCD(void);

void print_Char(const unsigned char nPos, const unsigned char nLine, const unsigned char nVal);
void print_String(const unsigned char nPos, const unsigned char nLine, const char* pStr);
void print_Blank(const unsigned char nPos, const unsigned char nLine, const unsigned char nLen);

void print_IntegerValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue);
void print_LongValue(const unsigned char nPos, const unsigned char nLine, const unsigned long lValue);
void print_LongValueUnsigned(const unsigned char nPos, const unsigned char nLine, const unsigned long lValue);
void print_ExceptionCode(const unsigned char nPos, const unsigned char nLine, const unsigned int nErrorCode);
void print_TimeDateValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue);
void print_IntegralDerivativeValue(const unsigned char nPos, const unsigned char nLine, unsigned int nValue);
void print_IntegerADCDigit(const unsigned char nPos, const unsigned char nLine, int nValue);
void print_IntegerMotorAccTime(const unsigned char nPos, const unsigned char nLine, unsigned int nValue);
void print_FloatValue(const unsigned char nPos, const unsigned char nLine, float fValue);
void print_FloatFlowQuantityValue(const unsigned char nPos, const unsigned char nLine, float fValue);
void print_FloatProgramShotTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue);
void print_FloatProgramFlowQuantityValue(const unsigned char nPos, const unsigned char nLine, float fValue);
#if AUTOMATION_SENSOR_ATTACHED
void print_FloatAutoFlushingModeTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue);
#else
void print_FloatFlushingDetectionTimeValue(const unsigned char nPos, const unsigned char nLine, float fValue);
#endif
void print_FloatPressureValue(const unsigned char nPos, const unsigned char nLine, float fValue);
void print_FloatInputBiasValue(const unsigned char nPos, const unsigned char nLine, float fValue);
void print_VersionCode(const unsigned char nPos, const unsigned char nLine, const unsigned char nVersion, const unsigned char nPatch, const unsigned char nTypedef);

// RGB LED
void initialize_PixelLED(void);
void prepare_LEDPalettOnBrewStandby(const CRGB rgbFgColor, const bool bBrewSetupMode);
void prepare_LEDPalettOnTimelapseHolding(const CRGB rgbFgColor, const bool bBrewSetupMode);
void prepare_LEDPalettOnBrewing(const CRGB rgbFgColor);
void prepare_LEDPalettOnSetupEntry(const CRGB rgbFgColor);
void prepare_LEDPalettOnSetupExit(const CRGB rgbFgColor);
void prepare_LEDPalettOnSleep(const CRGB rgbFgColor);
void prepare_LEDPalettOnCleaning(const CRGB rgbFgColor);
void prepare_LEDPalettOnException(const CRGB rgbFgColor);
void prepare_LEDPalettOnAirVentilation(const CRGB rgbFgColor);
void prepare_LEDPalettOnActivation(const CRGB rgbFgColor);
void prepare_LEDPalettOnLongPushing(const CRGB rgbFgColor);
#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
void prepare_LEDPalettOnVLongPushing(const CRGB rgbFgColor);
#endif


#endif //_MES_PERIPHERAL_INTERFACE_H_

