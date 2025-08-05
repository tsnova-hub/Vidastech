#ifndef _MES_DATASET_DEFINITION_H_
#define _MES_DATASET_DEFINITION_H_


#include "MES_Config.h"


//////////////////////////////////////////////////////////////////////////////////////////
//			GLOBAL TYPE DEFINITIONS OF PRESET PARAMETERS
//////////////////////////////////////////////////////////////////////////////////////////

#define PROGRAM_SET_ONE								(0)
#define PROGRAM_SET_TWO								(1)
#define MANUAL_SET_ONE 								(2)

#define BREW_SETUP_MODE_OFF							(0)
#define BREW_SETUP_MODE_ON							(1)

#define PROGRAM_MODE_FLOW 							(0)
#define PROGRAM_MODE_TIME							(1)

#define INJECTION_MODE_OFF							(0)
#define INJECTION_MODE_ON							(1)

#define MOTOR_RUN_OFF								(0)
#define MOTOR_RUN_ON								(1)

#define MOTOR_STANDBY_OFF							(0)
#define MOTOR_STANDBY_PREP							(1)
#define MOTOR_STANDBY_SHOT							(2)

#define LAMP_TURNOFF_ALWAYS							(0)
#define LAMP_TURNON_SHOT							(1)
#define LAMP_TURNON_ALWAYS							(2)

#define LAMP_LOW_INTENSITY							(0)
#define LAMP_HIGH_INTENSITY							(1)

#define LEDPIXEL_PROGRESS_MODE						(0)
#define LEDPIXEL_FLOWCNT_MODE						(LEDPIXEL_PROGRESS_MODE)
#define LEDPIXEL_TIMECNT_MODE						(LEDPIXEL_PROGRESS_MODE)
#define LEDPIXEL_TIMELAPSE_MODE						(1)
#define LEDPIXEL_FLOWRATE_MODE						(2)
#define LEDPIXEL_BREWPRESS_MODE						(3)
#define LEDPIXEL_PUMPPRESS_MODE 					(4)

#define POWER_SAVE_MODE_OFF							(0)
#define POWER_SAVE_MODE_ON							(1)
// PWS : POWER SAVING
#define PWS_NOT_WORKING								(0)
#define PWS_NOW_WORKING								(1)
#define PWS_PAUSED 									(2)
#define PWS_RESUMED									(3)

#define MAX_YEAR									(99UL)
#define MAX_MONTH									(12UL)
#define MAX_DAY										(31UL)
#define MAX_HOUR									(24UL)
#define MAX_MINUTE									(60UL)
#define MAX_SECOND									(60UL)

#define LOW_INLET_ALARM_OFF							(0)
#define LOW_INLET_ALARM_ON							(1)

#define CLEANING_MODE_OFF							(0)
#define CLEANING_MODE_ON							(1)

#define FLUSHING_DETECTION_MODE_OFF					(0)
#define FLUSHING_DETECTION_MODE_ON					(1)

#define AUTOMATION_MODE_OFF							(0)
#define AUTOMATION_MODE_ON							(1)
#define AUTOMATION_MODE_RESUME						(false)
#define AUTOMATION_MODE_SUSPEND						(true)

#define MAINTENANCE_MODE_OFF						(0)
#define MAINTENANCE_MODE_ON							(1)

#define DEFAULT_RESET_OFF							(0)
#define DEFAULT_RESET_ON							(1)

#define BREW_PROFILE_NORMAL_LEVEL					(0)				// difference < 15 % 
#define BREW_PROFILE_NOTICE_LEVEL					(1)				// difference >= 15 % 
#define BREW_PROFILE_ALARM_LEVEL					(2)				// difference >= 30 % 


//////////////////////////////////////////////////////////////
// Type-Definition of Preset Paramters

typedef struct _ST_ACQ_DATA_SET_T
{
	// Shot Time
	unsigned long ShotTime;				// Unit : Milli-Second
	
	// Flowmeter (count/rpm)
	unsigned int FlowCount;
	float FlowQuantity;
	float FlowRate;

	// Temperature of Heater (Degree C)
	float PreheatTemp;
	float BrewTemp;
	float GroupTemp;
	#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
	float BrewWaterTemp;
	#endif

	// Heating MV (%)
	float PreheatMV;
	float BrewMV;
	float GroupMV;

	// Present Motor Controller Status (OZBV-10A-D2M2)
	int PresentCurrent;
	int PresentSpeed;

	// Pressure (bar)
	float BrewPress;
	float PumpPress;
	#if AUX_CURRENT_SENSOR_ATTACHED
	float AuxPress;
	#endif

	// External Analog Input.
	#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	float AnalogInputVoltage;
	#endif
	
	// Averaged.
	float AvgFlowRate;
	float AvgBrewPress;
	#if PUMP_PRESSURE_SENSOR_ATTACHED
	float AvgPumpPress;
	#endif
	#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
	float AvgAnalogInputVoltage;
	#endif
} stAcqDataSet_t;

typedef struct _ST_SETUP_DATA_SET_T
{
	// Brew Setup Mode
	unsigned char BrewSetupMode;
	
	// Program Brew Mode
	unsigned char ProgramMode[3];
	unsigned int InjectionFlow[3];
	unsigned int ExtractionFlow[3];
	float InjectionTime[3];
	float ExtractionTime[3];
	
	// Manual Brew Mode
	unsigned char ProgramSetNum;

	// Injection Setup
	unsigned char InjectionMode;	
	float InjectionPower;

	// Heater Setup
	unsigned char SetHeaterChannel;
	float SetValueTemp[4];
	float ProportionalBandTemp[4];
	unsigned int IntegralTime[4];
	unsigned int DerivativeTime[4];
	float MVHighLimit[4];
	unsigned char TempSensorChannel;
	float SensorInputBias[4];

	// Pressure Sensor Setup
	unsigned char SetSensorChannel;
	int SensorADCDigit;
	float ZeroShift[3];
	float SpanFactor[3];

	// Motor Pump Setup
	unsigned char MotorRun;
	unsigned char MotorStandbyMode;
	float MotorPower;

	// Motor Acceleration Setup (OZBV-10A-D2M2)
	unsigned int MotorAccTime;
	unsigned int MotorAccTimeEST;

	// Lamp Setup
	unsigned char LampMode;
	unsigned char LampIntensity;
	
	// Visualization.
	unsigned char LedPixelMode[3];
	
	// Time/Date Setup
	unsigned char Year;
	unsigned char Month;
	unsigned char Day;
	unsigned char Weekday;
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
	
	// Power Saving Time Setup
	unsigned char PowerSaveMode;
	unsigned char StartHour;
	unsigned char StartMinute;
	// unsigned char StartSecond;	(>v2.0.0)
	unsigned char EndHour;
	unsigned char EndMinute;
	// unsigned char EndSecond;		(>v2.0.0)

	// Inlet Alarm
	#if PUMP_PRESSURE_SENSOR_ATTACHED
	unsigned char LowInletAlarm;
	float LowInletPress;
	#endif

	// Cleaning Mode
	unsigned char CleaningMode;
	
	#if AUTOMATION_SENSOR_ATTACHED
	// Auto-Flushing Mode (BF4RP) (>v2.0.0)
	unsigned char AutomationMode;
	float AutoFlushingTime;
	float AutoBrewWaitTime;
	#else
	// Flushing Detection Mode (<v1.4.0)
	unsigned char FlushingDetectionMode;
	float FlushingDetectionTime;
	float FlushingDetectionPress;
	#endif
	
	// Maintenance Mode
	unsigned char MaintenanceMode;
	
	// Reset
	unsigned char DefaultSet;

	// Password
	unsigned char PasswordInput[4];
} stSetupDataSet_t;

typedef struct _ST_MAINTENANCE_DATA_SET_T
{
	// Maintenance
	unsigned char ErrorHistoryIndex;
	unsigned char ErrorAddressIndex;
	unsigned int ErrorCode;
	unsigned int RecentErrorCode;

	unsigned int NoticeCode;
	unsigned int RecentNoticeCode;
	
	unsigned long TotBrewCount;
	unsigned long TotFlowQuantity;
	unsigned char Overflowed;
} stMaintenanceDataSet_t;

typedef struct _ST_INDICATOR_T
{
	unsigned char nShotTimeLevel;
	unsigned char nCurFlowrateLevel;
	unsigned char nAvgFlowrateLevel;
	unsigned char nAvgBrewPressLevel;
} stIndicator_t;

// need to implement "union structure" : (unsigend char type / specific data type)
typedef struct _ST_DATA_SET_T
{
	// Brew Number
	unsigned char ProgramNum;

	stAcqDataSet_t stAcquisition;
	stSetupDataSet_t stSetup;
	stMaintenanceDataSet_t stMaintenance;

	stIndicator_t stIndicator;
} stDataSet_t;

typedef struct _ST_ERROR_T
{
	bool bIsSystemOverloaded;			// Error code: 0x0001
	bool bFailToStandbyBrewPressure;	// Error code: 0x0002
	bool bIsBrewPressureOver;			// Error code: 0x0003
	bool bIsBrewPressureUnder;			// Error code: 0x0004
	#if PUMP_PRESSURE_SENSOR_ATTACHED
	bool bIsLowInletPressure;			// Error code: 0x0005
	#endif
	bool bIsGroupheadOverheated;		// Error code: 0x0006
	bool bIsBrewBoilerOverheated;		// Error code: 0x0007
	bool bIsPreheatBoilerOverheated;	// Error code: 0x0008
	bool bIsGroupheadUnderheated;		// Error code: 0x0009
	bool bIsBrewBoilerUnderheated;		// Error code: 0x000A
	bool bIsPreheatBoilerUnderheated;	// Error code: 0x000B
	bool bIsTemperatureSensorError; 	// Error code: 0x000C

	bool bIsFlowmeterFailed;			// Notice code: 0x0001
	bool bIsMotorAlarmReceived;			// Notice code: 0x0002
	bool bIsMaintenanceRequired;		// Notice code: 0x0003
	bool bIsFlowrateExceeded;			// Notice code: 0x0004 (v1.2r-171220)
	#if AUTOMATION_SENSOR_ATTACHED
	bool bIsOpticalFiberUnstable;		// Notice code: 0x0005 (v2.0.0b-180829)
	#endif
	#ifdef _USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_
	bool bIsEEPROMWorn;					// Notice code: 0x0006 (v1.2.1r-180209)
	#endif
} stError_t;


#endif//_MES_DATASET_DEFINITION_H_

