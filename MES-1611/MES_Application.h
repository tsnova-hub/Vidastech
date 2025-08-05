#ifndef _MES_APPLICATION_H_
#define _MES_APPLICATION_H_


#include "MES_Arduino.h"
#include "MES_Peripheral_Interface.h"
#include "MES_DataSet_Definition.h"
#include "MES_SetupMenu.h"

#include "PressureSensor/CPressureSensorThread.h"
#include "AnalogSensor/CAnalogSensorThread.h"

#include "Espresso/CEspresso.h"


////////////////////////////////////////////////////////////
// Definition for Framework of MOAI Espresso Station
////////////////////////////////////////////////////////////

#define DEF_PROGRAM_NUM 							(PROGRAM_SET_ONE)

#define DEF_BREW_SETUP_MODE							(BREW_SETUP_MODE_OFF)

#define DEF_PROGRAM_MODE							(PROGRAM_MODE_FLOW)
#define DEF_INJECTION_FLOW_COUNT					(60)
#define DEF_EXTRACTION_FLOW_COUNT					(80)
#define DEF_INJECTION_TIME							(4.0F)
#define DEF_EXTRACTION_TIME							(26.0F)

#define DEF_INJECTION_MODE							(INJECTION_MODE_ON)
#define DEF_PROGRAM_SET_NUM							(PROGRAM_SET_TWO)

#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
#define DEF_INJECTION_POWER							(MOTOR_UPPER_POWER * 0.50F)
#else
#define DEF_INJECTION_POWER							(MOTOR_UPPER_POWER * 0.30F)		// for OZBV-05A-D2M2
#endif

#define DEF_SETPOINT_TEMPERATURE					(DEFAULT_BOILER_TEMPERATURE)
#define DEF_CH1_SETPOINT_OFFSET						(-7.0F)
#define DEF_CH2_SETPOINT_OFFSET						(0.0F)
#define DEF_CH3_SETPOINT_OFFSET						(0.0F)
#define DEF_CH4_SETPOINT_OFFSET						(0.0F)
#define DEF_CH1_PBAND_TEMPERATURE					(PREHEAT_HEATER_P_BAND)
#define DEF_CH2_PBAND_TEMPERATURE					(BREW_HEATER_P_BAND)
#define DEF_CH3_PBAND_TEMPERATURE					(GROUPHEAD_HEATER_P_BAND)
#define DEF_CH4_PBAND_TEMPERATURE					(10.0F)
#define DEF_CH1_INTEGRAL_TIME 						(PREHEAT_HEATER_I_TIME)
#define DEF_CH2_INTEGRAL_TIME 						(BREW_HEATER_I_TIME)
#define DEF_CH3_INTEGRAL_TIME 						(GROUPHEAD_HEATER_I_TIME)
#define DEF_CH4_INTEGRAL_TIME 						(0)
#define DEF_CH1_DERIVATIVE_TIME						(PREHEAT_HEATER_D_TIME)
#define DEF_CH2_DERIVATIVE_TIME						(BREW_HEATER_D_TIME)
#define DEF_CH3_DERIVATIVE_TIME						(GROUPHEAD_HEATER_D_TIME)
#define DEF_CH4_DERIVATIVE_TIME						(0)
#define DEF_CH1_MV_HIGH_LIMIT						(PREHEAT_BOILER_HEATING_POWER)
#define DEF_CH2_MV_HIGH_LIMIT						(BREW_BOILER_HEATING_POWER)
#define DEF_CH3_MV_HIGH_LIMIT						(GROUPHEAD_HEATING_POWER)
#define DEF_CH4_MV_HIGH_LIMIT						(100.0F)
#define DEF_TM4_SENSOR_INPUT_BIAS					(DEFAULT_TEMPERATURE_SENSOR_INPUT_BIAS)

#define DEF_ZERO_SHIFT								(PRESSURE_SENSOR_ZERO_SHIFT)
#define DEF_SPAN_FACTOR								(PRESSURE_SENSOR_SPAN_FACTOR)

#define DEF_MOTOR_RUN								(MOTOR_RUN_OFF)
#define DEF_MOTOR_STANDBY_MODE						(MOTOR_STANDBY_OFF)
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
#define DEF_MOTOR_POWER								(MOTOR_UPPER_POWER * 0.70F)
#else
#define DEF_MOTOR_POWER								(MOTOR_UPPER_POWER * 0.80F)		// for OZBV-05A-D2M2
#define DEF_MOTOR_ACC_TIME							(MOTOR_ACC_TIME)				// for OZBV-05A-D2M2
#endif

#define DEF_LAMP_MODE								(LAMP_TURNON_SHOT)
#define DEF_LAMP_INTENSITY							(LAMP_LOW_INTENSITY)

#define DEF_POWER_SAVE_MODE							(POWER_SAVE_MODE_OFF)
#define DEF_POWER_SAVE_START_HOUR					(0)
#define DEF_POWER_SAVE_START_MINUTE					(0)
#define DEF_POWER_SAVE_START_SECOND					(0)
#define DEF_POWER_SAVE_END_HOUR						(0)
#define DEF_POWER_SAVE_END_MINUTE					(0)
#define DEF_POWER_SAVE_END_SECOND					(0)
#define DEF_POWER_SAVE_DURATION						(7)

#define DEF_LOW_INLET_ALARM_PRESSURE				(0.5F)

#define DEF_LEDPIXEL_MODE_PROGRAM					(LEDPIXEL_PROGRESS_MODE)
#define DEF_LEDPIXEL_MODE_MANUAL					(LEDPIXEL_BREWPRESS_MODE)

#define DEF_CLEANING_MODE							(CLEANING_MODE_OFF)

#if AUTOMATION_SENSOR_ATTACHED
#define DEF_AUTOMATION_MODE							(AUTOMATION_MODE_ON)
#define DEF_AUTO_FLUSHING_TIME						(2.5F)
#define DEF_AUTO_BREW_WAIT_TIME						(2.0F)
#else
#define DEF_FLUSHING_DETECTION_MODE					(FLUSHING_DETECTION_MODE_OFF)
#define DEF_FLUSHING_DETECTION_TIME					(5.0F)
#define DEF_FLUSHING_DETECTION_PRESS				(2.0F)							// @ flowrate : 8.0 [ml/sec]
#endif

#define DEF_MAINTENANCE_MODE						(MAINTENANCE_MODE_OFF)

#define DEF_DEFAULT_SET								(DEFAULT_RESET_OFF)

#define DEF_ERROR_NONE								(0x0000)
#define DEF_NOTICE_NONE								(0x0000)

#define DEF_LCDVIEW_ENTRY 							(DEFAULT_LCDVIEW)
#define DEF_DASHBOARD_ENTRY							(DEFAULT_DASHBOARD)
#define DEF_MENUBOARD_ENTRY							(DEFAULT_MENUBOARD)
#define DEF_MENUPAGE_ENTRY							(DEFAULT_MENUPAGE)

#define BREW_PROFILE_NOTICE_CONDITION				(0.85F)				// difference >= 15 % 
#define BREW_PROFILE_ALARM_CONDITION				(0.70F)				// difference >= 30 %


////////////////////////////////////////////////////////////
// Definition of System States

#define NORMAL_STATE_MASK							(0x0F)
#define ABNORMAL_STATE_MASK							(0xF0)

#define SYSTEM_NOT_READY							(0x0F)
#define SYSTEM_SUSPENDED							(SYSTEM_NOT_READY)
#define SYSTEM_ERROR_OCCURRED						(0xF0)


#define BREW_STANDBY_STATE							(0)
#define BREW_STOP_STATE								(1)
#define BREW_START_STATE 							(2)
#define BREW_STAGE_CHANGE_STATE 					(3)
#define BREW_MANUAL_STATE							(4)
#define BREW_SETUP_STATE 							(5)
#define BREW_PROGRAM_STATE							(6)
#define BREW_MODE_CHANGE_STATE						(7)
#define BREW_SETUP_ENTRY_STATE						(8)
#define BREW_SETUP_CANCEL_STATE						(9)
#define BREW_SETUP_EXIT_STATE						(10)
#define POWER_SAVING_STATE							(11)
#define POWER_SAVING_RESUME_STATE					(12)
#define POWER_SAVING_PAUSE_STATE					(13)
// #define POWER_SAVING_STOP_STATE 					(13)
#define POWER_SAVING_CONTINUE_STATE					(14)
#define CLEANING_MACHINE_START_STATE				(15)
#define CLEANING_MACHINE_STOP_STATE					(16)
#if AUTOMATION_SENSOR_ATTACHED
#define AUTO_FLUSHING_START_STATE					(17)
#define AUTO_FLUSHING_STOP_STATE					(18)
#define SYSTEM_ERROR_STATE							(19)
#else
#define SYSTEM_ERROR_STATE							(17)
#endif


#define LCD_DASHBOARD_ENTRY							(0)
#define LCD_DASHBOARD_CHANGE 						(1)
#define LCD_MENUBOARD_ENTRY							(2)
#define LCD_MENUBOARD_LIST_CHANGE					(3)
#define LCD_MENUPAGE_ENTRY							(4)
#define LCD_MENUPAGE_CANCEL_EXIT					(5)
#define LCD_MENUPAGE_SAVE_EXIT						(6)
#define LCD_MENUPAGE_LIST_VIEW						(7)
#define LCD_MENUPAGE_LIST_CHANGE					(8)
#define LCD_MENUPAGE_LIST_SELECT					(9)
#define LCD_MENUPAGE_VALUE_CHANGE					(10)
#define LCD_CLEANING_MODE_ENTRY						(11)
#define LCD_CLEANING_MODE_EXIT						(12)
#define LCD_LONGPUSH_STATE							(13)


#define CLEANING_STAGE_BEGIN						(0)
#define CLEANING_STAGE_READY						(CLEANING_STAGE_BEGIN)
#define CLEANING_STAGE_FIRST						(1)
#define CLEANING_STAGE_SECOND						(2)
#define CLEANING_STAGE_THIRD						(3)
#define CLEANING_STAGE_FORTH						(4)
#define CLEANING_STAGE_END							(5)
#define CLEANING_SUBSTAGE_READY						(CLEANING_STAGE_READY)
#define CLEANING_SUBSTAGE_FIRST						(CLEANING_STAGE_FIRST)
#define CLEANING_SUBSTAGE_SECOND					(CLEANING_STAGE_SECOND)
#define CLEANING_SUBSTAGE_THIRD						(CLEANING_STAGE_THIRD)
#define CLEANING_SUBSTAGE_FORTH 					(CLEANING_STAGE_FORTH)


#define ERROR_MSG_BASE								(DEF_ERROR_NONE)
#define ERROR_MSG_NONE								(ERROR_MSG_BASE)
#define ERROR_MSG_1									(ERROR_MSG_NONE+1)
#define ERROR_MSG_2									(ERROR_MSG_1+1)
#define ERROR_MSG_3									(ERROR_MSG_2+1)
#define ERROR_MSG_4									(ERROR_MSG_3+1)
#define ERROR_MSG_5									(ERROR_MSG_4+1)
#define ERROR_MSG_6									(ERROR_MSG_5+1)
#define ERROR_MSG_7									(ERROR_MSG_6+1)
#define ERROR_MSG_8									(ERROR_MSG_7+1)
#define ERROR_MSG_9									(ERROR_MSG_8+1)
#define ERROR_MSG_10								(ERROR_MSG_9+1)
#define ERROR_MSG_11								(ERROR_MSG_10+1)
#define ERROR_MSG_12								(ERROR_MSG_11+1)
#define TOTAL_NUM_OF_ERROR_MESSAGES					(ERROR_MSG_12)

#define NOITCE_MSG_BASE								(DEF_NOTICE_NONE)
#define NOTICE_MSG_NONE								(NOITCE_MSG_BASE)
#define NOTICE_MSG_1								(NOTICE_MSG_NONE+1)
#define NOTICE_MSG_2								(NOTICE_MSG_1+1)
#define NOTICE_MSG_3								(NOTICE_MSG_2+1)
#define NOTICE_MSG_4								(NOTICE_MSG_3+1)
#define NOTICE_MSG_5								(NOTICE_MSG_4+1)
#define NOTICE_MSG_6								(NOTICE_MSG_5+1)
#define NOTICE_MSG_7								(NOTICE_MSG_6+1)
#define NOTICE_MSG_8								(NOTICE_MSG_7+1)
#define TOTAL_NUM_OF_NOTICE_MESSAGES				(NOTICE_MSG_8)


////////////////////////////////////////////////////////////
// Definition on Application

#define SYSTEM_WAIT_FOR_MEASURING_SENSOR_INITIALLY	(2000UL)								// > 2 sec (250 [msec] * 8 samples)
#define SYSTEM_WAIT_FOR_AIR_VENTILATION_INITIALLY	(10000UL)								// < 10 sec
#define SYSTEM_FACTORY_DEFAULT_KEY_HOLD_TIME		(2000UL)								// < 2 sec
#define SYSTEM_AIR_VENTILATION_KEY_HOLD_TIME		(500UL)									// < 1 sec
#define SYSTEM_ACTIVATION_KEY_HOLD_TIME				(1000UL)								// < 2 sec

#define BOILER_TEMPERATURE_UNDER_POWER_SAVING		(MIN_BOILER_TEMPERATURE)

#define LOWER_BREW_PRESSURE_UNDER_STANDBY1			(6.0F)
#define LOWER_BREW_PRESSURE_UNDER_STANDBY2			(8.0F)									// 8.0 ~ 9.0 bar
#define LOWER_BREW_PRESSURE_UNDER_STANDBY3			(9.0F)									// 8.0 ~ 9.0 bar
#define UPPER_BREW_PRESSURE_UNDER_STANDBY1			(8.0F)									// < 10.0F bar (expansion valve : > 10.0 bar)
#define UPPER_BREW_PRESSURE_UNDER_STANDBY2			(9.0F)									// < 10.0F bar (expansion valve : > 10.0 bar)
#define MIN_BREW_PRESSURE_UNDER_STANDBY				(DEF_LOW_INLET_ALARM_PRESSURE)
#define MAX_BREW_PRESSURE_UNDER_STANDBY				(PRESSURE_VALUE_LIMIT+1.0F)				// 13.0 bar (Brew Boiler Pressure Limit : 12 bar)

#define MIN_MOTOR_POWER_TO_STANDBY_BREW_PRESSURE	(0.5F)									// >= 50 %
#define MIN_TIME_TO_STANDBY_BREW_PRESSURE			(1000UL)								// 1 sec
#define MAX_TIME_TO_STANDBY_BREW_PRESSURE			(5000UL)								// 5 sec
#define MAX_COUNT_FAILED_TO_STANDBY_BREW_PRESSURE	(10)									// 10
#define MIN_TEMP_TO_ENABLE_STANDBY_BREW_PRESSURE	(MIN_BOILER_TEMPERATURE * 0.90F)		// >= 63.0 Deg. C
#define DELAY_TIME_TO_STANDBY_BREW_PRESSURE			(2)										// 2 sec

#define TIME_TO_STABILIZE_BREWPRES_ON_STOP_CLEANING	(5000UL)								// < 5 sec
#if AUTOMATION_SENSOR_ATTACHED
#define TIME_TO_STABILIZE_BREWPRES_TO_AUTO_BREWING	(2000UL)								// < 2 sec
#endif

#define MIN_MOTOR_POWER_TO_ADJUST_PRESSURE_SENSOR	(0.5F)									// >= 50 %

#if PUMP_PRESSURE_SENSOR_ATTACHED
#define TIME_TO_DECIDE_LOW_INLET_PRESSURE			(5000UL)								// 5 sec
#endif
#define TIME_TO_DECIDE_LOW_BREW_PRESSURE			(30000UL)								// > 8 sec (related to flushing detection params)
#define TIME_TO_DECIDE_OVER_BREW_PRESSURE			(5000UL)								// 5 sec

#define TIME_TO_HOLD_PRESSURE_SENSOR_DEFAULT_RESET	(1000UL)
#define TIME_TO_PREPARE_PRESSURE_SENSOR_ZERO_SHIFT	(5000UL)
#define TIME_TO_PREPARE_PRESSURE_SENSOR_SPAN_TIME	(10000UL)

#define BOILER_OVERHEATING_TEMPERATURE				(99.9F)
#define BOILER_UNDERHEATING_TEMPERATURE				(35.0F)									// originally 50.0 deg C (modified at 1.2r-171220)
#define BOILER_UNDERHEATING_OBSERVATION_START_TIME	(30UL*60UL)								// after 30 min since system activation.

#define FLOWMETER_FAILURE_COUNT_CONDITION			(0UL)									// if Zero Flow Count, Notice.
#define FLOWMETER_FAILURE_TIME_CONDITION			(1000UL)								// > 1 sec
#define FLOWRATE_EXCEED_CONDITION					(FLOWRATE_VALUE_LIMIT+1.0F)				// 13.0 ml/sec (Flowrate Limit : 12 ml/sec)

#define SYSTEM_ERROR_MESSAGE_DISPLAY_PERIOD			(30UL)									// every 30 sec
#define SYSTEM_NOTICE_MESSAGE_DISPLAY_PERIOD		(5*60UL)								// every 5 min

#define BREW_WATER_QTY_REQUIRED_MAINTENANCE			(10000UL)
#define BREW_WATER_QTY_MAINTENANCE_NOTICE_RANGE		(2UL)
#define MAXIMUM_BREW_WATER_QUANTITY_DIGIT			(MAX_LONG_VALUE_NUM-1)
#define MAXIMUM_BREW_COUNT_DIGIT					(MAX_LONG_VALUE_NUM-1)

#define MOTOR_POWER_FOR_CLEANING_MODE				(0.7F)									// 70 %
#define CLEANING_SUBSTAGE_FiRST_ITERATION			(3)
#define CLEANING_SUBSTAGE_THIRD_ITERATION			(3)
#define CLEANING_SUBSTAGE_FORTH_ITERATION			(10)

#if AUTOMATION_SENSOR_ATTACHED

#define MIN_MOTOR_POWER_TO_AUTO_FLUSHING			(0.5F)									// >= 50 %
#define MAX_TIME_TO_DETECT_OPTICAL_SIGNAL_UNSTABLE	(3*60000UL)								// 180 sec (3 min)
#define MIN_TIME_TO_AUTO_FLUSHING					(2.0F)									// 2.0 sec
#define MAX_TIME_TO_AUTO_FLUSHING					(10.0F)									// 10.0 sec
#define MIN_WAIT_TIME_TO_AUTO_BREW					(1.0F)									// 1.0 sec
#define MAX_WAIT_TIME_TO_AUTO_BREW					(5.0F)									// 5.0 sec

#else

#define POLLING_DURATION_FOR_FLUSHING_DETECTION		(2000UL)								// 2000 msec, ex: 5 ~ 7 sec @ set = 5 sec
#define MIN_TIME_FOR_FLUSHING_DETECTION				(1.0F)									// >= 1 sec
#define MAX_TIME_FOR_FLUSHING_DETECTION				(30.0F)									// <= 30.0 sec
#define MIN_BREW_PRESSURE_FOR_FLUSHING_DETECTION	(0.0F)									// >= 0.0 bar
#define MAX_BREW_PRESSURE_FOR_FLUSHING_DETECTION	(10.0F)									// <= 10.0 bar
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
#define MIN_MOTOR_POWER_FOR_FLUSHING_DETECTION		(0.10F)									// >= 10 %
#else
#define MIN_MOTOR_POWER_FOR_FLUSHING_DETECTION		(0.01F)									// >= 1 %, for OZBV-05A-D2M2
#endif

#endif

#define TIME_TO_SAVE_AND_EXIT_MENU					(1UL*60UL)								// goto DASHBOARD in 1 min since last button event.
#define TIME_TO_DETECT_VLONG_PUSHING_EVENT			(TIME_VCLICKED_BUTTON)					// > 4000 msec

#define BREW_BUTTON_MUTEX_ACTIVATED_MASK 			(0b01)
#define ROTARY_ENCODER_MUTEX_ACTIVATED_MASK 		(0b10)
#define BREW_BUTTON_ACTIVATED						(BREW_BUTTON_MUTEX_ACTIVATED_MASK)
#define ROTARY_ENCODER_ACTIVATED					(ROTARY_ENCODER_MUTEX_ACTIVATED_MASK)
#define ALL_USER_INTERFACE_ACTIVATED				(0b11)
#define ALL_USER_INTERFACE_DEACTIVATED				(0b00)


////////////////////////////////////////////////////////////
// System Time Constant Definition (msec)

#define SYSTEM_ONE_SECOND							(1000UL)								// 1,000 msec
#define SYSTEM_ONE_MINUTE							(60000UL)								// 60,000 msec
#define SYSTEM_ONE_HOUR								(3600000UL)								// 3,600,000 msec
#define SYSTEM_ONE_DAY								(86400000UL)							// 86,400,000 msec


////////////////////////////////////////////////////////////
// Definition of Execution Period of Global Timer Handler

#define MAX_TIEMR_INTERRUPT_COUNT					(5000UL)								// 5000 msec

#define TIMER3_INTERRUPT_PERIOD						(1UL)									// 5 msec

#define ROTARY_ENCODER_SW_POLLING_PERIOD			(1UL) 									// 5 msec (< 50 msec)
#define BREW_BUTTON_POLLING_PERIOD					(5UL) 									// 10 msec (< 50 msec)
#define APP_THREAD_SHOULD_RUN_PERIOD				(25UL)									// < 50 msec
#define TIME_DATE_UPDATE_PERIOD						(100UL)									// < 100 msec
#define WATER_FLOW_STATUS_POLLING_PERIOD			(250UL)									// < 500 msec
#if AUTOMATION_SENSOR_ATTACHED
#define OPTICAL_FIBER_AMP_POLLING_PERIOD			(250UL)									// < 300 msec (for BF4RP)
#endif
#define LAMP_DELAYED_TURNOFF_POLLING_PERIOD			(500UL)									// < 500 msec
#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
#define MOTOR_ALARM_POLLING_PERIOD					(5000UL)								// < 50000 msec
#else
#define MOTOR_PRESENT_STATUS_POLLING_PERIOD			(250UL)									// < 1000 msec (for OZBV-05A-D2M2)
#define MOTOR_ALARM_POLLING_PERIOD					(1000UL)								// < 1000 msec (for OZBV-05A-D2M2)
#endif

#if defined(_USE_BREW_PROFILE_ACQUISITION_)
#define PERIOD_TO_SEND_SENSOR_OBSERVATION			(1000UL)
#endif


////////////////////////////////////////////////////////////
// Definition of Period of Thread Execution

#define PRESSURE_MEARSUREMENT_PERIOD				(APP_THREAD_SHOULD_RUN_PERIOD * 1)		// < 50 msec
#define ANALOG_INPUT_MEASUREMENT_PERIOD				(APP_THREAD_SHOULD_RUN_PERIOD * 4)		// < 100 msec
#define REALTIME_CLOCK_FETCH_PERIOD					(APP_THREAD_SHOULD_RUN_PERIOD * 10)		// < 250 msec


////////////////////////////////////////////////////////////
// Definition of Period of Polling Execution

#define TM4CONTROLLER_TEMP_MEASUREMENT_PERIOD		(250UL)									// < 250 msec
#define TM4CONTROLLER_MV_MEASUREMENT_PERIOD			(350UL)									// < 500 msec


////////////////////////////////////////////////////////////
// System EEPROM Address Map (Arduino MEGA)

#define MAX_EEPROM_MEMORY_SIZE						(4096)

#define ERROR_HISTORY_SIZE							(10)
#define ERROR_CODE_HISTORY_SIZE						(ERROR_HISTORY_SIZE * 3)
#define MAX_ERROR_HISTORY_INDEX						(200)

#define EEPROM_PAGE1_ADDRESS						(0x0000)								// size : 2048 bytes
#define EEPROM_PAGE2_ADDRESS						(0x0800)								// size : 2048 bytes

#define MAX_BACKUP_LOCATIONS						(30)									// 30 locations
#define MAX_BACKUP_PARAMETERS						(18)									// size : 18 bytes
#define TOT_BREW_COUNT_PTR_INDEX					(0)
#define TOT_FLOW_QUANTITY_PTR_INDEX					(TOT_BREW_COUNT_PTR_INDEX + 1)
#define CNT_OVERFLOW_PTR_INDEX						(TOT_FLOW_QUANTITY_PTR_INDEX + 1)
#define FIRST_PROGRAM_MODE_PTR_INDEX				(CNT_OVERFLOW_PTR_INDEX + 1)
#define FIRST_INJECTION_FLOWCNT_PTR_INDEX			(FIRST_PROGRAM_MODE_PTR_INDEX + 1)
#define FIRST_EXTRACTION_FLOWCNT_PTR_INDEX			(FIRST_INJECTION_FLOWCNT_PTR_INDEX + 1)
#define FIRST_INJECTION_TIME_PTR_INDEX				(FIRST_EXTRACTION_FLOWCNT_PTR_INDEX + 1)
#define FIRST_EXTRACTION_TIME_PTR_INDEX				(FIRST_INJECTION_TIME_PTR_INDEX + 1)
#define SECOND_PROGRAM_MODE_PTR_INDEX				(FIRST_EXTRACTION_TIME_PTR_INDEX + 1)
#define SECOND_INJECTION_FLOWCNT_PTR_INDEX			(SECOND_PROGRAM_MODE_PTR_INDEX + 1)
#define SECOND_EXTRACTION_FLOWCNT_PTR_INDEX			(SECOND_INJECTION_FLOWCNT_PTR_INDEX + 1)
#define SECOND_INJECTION_TIME_PTR_INDEX				(SECOND_EXTRACTION_FLOWCNT_PTR_INDEX + 1)
#define SECOND_EXTRACTION_TIME_PTR_INDEX			(SECOND_INJECTION_TIME_PTR_INDEX + 1)
#define MANUAL_RESERVED_DATA_PTR_INDEX				(SECOND_EXTRACTION_TIME_PTR_INDEX + 1)
#define MANUAL_INJECTION_FLOWCNT_PTR_INDEX			(MANUAL_PROGRAM_MODE_PTR_INDEX + 1)
#define MANUAL_EXTRACTION_FLOWCNT_PTR_INDEX			(MANUAL_INJECTION_FLOWCNT_PTR_INDEX + 1)
#define MANUAL_INJECTION_TIME_PTR_INDEX				(MANUAL_EXTRACTION_FLOWCNT_PTR_INDEX + 1)
#define MANUAL_EXTRACTION_TIME_PTR_INDEX			(MANUAL_INJECTION_TIME_PTR_INDEX + 1)

#define RESERVED_0_LOCATIONS						(1)
#define RESERVED_1_LOCATIONS						(1)
#define RESERVED_2_LOCATIONS						(60)									// size : 60 bytes
#define RESERVED_3_LOCATIONS						(1)
#define RESERVED_4_LOCATIONS						(12)									// size : 12 bytes
#define RESERVED_5_LOCATIONS						(2 * RESERVED_4_LOCATIONS * MAX_BACKUP_LOCATIONS) // size : 720 bytes
#define RESERVED_6_LOCATIONS						(1)

#define EEPROM_BASE_ADDRESS							(EEPROM_PAGE1_ADDRESS)

#define FACTORY_RESET_FLAG_ADDR						(EEPROM_BASE_ADDRESS)
#define HARDWARE_VERSION_ADDR						(FACTORY_RESET_FLAG_ADDR + 0x01)
#define FIRMWARE_VERSION_ADDR						(HARDWARE_VERSION_ADDR + 0x01)

#define RESERVED_0_ADDR								(FIRMWARE_VERSION_ADDR + 0x01)

#define DEFAULT_PROGRAM_NUM_ADDR					(RESERVED_0_ADDR + RESERVED_0_LOCATIONS)

#define RESERVED_1_ADDR								(DEFAULT_PROGRAM_NUM_ADDR + 0x01)

#define MAINTENANCE_SECTOR_BASE_ADDR				(RESERVED_1_ADDR + RESERVED_1_LOCATIONS)
#define ERROR_NEXT_INDEX_ADDR						(MAINTENANCE_SECTOR_BASE_ADDR)
#define ERROR_NEXT_POINTER_ADDR						(ERROR_NEXT_INDEX_ADDR + 0x01)
#define ERROR_CODE_HISTORY_ADDR						(ERROR_NEXT_POINTER_ADDR + 0x01)
#define TOTAL_BREW_COUNT_ADDR						(ERROR_CODE_HISTORY_ADDR + ERROR_CODE_HISTORY_SIZE)	// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define TOTAL_FLOW_QUANTITY_ADDR					(TOTAL_BREW_COUNT_ADDR + 0x04)						// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define COUNT_OVERFLOW_ADDR							(TOTAL_FLOW_QUANTITY_ADDR + 0x04)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)

#define PROGRAM_SETUP_BASE_ADDR						(COUNT_OVERFLOW_ADDR + 0x01)						// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define FIRST_PROGRAM_MODE_ADDR						(PROGRAM_SETUP_BASE_ADDR)							// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define FIRST_INJECTION_FLOW_COUNT_ADDR				(FIRST_PROGRAM_MODE_ADDR + 0x01)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define FIRST_EXTRACTION_FLOW_COUNT_ADDR			(FIRST_INJECTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define FIRST_INJECTION_TIME_ADDR					(FIRST_EXTRACTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define FIRST_EXTRACTION_TIME_ADDR					(FIRST_INJECTION_TIME_ADDR + 0x02)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define SECOND_PROGRAM_MODE_ADDR					(FIRST_EXTRACTION_TIME_ADDR + 0x02)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define SECOND_INJECTION_FLOW_COUNT_ADDR			(SECOND_PROGRAM_MODE_ADDR + 0x01)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define SECOND_EXTRACTION_FLOW_COUNT_ADDR			(SECOND_INJECTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define SECOND_INJECTION_TIME_ADDR					(SECOND_EXTRACTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define SECOND_EXTRACTION_TIME_ADDR					(SECOND_INJECTION_TIME_ADDR + 0x02)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define MANUAL_RESERVED_DATA_ADDR					(SECOND_EXTRACTION_TIME_ADDR + 0x02)				// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define MANUAL_INJECTION_FLOW_COUNT_ADDR			(MANUAL_RESERVED_DATA_ADDR + 0x01)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define MANUAL_EXTRACTION_FLOW_COUNT_ADDR			(MANUAL_INJECTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define MANUAL_INJECTION_TIME_ADDR					(MANUAL_EXTRACTION_FLOW_COUNT_ADDR + 0x02)			// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)
#define MANUAL_EXTRACTION_TIME_ADDR					(MANUAL_INJECTION_TIME_ADDR + 0x02)					// Not used (Reserved) : Deprecated (>= v1.2.1r-180209)

#define MANUAL_SETUP_BASE_ADDR						(MANUAL_EXTRACTION_TIME_ADDR + 0x02)
#define PROGRAM_SET_NUMBER_ADDR						(MANUAL_SETUP_BASE_ADDR)

#define INJECTION_SETUP_BASE_ADDR					(PROGRAM_SET_NUMBER_ADDR + 0x01)
#define INJECTION_MODE_ADDR							(INJECTION_SETUP_BASE_ADDR)
#define INJECTION_POWER_ADDR						(INJECTION_MODE_ADDR + 0x01)

#define HEATER_SETUP_BASE_ADDR						(INJECTION_POWER_ADDR + 0x02)
#define PREHEAT_SET_VALUE_ADDR						(HEATER_SETUP_BASE_ADDR)
#define BREW_SET_VALUE_ADDR							(PREHEAT_SET_VALUE_ADDR + 0x02)
#define GROUPHEAD_SET_VALUE_ADDR					(BREW_SET_VALUE_ADDR + 0x02)
#define AUX_CH_SET_VALUE_ADDR						(GROUPHEAD_SET_VALUE_ADDR + 0x02)

#define HEATER_PID_SETUP_BASE_ADDR					(AUX_CH_SET_VALUE_ADDR + 0x02)
#define HEATER_PBAND_VALUE_ADDR						(HEATER_PID_SETUP_BASE_ADDR)
#define HEATER_ITIME_VALUE_ADDR						(HEATER_PBAND_VALUE_ADDR + 0x06)
#define HEATER_DTIME_VALUE_ADDR						(HEATER_ITIME_VALUE_ADDR + 0x06)

#define HEATER_MV_SETUP_BASE_ADDR					(HEATER_DTIME_VALUE_ADDR + 0x06)
#define HEATER_PREHEAT_MV_VALUE_ADDR				(HEATER_MV_SETUP_BASE_ADDR)
#define HEATER_BREW_MV_VALUE_ADDR					(HEATER_PREHEAT_MV_VALUE_ADDR + 0x02)
#define HEATER_GROUPHEAD_MV_VALUE_ADDR				(HEATER_BREW_MV_VALUE_ADDR + 0x02)

#define HEATER_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR	(HEATER_GROUPHEAD_MV_VALUE_ADDR + 0x02)				// Not used (Reserved) : Deprecated (> v1.2r-171215)
#define HEATER_SENSOR_INPUT_BIAS_VALUE_ADDR			(HEATER_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR)			// Not used (Reserved) : Deprecated (> v1.2r-171215)

#define SENSOR_ADJ_SETUP_BASE_ADDR					(HEATER_SENSOR_INPUT_BIAS_VALUE_ADDR + 0x02)
#define SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR			(SENSOR_ADJ_SETUP_BASE_ADDR)
#define SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR			(SENSOR_ADJ_ZERO_SHIFT_VALUE_ADDR + 0x06)

#define MOTORPUMP_SETUP_BASE_ADDR					(SENSOR_ADJ_SPAN_FACTOR_VALUE_ADDR + 0x06)
#define MOTOR_RUN_ADDR								(MOTORPUMP_SETUP_BASE_ADDR)
#define MOTOR_STANDBY_MODE_ADDR						(MOTOR_RUN_ADDR + 0x01)
#define MOTOR_POWER_ADDR							(MOTOR_STANDBY_MODE_ADDR + 0x01)

#define LAMP_SETUP_BASE_ADDR						(MOTOR_POWER_ADDR + 0x02)
#define LAMP_MODE_ADDR								(LAMP_SETUP_BASE_ADDR)
#define LAMP_INTENSITY_ADDR							(LAMP_MODE_ADDR + 0x01)

#define POWER_SAVE_SETUP_BASE_ADDR					(LAMP_INTENSITY_ADDR + 0x01)
#define POWER_SAVE_MODE_ADDR						(POWER_SAVE_SETUP_BASE_ADDR)
#define START_HOUR_ADDR								(POWER_SAVE_MODE_ADDR + 0x01)
#define START_MINUTE_ADDR							(START_HOUR_ADDR + 0x01)
#define START_SECOND_ADDR							(START_MINUTE_ADDR + 0x01)
#define END_HOUR_ADDR								(START_SECOND_ADDR + 0x01)
#define END_MINUTE_ADDR								(END_HOUR_ADDR + 0x01)
#define END_SECOND_ADDR								(END_MINUTE_ADDR + 0x01)

#define LEDPIXEL_MODE_SETUP_BASE_ADDR				(END_SECOND_ADDR + 0x01)
#define LEDPIXEL_MODE_PROGRAM1_ADDR					(LEDPIXEL_MODE_SETUP_BASE_ADDR)
#define LEDPIXEL_MODE_PROGRAM2_ADDR					(LEDPIXEL_MODE_PROGRAM1_ADDR + 0x01)
#define LEDPIXEL_MODE_MANUAL1_ADDR					(LEDPIXEL_MODE_PROGRAM2_ADDR + 0x01)

#define MOTOR_ACC_TIME_SETUP_BASE_ADDR				(LEDPIXEL_MODE_MANUAL1_ADDR + 0x01)				// > v2.0r-180319
#define MOTOR_ACC_TIME_RESERVED_ADDR				(MOTOR_ACC_TIME_SETUP_BASE_ADDR)				// > v2.0r-180319
#define MOTOR_ACC_TIME_MSEC_ADDR					(MOTOR_ACC_TIME_RESERVED_ADDR + 0x01)			// > v2.0r-180319

#if AUTOMATION_SENSOR_ATTACHED
#define AUTOMATION_SETUP_BASE_ADDR					(MOTOR_ACC_TIME_MSEC_ADDR + 0x02)
#define AUTOMATION_MODE_ADDR 						(AUTOMATION_SETUP_BASE_ADDR)
#define AUTO_FLUSHING_TIME_ADDR 					(AUTOMATION_MODE_ADDR + 0x01)
#define AUTO_BREW_WAIT_TIME_ADDR					(AUTO_FLUSHING_TIME_ADDR + 0x02)
#else
#define FLUSHING_DETECTION_SETUP_BASE_ADDR			(MOTOR_ACC_TIME_MSEC_ADDR + 0x02)
#define FLUSHING_DETECTION_MODE_ADDR 				(FLUSHING_DETECTION_SETUP_BASE_ADDR)
#define FLUSHING_DETECTION_TIME_ADDR 				(FLUSHING_DETECTION_MODE_ADDR + 0x01)
#define FLUSHING_DETECTION_PRES_ADDR				(FLUSHING_DETECTION_TIME_ADDR + 0x02)
#endif

#if AUTOMATION_SENSOR_ATTACHED
#define TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR		(AUTO_BREW_WAIT_TIME_ADDR + 0x02)
#else
#define TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR		(FLUSHING_DETECTION_PRES_ADDR + 0x02)
#endif
#define TEMP_SENSOR_INPUT_BIAS_VALUE_CH1_ADDR		(TEMP_SENSOR_INPUT_BIAS_SETUP_BASE_ADDR)
#define TEMP_SENSOR_INPUT_BIAS_VALUE_CH2_ADDR		(TEMP_SENSOR_INPUT_BIAS_VALUE_CH1_ADDR + 0x02)
#define TEMP_SENSOR_INPUT_BIAS_VALUE_CH3_ADDR		(TEMP_SENSOR_INPUT_BIAS_VALUE_CH2_ADDR + 0x02)
#define TEMP_SENSOR_INPUT_BIAS_VALUE_CH4_ADDR		(TEMP_SENSOR_INPUT_BIAS_VALUE_CH3_ADDR + 0x02)	// 156 bytes / 4096 bytes

#define RESERVED_2_ADDR 							(TEMP_SENSOR_INPUT_BIAS_VALUE_CH4_ADDR + 0x02)	// (60 bytes), 216 bytes / 4096 bytes

#define RESERVED_3_ADDR 							(RESERVED_2_ADDR + RESERVED_2_LOCATIONS)		// 217 bytes / 4096 bytes

#define TABLE_FOR_BACKUP_POINTER_BASE_ADDR			(RESERVED_3_ADDR + RESERVED_3_LOCATIONS)
#define TOTAL_BREW_COUNT_BACKUP_POINTER				(TABLE_FOR_BACKUP_POINTER_BASE_ADDR)
#define TOTAL_FLOW_QTY_BACKUP_POINTER				(TOTAL_BREW_COUNT_BACKUP_POINTER + 0x01)
#define COUNT_OVERFLOW_BACKUP_POINTER				(TOTAL_FLOW_QTY_BACKUP_POINTER + 0x01)
#define FIRST_PROGRAM_MODE_BACKUP_POINTER			(COUNT_OVERFLOW_BACKUP_POINTER + 0x01)
#define FIRST_INJECTION_FLOW_CNT_BACKUP_POINTER		(FIRST_PROGRAM_MODE_BACKUP_POINTER + 0x01)
#define FIRST_EXTRACTION_FLOW_CNT_BACKUP_POINTER	(FIRST_INJECTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define FIRST_INJECTION_TIME_BACKUP_POINTER			(FIRST_EXTRACTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define FIRST_EXTRACTION_TIME_BACKUP_POINTER		(FIRST_INJECTION_TIME_BACKUP_POINTER + 0x01)
#define SECOND_PROGRAM_MODE_BACKUP_POINTER			(FIRST_EXTRACTION_TIME_BACKUP_POINTER + 0x01)
#define SECOND_INJECTION_FLOW_CNT_BACKUP_POINTER	(SECOND_PROGRAM_MODE_BACKUP_POINTER + 0x01)
#define SECOND_EXTRACTION_FLOW_CNT_BACKUP_POINTER	(SECOND_INJECTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define SECOND_INJECTION_TIME_BACKUP_POINTER		(SECOND_EXTRACTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define SECOND_EXTRACTION_TIME_BACKUP_POINTER		(SECOND_INJECTION_TIME_BACKUP_POINTER + 0x01)
#define MANUAL_RESERVED_DATA_BACKUP_POINTER			(SECOND_EXTRACTION_TIME_BACKUP_POINTER + 0x01)
#define MANUAL_INJECTION_FLOW_CNT_BACKUP_POINTER	(MANUAL_RESERVED_DATA_BACKUP_POINTER + 0x01)
#define MANUAL_EXTRACTION_FLOW_CNT_BACKUP_POINTER	(MANUAL_INJECTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define MANUAL_INJECTION_TIME_BACKUP_POINTER		(MANUAL_EXTRACTION_FLOW_CNT_BACKUP_POINTER + 0x01)
#define MANUAL_EXTRACTION_TIME_BACKUP_POINTER		(MANUAL_INJECTION_TIME_BACKUP_POINTER + 0x01)	// (18 bytes), 235 bytes / 4096 bytes

#define RESERVED_4_ADDR								(MANUAL_EXTRACTION_TIME_BACKUP_POINTER + 0x01)	// (12 bytes), 247 bytes / 4096 bytes

#define TABLE_FOR_BACKUP_BASE_ADDRESS				(RESERVED_4_ADDR + RESERVED_4_LOCATIONS)
#define TOTAL_BREW_COUNT_BACKUP_BASE_ADDR			(TABLE_FOR_BACKUP_BASE_ADDRESS)
#define TOTAL_FLOW_QTY_BACKUP_BASE_ADDR				(TOTAL_BREW_COUNT_BACKUP_BASE_ADDR + 0x04 * MAX_BACKUP_LOCATIONS)
#define COUNT_OVERFLOW_BACKUP_BASE_ADDR				(TOTAL_FLOW_QTY_BACKUP_BASE_ADDR + 0x04 * MAX_BACKUP_LOCATIONS)
#define FIRST_PROGRAM_MODE_BACKUP_BASE_ADDR			(COUNT_OVERFLOW_BACKUP_BASE_ADDR + 0x01 * MAX_BACKUP_LOCATIONS)
#define FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR	(FIRST_PROGRAM_MODE_BACKUP_BASE_ADDR + 0x01 * MAX_BACKUP_LOCATIONS)
#define FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR	(FIRST_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define FIRST_INJECTION_TIME_BACKUP_BASE_ADDR		(FIRST_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR		(FIRST_INJECTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define SECOND_PROGRAM_MODE_BACKUP_BASE_ADDR		(FIRST_EXTRACTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define SECOND_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR	(SECOND_PROGRAM_MODE_BACKUP_BASE_ADDR + 0x01 * MAX_BACKUP_LOCATIONS)
#define SECOND_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR	(SECOND_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define SECOND_INJECTION_TIME_BACKUP_BASE_ADDR		(SECOND_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define SECOND_EXTRACTION_TIME_BACKUP_BASE_ADDR		(SECOND_INJECTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define MANUAL_RESERVED_DATA_BACKUP_BASE_ADDR		(SECOND_EXTRACTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define MANUAL_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR	(MANUAL_RESERVED_DATA_BACKUP_BASE_ADDR + 0x01 * MAX_BACKUP_LOCATIONS)
#define MANUAL_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR	(MANUAL_INJECTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define MANUAL_INJECTION_TIME_BACKUP_BASE_ADDR		(MANUAL_EXTRACTION_FLOW_CNT_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)
#define MANUAL_EXTRACTION_TIME_BACKUP_BASE_ADDR		(MANUAL_INJECTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)	// 1327 bytes / 4096 bytes

#define RESERVED_5_ADDR								(MANUAL_EXTRACTION_TIME_BACKUP_BASE_ADDR + 0x02 * MAX_BACKUP_LOCATIONS)	// (720 bytes), 2047 bytes / 4096 bytes

#define RESERVED_6_ADDR								(RESERVED_5_ADDR + RESERVED_5_LOCATIONS)		// 2048 bytes / 4096 bytes

#define EEPROM_PAGE2_BASE_ADDR						(RESERVED_6_ADDR + RESERVED_6_LOCATIONS)		// Page 2 Base Address



////////////////////////////////////////////////////////////
// Variable Declarations

// extern stDataSet_t stDataSet;

// extern stTimeDate_t stCurTimeDate;

// extern stError_t stError;



////////////////////////////////////////////////////////////
// Interface to MOAI Espresso Station Framework

// System Init / Boot Sequences
void initialize_SystemParameters(void);
void initialize_StructureDataSet(void);
void restore_StructureDataSet(void);
void reset_StructureDataSet(void);
void WaitForFirstSensorMeasurement(void);
void initialize_MenupageFunctionPointerTable(void);
bool isFactoryDefault(void);

void activate_SystemStateMachine(void);
void reset_SystemParametersToFactoryDefault(void);
bool WaitforSystemActivationKey(void);
void display_SystemActivation(void);
void activate_AllUserInterface(void);
void deactivate_AllUserInterface(void);
void config_UserInterfaceActivation(void);
bool isMutuallyExclusiveActivated(const unsigned char nQueryMask);

// Application Threads Init / Callback
void initialize_AppThreads(void);
void AppThreadShouldRun_Callback(void);

// User Interface(Button, Rotary Encoder with Switch) Binding
int bind_SignalBrewUserInterface(void);
int bind_SignalSetupUserInterface(void);

// System State Machine and Setup-Menu State Machine
bool app_BrewStateMachine(void);
bool app_SetupStateMachine(void);

// System Exception Handler
int app_SystemExceptions(void);

// Brew Pressure Stabilization
void wait_stabilizeBrewPressure(void);
void standby_BrewBoilerPressure(void);
#if AUTOMATION_SENSOR_ATTACHED
void stabilize_BrewBoilerPressure(void);
#endif

// Brewing Water Qauntity and Brew Count
void increase_BrewingCount(void);
void accumulate_BrewWaterQuantity(void);
void reset_BrewingCount(void);
void reset_BrewWaterQuantity(void);

// Brew Observation(Profile) Confidence
void observe_BrewProfileConfidence(const stAcqDataSet_t& stAcqQueue);
void queue_BrewObservations(const bool bStartBrewing);

// Cleaning Mode
bool procedure_CleaningMachine(const int nSignal);

#if AUTOMATION_SENSOR_ATTACHED
// Optical Fiber Amplifier (BF4RP) (>v2.0.0)
void setup_AutomationMode(const bool bSuspendByForce = false);
void suspend_AutomationMode(void);
void resume_AutomationMode(void);
void init_AutomationMode(void);
void enable_AutomationMode(const bool bSkipAutoFlushingByForce = true);
void disable_AutomationMode(void);
void start_AutoFlushing(void);
void stop_AutoFlushing(void);
bool start_AutoFlushingMode(void);
void stop_AutoFlushingMode(void);
bool procedure_AutoFlushingMode(void);

bool start_AutoBrewingMode(void);
void stop_AutoBrewingMode(void);
#else
// Flushing Detection Mode (<v1.4.0)
void verify_FlushingDetectionMode(const float fMotorPower);
void verify_FlushingDetectionModeWithMotorPump(void);
#endif

// Maintenance Mode
bool enable_SystemMaintenanceMode(void);
bool disable_SystemMaintenanceMode(void);

// Pressure Sensor Adjustment
void setup_PressureSensorAdjustment(void);
void config_PressureSensorAdjustmentToFactoryDefault(void);
void config_PressureSensorZeroShift(void);
void config_PressureSensorSpanFactor(void);

// Authorization Utilities
void add_AuthorizationCode(const unsigned char nNumberPosition);
bool request_Authorization(void);
void release_Authorization(void);
bool IsAuthorized(void);
bool verify_AuthorizationToMenupageEntry(const eMenuPage nMenupage);

// Setup Menu Timer Utilities
void start_SetupMenuTimer(void);
void stop_SetupMenuTimer(void);
void restart_SetupMenuTimer(void);
bool isSetupMenuTimedOver(void);

void start_AlarmMessageDisplayTimer(void);
void stop_AlarmMessageDisplayTimer(void);
bool isAlarmMessageDisplay(const unsigned long ulDisplayTimePeriod);

// Setup Menu Procedures
int procedure_SetupMenuboard(const int nCurrentMenuboard);
int procedure_ProgramOneSetup(const int nSignal);
int procedure_ProgramTwoSetup(const int nSignal);
int procedure_ManualOneSetup(const int nSignal);
int procedure_InjectionSetup(const int nSignal);
int procedure_GroupheadTempSetup(const int nSignal);
int procedure_BrewBoilerTempSetup(const int nSignal);
int procedure_PreheatBoilerTempSetup(const int nSignal);
int procedure_MotorPumpSetup(const int nSignal);
int procedure_LampSetup(const int nSignal);
int procedure_LEDPixelModeSetup(const int nSignal);
int procedure_TimeDateSetup(const int nSignal);
int procedure_PowerSavingSetup(const int nSignal);
int procedure_HeaterParameterSetup(const int nSignal);
int procedure_PressureSensorAdjustmentSetup(const int nSignal);
int procedure_MotorAccelerationSetup(const int nSignal);
#if AUTOMATION_SENSOR_ATTACHED
int procedure_AutomationSetup(const int nSignal);
#else
int procedure_FlushingDetectionModeSetup(const int nSignal);
#endif
int procedure_MaintenanceSetup(const int nSignal);
int procedure_DefaultResetSetup(const int nSignal);
int procedure_SystemInformationSetup(const int nSignal);
int procedure_PasswordAuthorizationSetup(const int nSignal);

// Motor Pump Controller Utilities
#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
void read_MotorControllerObservation(void);
void clear_MotorControllerStatusObservation(void);
void verify_MotorControllerACCTime(void);
#endif

// Boiler/Heater Controller
void read_BoilerTemperatureControllerPID(void);
bool verify_BoilerTemperatureConstrollerPID(void);
bool setup_BoilerTemperatureControllerPID(void);
bool config_BoilerTemperatureControllerPID(void);
void read_BoilerTemperatureControllerMV(void);
bool verify_BoilerTemperatureControllerMV(void);
bool setup_BoilerTemperatureControllerMV(void);
bool config_BoilerTemperatureControllerMV(void);
bool verify_BoilerTemperatureControllerBIAS(void);
bool setup_BoilerTemperatureControllerBIAS(void);
bool config_BoilerTemperatureControllerBIAS(void);
bool read_BoilerTemperatureControllerSV(void);
bool verify_BoilerTemperatureControllerSV(void);
bool setup_BoilerTemperatureControllerSV(void);
bool setup_BoilerTemperatureControllerSetPointWithValue(const float fSetPoint);
bool config_BoilerTemperatureControllerSetPointWithChannel(const enum CTM4Controller::TM4Channel eChannel, const float fSetPoint);
bool config_PreheatBoilerSetPointTemperature(const float fSetPoint);
bool config_BrewBoilerSetPointTemperature(const float fSetPoint);
bool config_GroupheadHeaterSetPointTemperature(const float fSetPoint);
bool sleep_BoilerHeater(void);
bool wakeup_BoilerHeater(void);
bool isPowerSavingTime(const stTimeDate_t& stTimeDate);
void verify_PowerSavingDuration(const unsigned char nCurrentMinute, unsigned int& nHours, unsigned int& nMins);
bool suspend_BoilerHeater(void);
bool resume_BoilerHeater(void);

// Sensor Measurement (Observation)
void read_TemperatureSensorAll(void);
void read_TM4ControllerObservation(void);
void measure_PressureSensorAll(void);

void update_SensorObservationAll(void);
void clear_SensorObservationAll(void);

void clear_PressureSensorObservation(void);
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void init_AnalogInputAveragedObservation(void);
void clear_AnalogInputObservation(void);
#endif
void clear_TemperatureSensorObservation(void);

void update_PressureSensorObservation(void);
void update_PressureSensorAnalogDigit(const bool bImmediately);
#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
void update_AnalogInputObservation(void);
#endif
void update_TemperatureSensorObservation(void);
void update_HeatingMVObservation(void);

#if EXTERNAL_ANALOG_INPUT_CH4_CONNECTED
float calculate_AveragedAnalogInputVoltage(const bool bReset);
#endif

void observe_SystemStatusOnStandby(void);
void observe_SystemStatusOnBrewing(void);
#if AUTOMATION_SENSOR_ATTACHED
void observe_SystemStatusOnFlushing(void);
#endif
void observe_SystemControllerOVL(void);
#if PUMP_PRESSURE_SENSOR_ATTACHED
void observe_WaterPumpPressure(void);
#endif
void observe_BrewBoilerPressureStatus(void);
void observe_BoilerHeatingStatus(void);
void observe_TemperatureSensorStatus(void);
void observe_MotorPumpStatus(void);
void observe_FlowmeterFailure(/*const bool bOnBrewing = true*/ void);
void observe_FlowrateExceeded(void);
#if AUTOMATION_SENSOR_ATTACHED
void observe_OpticalFiberAmp(void);
#endif
void observe_TotalBrewWaterQuantity(void);

void start_UnderheatingObservationTimer(void);
void stop_UnderheatingObservationTimer(void);
bool isUnderheatingObservationEnabled(void);

// Real Time Clock Utilities
void clear_CurrentTimeDate(void);
void copy_TimeDate2SetupParams(void);

void UpdatingCurrentTimeDate_Callback(void);

// Text LCD Display
void display_TextLCDOnUserInterface(void);

// RGB LED Visualization
void draw_LEDPalettOnBrewStandby(void);
void draw_LEDPalettOnBrewStop(void);
void draw_LEDPalettOnBrewing(void);
void draw_LEDPalettOnSetupEntry(void);
void draw_LEDPalettOnSetupExit(void);
void draw_LEDPalettOnSleep(void);
void draw_LEDPalettOnWake(void);
void draw_LEDPalettOnCleaning(void);
void draw_LEDPalettOnException(void);
void draw_LEDPalettOnLongPushing(void);
#if defined(_PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_)
void draw_LEDPalettOnVLongPushing(void);
#endif
void draw_LEDPalettOnAirVentilation(void);
void draw_LEDPalettOnActivation(void);
void draw_LEDPixelOnUserInterface(void);
void update_LEDPalettOnBrewStandby(void);
void update_LEDPalettOnBrewing(void);
void update_LEDPalettOnBrewingProgress(void);
void update_LEDPalettOnCleaningProgress(void);

// System EEPROM Utilities
void save_FactoryResetFlag(const unsigned char nFactoryResetFlag);
unsigned char load_FactoryResetFlag(void);
void save_EEPROMBackupAddressPointerTable(void);
void load_EEPROMBackupAddressPointerTable(void);
void save_HardwareVersion(const unsigned char nVersion);
unsigned char load_HardwareVersion(void);
void save_FirmwareVersion(const unsigned char nVersion);
unsigned char load_FirmwareVersion(void);
void save_DefaultProgramNum(const unsigned char nProgramNum);
void load_DefaultProgramNum(stDataSet_t& stDataSet);
unsigned char load_DefaultProgramNum(void);
void clear_SystemErrorHistory(stMaintenanceDataSet_t& stMaintenance);
void restore_SystemErrorIndex(stMaintenanceDataSet_t& stMaintenance);
unsigned long get_SystemErrorCode(const bool bReload, const unsigned char nHistIndex);
int compare_ErrorHistoryIndex(const void *firstElem, const void *secondElem);
void save_SystemErrorCode(stMaintenanceDataSet_t& stMaintenance);
void load_SystemErrorHistory(unsigned long ulErrorHistory[]);
void save_MaintenanceBrewUsage(const stMaintenanceDataSet_t stMaintenance);
void load_MaintenanceBrewUsage(stMaintenanceDataSet_t& stMaintenance);
void save_TotalBrewCount(const unsigned long ulTotBrewCount);
unsigned long load_TotalBrewCount(void);
void save_TotalFlowQuantity(const unsigned long ulTotFlowQuantity);
unsigned long load_TotalFlowQuantity(void);
void save_OverflowedUsage(const unsigned char nOverflowed);
unsigned char load_OverflowedUsage(void);
void save_ProgramParam(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
void load_ProgramParam(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void copy_ProgramParam(const int nProgramSetNum, const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void save_ProgramMode(const int nProgramNum, const unsigned char nProgramMode);
unsigned char load_ProgramMode(const int nProgramNum);
// void save_ProgramFlow(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
// void load_ProgramFlow(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void save_ProgramInjectionFlow(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
void save_ProgramExtractionFlow(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
void load_ProgramInjectionFlow(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void load_ProgramExtractionFlow(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
unsigned int load_ProgramInjectionFlow(const int nProgramNum);
unsigned int load_ProgramExtractionFlow(const int nProgramNum);
// void save_ProgramTime(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
// void load_ProgramTime(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void save_ProgramInjectionTime(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
void save_ProgramExtractionTime(const int nProgramNum, const stSetupDataSet_t stSetupDataSet);
void load_ProgramInjectionTime(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
void load_ProgramExtractionTime(const int nProgramNum, stSetupDataSet_t& stSetupDataSet);
float load_ProgramInjectionTime(const int nProgramNum);
float load_ProgramExtractionTime(const int nProgramNum);
void save_ManualProgramSetNumber(const unsigned char nProgramSetNum);
void load_ManualProgramSetNumber(stSetupDataSet_t& stSetupDataSet);
unsigned char load_ManualProgramSetNumber(void);
void save_InjectionSetupParam(const stSetupDataSet_t stSetupDataSet);
void load_InjectionSetupParam(stSetupDataSet_t& stSetupDataSet);
void save_InjectionMode(const unsigned char nInjectionMode);
void load_InjectionMode(stSetupDataSet_t& stSetupDataSet);
unsigned char load_InjectionMode(void);
void save_InjectionPower(const float fInjectionPower);
void load_InjectionPower(stSetupDataSet_t& stSetupDataSet);
float load_InjectionPower(void);
void save_HeaterSetValueTemperature(const stSetupDataSet_t stSetupDataSet);
void load_HeaterSetValueTemperature(stSetupDataSet_t& stSetupDataSet);
void save_HeaterSetValueTemperatureWithChannel(const int nChannel, const float fSetValueTemp);
float load_HeaterSetValueTemperatureWithChannel(const int nChannel);
void save_PreheatBoilerSetValueTemperature(const float fSetValueTemp);
float load_PreheatBoilerSetValueTemperature(void);
void save_BrewBoilerSetValueTemperature(const float fSetValueTemp);
float load_BrewBoilerSetValueTemperature(void);
void save_GroupheadSetValueTemperature(const float fSetValueTemp);
float load_GroupheadSetValueTemperature(void);
void save_HeaterProportionalBandTemperature(const stSetupDataSet_t stSetupDataSet);
void load_HeaterProportionalBandTemperature(stSetupDataSet_t& stSetupDataSet);
void save_HeaterProportionalBandTemperatureWithChannel(const int nChannel, const float fPBandTemperature);
float load_HeaterProportionalBandTemperatureWithChannel(const int nChannel);
void save_HeaterIntegralTime(const stSetupDataSet_t stSetupDataSet);
void load_HeaterIntegralTime(stSetupDataSet_t& stSetupDataSet);
void save_HeaterIntegralTimeWithChannel(const int nChannel, const unsigned int nIntegralTime);
unsigned int load_HeaterIntegralTimeWithChannel(const int nChannel);
void save_HeaterDerivativeTime(const stSetupDataSet_t stSetupDataSet);
void load_HeaterDerivativeTime(stSetupDataSet_t& stSetupDataSet);
void save_HeaterDerivativeTimeWithChannel(const int nChannel, const unsigned int nDerivativeTime);
unsigned int load_HeaterDerivativeTimeWithChannel(const int nChannel);
void save_HeaterMVHighLimit(const stSetupDataSet_t stSetupDataSet);
void load_HeaterMVHighLimit(stSetupDataSet_t& stSetupDataSet);
void save_HeaterMVHighLimitWithChannel(const int nChannel, const float fMVHighLimit);
float load_HeaterMVHighLimitWithChannel(const int nChannel);
void save_HeaterInputBias(const stSetupDataSet_t stSetupDataSet);
void load_HeaterInputBias(stSetupDataSet_t& stSetupDataSet);
void save_HeaterInputBias(const int nChannel, const float fSensorInputBias);
float load_HeaterInputBias(const int nChannel);
void save_PressureSensorZeroShift(const stSetupDataSet_t stSetupDataSet);
void load_PressureSensorZeroShift(stSetupDataSet_t& stSetupDataSet);
float load_PressureSensorZeroShift(const int nChannel);
void save_PressureSensorSpanFactor(const stSetupDataSet_t stSetupDataSet);
void load_PressureSensorSpanFactor(stSetupDataSet_t& stSetupDataSet);
float load_PressureSensorSpanFactor(const int nChannel);
void save_MotorPumpParam(const stSetupDataSet_t stSetupDataSet);
void load_MotorPumpParam(stSetupDataSet_t& stSetupDataSet);
void save_MotorRunFlag(const unsigned char nMotorRunFlag);
unsigned char load_MotorRunFlag(void);
void save_MotorStandbyMode(const unsigned char nMotorStandbyMode);
unsigned char load_MotorStandbyMode(void);
void save_MotorPower(const float fMotorPower);
float load_MotorPower(void);
void save_GroupheadLampParam(const stSetupDataSet_t stSetupDataSet);
void load_GroupheadLampParam(stSetupDataSet_t& stSetupDataSet);
void save_GroupheadLampMode(const unsigned char nLampMode);
unsigned char load_GroupheadLampMode(void);
void save_GroupheadLampIntensity(const unsigned char nLampIntensity);
unsigned char load_GroupheadLampIntensity(void);
void save_LEDPixelMode(const stSetupDataSet_t& stSetupDataSet);
void load_LEDPixelMode(stSetupDataSet_t& stSetupDataSet);
unsigned char load_LEDPixelMode(const int nProgramNum);
void save_MotorACCTimeParam(stSetupDataSet_t& stSetupDataSet);
void load_MotorACCTimeParam(stSetupDataSet_t& stSetupDataSet);
void save_MotorACCTime(const unsigned int nMotorAccTime);
unsigned int load_MotorACCTime(void);
#if AUTOMATION_SENSOR_ATTACHED
void save_AutomationParam(const stSetupDataSet_t stSetupDataSet);
void load_AutomationParam(stSetupDataSet_t& stSetupDataSet);
void save_AutomationMode(const unsigned char nAutoFlushingMode);
unsigned char load_AutomationMode(void);
void save_AutoFlushingTime(const float fAutoFlushingTime);
float load_AutoFlushingTime(void);
void save_AutoBrewWaitTime(const float fAutoBrewWaitTime);
float load_AutoBrewWaitTime(void);
#else
void save_FlushingDetectionParam(const stSetupDataSet_t stSetupDataSet);
void load_FlushingDetectionParam(stSetupDataSet_t& stSetupDataSet);
void save_FlushingDetectionMode(const unsigned char nFlushingDetectionMode);
unsigned char load_FlushingDetectionMode(void);
void save_FlushingDetectionTime(const float fFlushingDetectionTime);
float load_FlushingDetectionTime(void);
void save_FlushingDetectionPressure(const float fFlushingDetectionPress);
float load_FlushingDetectionPressure(void);
#endif
void save_PowerSavingParam(const stSetupDataSet_t stSetupDataSet);
void load_PowerSavingParam(stSetupDataSet_t& stSetupDataSet);
void save_PowerSavingMode(const unsigned char nPowerSavingMode);
unsigned char load_PowerSavingMode(stSetupDataSet_t& stSetupDataSet);
void save_PowerSavingTime(const stSetupDataSet_t stSetupDataSet);
void load_PowerSavingTime(stSetupDataSet_t& stSetupDataSet);

// Print Data-log and System Informations
void print_SystemInformation(void);

void print_SystemErrorList(const bool bReload);
#if defined(_USE_APPLICATION_DEBUG_MSG_)
void print_EEPROMBackupAddressPointer(void);
void print_StructureDataSet(void);
void print_BrewingLogHeader(void);
void print_BrewingStandbyObservation(void);
#else
#if defined(_USE_BREW_PROFILE_ACQUISITION_)
void print_BrewingLogHeader(void);
void print_SensorObservation(void);
#endif
#endif

#endif //_MES_APPLICATION_H_

