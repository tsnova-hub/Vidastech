#ifndef _MES_CONFIG_H_
#define _MES_CONFIG_H_


//////////////////////////////////////////////////////////////////////////////////////////
//			GLOBAL CONSTANT DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef MACHINE_FAMILY
	#define MACHINE_FAMILY											2			// 0 : LE,  1 : SD,  2 : SD-OZBV,  3 : EV
#endif

#if MACHINE_FAMILY == 0		// MES1611-LE
	#define HARDWARE_MODELNAME										"MES1611-LE"
	#define HARDWARE_VERSION										1
	#define HARDWARE_TYPEDEF										'b'
	#define FIRMWARE_VERSION 										12 			// ex) Ver. 1.5 ==> 15 (Max : 255 ==> Ver. 25.5)
	#define FIRMWARE_PATCH											3			// ex) Ver. 1.5.1 ==> 1
	#define FIRMWARE_TYPEDEF										'r'			// ex) Beta ==> b, Alpha ==> a, Released ==> r
#elif MACHINE_FAMILY == 1		// MES1611-SD
	#define HARDWARE_MODELNAME										"MES1611-SD"
	#define HARDWARE_VERSION										11
	#define HARDWARE_TYPEDEF										'r'
	#define FIRMWARE_VERSION 										14
	#define FIRMWARE_PATCH											0
	#define FIRMWARE_TYPEDEF										'r'
#elif MACHINE_FAMILY == 2		// MES1611-SD (OZBV)
	#define HARDWARE_MODELNAME										"MES1611-SD"
	#define HARDWARE_VERSION										20
	#define HARDWARE_TYPEDEF										'r'
	#define FIRMWARE_VERSION 										20
	#define FIRMWARE_PATCH											0
	#define FIRMWARE_TYPEDEF										'r'
#else // MACHINE_FAMILY == 3	// MES1611-EV (Future)
	#define HARDWARE_MODELNAME										"MES1611-EV"
	#define HARDWARE_VERSION										0
	#define HARDWARE_TYPEDEF										'b'
	#define FIRMWARE_VERSION 										0
	#define FIRMWARE_PATCH											0
	#define FIRMWARE_TYPEDEF										'b'
#endif

#define FACTORY_DEFAULT_CODE										0xE5		// 0xE5 : Already Factory Default Reset
#define APPLICATION_RUN_CODE										0x5E 		// 0x5E : Ready to Run in Application Mode
#define AUTHORIZATION_CODE											7416

#define RTC_FACTORY_DEFAULT_DAY										1			// 2019.01.01 (TUE) : Factory Default Time-Date
#define RTC_FACTORY_DEFAULT_MONTH									1
#define RTC_FACTORY_DEFAULT_YEAR									19


//////////////////////////////////////////////////////////////////////////////////////////
// Definition for Functional Constraints, Testing and Debugging, Build Configurations

#define _MES_HAS_BUILD_MESSAGE_

// #define _USE_APPLICATION_DEBUG_MSG_

// #define _SYSTEM_ENGINEERING_MODE_

// #define _AUTONICS_DATA_ACQ_CLIENT_CONNECTED_

// #define _SERIAL_COMMUNICATION_WITH_CLIENT_ENABLED_

// #define _SETUP_WEEK_DAY_


#define _USE_PSEUDO_CONVERT_FUNCTION_											// > v1.2r (b
#define _USE_MOVING_AVERAGED_TEMPERATURE_										// > v1.3r (func-added)

#define _ROTARY_ENCODER_ACCELERATION_											// > v1.4r (func-added)

#define _USE_BREW_WATER_TEMP_SENSOR_K_L_TYPE_FOR_TEST_							// > v1.4r (func-added)

#define _LAMP_DELAYED_OFF_														// > v2.0b (func-added)

#define _PIXELLED_DISPLAY_VLONG_PUSHED_BREW_BUTTON_								// > v2.0b (func-added)

// #define _USE_EEPROM_WORN_OUT_NOTICE_MESSAGE_									// < v2.0b (func-deprecated)


#define FLOAT_POINT_PRECISION_FOR_DATA_ACQ							(3)

#if (MACHINE_FAMILY  <= 1)

	// TM4 Interfaced
	#define PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED				(true)
	#define BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define BREW_WATER_TEMPERATURE_SENSOR_ATTACHED					(false)

	// OP-AMP Interfaced
	#define BREW_PRESSURE_SENSOR_ATTACHED							(true)
	#define PUMP_PRESSURE_SENSOR_ATTACHED							(false)
	#define AUX_CURRENT_SENSOR_ATTACHED								(false)
	// #define AUX_VOLTAGE_SENSOR_ATTACHED							(false)		// deprecated : > v1.3r (hw-modified)

	// External Teminal Interfaced
	#define EXTERNAL_ANALOG_INPUT_CH4_CONNECTED						(false)
	#define EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED					(false)

	// Brew-Boiler Pressure Stabilized
	#define BREW_BOILER_PRESSURE_STABILIZATION						(true)

	// Auto-Flushing / Auto-Brewing
	#define AUTOMATION_SENSOR_ATTACHED								(false)

#elif (MACHINE_FAMILY == 2)

	// TM4 Interfaced
	#define PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED				(true)
	#define BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define BREW_WATER_TEMPERATURE_SENSOR_ATTACHED					(false)

	// OP-AMP Interfaced
	#define BREW_PRESSURE_SENSOR_ATTACHED							(true)
	#define PUMP_PRESSURE_SENSOR_ATTACHED							(false)
	#define AUX_CURRENT_SENSOR_ATTACHED								(false)

	// External Teminal Interfaced
	#define EXTERNAL_ANALOG_INPUT_CH4_CONNECTED						(false)
	#define EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED					(false)

	// Brew-Boiler Pressure Stabilized
	#define BREW_BOILER_PRESSURE_STABILIZATION						(true)

	// Auto-Flushing / Auto-Brewing
	#define AUTOMATION_SENSOR_ATTACHED								(true)


	#define _USE_OZBV10AD2M2_MOTOR_CONTROLLER_									// > v2.0b (hw-replaced)

	#ifdef _USE_OZBV10AD2M2_MOTOR_CONTROLLER_

		// #define _OBSERVE_MOTOR_SPEED_VALUE_									// > v2.0b (hw-replaced), for meetup

		// #define _OBSERVE_MOTOR_CURRENT_VALUE_								// > v2.0b (hw-replaced), for meetup

	#endif

	#ifndef _USE_APPLICATION_DEBUG_MSG_
		
		// #define _USE_BREW_PROFILE_ACQUISITION_								// for meetup

	#endif

#else // (MACHINE_FAMILY == 3)

	// TM4 Interfaced
	#define PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED				(true)
	#define BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED					(true)
	#define BREW_WATER_TEMPERATURE_SENSOR_ATTACHED					(true)

	// OP-AMP Interfaced
	#define BREW_PRESSURE_SENSOR_ATTACHED							(true)
	#define PUMP_PRESSURE_SENSOR_ATTACHED							(true)
	#define AUX_CURRENT_SENSOR_ATTACHED								(false)

	// External Teminal Interfaced
	#define EXTERNAL_ANALOG_INPUT_CH4_CONNECTED						(false)
	#define EXTERNAL_INTERRUPT_SIGNAL4_CONNECTED					(false)

	// Brew-Boiler Pressure Stabilized
	#define BREW_BOILER_PRESSURE_STABILIZATION						(true)

	// Auto-Flushing / Auto-Brewing
	#define AUTOMATION_SENSOR_ATTACHED								(true)


	#define _USE_OZBV10AD2M2_MOTOR_CONTROLLER_									// > v2.0b (hw-replaced)

#endif


// SERIAL COMM. BAUDRATE FOR CONSOLE/USB
//	115200 [Baud] for Console/USB (Serial)
//	8 sets data, Non parity bits, 1 stop bits

// SERIAL COMM. BAUDRATE FOR MODBUS RTU BUS
#if (MACHINE_FAMILY == 0)

	// Max. Baud Rate of TM4 : 19200 [bps]
	// 8 sets data, Non parity bits, 1 stop bits
	#define USE_TM4_RS485_COMM_PROTOCOL								(false) // (true) : for KC Certi.

#else

	// Max. Baud Rate of TM4 : 19200 [bps]
	// 8 sets data, Non parity bits, 2 stop bits
	#define USE_TM4_RS485_COMM_PROTOCOL								(true)

#endif


#endif//_MES_CONFIG_H_

