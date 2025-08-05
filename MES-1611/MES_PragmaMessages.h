#ifndef _MES_PRAGMA_MESSAGES_H_
#define _MES_PRAGMA_MESSAGES_H_


#include "MES_Config.h"


#ifdef _MES_HAS_BUILD_MESSAGE_

	#if MACHINE_FAMILY == 0		// MES1611-LE
		#define BUILD_NOTICE_MESSAGE								"[NOTICE] MES-1611-LE v1.2.3-release (H/W v0.1-beta)"
	#elif MACHINE_FAMILY == 1	// MES1611-SD
		#define BUILD_NOTICE_MESSAGE								"[NOTICE] MES-1611-SD v1.4.0-release (H/W v1.1-release)"
	#elif MACHINE_FAMILY == 2	// MES1611-SD (OZBV)
		#define BUILD_NOTICE_MESSAGE								"[NOTICE] MES-1611-SD v2.0.0-alpha0 (H/W v2.0-alpha)"
	#else 						// MES1611-EV (Future)
		#define BUILD_NOTICE_MESSAGE								"[NOTICE] MES-1611-SD v0.0.0-future (H/W v0.0-future)"
	#endif

	#define BOARD_SUPPORT_PACKAGE_MESSAGE							"[NOTICE] Arduino AVR Board v1.6.8, Controllino Board v2.0.1"

	#ifdef _SYSTEM_ENGINEERING_MODE_
		#define SYSTEM_ENGINEERING_MODE_MESSAGE						"[NOTICE] System engineering mode enabled"
	#endif

	#ifdef _AUTONICS_DATA_ACQ_CLIENT_CONNECTED_
		#define AUTONICS_DAQ_CLIENT_CONNECTED_MESSAGE				"[NOTICE] Autonics DAQ client connection enabled"
	#endif

	#ifdef _SERIAL_COMMUNICATION_WITH_CLIENT_ENABLED_
		#define SERIAL_COMM_WITH_CLIENT_MESSAGE						"[NOTICE] Serial communication with client enabled"
	#endif

	#ifdef _USE_APPLICATION_DEBUG_MSG_
		#define APPLICATION_DEBUG_LOG_MESSAGE						"[NOTICE] Application log enabled"
	#endif

	#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
		#ifdef _USE_BREW_WATER_TEMP_SENSOR_K_L_TYPE_FOR_TEST_
			#define BREW_WATER_TEMP_SENSOR_K_L_TYPE_MESSAGE			"[NOTICE] Brew water temperature sensor type : K.L"
		#endif
	#endif

	#ifdef _USE_OZBV10AD2M2_MOTOR_CONTROLLER_
		#define MOTOR_PUMP_OZBV10AD2M2_MESSAGE						"[NOTICE] Motor Controller (OZBV-10A-D2M2) selected"
	#endif

	#if defined(_OBSERVE_MOTOR_SPEED_VALUE_) || defined(_OBSERVE_MOTOR_CURRENT_VALUE_)
		#define MOTOR_ELEC_OBSERVATION_MESSAGE						"[NOTICE] Motor observation enabled"
	#endif

	#if AUTOMATION_SENSOR_ATTACHED
		#define OPTICAL_FIBER_AMPLIFIER_MESSAGE						"[NOTICE] Optical Fiber Amplifier (BF4RP) attached"
	#endif

#endif


#ifndef MES_BUILD_NOTICE

	#define MES_BUILD_NOTICE

	#ifdef _MES_HAS_BUILD_MESSAGE_

		#pragma message (BUILD_NOTICE_MESSAGE)
		#pragma message (BOARD_SUPPORT_PACKAGE_MESSAGE)
		#pragma message (SYSTEM_ENGINEERING_MODE_MESSAGE)
		#pragma message (AUTONICS_DAQ_CLIENT_CONNECTED_MESSAGE)
		#pragma message (SERIAL_COMM_WITH_CLIENT_MESSAGE)
		#pragma message (APPLICATION_DEBUG_LOG_MESSAGE)
		#pragma message (BREW_WATER_TEMP_SENSOR_K_L_TYPE_MESSAGE)
		#pragma message (MOTOR_PUMP_OZBV10AD2M2_MESSAGE)
		#pragma message (MOTOR_ELEC_OBSERVATION_MESSAGE)
		#pragma message (OPTICAL_FIBER_AMPLIFIER_MESSAGE)

	#else

		#warning [NOTICE] MOAI Espresso Station Software (not really a warning)
		#warning 		  Check MES configuration file - MES_Config.h

	#endif

#endif



#endif//_MES_PRAGMA_MESSAGE_H_
