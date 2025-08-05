#ifndef _CMOTORPUMP_OZBV10A_H_
#define _CMOTORPUMP_OZBV10A_H_

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


// #ifndef _FlashString
// 	#define _FlashString 							__FlashStringHelper	// char
// #endif


#if defined(_USE_APPLICATION_DEBUG_MSG_)
	#define _TEST_CMOTORPUMP_BLDC_
#endif


// #define _USE_MOTOR_PUMP_ALARM_PROC_MODE_


// SERIAL COMM. BAUDRATE FOR MOTOR CONTROLLER (RS232-MODBUS RTU)
#define OZBV10A_BAUD_RATE							(9600)				// Max. Baud Rate of Motor Controller : 9600 [bps]
#define OZBV10A_COMM_CONFIG							(SERIAL_8N1)		// 8 sets data, Non parity bits, 1 stop bits
#define OZBV10A_RCV_WAIT_TIME						(20)				// 20 msec, Max. Time to wait for serial incoming data

#define OZBV10A_RS232_MODEL							(true)				// RS232 Version


// PROCON MOTOR PUMP AND OZ MOTOR CONTROLLER DESCRIPTIONS
// "The flow rate @ 10bar at 1800 is 30LPH. The pump model number should be changed to 61A1FCF2QAP.
// There is no problem to run the pump at 3000 rpm. The normal speed range is 1000~2500 rpm.
// The max speed of this motor can be 3700rpm.
// We do not suggest to run the motor below 800rpm.”

// Procon Motor Pump : 0 ~ 3700 [rpm] (effective : 800 ~ 3000 [rpm])
// OZBV-10A-D2M2 Motor Controller : 0 ~ 3000 [rpm] (effective : 300 ~ 3000 [rpm])
// ==> Effective Motor Pump : 800 ~ 3000 [rpm]
#define _LOWER_RPM_LIMIT_							(800UL)
#define _UPPER_RPM_LIMIT_							(3000UL)

#define _ZERO_RPM_VALUE_							(0UL)
#define _MIN_RPM_VALUE_								(1000UL)
#define _MAX_RPM_VALUE_								(2500UL)
#define _DEFAULT_RPM_VALUE_							(1800UL) 			// SPEC : about 1800 [rpm] @ 10 bar, 30 Liters/Hour

#define _ZERO_RPM_RATE_								(0.000F)			// rate=0%   : value=0 [rpm]
#define _MIN_RPM_RATE_								(0.010F)			// rate=1%   : value=1015 [rpm]
#define _MAX_RPM_RATE_								(1.000F)			// rate=100% : value=2500 [rpm]

// Effective RPM Values
#define _ZERO_RPM_VALUE_EFF_						(0UL)				// RPM_RATE = 0.00 (FOR WATER PRESSURE)
#define _MIN_RPM_VALUE_EFF_							(1015UL)			// RPM_RATE = 0.01 (DEFAULT TO INJECTION)
#define _MAX_RPM_VALUE_EFF_							(2500UL)            // RPM_RATE = 1.00
#define _DEFAULT_RPM_VALUE_EFF_						(2200UL) 			// RPM_RATE = 0.80 (DEFAULT TO EXTRACTION)

#define _CW_DIRECTION_								(1)
#define _CCW_DIRECTION_								(-1)
#define _DEFAULT_DIRECTION_							(_CCW_DIRECTION_)

#define _MIN_ACC_TIME_								(1UL)				// 0.001 [sec] @ 1000 [rpm]
#define _MAX_ACC_TIME_								(20000UL)			// 20 [sec] @ 1000 [rpm]
#define _MIN_DEC_TIME_								(_MIN_ACC_TIME_)
#define _MAX_DEC_TIME_								(_MAX_ACC_TIME_)
#define _ACC_TIME_BASIS_RPM_						(1000UL)			// 1000 [rpm]
#define _DEC_TIME_BASIS_RPM_						(_ACC_TIME_BASIS_RPM_)

#define MOTOR_RESET_DURATION						(50UL)				// < 100 [msec]
#define MOTOR_PARAMS_SAVE_LATENCY					(500UL)				// < 1000 [msec]


//////////////////////////////////////////////////////////////////////////////////////////
// DEFAULT REGISTER VALUE OF MOTOR CONTROLLER

#define _DEFAULT_COMM_ID_							(0)					// [5] Communication ID : 0 (Internally ID = 1)
#define _DEFAULT_COMM_BAUDRATE_						(0)					// [6] Communication Baudrate : 0 (9600 [bps])

#define _DEFAULT_CURRENT_P_GAIN_					(1600)				// [13] Current P Gain : 1600
#define _DEFAULT_CURRENT_I_GAIN_					(0)					// [14] Current I Gain : 0
#define _DEFAULT_SPEED_P_GAIN_						(3200)				// [15] Speed P Gain : 3200
#define _DEFAULT_SPEED_I_GAIN_						(160)				// [16] Speed I Gain : 160

#define _DEFAULT_ACC_TIME_							(100UL)				// [20] 100 [msec] @ up to 1000 [rpm] (about 220 [msec] @ up to 2200 [rpm])
#define _DEFAULT_DEC_TIME_							(500UL)				// [21] 500 [msec] @ up to 1000 [rpm] (about 1100 [msec] @ up to 2200 [rpm])

#define _DEFAULT_MIN_SPEED_							(1UL)				// [22] Minimum Speed : 1 [rpm]
#define _DEFAULT_MAX_SPEED_							(_MAX_RPM_VALUE_)	// [23] Maximum Speed : 2500 [rpm]

#define _DEFAULT_CURRENT_LIMIT_						(6900)				// [25]	Current Limit : 6900 [mA]
#define _DEFAULT_CURRENT_LIMIT_TIME_				(30)				// [25]	Current Limit Time Condition : 30 [msec]

#define _DEFAULT_OVERHEAT_LIMIT_					(70)				// [27] Overheat Limit : 70 [dec C]
#define _DEFAULT_OVERHEAT_LIMIT_TIME_				(2000)				// [28] Overheat Limit Time Condition : 2000 [msec]

#define _DEFAULT_POWER_DIFF_LIMIT_TIME_				(300)				// [29] Power Diff. Limit Time Condition : 300 [msec]

#define _DEFAULT_MOTOR_STOP_MODE_					(2)					// [30] Stop Mode : 2 (Stop w/o DEC Time + Motor Free)
#define _DEFAULT_MOTOR_EMG_STOP_MODE_				(2)					// [31] Emergency Stop Mode : 2 (Stop w/o DEC Time + Motor Free)

#define _DEFAULT_MONITORING_MODE_					(0)					// [37] Monitoring Mode : 0

#define _DEFAULT_SPEED_PERMISSIBLE_RANGE_			(10)				// [38] Speed Permissible Range : 10 (+/- 5 [rpm])

#define _DEFAULT_IN_POLARITY_						(15)				// [40] Input IO Polarity : 15 (IN1, IN2, IN3, IN4 = A Contact)
#define _DEFAULT_OUT_POLARITY_						(7)					// [41] Output IO Polarity : 7 (OUT1, OUT2, OUT3 = A Contact)

#define _DEFAULT_MOTOR_RUN_MODE_					(0)					// [42] Drive Mode : 0 (RS232 / Modbus RTU Protocol)

#define _DEFAULT_IN1_FUNCTION_						(0)					// [43] Input IO Function : (User)
#define _DEFAULT_IN2_FUNCTION_						(0)					// [44] Input IO Function : (User)
#define _DEFAULT_IN3_FUNCTION_						(0)					// [45] Input IO Function : (User)
#define _DEFAULT_IN4_FUNCTION_						(0)					// [46] Input IO Function : (User)

#define _DEFAULT_OUT1_FUNCTION_						(1)					// [47] Output IO Function : Alarm
#define _DEFAULT_OUT2_FUNCTION_						(2)					// [48] Output IO Function : Speed Out
#define _DEFAULT_OUT3_FUNCTION_						(3)					// [49] Output IO Function : Direction (CW/CCW)

#define _DEFAULT_ANALOG_IN_POLARITY_				(1)					// [60] Analog IN Polarity : 1 (Unipolar)
#define _DEFAULT_ANALOG_IN_MINIMUM_					(100)				// [61] Analog IN Minimum : 100 [mA]

#define _DEFAULT_TORQUE_CHECK_						(0)					// [63] Motor Torque Check : 0
#define _DEFAULT_TORQUE00_RPM_						(3000)				// [64] Motor Torque 0_0 RPM : 3000 [rpm]
#define _DEFAULT_TORQUE00_CURRENT_BASE_				(2920)				// [65] Motor Torque 0_0 Current Base : 2920 [mA]
#define _DEFAULT_TORQUE00_CURRENT_LIMIT_			(3980)				// [66] Motor Torque 0_0 Current Limit : 3980 [mA]
#define _DEFAULT_TORQUE01_RPM_						(2500)				// [67] Motor Torque 0_1 RPM : 2500 [rpm]
#define _DEFAULT_TORQUE01_CURRENT_BASE_				(3000)				// [68] Motor Torque 0_1 Current Base : 3000 [mA]
#define _DEFAULT_TORQUE01_CURRENT_LIMIT_			(4240)				// [69] Motor Torque 0_1 Current Limit : 4240 [mA]
#define _DEFAULT_TORQUE10_RPM_						(2500)				// [70] Motor Torque 0_1 RPM : 2500 [rpm]
#define _DEFAULT_TORQUE10_CURRENT_BASE_				(3000)				// [71] Motor Torque 0_1 Current Base : 3000 [mA]
#define _DEFAULT_TORQUE10_CURRENT_LIMIT_			(4240)				// [72] Motor Torque 0_1 Current Limit : 4240 [mA]
#define _DEFAULT_TORQUE11_RPM_						(1500)				// [73] Motor Torque 0_1 RPM : 1500 [rpm]
#define _DEFAULT_TORQUE11_CURRENT_BASE_				(1980)				// [74] Motor Torque 0_1 Current Base : 1980 [mA]
#define _DEFAULT_TORQUE11_CURRENT_LIMIT_			(2870)				// [75] Motor Torque 0_1 Current Limit : 2870 [mA]
#define _DEFAULT_TORQUE20_RPM_						(1500)				// [76] Motor Torque 0_1 RPM : 1500 [rpm]
#define _DEFAULT_TORQUE20_CURRENT_BASE_				(1980)				// [77] Motor Torque 0_1 Current Base : 1980 [mA]
#define _DEFAULT_TORQUE20_CURRENT_LIMIT_			(2870)				// [78] Motor Torque 0_1 Current Limit : 2870 [mA]
#define _DEFAULT_TORQUE21_RPM_						(500)				// [79] Motor Torque 0_1 RPM : 500 [rpm]
#define _DEFAULT_TORQUE21_CURRENT_BASE_				(940)				// [80] Motor Torque 0_1 Current Base : 940 [mA]
#define _DEFAULT_TORQUE21_CURRENT_LIMIT_			(1440)				// [81] Motor Torque 0_1 Current Limit : 1440 [mA]
#define _DEFAULT_TORQUE30_RPM_						(500)				// [82] Motor Torque 0_1 RPM : 500 [rpm]
#define _DEFAULT_TORQUE30_CURRENT_BASE_				(940)				// [83] Motor Torque 0_1 Current Base : 940 [mA]
#define _DEFAULT_TORQUE30_CURRENT_LIMIT_			(100)				// [84] Motor Torque 0_1 Current Limit : 100 [mA]
#define _DEFAULT_TORQUE31_RPM_						(300)				// [85] Motor Torque 0_1 RPM : 300 [rpm]
#define _DEFAULT_TORQUE31_CURRENT_BASE_				(750)				// [86] Motor Torque 0_1 Current Base : 750 [mA]
#define _DEFAULT_TORQUE31_CURRENT_LIMIT_			(1190)				// [87] Motor Torque 0_1 Current Limit : 1190 [mA]
#define _DEFAULT_TORQUE40_RPM_						(300)				// [88] Motor Torque 0_1 RPM : 300 [rpm]
#define _DEFAULT_TORQUE40_CURRENT_BASE_				(750)				// [89] Motor Torque 0_1 Current Base : 750 [mA]
#define _DEFAULT_TORQUE40_CURRENT_LIMIT_			(1190)				// [90] Motor Torque 0_1 Current Limit : 1190 [mA]
#define _DEFAULT_TORQUE41_RPM_						(100)				// [91] Motor Torque 0_1 RPM : 100 [rpm]
#define _DEFAULT_TORQUE41_CURRENT_BASE_				(560)				// [92] Motor Torque 0_1 Current Base : 560 [mA]
#define _DEFAULT_TORQUE41_CURRENT_LIMIT_			(100)				// [93] Motor Torque 0_1 Current Limit : 100 [mA]
#define _DEFAULT_SPEED_OFFSET0_						(0)					// [94] Motor Speed Offset0 : 0 [rpm]
#define _DEFAULT_SPEED_OFFSET1_						(0)					// [95] Motor Speed Offset1 : 0 [rpm]
#define _DEFAULT_SPEED_OFFSET2_						(0)					// [96] Motor Speed Offset2 : 0 [rpm]
#define _DEFAULT_TORQUE_CURRENT_LIMIT_TIME_			(200)				// [97] Motor Torque Current Limit Time Condition : 200 [msec]

#define _DEFAULT_RUN_DIRECTION_						(0)					// [100] Run Direction : 0 (Procon Motor Pump)
#define _DEFAULT_POLARITY_NUMBER_					(6)					// [101] Polarity Number : 6 (Procon Motor Pump)

#define _DEFAULT_CURRENT_FILTER_					(512)				// [103] Current Filter
#define _DEFAULT_SPEED_FILTER_						(256)				// [104] Speed Filter

#define _DEFAULT_EXTERNAL_VOLUME_FILTER_			(128)				// [105] External Volume Filter

#define _DEFAULT_CURRENT_SENSOR_CAPACITY_			(10000)				// [106] Current Sensor Capacity : 10000 [mA]


//////////////////////////////////////////////////////////////////////////////////////////
// MODBUS RTU LIBRARY RELATED CONSTANTS DEFINITION

// OZBV-10A-D2M2 RELATED DEFINITIONS
#define OZBV_TOTAL_CH_PER_UNIT						(1)
#define OZBV_TOTAL_NO_OF_CH							(OZBV_TOTAL_CH_PER_UNIT*1)
#define OZBV_UNIT_BASE_ADDR							(0x00)
#define OZBV_MASTER_ADDR 							(OZBV_UNIT_BASE_ADDR+1)

// OUPUT-IO STATUS (MODBUS RTU FUNC 01H/05H (COIL READ/WRITE), R/W)
#define OZBV_IO_OUTPUT_REG_BASE_ADDR				(0x0000)
#define OZBV_OUT1_REG_ADDR							(0x0000)
#define OZBV_OUT2_REG_ADDR							(0x0001)
#define OZBV_OUT3_REG_ADDR							(0x0002)

// INPUT-IO STATUS (MODBUS RTU FUNC 02H (DISCRETES INPUT READ), R)
#define OZBV_IO_INPUT_REG_BASE_ADDR					(0x0000)
#define OZBV_IN1_REG_ADDR							(0x0000)
#define OZBV_IN2_REG_ADDR							(0x0001)
#define OZBV_IN3_REG_ADDR							(0x0002)
#define OZBV_IN4_REG_ADDR							(0x0003)

// PRESENT STATUS (MODBUS RTU FUNC 04H (INPUT REGISTER READ), R)
#define OZBV_PRESENT_STATE_REG_BASE_ADDR			(0x0000)
#define OZBV_PRESENT_STATE_REG_ADDR 				(0x0000)
#define OZBV_PRESENT_ALARM_REG_ADDR					(0x0001)
#define OZBV_PRESENT_SPEED_SETPOINT_REG_ADDR		(0x0002)			// [rpm]
#define OZBV_PRESENT_SPEED_REG_ADDR					(0x0003)			// [rpm]
#define OZBV_PRESENT_CURRENT_SETPOINT_REG_ADDR		(0x0004)			// [mA]
#define OZBV_PRESENT_CURRENT_REG_ADDR				(0x0005)			// [mA]
#define OZBV_PRESENT_EXTERNAL_VOLUME_REG_ADDR		(0x0006)

// PARAMETERS TABLE (MODBUS RTU FUNC 03H/06H (SINGLE HOLDING REGISTER READ/WRITE), R/W)
#define OZBV_CONTROL_SETTING_GRP_BASE_ADDR			(0x0000)
#define OZBV_COMM_SLAVE_ID_REG_ADDR					(0x0005)			// 0
#define OZBV_COMM_BAUDRATE_REG_ADDR					(0x0006)			// 0
#define OZBV_CURRENT_LOOP_REG_ADDR					(0x000A)			// 1,			do not change
#define OZBV_SPPED_LOOP_REG_ADDR					(0x000B)			// 1,			do not change
#define OZBV_CURRENT_P_GAIN_REG_ADDR				(0x000D)			// 1600
#define OZBV_CURRENT_I_GAIN_REG_ADDR				(0x000E)			// 0
#define OZBV_SPEED_P_GAIN_REG_ADDR					(0x000F)			// 3200
#define OZBV_SPEED_I_GAIN_REG_ADDR					(0x0010)			// 160
#define OZBV_ACC_TIME_REG_ADDR						(0x0014)			// 1000	[msec]
#define OZBV_DEC_TIME_REG_ADDR						(0x0015)			// 500	[msec]
#define OZBV_MIN_SPEED_REG_ADDR						(0x0016)			// 1	[rpm]
#define OZBV_MAX_SPEED_REG_ADDR						(0x0017)			// 2500	[rpm]
#define OZBV_CURRENT_LIMIT_REG_ADDR					(0x0019)			// 6900 [mA]
#define OZBV_CURRENT_LIMIT_TIME_REG_ADDR			(0x001A)			// 30	[msec]
#define OZBV_OVERHEAT_LIMIT_REG_ADDR				(0x001B)			// 70	[dec C]
#define OZBV_OVERHEAT_LIMIT_TIME_REG_ADDR			(0x001C)			// 2000	[msec]
#define OZBV_POWER_DIFF_LIMIT_TIME_REG_ADDR			(0x001D)			// 300 	[msec]
#define OZBV_MOTOR_STOP_MODE_REG_ADDR				(0x001E)			// 2
#define OZBV_MOTOR_EMG_STOP_MODE_REG_ADDR			(0x001F)			// 2
#define OZBV_MONITORING_MODE_REG_ADDR				(0x0025)			// 0
#define OZBV_SPEED_PERMISSIBLE_RANGE_REG_ADDR		(0x0026)			// 10	[rpm]
#define OZBV_IN_POLARITY_SEL_REG_ADDR				(0x0028)			// 15
#define OZBV_OUT_POLARITY_SEL_REG_ADDR				(0x0029)			// 7
#define OZBV_MOTOR_RUN_MODE_REG_ADDR				(0x002A)			// 0
#define OZBV_IN1_FUNCTION_REG_ADDR					(0x002B)			// 5
#define OZBV_IN2_FUNCTION_REG_ADDR					(0x002C)			// 0
#define OZBV_IN3_FUNCTION_REG_ADDR					(0x002D)			// 0
#define OZBV_IN4_FUNCTION_REG_ADDR					(0x002E)			// 6
#define OZBV_OUT1_FUNCTION_REG_ADDR					(0x002F)			// 1
#define OZBV_OUT2_FUNCTION_REG_ADDR					(0x0030)			// 2
#define OZBV_OUT3_FUNCTION_REG_ADDR					(0x0031)			// 3
#define OZBV_SPEED_INDEX0_REG_ADDR					(0x0032)			// 500	[rpm]
#define OZBV_SPEED_INDEX1_REG_ADDR					(0x0033)			// 1000	[rpm]
#define OZBV_SPEED_INDEX2_REG_ADDR					(0x0034)			// 2000	[rpm]
#define OZBV_SPEED_INDEX3_REG_ADDR					(0x0035)			// 3000	[rpm]
#define OZBV_SPEED_INDEX4_REG_ADDR					(0x0036)			// -500	[rpm]
#define OZBV_SPEED_INDEX5_REG_ADDR					(0x0037)			// -1000[rpm]
#define OZBV_SPEED_INDEX6_REG_ADDR					(0x0038)			// -2000[rpm]
#define OZBV_SPEED_INDEX7_REG_ADDR					(0x0039)			// -3000[rpm]
#define OZBV_ANALOG_INPUT_POLARITY_REG_ADDR			(0x003C)			// 1
#define OZBV_ANALOG_INPUT_MINIMUM_REG_ADDR			(0x003D)			// 100	[mV]
#define OZBV_FND_DISPLAY_DEFAULT_REG_ADDR			(0x003E)			// 0
#define OZBV_MOTOR_TORQUE_CHECK_REG_ADDR			(0x003F)			// 0, 			do not change
#define OZBV_MOTOR_TORQUE00_RPM_REG_ADDR			(0x0040)			// 3000 [rpm],	do not change
#define OZBV_MOTOR_TORQUE00_CURRENT_BASE_REG_ADDR	(0x0041)			// 2920 [mA],	do not change
#define OZBV_MOTOR_TORQUE00_CURRENT_LIMIT_REG_ADDR	(0x0042)			// 3980 [mA],	do not change
#define OZBV_MOTOR_TORQUE01_RPM_REG_ADDR			(0x0043)			// 2500 [rpm],	do not change
#define OZBV_MOTOR_TORQUE01_CURRENT_BASE_REG_ADDR	(0x0044)			// 3000 [mA],	do not change
#define OZBV_MOTOR_TORQUE01_CURRENT_LIMIT_REG_ADDR	(0x0045)			// 4240 [mA],	do not change
#define OZBV_MOTOR_TORQUE10_RPM_REG_ADDR			(0x0046)			// 2500 [rpm],	do not change
#define OZBV_MOTOR_TORQUE10_CURRENT_BASE_REG_ADDR	(0x0047)			// 3000 [mA],	do not change
#define OZBV_MOTOR_TORQUE10_CURRENT_LIMIT_REG_ADDR	(0x0048)			// 4240 [mA],	do not change
#define OZBV_MOTOR_TORQUE11_RPM_REG_ADDR			(0x0049)			// 1500 [rpm],	do not change
#define OZBV_MOTOR_TORQUE11_CURRENT_BASE_REG_ADDR	(0x004A)			// 1980 [mA],	do not change
#define OZBV_MOTOR_TORQUE11_CURRENT_LIMIT_REG_ADDR	(0x004B)			// 2870 [mA],	do not change
#define OZBV_MOTOR_TORQUE20_RPM_REG_ADDR			(0x004C)			// 1500 [rpm],	do not change
#define OZBV_MOTOR_TORQUE20_CURRENT_BASE_REG_ADDR	(0x004D)			// 1980 [mA],	do not change
#define OZBV_MOTOR_TORQUE20_CURRENT_LIMIT_REG_ADDR	(0x004E)			// 2870 [mA],	do not change
#define OZBV_MOTOR_TORQUE21_RPM_REG_ADDR			(0x004F)			// 500	[rpm],	do not change
#define OZBV_MOTOR_TORQUE21_CURRENT_BASE_REG_ADDR	(0x0050)			// 940	[mA],	do not change
#define OZBV_MOTOR_TORQUE21_CURRENT_LIMIT_REG_ADDR	(0x0051)			// 1440 [mA],	do not change
#define OZBV_MOTOR_TORQUE30_RPM_REG_ADDR			(0x0052)			// 500	[rpm],	do not change
#define OZBV_MOTOR_TORQUE30_CURRENT_BASE_REG_ADDR	(0x0053)			// 940	[mA],	do not change
#define OZBV_MOTOR_TORQUE30_CURRENT_LIMIT_REG_ADDR	(0x0054)			// 100	[mA],	do not change
#define OZBV_MOTOR_TORQUE31_RPM_REG_ADDR			(0x0055)			// 300	[rpm],	do not change
#define OZBV_MOTOR_TORQUE31_CURRENT_BASE_REG_ADDR	(0x0056)			// 750	[mA],	do not change
#define OZBV_MOTOR_TORQUE31_CURRENT_LIMIT_REG_ADDR	(0x0057)			// 1190 [mA],	do not change
#define OZBV_MOTOR_TORQUE40_RPM_REG_ADDR			(0x0058)			// 300	[rpm],	do not change
#define OZBV_MOTOR_TORQUE40_CURRENT_BASE_REG_ADDR	(0x0059)			// 750	[mA],	do not change
#define OZBV_MOTOR_TORQUE40_CURRENT_LIMIT_REG_ADDR	(0x005A)			// 1190 [mA],	do not change
#define OZBV_MOTOR_TORQUE41_RPM_REG_ADDR			(0x005B)			// 100	[rpm],	do not change
#define OZBV_MOTOR_TORQUE41_CURRENT_BASE_REG_ADDR	(0x005C)			// 560	[mA],	do not change
#define OZBV_MOTOR_TORQUE41_CURRENT_LIMIT_REG_ADDR	(0x005D)			// 100	[mA],	do not change
#define OZBV_SPEED_OFFSET0_REG_ADDR					(0x005E)			// 0	[rpm],	do not change
#define OZBV_SPEED_OFFSET1_REG_ADDR					(0x005F)			// 0	[rpm],	do not change
#define OZBV_SPEED_OFFSET2_REG_ADDR					(0x0060)			// 0,	[rpm],	do not change
#define OZBV_TORQUE_CURRENT_LIMIT_TIME_REG_ADDR		(0x0061)			// 200	[msec],	do not change
#define OZBV_MOTOR_RUN_DIRECTION_REG_ADDR			(0x0064)			// 0
#define OZBV_MOTOR_POLARITY_NUMBER_REG_ADDR			(0x0065)			// 6
#define OZBV_CURRENT_FILTER_REG_ADDR				(0x0067)			// 512,			do not change
#define OZBV_SPEED_FILTER_REG_ADDR					(0x0068)			// 256,			do not change
#define OZBV_EXTERNAL_VOLUME_FILTER_REG_ADDR		(0x0069)			// 128,			do not change
#define OZBV_CURRENT_SENSOR_CAPACITY_REG_ADDR		(0x006A)			// 10000,		do not change

#define OZBV_COMMAND_GRP_BASE_ADDR					(0x0078)
#define OZBV_COMMAND_REG_ADDR 						(0x0078)
#define OZBV_SPEED_SETPOINT_REG_ADDR 				(0x0079)
#define OZBV_HARDWARE_VERSION_REG_ADDR				(0x008E)			// 12001 = 12.0.01
#define OZBV_SOFTWARE_VERSION_REG_ADDR				(0x008F)			// 14001 = 14.0.01


// MOTOR CONTROLLER : COMMAND
#define COMMAND_NONE								(0)
#define COMMAND_RUN									(1)
#define COMMAND_STOP								(2)
#define COMMAND_EMG_STOP							(3)
#define COMMAND_RESET								(4)
#define COMMAND_RESERVED1							(5)
#define COMMAND_RESERVED2							(6)
#define COMMAND_RESERVED3							(7)
#define COMMAND_RESERVED4							(8)
#define COMMAND_RESERVED5							(9)
#define COMMAND_RESERVED6							(10)
#define COMMAND_FREE_STOP							(11)
#define COMMAND_BRAKE_STOP							(12)
#define COMMAND_PARAMS_SAVE							(13)

// MOTOR CONTROLLER : OUTPUT IO STATUS
#define MOTOR_OUT1_STATE							(0b00000001)		// ALARM
#define MOTOR_OUT2_STATE							(0b00000010)		// SPEED OUT : Pulse
#define MOTOR_OUT3_STATE							(0b00000100)		// DIR : CW = 0, CCW = 1

// MOTOR CONTROLLER : INPUT IO STATUS
#define MOTOR_IN1_STATE								(0b00000001)
#define MOTOR_IN2_STATE								(0b00000010)
#define MOTOR_IN3_STATE								(0b00000100)
#define MOTOR_IN4_STATE								(0b00001000)

// MOTOR CONTROLLER : PRESENT STATES
#define MOTOR_BRAKE_STATE							(0b00000001)		// BRK
#define MOTOR_FREE_STATE							(0b00000010)		// FRE
#define MOTOR_ALARM_STATE							(0b00000100)		// ALM
#define MOTOR_EMERGENCY_STOP_STATE					(0b00001000)		// EMG
#define MOTOR_DEC_STATE								(0b00010000)		// DEC
#define MOTOR_ACC_STATE								(0b00100000)		// ACC
#define MOTOR_CCW_STATE								(0b01000000)		// DIR : CW = 0, CCW = 1
#define MOTOR_RUN_STATE								(0b10000000)		// RUN : STOP = 0, RUN = 1

// MOTOR CONTROLLER : PRESENT ALARM
#define MOTOR_NORMAL								(0)
#define MOTOR_EMERGENCY_STOP						(1)
#define MOTOR_OVERCURRENT							(2)
#define MOTOR_OVERHEAT								(3)
#define MOTOR_HALLSENSOR_ABNORMAL					(4)
#define MOTOR_OVERVOLTAGE							(5)
#define MOTOR_UNDERVOLTAGE							(6)
#define MOTOR_OVERLOAD								(7)
#define MOTOR_SHORT_DRIVE							(8)


// MOTOR CONTROLLER : INVALID VALUE (RECEIVED VALUE)
#define OZBV_INVALID_OUT_STATUS						(MODBUS_RXBUF_RESET)
#define OZBV_INVALID_IN_STATUS						(MODBUS_RXBUF_RESET)
#define OZBV_INVALID_PRESENT_STATUS					(MODBUS_RXBUF_RESET)
#define OZBV_INVALID_CONTROL_PARAMS					(MODBUS_RXBUF_RESET)


class CMotorPumpBLDC : public CModbusRTU
{
public:
	CMotorPumpBLDC();

	void setModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr = OZBV_MASTER_ADDR);
#if OZBV10A_RS232_MODEL == false
	void setPreTransmission(void (*preTransmission)(void));
	void setPostTransmission(void (*postTransmission)(void));
#endif

	void clear(void);

#ifdef _USE_MOTOR_PUMP_ALARM_PROC_MODE_
	void setPostAlarm(void (*postAlarm)(void));
#endif

	// Motor Controller APIs
	bool init(void);					// Factory default set
	void reset(void);
	void start(void);
	void start(const unsigned int nRPMValue);
	void start(const float fRPMRate);
	void stop(void);
	void stopEMG(void);
	void setRPM(const unsigned int nRPMValue);
	void setRPM(const float fRPMRate);
	void setACC(const unsigned int nACCTime);
	void setDEC(const unsigned int nDECTime);

	int getParams(const unsigned int nStartAddr);
	int getSpeed(void);
	int getCurrent(void);
	int getVersion(int* pVersion);
	unsigned char getState(void);
	unsigned char getAlarmCode(void);

	void pollingAlarm(void);

	// Utilities
	unsigned int verifyACCTime(void);
	unsigned int convACCTimeWithRPM(const unsigned int nACCTime, const float fRPMRate);
	unsigned int convDECTimeWithRPM(const unsigned int nDECTime, const float fRPMRate);

	bool isRunning(void) const { return _bIsRunning; }
	bool isAlarm(void) const { return _bIsAlarm; }

	unsigned int getRPMValue(void) const { return _u16SetRPMValue; }
	float getRPMRate(void) const { return _fSetRPMRate; }

	unsigned int getACCTime(void) const { return _u16ACCTime; }
	unsigned int getDECTime(void) const { return _u16DECTime; }


private:
	bool _bIsAlarm;
	bool _bIsRunning;
	bool _bIsPseudoRunning;

	unsigned int _u16SetRPMValue;
	float _fSetRPMRate;

	unsigned int _u16ACCTime; 			// ACC Time [msec] @ up to 1000 [rpm]
	unsigned int _u16DECTime;			// DEC Time [msec] @ up to 1000 [rpm]

#ifdef _USE_MOTOR_PUMP_ALARM_PROC_MODE_
	void (*_postAlarm)(void);
#endif

	// Motor Controller Interfaces
	int read_OutputStatus(const unsigned int nStartAddr = OZBV_IO_OUTPUT_REG_BASE_ADDR);											// FUNC 01
	bool write_SignleOutputStatus(const unsigned int nStartAddr, const unsigned char nOutputStatus);								// FUNC 05
	int read_InputStatus(const unsigned int nStartAddr = OZBV_IO_INPUT_REG_BASE_ADDR);												// FUNC 02
	int read_PresentStatus(const unsigned int nStartAddr = OZBV_PRESENT_ALARM_REG_ADDR);											// FUNC 04
	int read_PresentElec(const unsigned int nStartAddr, int* pElecValBuf);															// FUNC 04
	int read_ControlParams(const unsigned int nStartAddr);																			// FUNC 03
	bool write_ControlParams(const unsigned int nStartAddr, const int nSetParams);													// FUNC 06
	bool write_CommandSet(const unsigned int nStartAddr = OZBV_COMMAND_REG_ADDR, const unsigned char nCommand = COMMAND_RUN);		// FUNC 06
	bool write_SpeedSet(const unsigned int nStartAddr = OZBV_SPEED_SETPOINT_REG_ADDR, const int nSpeedRPM = _DEFAULT_RPM_VALUE_);	// FUNC 06
	int read_ControllerVersion(int* pVersionBuf);																					// FUNC 03
};

#endif //_CMOTORPUMP_OZBV10A_H_

