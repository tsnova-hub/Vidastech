#ifndef _MES_ARDUINO_H_
#define _MES_ARDUINO_H_


#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>
// #include <string>
// #include <vector>
// #include <queue>
// #include <stack>
// #include <iterator>
// #include <algorithm>
// #include <iomanip>


#include <avr/pgmspace.h>

#include <Controllino.h>

#include <SPI.h>
// #include <Wire.h>
// #include <Ethernet.h>
// #include <WiFi.h>
#include <EEPROM.h>
// #include <LiquidTWI.h>

#include <TimerThree.h>
// #include <TimerFour.h>
// #include <TimerFive.h>

#include <StopWatch.h>

#include <Thread.h>
#include <ThreadController.h>


#include "MES_Config.h"
#include "MES_Hardware_Map.h"


using namespace std;


////////////////////////////////////////////////////////////
// CONSOLE (Serial)

void config_FloatPrecisionConsole(void);

void wait_RebootSystemWithConsoleMsg(void);


////////////////////////////////////////////////////////////
// WAIT FOR POWER ON RESET

#define SYSTEM_WAIT_FOR_PWR_ON_RESET 				(50UL)						// 50 msec


////////////////////////////////////////////////////////////
// RS485 Utilities

#if defined(CONTROLLINO_MINI) || defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
	#define _init_RS485								Controllino_RS485Init
	#define _USE_PORT_ACCESS_INTRINSIC_
	#if !defined(_USE_PORT_ACCESS_INTRINSIC_)
	#define _set_RS485RE(_val)						Controllino_SwitchRS485RE(_val)
	#define _set_RS485DE(_val)						Controllino_SwitchRS485DE(_val)
	#endif //_USE_PORT_ACCESS_INTRINSIC_
#else
	#define _init_RS485											
	#define _set_RS485RE(_val)						(_val)
	#define _set_RS485DE(_val)						(_val)
#endif


////////////////////////////////////////////////////////////
// EEPROM Utilities

#define _low_byte(_int16)							((unsigned char) ((_int16) & 0xFF))
#define _high_byte(_int16)							((unsigned char) ((_int16) >> 8) & 0xFF)
#define _word16(_high8, _low8)						((unsigned int)  (((unsigned int)(_high8) << 8) | ((unsigned int)(_low8) & 0xFF)))

#define _low_word(_long32)							((unsigned int)  ((_long32) & 0xFFFF))
#define _high_word(_long32)							((unsigned int)  ((_long32) >> 16) & 0xFFFF)
#define _long32(_high16, _low16)					((unsigned long) (((unsigned long)(_high16) << 16) | ((unsigned long)(_low16) & 0xFFFF)))

#if defined(_USE_PSEUDO_CONVERT_FUNCTION_)
	// cause of fault to convert float to integer
	#define _convert_float_to_decimal(_float, _factor)	(roundf(_float * _factor))
#else
	#define _convert_float_to_decimal(_float, _factor)	(_float * _factor)
#endif

void clear_ContentsEEPROM(void);
void write_Byte8ParameterEEPROM(const unsigned int nStartAddr, const unsigned char nByte8Param);
void write_Word16ParameterEEPROM(const unsigned int nStartAddr, const unsigned int nWord16Param);
void write_Word32ParameterEEPROM(const unsigned int nStartAddr, const unsigned long nWord32Param);
unsigned char read_Byte8ParameterEEPROM(const unsigned int nStartAddr);
unsigned int read_Word16ParameterEEPROM(const unsigned int nStartAddr);
unsigned long read_Word32ParameterEEPROM(const unsigned int nStartAddr);


////////////////////////////////////////////////////////////
// Timer Utilities

void start_TimerThree(const unsigned long ulPeriod);
void stop_TimerThree(void);
void attach_Timer3Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/);
void detach_Timer3Interrupt(void);
// void attach_Timer4Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/);
// void detach_Timer4Interrupt(void);
// void attach_Timer5Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/);
// void detach_Timer5Interrupt(void);
void enable_ExternalInterrupt(void (*callback)(void), int nInterrupt, int nTriggerMode);
void disable_ExternalInterrupt(int nInterrupt);


#endif //_MES_ARDUINO_H_

