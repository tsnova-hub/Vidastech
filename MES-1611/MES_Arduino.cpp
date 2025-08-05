

#include "MES_Arduino.h"


////////////////////////////////////////////////////////////
// CONSOLE (Serial)

namespace std
{
	ohserialstream cout(_console);
}

void config_FloatPrecisionConsole(void)
{
	cout.setf(ios::fixed, ios_base::floatfield);			// floating point fixed.
	cout.setf(ios::showpoint); 								// zero-padding
	cout.precision(FLOAT_POINT_PRECISION_FOR_DATA_ACQ);		// ex) 3 : 0.123, 6 : 0.123456

#if defined(_USE_APPLICATION_DEBUG_MSG_)
	// cout << "Float " << scientific << __FLT_MAX__ << endl; 
	// cout << "Float " << scientific << __FLT_MIN__ << endl;
	// cout << F("\n[LOG] Float Precision ::") << FLOAT_POINT_PRECISION_FOR_DATA_ACQ << F("\n\r") << endl;
#endif
}

void wait_RebootSystemWithConsoleMsg(void)
{
	_console.print(F("[NOTICE] Reboot.\n\r"));

	while (true);
}

#ifdef __REFERENCE_CODE_SNIPPET__
void reference_code_snippet(void)
{
	////
	cout.setf(ios::dec, ios_base::basefield);
	cout.setf(ios::showbase);
	cout << ">> It's running on (1): " << millis() << endl;
	cout.unsetf(ios::showbase);
	////
	float fmax = __FLT_MAX__, fmin = __FLT_MIN__;
	cout.precision(7);
	// cout.setf(ios::fixed, ios_base::floatfield);
	cout << "Float " << scientific << fmax << endl; 
	cout << "Float " << scientific << fmin << endl;

	cout << "OK" << endl;
	////
	cout.setf(ios::fixed, ios_base::floatfield);
	cout << "charater string " << float(vNumber.front()) << " " << double(vNumber.back()) << endl;

	cout.setf(ios::hex, ios_base::basefield);
	cout.setf(ios::showbase);
	cout << hex << vNumber.at(5) << endl;
	cout << hex << 10 << endl;
	cout.unsetf(ios::showbase);

	cout << "charater string " << float(vNumber.front()) << " " << double(vNumber.back()) << endl;
	////
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
//			ARDUINO EEPROM Utilities

void clear_ContentsEEPROM(void)
{
	for (int nAddress = 0x0000; nAddress < EEPROM.length(); nAddress++) {

		EEPROM.update(nAddress, 0x00);

	}
}

void write_Byte8ParameterEEPROM(const unsigned int nStartAddr, const unsigned char nByte8Param)
{
	EEPROM.update(nStartAddr, nByte8Param);
}

void write_Word16ParameterEEPROM(const unsigned int nStartAddr, const unsigned int nWord16Param)
{
	EEPROM.update(nStartAddr,	   _low_byte(nWord16Param));
	EEPROM.update(nStartAddr+0x01, _high_byte(nWord16Param));
}

void write_Word32ParameterEEPROM(const unsigned int nStartAddr, const unsigned long nWord32Param)
{
	unsigned int nLowWord = _low_word(nWord32Param);
	unsigned int nHighWord = _high_word(nWord32Param);

	EEPROM.update(nStartAddr,	   _low_byte(nLowWord));
	EEPROM.update(nStartAddr+0x01, _high_byte(nLowWord));
	EEPROM.update(nStartAddr+0x02, _low_byte(nHighWord));
	EEPROM.update(nStartAddr+0x03, _high_byte(nHighWord));
}

unsigned char read_Byte8ParameterEEPROM(const unsigned int nStartAddr)
{
	unsigned char nByte = (unsigned char)EEPROM.read(nStartAddr);

	unsigned char nByte8Param = (unsigned char)constrain(nByte, 0, 0xFF);

	return nByte8Param;//(unsigned char)EEPROM.read(nStartAddr);
}

unsigned int read_Word16ParameterEEPROM(const unsigned int nStartAddr)
{
	unsigned char nByte[2];
	unsigned int nWord16Param;

	nByte[0] = (unsigned char)EEPROM.read(nStartAddr);
	nByte[1] = (unsigned char)EEPROM.read(nStartAddr+0x01);

	nWord16Param = (unsigned int)constrain(_word16(nByte[1], nByte[0]), 0, 0xFFFF);

	return nWord16Param;
}

unsigned long read_Word32ParameterEEPROM(const unsigned int nStartAddr)
{
	unsigned char nByte[4];
	unsigned int nLowWord, nHighWord;
	unsigned long nWord32Param;

	nByte[0] = (unsigned char)EEPROM.read(nStartAddr);
	nByte[1] = (unsigned char)EEPROM.read(nStartAddr+0x01);
	nByte[2] = (unsigned char)EEPROM.read(nStartAddr+0x02);
	nByte[3] = (unsigned char)EEPROM.read(nStartAddr+0x03);

	nLowWord = _word16(nByte[1], nByte[0]);
	nHighWord = _word16(nByte[3], nByte[2]);

	nWord32Param = (unsigned long)constrain(_long32(nHighWord, nLowWord), 0, 0xFFFFFFFF);

	return nWord32Param;
}


//////////////////////////////////////////////////////////////////////////////////////////
//			OPERATIONS OF TIMERS AND EXTERNAL INTERRUPTS

void start_TimerThree(const unsigned long ulPeriod)
{
	Timer3.initialize(ulPeriod*1000UL);
}

void stop_TimerThree(void)
{
	Timer3.stop();
}

void attach_Timer3Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/)
{
	Timer3.initialize(period*1000UL);						// in microseconds
	Timer3.attachInterrupt(callback/*, period*1000*/);		// in microseconds
}

void detach_Timer3Interrupt(void)
{
	Timer3.stop();
	Timer3.detachInterrupt();
}

// void attach_Timer4Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/)
// {
// 	Timer4.initialize(period*1000UL);						// in microseconds
// 	Timer4.attachInterrupt(callback/*, period*1000*/);		// in microseconds
// }

// void detach_Timer4Interrupt(void)
// {
// 	Timer4.stop();
// 	Timer4.detachInterrupt();
// }

// void attach_Timer5Interrupt(void (*callback)(void), unsigned long period /*in milliseconds*/)
// {
// 	Timer5.initialize(period*1000UL);						// in microseconds
// 	Timer5.attachInterrupt(callback/*, period*1000*/);		// in microseconds
// }

// void detach_Timer5Interrupt(void)
// {
// 	Timer5.stop();
// 	Timer5.detachInterrupt();
// }

void enable_ExternalInterrupt(void (*callback)(void), int nInterrupt, int nTriggerMode)
{
	attachInterrupt(nInterrupt, callback, nTriggerMode);
}

void disable_ExternalInterrupt(int nInterrupt)
{
	detachInterrupt(nInterrupt);
}

