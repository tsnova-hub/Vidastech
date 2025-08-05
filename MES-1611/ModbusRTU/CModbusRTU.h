#ifndef _CMODBUSRTU_H_
#define _CMODBUSRTU_H_

#if defined(ARDUINO) && ARDUINO >= 100

#include <Arduino.h>

#else

#include <WProgram.h>

#endif


#include <StandardCplusplus.h>
#include <serstream>


#include <ModbusMaster.h>
#include <util/word.h>


#include "../MES_Config.h"


using namespace std;


#ifndef _FlashString
	#define _FlashString 							__FlashStringHelper	// char
#endif


#if defined(_USE_APPLICATION_DEBUG_MSG_)
	#define _USE_CMODBUS_RTU_DEBUG_MSG_
	#if defined(_USE_CMODBUS_RTU_DEBUG_MSG_)
		#define _USE_CMODBUS_RTU_EXCEPTION_HANDLER_
		#define _USE_CMODBUS_RTU_RESPONSE_HANDLER_
	#endif
#endif


//////////////////////////////////////////////////////////////////////////////////////////
// MODBUS RTU LIBRARY RELATED CONSTANTS DEFINITION

// Protocol-defined exception codes
#define MODBUS_RTU_ILLEGAL_FUNCTION					0x01				// Modbus protocol illegal function exception.
#define MODBUS_RTU_ILLEGAL_DATA_ADDR				0x02				// Modbus protocol illegal data address exception.
#define MODBUS_RTU_ILLEGAL_DATA_VALUE				0x03 				// Modbus protocol illegal data value exception.
#define MODBUS_RTU_SLAVE_DEVICE_FAILURE				0x04				// Modbus protocol slave device failure exception.
// Class-defined success/exception codes
#define MODBUS_RTU_SUCCESS							0x00	    		// ModbusMaster success.
#define MODBUS_RTU_INVALID_SLAVE_ID					0xE0				// ModbusMaster invalid response slave ID exception.
#define MODBUS_RTU_INVALID_FUNCTION					0xE1				// ModbusMaster invalid response function exception.
#define MODBUS_RTU_RESPONSE_TIMEOUT					0xE2				// ModbusMaster response timed out exception.
#define MODBUS_RTU_INVALID_CRC_CODE					0xE3				// ModbusMaster invalid response CRC exception.  
// Custom-defined exception codes
#define MODBUS_RTU_RETRY							0xFF
#define MODBUS_RTU_FAILED							0xFA

#define _MODBUS_WAIT_LATENCY_TIME_					/*delay(1)*/


// MOTOR CONTROLLER : INVALID VALUE (RECEIVED VALUE)
#define MODBUS_RXBUF_RESET							0xFFFF


class CModbusRTU : public ModbusMaster
{
public:
	CModbusRTU();

	void set_ModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr = 0x00);
	void set_PreTransmission(void (*preTransmission)(void));
	void set_PostTransmission(void (*postTransmission)(void));

	// Modbus RTU Protocols
	unsigned char read_Coils(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf);				// MODBUS RTU FUNC 01
	unsigned char read_DiscreteInputs(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf);		// MODBUS RTU FUNC 02
	unsigned char read_HoldingRegisters(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf);	// MODBUS RTU FUNC 03
	unsigned char read_InputRegisters(unsigned int nStartAddr, unsigned char cReadQuantity, unsigned int* pReceivedBuf);	// MODBUS RTU FUNC 04
	unsigned char write_SingleCoil(unsigned int nStartAddr, unsigned char nTransmitStatus);									// MODBUS RTU FUNC 05
	unsigned char write_SingleRegister(unsigned int nStartAddr, unsigned int nTransmitData);								// MODBUS RTU FUNC 06
	
#ifdef _USE_CMODBUS_RTU_DEBUG_MSG_
	void print_ExceptionMessage(const _FlashString* pstrErrorMsg);
#endif
#ifdef _USE_CMODBUS_RTU_EXCEPTION_HANDLER_
	unsigned char handle_ExceptionModbusRTU(const unsigned char cExceptionCode);
#endif
#ifdef _USE_CMODBUS_RTU_RESPONSE_HANDLER_
	unsigned int handle_InvalidResponse(const unsigned int nInvalidResponse);
#endif
};

#endif //_CMODBUSRTU_H_

