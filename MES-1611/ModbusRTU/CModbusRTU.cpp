

#include "CModbusRTU.h"


CModbusRTU::CModbusRTU(): ModbusMaster()
{
	
}

void CModbusRTU::set_ModbusNode(Stream& rSerial, const unsigned char nMBSlaveAddr)
{
	this->begin(nMBSlaveAddr, rSerial);
}

void CModbusRTU::set_PreTransmission(void (*preTransmission)(void))
{
	if (preTransmission != NULL) {

		this->preTransmission(preTransmission);

	}
}

void CModbusRTU::set_PostTransmission(void (*postTransmission)(void))
{
	if (postTransmission != NULL) {

		this->postTransmission(postTransmission);

	}
}


// MODBUS RTU FUNC 01
unsigned char CModbusRTU::read_Coils(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf)
{
	unsigned char cResult;

	int nBufSize = int(nReadQuantity / (sizeof(nReadQuantity) * 8) + 1);

	if (!pReceivedBuf) {
		return MODBUS_RXBUF_RESET;
	}

	cResult = this->readCoils(nStartAddr, nReadQuantity);

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif

	for (int nBufIdx=0; nBufIdx<nBufSize; nBufIdx++)
	{
		pReceivedBuf[nBufIdx] = this->getResponseBuffer(nBufIdx);
	}

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;
}

// MODBUS RTU FUNC 02
unsigned char CModbusRTU::read_DiscreteInputs(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf)
{
	unsigned char cResult;

	int nBufSize = int(nReadQuantity / (sizeof(nReadQuantity) * 8) + 1);

	if (!pReceivedBuf) {
		return MODBUS_RXBUF_RESET;
	}

	cResult = this->readDiscreteInputs(nStartAddr, nReadQuantity);

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif
	
	for (int nBufIdx=0; nBufIdx<nBufSize; nBufIdx++)
	{
		pReceivedBuf[nBufIdx] = this->getResponseBuffer(nBufIdx);
	}

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;
}

// MODBUS RTU FUNC 03
unsigned char CModbusRTU::read_HoldingRegisters(unsigned int nStartAddr, unsigned int nReadQuantity, unsigned int* pReceivedBuf)
{
	unsigned char cResult;

	if (!pReceivedBuf) {
		return MODBUS_RXBUF_RESET;
	}

	cResult = this->readHoldingRegisters(nStartAddr, nReadQuantity);

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif

	for (int nBufIdx=0; nBufIdx<nReadQuantity; nBufIdx++)
	{
		pReceivedBuf[nBufIdx] = this->getResponseBuffer(nBufIdx);
	}

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;	
}

// MODBUS RTU FUNC 04
unsigned char CModbusRTU::read_InputRegisters(unsigned int nStartAddr, unsigned char cReadQuantity, unsigned int* pReceivedBuf)
{
	unsigned char cResult;

	if (!pReceivedBuf) {
		return MODBUS_RXBUF_RESET;
	}

	cResult = this->readInputRegisters(nStartAddr, cReadQuantity);

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif

	for (int nBufIdx=0; nBufIdx<cReadQuantity; nBufIdx++)
	{
		pReceivedBuf[nBufIdx] = this->getResponseBuffer(nBufIdx);
	}

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;
}

// MODBUS RTU FUNC 05
unsigned char CModbusRTU::write_SingleCoil(unsigned int nStartAddr, unsigned char nTransmitStatus)
{
	unsigned char cResult;

	cResult = this->writeSingleCoil(nStartAddr, nTransmitStatus);

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;
}

// MODBUS RTU FUNC 06
unsigned char CModbusRTU::write_SingleRegister(unsigned int nStartAddr, unsigned int nTransmitData)
{
	unsigned char cResult;

	cResult = this->writeSingleRegister(nStartAddr, lowWord(nTransmitData));

	#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)	
	if (cResult != MODBUS_RTU_SUCCESS)
	{
		handle_ExceptionModbusRTU(cResult);
	}
	#endif

	_MODBUS_WAIT_LATENCY_TIME_;

	return cResult;
}

// MODBUS RTU EXCEPTION HANDLER
#if defined(_USE_CMODBUS_RTU_DEBUG_MSG_)
void CModbusRTU::print_ExceptionMessage(const _FlashString* pstrErrorMsg)
{
	cout << F("[MODBUS.ERR] ") << pstrErrorMsg << endl;
}
#endif

#if defined(_USE_CMODBUS_RTU_EXCEPTION_HANDLER_)
unsigned char CModbusRTU::handle_ExceptionModbusRTU(const unsigned char cExceptionCode)
{
	switch (cExceptionCode)
	{
		case MODBUS_RTU_INVALID_SLAVE_ID :
			print_ExceptionMessage(F("ModbusMaster invalid response slave ID exception."));
		break;
		case MODBUS_RTU_INVALID_FUNCTION :
			print_ExceptionMessage(F("ModbusMaster invalid response function exception."));
		break;
		case MODBUS_RTU_RESPONSE_TIMEOUT :
			print_ExceptionMessage(F("ModbusMaster response timed out exception."));
		break;
		case MODBUS_RTU_INVALID_CRC_CODE :
			print_ExceptionMessage(F("ModbusMaster invalid response CRC exception."));
		break;
	}

	return MODBUS_RTU_FAILED;
}
#endif

#if defined(_USE_CMODBUS_RTU_RESPONSE_HANDLER_)
unsigned int CModbusRTU::handle_InvalidResponse(const unsigned int nInvalidResponse)
{
	switch (nInvalidResponse)
	{
		case MODBUS_RTU_ILLEGAL_FUNCTION :
			print_ExceptionMessage(F("Modbus protocol illegal function."));
		break;
		case MODBUS_RTU_ILLEGAL_DATA_ADDR :
			print_ExceptionMessage(F("Modbus protocol illegal data address."));
		break;
		case MODBUS_RTU_ILLEGAL_DATA_VALUE :
			print_ExceptionMessage(F("Modbus protocol illegal data value."));
		break;
		case MODBUS_RTU_SLAVE_DEVICE_FAILURE :
			print_ExceptionMessage(F("Modbus protocol slave device failure."));
		break;
		case MODBUS_RXBUF_RESET :
		default:
			print_ExceptionMessage(F("Modbus response invalid."));
		break;
	}

	return nInvalidResponse;
}
#endif

