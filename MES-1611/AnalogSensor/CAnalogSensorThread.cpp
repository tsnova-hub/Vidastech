

#include "CAnalogSensorThread.h"


CAnalogSensorThread::CAnalogSensorThread(): Thread()
{
	this->enabled = false;

	_bRunned = false;

	_vSensorList.clear();

#if defined(_AS_EVENT_DRIVEN_ACCESS_)
	_bEventTriggered = false;
#endif
}

void CAnalogSensorThread::begin(const long lInterval)
{
	this->setInterval(lInterval);
	this->enabled = true;
}

void CAnalogSensorThread::stop(void)
{
	this->enabled = false;
}

bool CAnalogSensorThread::add(CAnalogSensor* phSensor)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			if (_vSensorList[index] == phSensor)

				return true;

		}

	}

	if (_vSensorList.size() < MAX_ANALOG_SENSORS) {

		_vSensorList.push_back(phSensor);

		return true;

	}

	return false;
}

void CAnalogSensorThread::remove(const unsigned char u8Index)
{
	if (u8Index < _vSensorList.size()) {

		_vSensorList.erase(_vSensorList.begin() + u8Index);

	}
}

void CAnalogSensorThread::remove(const CAnalogSensor* phSensor)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			if (_vSensorList[index] == phSensor) {

				_vSensorList.erase(_vSensorList.begin() + index);

			}

		}

	}
}

CAnalogSensor* CAnalogSensorThread::getSensorObject(const unsigned char u8Index) const
{
	if (!_vSensorList.empty()) {

		if (u8Index < _vSensorList.size()) {

			return _vSensorList[u8Index];

		}

	}

	return NULL;
}

void CAnalogSensorThread::measure_AnalogSensorList(void)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			_vSensorList[index]->measureAnalogSensor();

		}

#if defined(_AS_EVENT_DRIVEN_ACCESS_)
		// if (!_bEventTriggered) {

			_bEventTriggered = true;

		// }
#endif

	}
}

bool CAnalogSensorThread::isTriggered(void)
{
#if defined(_AS_EVENT_DRIVEN_ACCESS_)
	if (_bEventTriggered) {

		noInterrupts();

		_bEventTriggered = false;

		interrupts();

		return true;

	}

	return false;
#else
	return true;
#endif
}

void CAnalogSensorThread::run(void)
{
#if defined(_AS_THREAD_DEBUG_MSG_)
	cout << F(">> It's running on (") << this->ThreadID << F("): ") << millis() << endl;
#endif

	measure_AnalogSensorList();

	_bRunned = false;

	Thread::run();
}

bool CAnalogSensorThread::shouldRun(void)
{
	return (_bRunned = Thread::shouldRun(millis()));
}

// if 'Thread' use within Timer Interrupts, 'shouldRun' Method must override like below code
bool CAnalogSensorThread::shouldRun(long time)
{
	return (_bRunned == false ? _bRunned = Thread::shouldRun(time) : false);
}

