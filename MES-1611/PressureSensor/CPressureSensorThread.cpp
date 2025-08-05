

#include "CPressureSensorThread.h"


CPressureSensorThread::CPressureSensorThread(): Thread()
{
	this->enabled = false;

	_bRunned = false;

	_vSensorList.clear();

#if defined(_PS_EVENT_DRIVEN_ACCESS_)
	_bEventTriggered = false;
#endif
}

void CPressureSensorThread::begin(const long lInterval)
{
	this->setInterval(lInterval);
	this->enabled = true;
}

void CPressureSensorThread::stop(void)
{
	this->enabled = false;
}

bool CPressureSensorThread::add(CPressureSensor* phSensor)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			if (_vSensorList[index] == phSensor)

				return true;

		}

	}

	if (_vSensorList.size() < MAX_PRESSURE_SENSORS) {

		_vSensorList.push_back(phSensor);

		return true;

	}

	return false;
}

void CPressureSensorThread::remove(const unsigned char u8Index)
{
	if (u8Index < _vSensorList.size()) {

		_vSensorList.erase(_vSensorList.begin() + u8Index);

	}
}

void CPressureSensorThread::remove(const CPressureSensor* phSensor)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			if (_vSensorList[index] == phSensor) {

				_vSensorList.erase(_vSensorList.begin() + index);

			}

		}

	}
}

CPressureSensor* CPressureSensorThread::getSensorObject(const unsigned char u8Index) const
{
	if (!_vSensorList.empty()) {

		if (u8Index < _vSensorList.size()) {

			return _vSensorList[u8Index];

		}

	}

	return NULL;
}

void CPressureSensorThread::measure_PressureSensorList(void)
{
	if (!_vSensorList.empty()) {

		for (unsigned index = 0; index < _vSensorList.size(); index++) {

			_vSensorList[index]->measurePressure();

		}

#if defined(_PS_EVENT_DRIVEN_ACCESS_)
		// if (!_bEventTriggered) {

			_bEventTriggered = true;

		// }
#endif

	}
}

bool CPressureSensorThread::isTriggered(void)
{
#if defined(_PS_EVENT_DRIVEN_ACCESS_)
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

void CPressureSensorThread::run(void)
{
#if defined(_PS_THREAD_DEBUG_MSG_)
	cout << F(">> It's running on (") << this->ThreadID << F("): ") << millis() << endl;
#endif

	measure_PressureSensorList();

	_bRunned = false;

	Thread::run();
}

bool CPressureSensorThread::shouldRun(void)
{
	return (_bRunned = Thread::shouldRun(millis()));
}

// if 'Thread' use within Timer Interrupts, 'shouldRun' Method must override like below code
bool CPressureSensorThread::shouldRun(long time)
{
	return (_bRunned == false ? _bRunned = Thread::shouldRun(time) : false);
}

