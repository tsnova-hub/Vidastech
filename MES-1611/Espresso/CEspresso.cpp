

#include "CEspresso.h"


CEspresso::CEspresso(): StopWatch(StopWatch::MILLIS)
{
	_BrewMode = CEspresso::BREW_MODE;
	_BrewBase = CEspresso::BREW_FLOWCNT;
	_BrewState = CEspresso::BREW_STOP;
}

void CEspresso::init(stDataSet_t* pstdataSet)
{
	_fInjectionPower = 0.0F;
	_fExtractionPower = 0.0F;

	_nProgramNum = 0;

	memset(_fInjectionSet, 0.0f, sizeof(float)*2);
	memset(_fExtractionSet, 0.0f, sizeof(float)*2);

	memset(_fInjectionSetQueue, 0.0f, sizeof(float)*2);
	memset(_fExtractionSetQueue, 0.0f, sizeof(float)*2);

	_bIsTimeover = false;
	_bIsFlushingDetected = false;
	_bIsBrewPrepared = false;
	_bIsStateChanged = false;
	_bManualStateTransition = false;

	if (pstdataSet != NULL) {

		_pstDataSet = pstdataSet;

	}

	_idleProcessing = NULL;
}

void CEspresso::setIdleProcessing(void (*callback)(void))
{
	if (callback != NULL) {

		_idleProcessing = callback;

	}
}

void CEspresso::clearProgramSet(void)
{
	clear_Condition();
	clear_ConditionQueue();
}

void CEspresso::clear_Condition(void)
{
	memset(_fInjectionSet, 0.0f, sizeof(float)*2);
	memset(_fExtractionSet, 0.0f, sizeof(float)*2);
}

void CEspresso::clear_ConditionQueue(void)
{
	memset(_fInjectionSetQueue, 0.0f, sizeof(float)*2);
	memset(_fExtractionSetQueue, 0.0f, sizeof(float)*2);
}

void CEspresso::load_Condition(void)
{
	stSetupDataSet_t* pstSetup = &(_pstDataSet->stSetup);

	if (_BrewBase == CEspresso::BREW_FLOWCNT) {

		_fInjectionSet[0] = pstSetup->InjectionFlow[_nProgramNum];
		_fExtractionSet[0] = pstSetup->ExtractionFlow[_nProgramNum] + pstSetup->InjectionFlow[_nProgramNum];

	}
	else {

		_fInjectionSet[0] = pstSetup->InjectionTime[_nProgramNum];
		_fExtractionSet[0] = pstSetup->ExtractionTime[_nProgramNum] + pstSetup->InjectionTime[_nProgramNum];

	}

	_fInjectionSet[1] = 0.0f;
	_fExtractionSet[1] = 0.0f;
}

void CEspresso::update_Condition(void)
{
	stSetupDataSet_t* pstSetup = &(_pstDataSet->stSetup);

	pstSetup->InjectionFlow[_nProgramNum] = _fInjectionSet[CEspresso::BREW_FLOWCNT];
	pstSetup->ExtractionFlow[_nProgramNum] = _fExtractionSet[CEspresso::BREW_FLOWCNT] - _fInjectionSet[CEspresso::BREW_FLOWCNT];

	pstSetup->InjectionTime[_nProgramNum] = _fInjectionSet[CEspresso::BREW_TIMECNT];
	pstSetup->ExtractionTime[_nProgramNum] = _fExtractionSet[CEspresso::BREW_TIMECNT] - _fInjectionSet[CEspresso::BREW_TIMECNT];
}

bool CEspresso::invalidateCondition(const unsigned int nInjectionFlow, const unsigned int nExtractionFlow, const float fInjectionTime, const float fExtractionTime)
{
	unsigned long ulTotFlowCount = nInjectionFlow + nExtractionFlow;
	float fTotBrewTime = fInjectionTime + fExtractionTime;

	if ((ulTotFlowCount >= MIN_BREWING_FLOWCNT && ulTotFlowCount <= MAX_BREWING_FLOWCNT) && (fTotBrewTime >= (MIN_BREWING_TIME/1000UL) && fTotBrewTime <= (MAX_BREWING_TIME/1000UL))) {

		if ((nInjectionFlow >= MIN_INJECTION_FLOWCNT && nExtractionFlow >= MIN_EXTRACTION_FLOWCNT) &&
			(nInjectionFlow <= MAX_INJECTION_FLOWCNT && nExtractionFlow <= MAX_EXTRACTION_FLOWCNT)) {

			if ((fInjectionTime >= (MIN_INJECTION_TIME/1000UL) && fExtractionTime >= (MIN_EXTRACTION_TIME/1000UL)) &&
				(fInjectionTime <= (MAX_INJECTION_TIME/1000UL) && fExtractionTime <= (MAX_EXTRACTION_TIME/1000UL))) {

				if (fTotBrewTime > MIN_EFFECTIVE_BREW_TIME_FOR_FLUSHING_DETECTION) {

					return true;

				}

			}

		}

	}

	return false;
}

void CEspresso::clear_Observation(void)
{
	stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);

	pstAcquisition->ShotTime = 0UL;
	pstAcquisition->FlowCount = 0UL;
	pstAcquisition->FlowQuantity = 0.0F;
	pstAcquisition->FlowRate = 0.0F;

	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
	pstAcquisition->PresentSpeed = 0L;
	pstAcquisition->PresentCurrent = 0L;
	#endif

	pstAcquisition->AvgFlowRate = calculate_AveragedFlowRate(true);

	#if BREW_PRESSURE_SENSOR_ATTACHED
	pstAcquisition->AvgBrewPress = calculate_AveragedBrewPressure(true);
	#endif

	#if PUMP_PRESSURE_SENSOR_ATTACHED
	pstAcquisition->AvgPumpPress = calculate_AveragedPumpPressure(true);
	#endif
}

void CEspresso::update_Observation(void)
{
	stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);

	pstAcquisition->ShotTime = StopWatch::elapsed();
	pstAcquisition->FlowCount = get_FlowmeterCNT();
	pstAcquisition->FlowQuantity = get_WaterFlowQuantity(CFlowmeter::UNIT_MILLIS);
	pstAcquisition->FlowRate = get_AveragedFlowmeterQTY(CFlowmeter::UNIT_LPS);

	pstAcquisition->AvgFlowRate = calculate_AveragedFlowRate(false);

	#if BREW_PRESSURE_SENSOR_ATTACHED
	pstAcquisition->BrewPress = get_BrewBoilerPressureValue();
	pstAcquisition->AvgBrewPress = calculate_AveragedBrewPressure(false);
	#endif

	#if PUMP_PRESSURE_SENSOR_ATTACHED
	pstAcquisition->PumpPress = get_PumpPressureValue();
	pstAcquisition->AvgPumpPress = calculate_AveragedPumpPressure(false);
	#endif
}

float CEspresso::calculate_AveragedFlowRate(const bool bReset)
{
	static unsigned long _nNum_of_samples = 0UL;
	static float _fSum_of_samples = 0.0F;
	static float _fAvgFlowRate = 0.0F;

	if (bReset) {

		_nNum_of_samples = 0UL;
		_fSum_of_samples = 0.0F;
		_fAvgFlowRate = 0.0F;

	}
	else {

		stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);

		_fSum_of_samples += pstAcquisition->FlowRate;

		_fAvgFlowRate = _fSum_of_samples / float(++_nNum_of_samples);

	}

	return _fAvgFlowRate;
}

float CEspresso::calculate_AveragedBrewPressure(const bool bReset)
{
	static unsigned long _nNum_of_samples = 0UL;
	static float _fSum_of_samples = 0.0F;
	static float _fAvgBrewPressure = 0.0F;

	stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);

	if (bReset) {

		_nNum_of_samples = 0UL;
		_fSum_of_samples = 0.0F;
		_fAvgBrewPressure = pstAcquisition->BrewPress;

	}
	else {

		_fSum_of_samples += pstAcquisition->BrewPress;

		_fAvgBrewPressure = _fSum_of_samples / float(++_nNum_of_samples);

	}

	return _fAvgBrewPressure;
}

#if PUMP_PRESSURE_SENSOR_ATTACHED
float CEspresso::calculate_AveragedPumpPressure(const bool bReset)
{
	static unsigned long _nNum_of_samples = 0UL;
	static float _fSum_of_samples = 0.0F;
	static float _fAvgPumpPressure = 0.0F;

	stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);

	if (bReset) {

		_nNum_of_samples = 0UL;
		_fSum_of_samples = 0.0F;
		_fAvgPumpPressure = pstAcquisition->PumpPress;

	}
	else {

		_fSum_of_samples += pstAcquisition->PumpPress;

		_fAvgPumpPressure = _fSum_of_samples / float(++_nNum_of_samples);

	}

	return _fAvgPumpPressure;
}
#endif

void CEspresso::stabilize_BrewPressure(void)
{
	unsigned long ulStabilizationTime = millis();

	while (millis() - ulStabilizationTime < TIME_FOR_PRESSURE_STABILIZATION) {

		if (_idleProcessing) {

			_idleProcessing();

		}

	}
}

void CEspresso::prepareToBrew(void)
{
	stSetupDataSet_t* pstSetup = &(_pstDataSet->stSetup);

	_nProgramNum = _pstDataSet->ProgramNum;

	_fInjectionPower = pstSetup->InjectionPower;
	_fExtractionPower = pstSetup->MotorPower;

	switch (_BrewMode)
	{
		case CEspresso::FREE_MODE:
		case CEspresso::SETUP_MODE:

			if (pstSetup->InjectionMode == INJECTION_MODE_ON) {

				_fInjectionPower = pstSetup->InjectionPower;
				_BrewState = CEspresso::BREW_INJECTION;

			}
			else {

				_fInjectionPower = 0.0f;
				_BrewState = CEspresso::BREW_EXTRACTION;

			}

			clear_Condition();

		break;

		case CEspresso::BREW_MODE:
		default:

			if (pstSetup->ProgramMode[_nProgramNum] == PROGRAM_MODE_FLOW) {

				_BrewBase = CEspresso::BREW_FLOWCNT;

			}
			else {

				_BrewBase = CEspresso::BREW_TIMECNT;

			}

			load_Condition();

			_BrewState = _fInjectionSet[0] ? CEspresso::BREW_INJECTION : CEspresso::BREW_EXTRACTION;

		break;
	}

	_bIsTimeover = false;
	_bIsFlushingDetected = false;
	_bIsBrewPrepared = true;
	_bManualStateTransition = false;

	#if defined(_USE_ESPRESSO_DEBUG_MSG_)
	print_PreparedBrewCondition(F("CEspresso::prepareToBrew"));
	#else
	#if defined(_USE_BREW_PROFILE_ACQUISITION_)
		print_PreparedBrewCondition();
	#endif
	#endif
}

void CEspresso::postBrew(void)
{
	// if (_pstDataSet->stSetup.FlushingDetectionMode == FLUSHING_DETECTION_MODE_ON) {

		if (_pstDataSet->stAcquisition.ShotTime <= MIN_EFFECTIVE_BREW_TIME_FOR_FLUSHING_DETECTION * 1000UL) {

			if (!_bIsFlushingDetected) {

				#if defined(_USE_ESPRESSO_DEBUG_MSG_)
				cout << F("CEspresso::FlushingDetect") << endl;
				#endif

				_bIsFlushingDetected = true;

			}

		}

	// }

	if (!_bIsTimeover && !_bIsFlushingDetected) {

		memcpy(_fInjectionSetQueue, _fInjectionSet, sizeof(float)*2);
		memcpy(_fExtractionSetQueue, _fExtractionSet, sizeof(float)*2);

	}

	if (_BrewMode == CEspresso::FREE_MODE) {

		stSetupDataSet_t* pstSetup = &(_pstDataSet->stSetup);

		pstSetup->InjectionFlow[_nProgramNum] = _fInjectionSet[CEspresso::BREW_FLOWCNT];
		pstSetup->ExtractionFlow[_nProgramNum] = _fExtractionSet[CEspresso::BREW_FLOWCNT] - _fInjectionSet[CEspresso::BREW_FLOWCNT];

		pstSetup->InjectionTime[_nProgramNum] = _fInjectionSet[CEspresso::BREW_TIMECNT];
		pstSetup->ExtractionTime[_nProgramNum] = _fExtractionSet[CEspresso::BREW_TIMECNT] - _fInjectionSet[CEspresso::BREW_TIMECNT];

	}

	#if defined(_USE_ESPRESSO_DEBUG_MSG_)
	print_PostBrewCondition(F("CEspresso::postBrew"));
	#else
		#if defined(_USE_BREW_PROFILE_ACQUISITION_)
		print_PostBrewCondition();
		#endif
	#endif
}

bool CEspresso::prepareToSave(void)
{
	if (_BrewMode != CEspresso::BREW_MODE) {

		if (_bIsTimeover || _bIsFlushingDetected) {

			memcpy(_fInjectionSet, _fInjectionSetQueue, sizeof(float)*2);
			memcpy(_fExtractionSet, _fExtractionSetQueue, sizeof(float)*2);

		}

		unsigned int nInjectionFlow = _fInjectionSet[CEspresso::BREW_FLOWCNT];
		unsigned int nExtractionFlow = _fExtractionSet[CEspresso::BREW_FLOWCNT] - _fInjectionSet[CEspresso::BREW_FLOWCNT];

		float fInjectionTime = _fInjectionSet[CEspresso::BREW_TIMECNT];
		float fExtractionTime = _fExtractionSet[CEspresso::BREW_TIMECNT] - _fInjectionSet[CEspresso::BREW_TIMECNT];

		if (invalidateCondition(nInjectionFlow, nExtractionFlow, fInjectionTime, fExtractionTime)) {

			_pstDataSet->ProgramNum = _nProgramNum;
		
			update_Condition();

			#if defined(_USE_ESPRESSO_DEBUG_MSG_)
			print_UpdatedBrewCondition(F("CEspresso::prepareToSave"));
			#endif

			return true;

		}

	}

	return false;
}

void CEspresso::startBrewing(void)
{
	prepareToBrew();

	start_Brewing();
}

void CEspresso::stopBrewing(void)
{
	_BrewState = CEspresso::BREW_STOP;

	stop_Brewing();

	postBrew();
}

void CEspresso::start_Brewing(void)
{
	#if defined(_USE_ESPRESSO_DEBUG_MSG_)
	cout << F("\n\rCEspresso::start_Brewing") << endl;
	cout << (_BrewState == CEspresso::BREW_INJECTION ? F("CEspresso::Injection") : F("CEspresso::Extraction")) << endl;
	print_BrewingInitObservation(true);
	#else
		#if defined(_USE_BREW_PROFILE_ACQUISITION_)
			print_BrewingInitObservation(true);
		#endif
	#endif

	if (!hBrewValve.isOpening()) {

		clear_Observation();

		start_Flowmeter();

		StopWatch::reset();
		StopWatch::start();

		hBrewValve.open();

		hMotorPump.start((_BrewState == CEspresso::BREW_INJECTION ? _fInjectionPower : _fExtractionPower));

	}

	#if defined(_USE_ESPRESSO_DEBUG_MSG_) || defined(_USE_BREW_PROFILE_ACQUISITION_)
	print_BrewingInitObservation(false);
	#endif
}

void CEspresso::stop_Brewing(void)
{
	#if defined(_USE_ESPRESSO_DEBUG_MSG_)
	cout << F("CEspresso::stop_Brewing") << endl;
	#endif

	if (hBrewValve.isOpening()) {

		stop_Flowmeter();

		hBrewValve.close();

		StopWatch::stop();

		update_Observation();

		if (hMotorPump.isRunning()) {

			#if BREW_BOILER_PRESSURE_STABILIZATION
			stabilize_BrewPressure();
			#endif

			hMotorPump.stop();

		}

	}
}

enum CEspresso::BrewState CEspresso::onBrewing(void)
{
	if (!hBrewValve.isOpening() || !hMotorPump.isRunning()) {

		stop_Brewing();

		postBrew();

		return (_BrewState = CEspresso::BREW_STOP);

	}

	stAcqDataSet_t* pstAcquisition = &(_pstDataSet->stAcquisition);
	stSetupDataSet_t* pstSetup = &(_pstDataSet->stSetup);

	_bIsStateChanged = false;

	update_Observation();

	switch (_BrewMode)
	{
		case CEspresso::BREW_MODE:
		{
			float fBrewCondition = _BrewBase == BREW_FLOWCNT ? float(pstAcquisition->FlowCount) : float(pstAcquisition->ShotTime / 1000.0f);

			switch (_BrewState)
			{
				case CEspresso::BREW_INJECTION:

					if (fBrewCondition >= _fInjectionSet[0]) {

						#if defined(_USE_ESPRESSO_DEBUG_MSG_)
						cout << F("CEspresso::Extraction") << endl;
						#endif

						#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
						if (hMotorPump.getDutyRate() != _fExtractionPower) {

							hMotorPump.setDuty(_fExtractionPower);

						}
						#else
						if (hMotorPump.getRPMRate() != _fExtractionPower) {

							hMotorPump.setRPM(_fExtractionPower);

						}
						#endif

						_bIsStateChanged = true;

						_BrewState = CEspresso::BREW_EXTRACTION;

					}

				break;

				case CEspresso::BREW_EXTRACTION:

					if (fBrewCondition >= _fExtractionSet[0]) {

						stop_Brewing();

						_BrewState = CEspresso::BREW_STOP;

					}

				break;
			}
		}
		break;

		case CEspresso::FREE_MODE:
		case CEspresso::SETUP_MODE:

			if (_bManualStateTransition == true) {

				switch (_BrewState)
				{
					case CEspresso::BREW_EXTRACTION:

						#if defined(_USE_ESPRESSO_DEBUG_MSG_)
						cout << F("CEspresso::Extraction") << endl;
						#endif

						#if !defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
						if (hMotorPump.getDutyRate() != _fExtractionPower) {

							hMotorPump.setDuty(_fExtractionPower);

						}
						#else
						if (hMotorPump.getRPMRate() != _fExtractionPower) {

							hMotorPump.setRPM(_fExtractionPower);

						}
						#endif

						_bIsStateChanged = true;

						_fInjectionSet[CEspresso::BREW_FLOWCNT] = pstAcquisition->FlowCount;
						_fInjectionSet[CEspresso::BREW_TIMECNT] = int((pstAcquisition->ShotTime / 1000.0f) * 10ul) / 10.0f;

					break;

					case CEspresso::BREW_STOP:

						stop_Brewing();

						_fExtractionSet[CEspresso::BREW_FLOWCNT] = pstAcquisition->FlowCount;
						_fExtractionSet[CEspresso::BREW_TIMECNT] = int((pstAcquisition->ShotTime / 1000.0f) * 10ul) / 10.0f;

					break;
				}

				_bManualStateTransition = false;

			}

		break;
	}

	if (_BrewState == CEspresso::BREW_STOP) {

		postBrew();

		return _BrewState;

	}
	else {

		#if defined(_USE_ESPRESSO_DEBUG_MSG_) || defined(_USE_BREW_PROFILE_ACQUISITION_)
		print_BrewingObservation();
		#endif

		if (pstAcquisition->ShotTime >= MAX_BREWING_TIME) {

			#if defined(_USE_ESPRESSO_DEBUG_MSG_)
			cout << F("CEspresso::Timeover") << endl;
			#endif

			_bIsTimeover = true;

			stop_Brewing();

			if (_BrewMode == CEspresso::FREE_MODE || _BrewMode == CEspresso::SETUP_MODE) {

				if (_BrewState == CEspresso::BREW_INJECTION) {

					_fInjectionSet[CEspresso::BREW_FLOWCNT] = pstAcquisition->FlowCount;
					_fInjectionSet[CEspresso::BREW_TIMECNT] = int((pstAcquisition->ShotTime / 1000.0f) * 10ul) / 10.0f;

				}
				else {

					_fExtractionSet[CEspresso::BREW_FLOWCNT] = pstAcquisition->FlowCount;
					_fExtractionSet[CEspresso::BREW_TIMECNT] = int((pstAcquisition->ShotTime / 1000.0f) * 10ul) / 10.0f;

				}

			}

			postBrew();

			return (_BrewState = CEspresso::BREW_STOP);

		}
		else {

#if AUTOMATION_SENSOR_ATTACHED
			unsigned long ulFlushingDetectionStartTime = MIN_EFFECTIVE_BREW_TIME_FOR_FLUSHING_DETECTION * 1000UL;
			unsigned long ulFlushingDetectionEndTime = ulFlushingDetectionStartTime + OBSERVATION_DURATION_FOR_FLUSHING_DETECTION;

			if (pstAcquisition->ShotTime >= ulFlushingDetectionStartTime && pstAcquisition->ShotTime < ulFlushingDetectionEndTime) {

				if (pstAcquisition->BrewPress < MIN_EFFECTIVE_BREW_PRES_FOR_FLUSHING_DETECTION) {

					if (!_bIsFlushingDetected) {

						#if defined(_USE_ESPRESSO_DEBUG_MSG_)
						cout << F("CEspresso::FlushingDetect") << endl;
						#endif

						_bIsFlushingDetected = true;

					}

				}

			}
#else
			if (pstSetup->FlushingDetectionMode == FLUSHING_DETECTION_MODE_ON) {

				unsigned long ulFlushingDetectionStartTime = pstSetup->FlushingDetectionTime * 1000UL;
				unsigned long ulFlushingDetectionEndTime = ulFlushingDetectionStartTime + OBSERVATION_DURATION_FOR_FLUSHING_DETECTION;

				if (pstAcquisition->ShotTime >= ulFlushingDetectionStartTime && pstAcquisition->ShotTime < ulFlushingDetectionEndTime) {

					if (pstAcquisition->BrewPress < pstSetup->FlushingDetectionPress) {

						if (!_bIsFlushingDetected) {

							#if defined(_USE_ESPRESSO_DEBUG_MSG_)
							cout << F("CEspresso::FlushingDetect") << endl;
							#endif

							_bIsFlushingDetected = true;

							if (_BrewMode != CEspresso::FREE_MODE && _BrewMode != CEspresso::SETUP_MODE) {

								stop_Brewing();

								postBrew();

								return (_BrewState = CEspresso::BREW_STOP);

							}

						}

					}

				}

			}
#endif

		}

		return _BrewState;

	}
}

#if defined(_USE_ESPRESSO_DEBUG_MSG_)
void CEspresso::print_PreparedBrewCondition(const _FlashString* pstrHeader)
{
	cout << F("---------------------------------------------") << endl;
	cout << pstrHeader << endl;
	cout << F("\tProgramNum::") << int(_nProgramNum) << endl;
	cout << F("\tBrewMode::") << _BrewMode << endl;
	cout << F("\tBrewBase::") << _BrewBase << endl;
	cout << F("\tInjectionSet::") << _fInjectionSet[0] << endl;
	cout << F("\tTotalBrewSet::") << _fExtractionSet[0] << endl;
	cout << F("\tInjectionPower::") << _fInjectionPower << endl;
	cout << F("\tExtractionPower::") << _fExtractionPower << endl;
	cout << F("---------------------------------------------") << endl;
}

void CEspresso::print_BrewingInitObservation(const bool bStandby)
{
	#if BREW_PRESSURE_SENSOR_ATTACHED
	static float _fStadbyBrewPress = 0.0F;
	#endif
	#if FEED_PRESSURE_SENSOR_ATTACHED
	static float _fStadbyPumpPress = 0.0F;
	#endif

	if (bStandby) {

		_ulLogPrintTime = 0UL;

		#if BREW_PRESSURE_SENSOR_ATTACHED
		_fStadbyBrewPress = _pstDataSet->stAcquisition.BrewPress;
		#endif
		#if FEED_PRESSURE_SENSOR_ATTACHED
		_fStadbyPumpPress = _pstDataSet->stAcquisition.PumpPress;
		#endif

	}
	else {

		cout << F("\t")
			 << F("0.000")									<< F("\t")
			 << F("0") 										<< F("\t")
			 << F("0.000") 									<< F("\t")
			 << F("0.000") 									<< F("\t")
		#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewWaterTemp 	<< F("\t")
		#endif
		#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewTemp 		<< F("\t")
		#endif
		#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.GroupTemp 		<< F("\t")
		#endif
		#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.PreheatTemp 		<< F("\t")
		#endif
		#if BREW_PRESSURE_SENSOR_ATTACHED
			 << _fStadbyBrewPress 							<< F("\t")
			 << _fStadbyBrewPress							<< F("\t")
		#endif
		#if PUMP_PRESSURE_SENSOR_ATTACHED
			 << _fStadbyPumpPress 					 		<< F("\t")
			 << _fStadbyPumpPress							<< F("\t")
		#endif
		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			 << _pstDataSet->stAcquisition.PresentSpeed		<< F("\t")
			 << _pstDataSet->stAcquisition.PresentCurrent	<< F("\t")
		#endif
			 << endl;

	}
}

void CEspresso::print_BrewingObservation(void)
{
	if (_pstDataSet->stAcquisition.ShotTime - _ulLogPrintTime >= LOG_BREWING_OBSERBATION_PERIOD) {

		_ulLogPrintTime += LOG_BREWING_OBSERBATION_PERIOD;

		cout << F("\t") 
			 << _pstDataSet->stAcquisition.ShotTime / 1000.0f 	<< F("\t")
			 << _pstDataSet->stAcquisition.FlowCount 			<< F("\t")
			 << _pstDataSet->stAcquisition.FlowRate 			<< F("\t")
			 << _pstDataSet->stAcquisition.AvgFlowRate 			<< F("\t")

		#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewWaterTemp 		<< F("\t")
		#endif
		#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewTemp 			<< F("\t")
		#endif
		#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.GroupTemp 			<< F("\t")
		#endif
		#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.PreheatTemp 			<< F("\t")
		#endif
		#if BREW_PRESSURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewPress 			<< F("\t")
			 << _pstDataSet->stAcquisition.AvgBrewPress 		<< F("\t")
		#endif
		#if PUMP_PRESSURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.PumpPress 			<< F("\t")
			 << _pstDataSet->stAcquisition.AvgPumpPress 		<< F("\t")
		#endif
		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			 << _pstDataSet->stAcquisition.PresentSpeed 		<< F("\t")
			 << _pstDataSet->stAcquisition.PresentCurrent		<< F("\t")
		#endif
			 << endl;

	}
}

void CEspresso::print_PostBrewCondition(const _FlashString* pstrHeader)
{
	cout << F("\t") 
		 << _pstDataSet->stAcquisition.ShotTime / 1000.0f 	<< F("\t")
		 << _pstDataSet->stAcquisition.FlowCount 			<< F("\t")
		 << _pstDataSet->stAcquisition.FlowRate 			<< F("\t")
		 << _pstDataSet->stAcquisition.AvgFlowRate 			<< F("\t")

	#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewWaterTemp 		<< F("\t")
	#endif
	#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewTemp 			<< F("\t")
	#endif
	#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.GroupTemp 			<< F("\t")
	#endif
	#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.PreheatTemp 			<< F("\t")
	#endif
	#if BREW_PRESSURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewPress 			<< F("\t")
		 << _pstDataSet->stAcquisition.AvgBrewPress 		<< F("\t")
	#endif
	#if PUMP_PRESSURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.PumpPress 			<< F("\t")
		 << _pstDataSet->stAcquisition.AvgPumpPress 		<< F("\t")
	#endif
	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
		 << _pstDataSet->stAcquisition.PresentSpeed 		<< F("\t")
		 << _pstDataSet->stAcquisition.PresentCurrent		<< F("\t")
	#endif
		 << F("\n\r") << endl;

	// cout << F("\n\r") << endl;
	cout << F("---------------------------------------------") << endl;
	cout << pstrHeader << endl;
	cout << F("\tProgramNum::") << int(_pstDataSet->ProgramNum) << endl;
	cout << F("\tShotTime::") << _pstDataSet->stAcquisition.ShotTime << endl;
	cout << F("\tFlowCount::") << _pstDataSet->stAcquisition.FlowCount << endl;
	cout << F("\tFlowQuantity::") << _pstDataSet->stAcquisition.FlowQuantity << endl;
	cout << F("\tAvgFlowRate::") << _pstDataSet->stAcquisition.AvgFlowRate << endl;
	#if BREW_PRESSURE_SENSOR_ATTACHED
	cout << F("\tAvgBrewPress::") << _pstDataSet->stAcquisition.AvgBrewPress << endl;
	#endif
	#if PUMP_PRESSURE_SENSOR_ATTACHED
	cout << F("\tAvgPumpPress::") << _pstDataSet->stAcquisition.AvgPumpPress << endl;
	#endif

	cout << F("\tInjectionSet[1]::") << getInjectionSet(CEspresso::BREW_FLOWCNT) << endl;
	cout << F("\tExtractionSet[1]::") << getExtractionSet(CEspresso::BREW_FLOWCNT) << endl;
	cout << F("\tTotalBrewSet[1]::") << getMaxBrewingSet(CEspresso::BREW_FLOWCNT) << endl;
	cout << F("\tInjectionSet[2]::") << getInjectionSet(CEspresso::BREW_TIMECNT) << endl;
	cout << F("\tExtractionSet[2]::") << getExtractionSet(CEspresso::BREW_TIMECNT) << endl;
	cout << F("\tTotalBrewSet[2]::") << getMaxBrewingSet(CEspresso::BREW_TIMECNT) << endl;

	cout << F("---------------------------------------------\n\r") << endl;
}

void CEspresso::print_UpdatedBrewCondition(const _FlashString* pstrHeader)
{
	cout << pstrHeader << endl;
	cout << F("\tProgramNum::") << int(_pstDataSet->ProgramNum) << endl;
	cout << F("\tInjectionFlow::") << _pstDataSet->stSetup.InjectionFlow[_nProgramNum] << endl;
	cout << F("\tExtractionFlow::") << _pstDataSet->stSetup.ExtractionFlow[_nProgramNum] << endl;
	cout << F("\tTotFlowCount::") << _pstDataSet->stSetup.InjectionFlow[_nProgramNum] + _pstDataSet->stSetup.ExtractionFlow[_nProgramNum] << endl;
	cout << F("\tInjectionTime::") << _pstDataSet->stSetup.InjectionTime[_nProgramNum] << endl;
	cout << F("\tExtractionTime::") << _pstDataSet->stSetup.ExtractionTime[_nProgramNum] << endl;
	cout << F("\tTotShotTime::") << _pstDataSet->stSetup.InjectionTime[_nProgramNum] + _pstDataSet->stSetup.ExtractionTime[_nProgramNum] << endl;
	cout << F("--------------------SAVED--------------------") << endl;
}
#else

#if defined(_USE_BREW_PROFILE_ACQUISITION_)
void CEspresso::print_PreparedBrewCondition(void)
{
	float fInjectionSet = _fInjectionSet[0] * 0.5f;
	float fExtractionSet = (_fExtractionSet[0] - _fInjectionSet[0]) * 0.5f;

	cout << F("CONF::")
		 << int(_nProgramNum) << F("::")
		 << _BrewMode << F("::")
		 << _BrewBase << F("::")
		 << fInjectionSet << F("::")
		 << fExtractionSet  << F("::")
		 << _fInjectionPower << F("::")
		 << _fExtractionPower
	<< endl;

	cout << F("TEMP::")
		 << _pstDataSet->stSetup.SetValueTemp[1] << F("::")
		 << _pstDataSet->stSetup.SetValueTemp[2] << F("::")
		 << _pstDataSet->stSetup.SetValueTemp[0]
	<< endl;
}

void CEspresso::print_BrewingInitObservation(const bool bStandby)
{
	#if BREW_PRESSURE_SENSOR_ATTACHED
	static float _fStadbyBrewPress = 0.0F;
	#endif
	#if FEED_PRESSURE_SENSOR_ATTACHED
	static float _fStadbyPumpPress = 0.0F;
	#endif

	if (bStandby) {

		_ulLogPrintTime = 0UL;

		#if BREW_PRESSURE_SENSOR_ATTACHED
		_fStadbyBrewPress = _pstDataSet->stAcquisition.BrewPress;
		#endif
		#if FEED_PRESSURE_SENSOR_ATTACHED
		_fStadbyPumpPress = _pstDataSet->stAcquisition.PumpPress;
		#endif

	}
	else {

		cout << F("ACQ::")
			 << 0.000f 															<< F("::")
			 << (_BrewState == CEspresso::BREW_INJECTION ? F("INJ") : F("EXT")) << F("::")
			 << 0 																<< F("::")
			 << 0.000f 															<< F("::")
			 << 0.000f 															<< F("::")
		#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewWaterTemp 						<< F("::")
		#else
			 << 0.000f															<< F("::")
		#endif
		#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewTemp 							<< F("::")
		#endif
		#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.GroupTemp 							<< F("::")
		#endif
		#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.PreheatTemp 							<< F("::")
		#endif
		#if BREW_PRESSURE_SENSOR_ATTACHED
			 << _fStadbyBrewPress 												<< F("::")
			 << _fStadbyBrewPress												<< F("::")
		#endif
		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			 << _pstDataSet->stAcquisition.PresentSpeed							<< F("::")
			 << _pstDataSet->stAcquisition.PresentCurrent
		#endif
		<< endl;

	}
}

void CEspresso::print_BrewingObservation(void)
{
	if (_pstDataSet->stAcquisition.ShotTime - _ulLogPrintTime >= LOG_BREWING_OBSERBATION_PERIOD) {

		_ulLogPrintTime += LOG_BREWING_OBSERBATION_PERIOD;

		// float fShotTime = _pstDataSet->stAcquisition.ShotTime / 1000.0f;
		float fShotTime = _ulLogPrintTime / 1000.0f;

		cout << F("ACQ::")
			 << floorf(fShotTime*10.0f)/10.0f 									<< F("::")
			 << (_BrewState == CEspresso::BREW_INJECTION ? F("INJ") : F("EXT")) << F("::")
			 << _pstDataSet->stAcquisition.FlowCount 							<< F("::")
			 << _pstDataSet->stAcquisition.FlowRate 							<< F("::")
			 << _pstDataSet->stAcquisition.AvgFlowRate 							<< F("::")

		#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewWaterTemp 						<< F("::")
		#else
			 << 0.000f															<< F("::")
		#endif
		#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewTemp 							<< F("::")
		#endif
		#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.GroupTemp 							<< F("::")
		#endif
		#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.PreheatTemp 							<< F("::")
		#endif
		#if BREW_PRESSURE_SENSOR_ATTACHED
			 << _pstDataSet->stAcquisition.BrewPress 							<< F("::")
			 << _pstDataSet->stAcquisition.AvgBrewPress 						<< F("::")
		#endif
		#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
			 << _pstDataSet->stAcquisition.PresentSpeed 						<< F("::")
			 << _pstDataSet->stAcquisition.PresentCurrent
		#endif
		<< endl;

	}
}

void CEspresso::print_PostBrewCondition(void)
{
	float fShotTime = _pstDataSet->stAcquisition.ShotTime / 1000.0f;

	cout << F("ACQ::")
		 << floorf(fShotTime*10.0f)/10.0f 									<< F("::")
		 << (_BrewState == CEspresso::BREW_INJECTION ? F("INJ") : F("EXT")) << F("::")
		 << _pstDataSet->stAcquisition.FlowCount 							<< F("::")
		 << _pstDataSet->stAcquisition.FlowRate 							<< F("::")
		 << _pstDataSet->stAcquisition.AvgFlowRate 							<< F("::")

	#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewWaterTemp 						<< F("::")
	#else
		 << 0.000f															<< F("::")
	#endif
	#if BREW_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewTemp 							<< F("::")
	#endif
	#if GROUPHEAD_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.GroupTemp 							<< F("::")
	#endif
	#if PREHEAT_BOILER_TEMPERATURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.PreheatTemp 							<< F("::")
	#endif
	#if BREW_PRESSURE_SENSOR_ATTACHED
		 << _pstDataSet->stAcquisition.BrewPress 							<< F("::")
		 << _pstDataSet->stAcquisition.AvgBrewPress 						<< F("::")
	#endif
	#if defined(_USE_OZBV10AD2M2_MOTOR_CONTROLLER_)
		 << _pstDataSet->stAcquisition.PresentSpeed 						<< F("::")
		 << _pstDataSet->stAcquisition.PresentCurrent
	#endif
	<< endl;

	cout << F("ENT::") << (_bIsFlushingDetected == true ? F("flush") : F("stop")) << endl;
}
#endif

#endif
