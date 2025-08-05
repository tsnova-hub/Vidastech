

#include "MES_SetupMenu.h"


const static char _strNumbers[] = {LCD_ZERO_CHAR, LCD_ONE_CHAR, LCD_TWO_CHAR, LCD_THREE_CHAR, LCD_FOUR_CHAR, LCD_FIVE_CHAR, LCD_SIX_CHAR, LCD_SEVEN_CHAR, LCD_EIGHT_CHAR, LCD_NINE_CHAR, LCD_ASTERISK_CHAR};
const static char _strProgramNum[] = {LCD_PROGRAM1_CHAR, LCD_PROGRAM2_CHAR};
const static char* _strProgramMode[] = {LCD_FLOW_STRING, LCD_TIME_STRING};
const static char* _strOnOff[] = {LCD_OFF_STRING, LCD_ON_STRING};
const static char* _strYesNo[] = {LCD_NO_STRING, LCD_YES_STRING};
const static char* _strLowHigh[] = {LCD_LOW_STRING, LCD_HIGH_STRING};
const static char* _strRunStop[] = {LCD_STOP_STRING, LCD_RUN_STRING};
#if AUTOMATION_SENSOR_ATTACHED
const static char* _strStandbyMode[] = {LCD_STANDBY_MODE_OFF_STRING, LCD_STANDBY_MODE_PREP_STRING, LCD_STANDBY_MODE_SHOT_STRING};
#endif
const static char* _strLampMode[] = {LCD_LAMP_OFF_STRING, LCD_LAMP_SHOT_STRING, LCD_LAMP_ALWAYS_STRING};
const static char* _strLEDMode[] = {LCD_LED_PROGRESS_STRING, LCD_LED_TIMELAPSE_STRING, LCD_LED_FLOWRATE_STRING, LCD_LED_BREW_PRESS_STRING, LCD_LED_PUMP_PRESS_STRING};
const static char* _strWeekday[] = {LCD_SUN_STRING, LCD_MON_STRING, LCD_TUE_STRING, LCD_WED_STRING, LCD_THU_STRING, LCD_FRI_STRING, LCD_SAT_STRING};
const static char* _strHeater[] = {LCD_PREHEAT_STRING, LCD_BREW_STRING, LCD_GROUPHEAD_STRING, LCD_MVHIGH_LIMIT_STRING, LCD_INPUT_BIAS_STRING};
const static char* _strTSensor[] = {LCD_PREHEAT_STRING, LCD_BREW_STRING, LCD_GROUPHEAD_STRING, LCD_BREWWATER_STRING};
const static char* _strPSensor[] = {LCD_BREWPRES_STRING, LCD_FEEDPRES_STRING, LCD_AUXPRES_STRING};

const static char* _strError[] = {LCD_SYSTEM_OVERLOAD_STRING, LCD_LOW_BREW_PRESSURE_STRING, LCD_BREW_PRESSURE_OVER_STRING, LCD_BREW_PRESSURE_UNDER_STRING,
								  LCD_LOW_INLET_PRESSURE_STRING, LCD_GROUPHEAD_OVERHEAT_STRING, LCD_BREWBOILER_OVERHEAT_STRING, LCD_PREHEATBOILER_OVERHEAT_STRING,
								  LCD_GROUPHEAD_UNDERHEAT_STRING, LCD_BREWBOILER_UNDERHEAT_STRING, LCD_PREHEATBOILER_UNDERHEAT_STRING, LCD_TEMPERATURE_SENSOR_ERROR_STRING};
const static char* _strNotice[] = {LCD_FLOWMETER_FAILURE_STRING, LCD_MOTOR_ALARM_STRING, LCD_MAINTENANCE_REQUIRED_STRING, LCD_FLOWRATE_EXCEEDED_STRING, LCD_OPTICAL_FIBER_UNSTABLE_STRING, LCD_EEPROM_WORN_STRING};


////////////////////////////////////////////////////////////
// Variables for Setup Menu Frameworks

enum eLCDView _nCurrentView;
enum eLCDView _nPreviousView;

enum eDashboard _nCurrentDashboard;
enum eMenuBoard _nCurrentMenuboard;
enum eMenuPage _nCurrentMenupage;

unsigned char _nCurrentPos;
unsigned char _nCurrentLine;
unsigned char _nPreviousPos;
unsigned char _nPreviousLine;

bool _bChangingTextLCD;
bool _bIsLeavingMenupage;
bool _bSelectedCursorTextLCD;
bool _bDefaultResetString;
bool _bAuthorizedSaveString;
bool _bNoSelectionPage;
bool _bSelectSetupTimeDateContents;

// unsigned long _ulUpdatedTimeCondition;


////////////////////////////////////////////////////////////
// Interfaces for Text LCD Display

void initialize_TextLCDParameters(void)
{
	_nCurrentView = DEFAULT_LCDVIEW;
	_nPreviousView = DEFAULT_LCDVIEW;

	_nCurrentDashboard = DEFAULT_DASHBOARD;
	_nCurrentMenuboard = DEFAULT_MENUBOARD;
	_nCurrentMenupage = DEFAULT_MENUPAGE;

	_nCurrentPos = 0;
	_nCurrentLine = 0;
	_nPreviousPos = 0;
	_nPreviousLine = 0;

	_bChangingTextLCD = false;
	_bIsLeavingMenupage = false;
	_bSelectedCursorTextLCD = false;
	_bDefaultResetString = false;
	_bAuthorizedSaveString = false;
	_bNoSelectionPage = false;
	_bSelectSetupTimeDateContents = false;

	// _ulUpdatedTimeCondition = TEXT_LCD_INVALIDATE_PERIOD1;
}

void set_TextLCDView(const eLCDView nView)
{
	_nCurrentView = nView;
	_bChangingTextLCD = true;
}

void set_TextLCDDashboard(const eDashboard nDashboard)
{
	_nCurrentDashboard = nDashboard;
	_bChangingTextLCD = true;
}

void set_TextLCDMenuboard(const eMenuBoard nMenuboard)
{
	_nCurrentMenuboard = nMenuboard;
	_bChangingTextLCD = true;
}

void set_TextLCDMenupage(const eMenuPage nMenupage)
{
	_nCurrentMenupage = nMenupage;
	_bChangingTextLCD = true;
}

eLCDView get_TextLCDView(void)
{
	return _nCurrentView;
}

eDashboard get_TextLCDDashboard(void)
{
	return _nCurrentDashboard;
}

eMenuBoard get_TextLCDMenuboard(void)
{
	return _nCurrentMenuboard;
}

eMenuPage get_TextLCDMenupage(void)
{
	return _nCurrentMenupage;
}

void set_SelectCursorPosition(const unsigned char nPos, const unsigned char nLine)
{
	_nCurrentPos = constrain(nPos, 0, LCD_MAX_COL-1);
	_nCurrentLine = constrain(nLine, 0, LCD_MAX_LINE-1);
	_nPreviousPos = _nCurrentPos;
	_nPreviousLine = _nCurrentLine;
}

void get_SelectCursorPosition(unsigned char& nPos, unsigned char& nLine)
{
	nPos = _nCurrentPos;
	nLine = _nCurrentLine;
}

void set_SelectCursorPos(const unsigned char nPos)
{
	_nPreviousPos = _nCurrentPos;
	_nCurrentPos = constrain(nPos, 0, LCD_MAX_COL-1);
}

unsigned char get_SelectCursorPos(void)
{
	return _nCurrentPos;
}

void set_SelectCursorLine(const unsigned char nLine)
{
	_nPreviousLine = _nCurrentLine;
	_nCurrentLine = constrain(nLine, 0, LCD_MAX_LINE-1);
}

unsigned char get_SelectCursorLine(void)
{
	return _nCurrentLine;
}

void set_ChangingTextLCD(const bool bChanging)
{
	_bChangingTextLCD = bChanging;
}

bool get_ChangingTextLCD(void)
{
	volatile bool bIsChanged = _bChangingTextLCD;

	_bChangingTextLCD = false;

	return bIsChanged;
}

void set_LeavingMenupage(const bool bLeaving)
{
	_bIsLeavingMenupage = bLeaving;
}

bool get_LeavingMenupage(void)
{
	return _bIsLeavingMenupage;
}

void set_SelectedCursor(const bool bSelected)
{
	_bSelectedCursorTextLCD = bSelected;
}

bool get_SelectedCursor(void)
{
	return _bSelectedCursorTextLCD;
}

void set_DefaultResetString(const bool bResetString)
{
	_bDefaultResetString = bResetString;
}

void set_AuthorizedString(const bool bAuthorizedString)
{
	_bAuthorizedSaveString = bAuthorizedString;
}

bool get_AuthorizedString(void)
{
	return _bAuthorizedSaveString;
}

void set_NoSelectionPage(const bool bNoSelectCursor)
{
	_bNoSelectionPage = bNoSelectCursor;
}

void select_SetupTimeDateContents(const bool bSetupContents)
{
	_bSelectSetupTimeDateContents = bSetupContents;
}

void display_SystemInitialization(void)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, 1, HEADER_MACHINE_INIT_STRING);

	hTextLCD.homeCursor();
}

void display_SystemFactoryReset(void)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, 1, HEADER_MACHINE_FACTORY_STRING);

	hTextLCD.homeCursor();
}

void display_SystemInformation(void)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, 1, HEADER_MACHINE_NAME_STRING);
	hTextLCD.printPos(5, 2, HEADER_MACHINE_MODEL_STRING);
	hTextLCD.printPos(0, 3, HEADER_MACHINE_MAKER_STRING);

	hTextLCD.homeCursor();
}

void display_TM4CommunicationSettingPass(void)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, 1, HEADER_TM4_COMM_SETTING_PASS_STRING);
	hTextLCD.printPos(0, 2, HEADER_SYSTEM_REBOOT_STRING);

	hTextLCD.homeCursor();
}

void display_TM4CommunicationSettingFail(void)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, 1, HEADER_TM4_COMM_SETTING_FAIL_STRING);
	hTextLCD.printPos(0, 2, HEADER_SYSTEM_REBOOT_STRING);

	hTextLCD.homeCursor();
}

void display_SensorAdjFailNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, NOTICE_HEADER_LINE, LCD_NOTICE_HEADER_STRING);
	hTextLCD.printPos(0, NOTICE_MESSAGE_LINE, LCD_SENSOR_ADJ_NOTICE_STRING);
	hTextLCD.printPos(0, NOTICE_CODE_LINE, LCD_SENSOR_ADJ_FAIL_STRING);

	disable_RotaryEncoder();

	delay(TEXT_LCD_INVALIDATE_PERIOD4);

	display_SystemMenupage(true, rDataSet, rTimeDate);

	set_SelectCursorLine(SENSOR_SPAN_VALUE_LINE);

	enable_RotaryEncoder();
}

void display_SensorZeroShiftNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, NOTICE_HEADER_LINE, LCD_NOTICE_HEADER_STRING);
	hTextLCD.printPos(0, NOTICE_MESSAGE_LINE, LCD_SENSOR_ZEROSHIFT_ADJ_STRING);

	disable_RotaryEncoder();

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	display_SystemMenupage(true, rDataSet, rTimeDate);

	set_SelectCursorLine(SENSOR_ZEROSHIFT_VALUE_LINE);

	enable_RotaryEncoder();
}

void display_SensorSpanNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, NOTICE_HEADER_LINE, LCD_NOTICE_HEADER_STRING);
	hTextLCD.printPos(0, NOTICE_MESSAGE_LINE, LCD_SENSOR_SPAN_ADJ_STRING);

	disable_RotaryEncoder();

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	display_SystemMenupage(true, rDataSet, rTimeDate);

	set_SelectCursorLine(SENSOR_SPAN_VALUE_LINE);

	enable_RotaryEncoder();
}

void display_SensorInitNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	hTextLCD.clear();

	hTextLCD.printPos(0, NOTICE_HEADER_LINE, LCD_NOTICE_HEADER_STRING);
	hTextLCD.printPos(0, NOTICE_MESSAGE_LINE, LCD_SENSOR_DEFAULT_ADJ_STRING);

	disable_RotaryEncoder();

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	display_SystemMenupage(true, rDataSet, rTimeDate);

	set_SelectCursorLine(SENSOR_DEFAULT_VALUE_LINE);

	enable_RotaryEncoder();
}

void display_ErrorMessage(const stMaintenanceDataSet_t& rMaintenance)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = AlarmView;

	hTextLCD.clear();

	hTextLCD.printPos(0, ERROR_HEADER_LINE, LCD_ERROR_HEADER_STRING);
	hTextLCD.printPos(0, ERROR_MESSAGE_LINE, _strError[(rMaintenance.RecentErrorCode & 0x000F)-1]);
	print_ExceptionCode(ERROR_CODE_POS, ERROR_CODE_LINE, rMaintenance.RecentErrorCode+0xE000);

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	hTextLCD.homeCursor();
}

void display_NoticeMessage(const stMaintenanceDataSet_t& rMaintenance)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = AlarmView;

	hTextLCD.clear();

	hTextLCD.printPos(0, NOTICE_HEADER_LINE, LCD_NOTICE_HEADER_STRING);
	hTextLCD.printPos(0, NOTICE_MESSAGE_LINE, _strNotice[(rMaintenance.RecentNoticeCode & 0x000F)-1]);
	print_ExceptionCode(NOTICE_CODE_POS, NOTICE_CODE_LINE, rMaintenance.RecentNoticeCode+0xA000);

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	hTextLCD.homeCursor();
}

void display_EntryMessage(const bool bDashboard)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = MessageView;

	hTextLCD.clear();

	hTextLCD.printPos(0, ENTRY_MESSAGE_LINE, (bDashboard ? LCD_DASHBOARD_ENTRY_STRING : LCD_SETUPMENU_ENTRY_STRING));

	delay(TEXT_LCD_INVALIDATE_PERIOD2);

	hTextLCD.homeCursor();
}

void display_MenupageExitMessage(const bool bSaveMsg)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = MessageView;

	hTextLCD.clear();

	switch (_nCurrentMenupage) {

		case Menupage14:

			if (bSaveMsg) {

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE, (_bAuthorizedSaveString ? LCD_SAVE_STRING : LCD_EXIT_STRING));

			}
			else {

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE, LCD_CANCEL_STRING);

			}

		break;

		case Menupage18:
		case Menupage19:

			if (bSaveMsg) {

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE, (_bDefaultResetString ? LCD_RESET_STRING : LCD_EXIT_STRING));

			}
			else {

				hTextLCD.printPos(0, SAVE_MESSAGE_LINE, LCD_CANCEL_STRING);

			}

		break;

		case Menupage20:

			hTextLCD.printPos(0, SAVE_MESSAGE_LINE, (bSaveMsg ? LCD_PW_VERIFY_STRING : LCD_CANCEL_STRING));

		break;

		default:

			hTextLCD.printPos(0, SAVE_MESSAGE_LINE, (bSaveMsg ? LCD_SAVE_STRING : LCD_CANCEL_STRING));

		break;

	}

	delay(TEXT_LCD_INVALIDATE_PERIOD2);

	hTextLCD.homeCursor();
}

void display_CleaningEntryMessage(const bool bCondition)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = MessageView;

	hTextLCD.clear();

	hTextLCD.printPos(0, ENTRY_MESSAGE_LINE, LCD_CLEANING_MODE_ENTRY_STRING);

	delay(TEXT_LCD_INVALIDATE_PERIOD3);

	hTextLCD.homeCursor();
}

void display_CleaningReadyMessage(void)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = CleaningView;

	hTextLCD.clear();

	hTextLCD.printPos(0, CLEANING_HEADER_LINE, LCD_CLEANING_MODE_ENTRY_STRING);
	hTextLCD.printPos(0, CLEANING_MESSAGE_LINE, LCD_CLEANING_READY_STRING);

	hTextLCD.homeCursor();
}

void display_CleaningModeMessage(void)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = CleaningView;

	hTextLCD.clear();

	hTextLCD.printPos(0, CLEANING_HEADER_LINE, LCD_CLEANING_MODE_STRING);

	hTextLCD.homeCursor();
}

void display_SystemDashboard(const bool bDisplayContents, const stDataSet_t& rDataSet)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = DashboardView;

	_nCurrentMenuboard = Menuboard1;
	_nCurrentMenupage = Menupage1;

	_nCurrentPos = HOME_CURSOR_POS;
	_nCurrentLine = HOME_CURSOR_LINE;
	_nPreviousPos = HOME_CURSOR_POS;
	_nPreviousLine = HOME_CURSOR_LINE;

	// _ulUpdatedTimeCondition = TEXT_LCD_INVALIDATE_PERIOD1;

	display_DashboardTitle();

	if (bDisplayContents) {

		display_DashboardContents(rDataSet);

	}
}

void display_SystemMenuboard(void)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = MenuboardView;

	// _ulUpdatedTimeCondition = TEXT_LCD_INVALIDATE_PERIOD3;//

	display_SetupMenuBoard();
}

void display_SystemMenupage(const bool bDisplayContents, const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	_nPreviousView = _nCurrentView;
	_nCurrentView = MenupageView;
	
	// _ulUpdatedTimeCondition = TEXT_LCD_INVALIDATE_PERIOD1;

	display_SetupMenuPageTitle();

	if (bDisplayContents) {

		display_SetupMenuPageContents(rDataSet, rTimeDate);

	}
}

void invalidate_TextLCD(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	static unsigned long _ulUpdatedTime = 0UL;

	if (millis() - _ulUpdatedTime >= TEXT_LCD_INVALIDATE_PERIOD1 /*_ulUpdatedTimeCondition*/) {

		_ulUpdatedTime = millis();

		switch (_nCurrentView)
		{
			case DashboardView:

				display_DashboardContents(rDataSet);

			break;

			case MenuboardView:

			break;

			case MenupageView:

				display_SetupMenuPageContents(rDataSet, rTimeDate);

			break;

			case MessageView:

				_nCurrentView = _nPreviousView;
				hRotaryEncoder.clearDirection();

			break;

			case AlarmView:

			break;

			case CleaningView:

			break;

		}

	}
}

void display_SelectCursor(void)
{
	if (_nCurrentView != MessageView && _nCurrentView != AlarmView && !_bNoSelectionPage) {

		if (_nCurrentPos != _nPreviousPos || _nCurrentLine != _nPreviousLine) {

			hTextLCD.writePos(_nPreviousPos, _nPreviousLine, LCD_BLANK_CHAR);

		}

		hTextLCD.writePos(_nCurrentPos, _nCurrentLine, (!_bSelectedCursorTextLCD ? (unsigned char)SCHAR_DESELECTED : (unsigned char)SCHAR_SELECTED));

	}

}

void display_DashboardTitle(void)
{
	hTextLCD.clear();

	switch (_nCurrentDashboard)
	{
		case DashboardA:

			hTextLCD.printPos(0, SHOT_TIME_VALUE_LINE,  LCD_DASHBOARD_LINE0_TEMPLATE_A);
			hTextLCD.printPos(0, FLOW_QTY_A_VALUE_LINE, LCD_DASHBOARD_LINE1_TEMPLATE_A);
			hTextLCD.printPos(0, BREW_TEMP_VALUE_LINE,  LCD_DASHBOARD_LINE2_TEMPLATE_A);
			hTextLCD.printPos(0, PRESSURE_VALUE_LINE,   LCD_DASHBOARD_LINE3_TEMPLATE_A);

		break;

		case DashboardB:

			#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			hTextLCD.printPos(0, GROUP_TEMP_VALUE_LINE,  	 LCD_DASHBOARD_LINE0_TEMPLATE_B);
			hTextLCD.printPos(0, BREW_B_TEMP_VALUE_LINE, 	 LCD_DASHBOARD_LINE1_TEMPLATE_B);
			hTextLCD.printPos(0, PRE_B_TEMP_VALUE_LINE,  	 LCD_DASHBOARD_LINE2_TEMPLATE_B);
			hTextLCD.printPos(0, BREW_WATER_TEMP_VALUE_LINE, LCD_DASHBOARD_LINE3_TEMPLATE_B);
			#else
			hTextLCD.printPos(0, SHOT_TIME_VALUE_LINE,  	 LCD_DASHBOARD_LINE0_TEMPLATE_B);
			hTextLCD.printPos(0, GROUP_TEMP_VALUE_LINE,  	 LCD_DASHBOARD_LINE1_TEMPLATE_B);
			hTextLCD.printPos(0, BREW_B_TEMP_VALUE_LINE, 	 LCD_DASHBOARD_LINE2_TEMPLATE_B);
			hTextLCD.printPos(0, PRE_B_TEMP_VALUE_LINE,  	 LCD_DASHBOARD_LINE3_TEMPLATE_B);
			#endif

		break;

		case DashboardC:

			hTextLCD.printPos(0, SHOT_TIME_VALUE_LINE,     LCD_DASHBOARD_LINE0_TEMPLATE_C);
			hTextLCD.printPos(0, FLOW_RATE_C_VALUE_LINE,   LCD_DASHBOARD_LINE1_TEMPLATE_C);
			hTextLCD.printPos(0, AVG_FLOW_RATE_VALUE_LINE, LCD_DASHBOARD_LINE2_TEMPLATE_C);
			hTextLCD.printPos(0, AVG_PRESSURE_VALUE_LINE,  LCD_DASHBOARD_LINE3_TEMPLATE_C);

		break;
	}

	hTextLCD.homeCursor();
}

void display_DashboardContents(const stDataSet_t& rDataSet)
{
	if (_nCurrentView == MessageView || _nCurrentView == AlarmView) {

		return;

	}

	switch (_nCurrentDashboard)
	{
		case DashboardA:

			print_FloatValue(DASHBOARD_A_SUBVALUE_POS, SHOT_TIME_VALUE_LINE, (rDataSet.stAcquisition.ShotTime/1000.0F));
			print_FloatFlowQuantityValue(DASHBOARD_A_SUBVALUE_POS, FLOW_QTY_A_VALUE_LINE, rDataSet.stAcquisition.FlowQuantity);
			print_FloatValue(DASHBOARD_A_SUBVALUE_POS, BREW_TEMP_VALUE_LINE, rDataSet.stAcquisition.BrewTemp);
			print_FloatPressureValue(DASHBOARD_A_SUBVALUE_POS, PRESSURE_VALUE_LINE, rDataSet.stAcquisition.BrewPress);

		break;

		case DashboardB:

			#if BREW_WATER_TEMPERATURE_SENSOR_ATTACHED
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, GROUP_TEMP_VALUE_LINE, rDataSet.stAcquisition.GroupTemp);
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, BREW_B_TEMP_VALUE_LINE, rDataSet.stAcquisition.BrewTemp);
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, PRE_B_TEMP_VALUE_LINE, rDataSet.stAcquisition.PreheatTemp);
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, BREW_WATER_TEMP_VALUE_LINE, rDataSet.stAcquisition.BrewWaterTemp);
			#else
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, SHOT_TIME_VALUE_LINE, (rDataSet.stAcquisition.ShotTime/1000.0F));
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, GROUP_TEMP_VALUE_LINE, rDataSet.stAcquisition.GroupTemp);
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, BREW_B_TEMP_VALUE_LINE, rDataSet.stAcquisition.BrewTemp);
			print_FloatValue(DASHBOARD_B_SUBVALUE_POS, PRE_B_TEMP_VALUE_LINE, rDataSet.stAcquisition.PreheatTemp);
			#endif

		break;

		case DashboardC:

			print_FloatValue(DASHBOARD_C_SUBVALUE_POS, SHOT_TIME_VALUE_LINE, (rDataSet.stAcquisition.ShotTime/1000.0F));
			print_FloatValue(DASHBOARD_C_SUBVALUE_POS, FLOW_RATE_C_VALUE_LINE, rDataSet.stAcquisition.FlowRate);
			print_FloatValue(DASHBOARD_C_SUBVALUE_POS, AVG_FLOW_RATE_VALUE_LINE, rDataSet.stAcquisition.AvgFlowRate);
			print_FloatPressureValue(DASHBOARD_C_SUBVALUE_POS, AVG_PRESSURE_VALUE_LINE, rDataSet.stAcquisition.AvgBrewPress);

			if (rDataSet.stIndicator.nShotTimeLevel)
				print_Char(DASHBOARD_C_INDICATOR_POS, SHOT_TIME_VALUE_LINE, (rDataSet.stIndicator.nShotTimeLevel < BREW_PROFILE_ALARM_LEVEL ? (unsigned char)SCHAR_DESELECTED : (unsigned char)SCHAR_SELECTED));
			else
				print_Char(DASHBOARD_C_INDICATOR_POS, SHOT_TIME_VALUE_LINE, LCD_BLANK_CHAR);

			if (rDataSet.stIndicator.nCurFlowrateLevel)
				print_Char(DASHBOARD_C_INDICATOR_POS, FLOW_RATE_C_VALUE_LINE, (rDataSet.stIndicator.nCurFlowrateLevel < BREW_PROFILE_ALARM_LEVEL ? (unsigned char)SCHAR_DESELECTED : (unsigned char)SCHAR_SELECTED));
			else
				print_Char(DASHBOARD_C_INDICATOR_POS, FLOW_RATE_C_VALUE_LINE, LCD_BLANK_CHAR);

			if (rDataSet.stIndicator.nAvgFlowrateLevel)
				print_Char(DASHBOARD_C_INDICATOR_POS, AVG_FLOW_RATE_VALUE_LINE, (rDataSet.stIndicator.nAvgFlowrateLevel < BREW_PROFILE_ALARM_LEVEL ? (unsigned char)SCHAR_DESELECTED : (unsigned char)SCHAR_SELECTED));
			else
				print_Char(DASHBOARD_C_INDICATOR_POS, AVG_FLOW_RATE_VALUE_LINE, LCD_BLANK_CHAR);
			
			if (rDataSet.stIndicator.nAvgBrewPressLevel)
				print_Char(DASHBOARD_C_INDICATOR_POS, AVG_PRESSURE_VALUE_LINE, (rDataSet.stIndicator.nAvgBrewPressLevel < BREW_PROFILE_ALARM_LEVEL ? (unsigned char)SCHAR_DESELECTED : (unsigned char)SCHAR_SELECTED));
			else
				print_Char(DASHBOARD_C_INDICATOR_POS, AVG_PRESSURE_VALUE_LINE, LCD_BLANK_CHAR);

		break;
	}

	hTextLCD.homeCursor();
}

void display_SetupMenuBoard(void)
{
	hTextLCD.clear();

	switch (_nCurrentMenuboard)
	{
		case Menuboard1:

			hTextLCD.printPos(0, 0, LCD_SETUPMENU_LINE0_TEMPLATE_1);
			hTextLCD.printPos(0, 1, LCD_SETUPMENU_LINE1_TEMPLATE_1);
			hTextLCD.printPos(0, 2, LCD_SETUPMENU_LINE2_TEMPLATE_1);
			hTextLCD.printPos(0, 3, LCD_SETUPMENU_LINE3_TEMPLATE_1);

		break;

		case Menuboard2:

			hTextLCD.printPos(0, 0, LCD_SETUPMENU_LINE0_TEMPLATE_2);
			hTextLCD.printPos(0, 1, LCD_SETUPMENU_LINE1_TEMPLATE_2);
			hTextLCD.printPos(0, 2, LCD_SETUPMENU_LINE2_TEMPLATE_2);
			hTextLCD.printPos(0, 3, LCD_SETUPMENU_LINE3_TEMPLATE_2);

		break;

		case Menuboard3:

			hTextLCD.printPos(0, 0, LCD_SETUPMENU_LINE0_TEMPLATE_3);
			hTextLCD.printPos(0, 1, LCD_SETUPMENU_LINE1_TEMPLATE_3);
			hTextLCD.printPos(0, 2, LCD_SETUPMENU_LINE2_TEMPLATE_3);
			hTextLCD.printPos(0, 3, LCD_SETUPMENU_LINE3_TEMPLATE_3);

		break;

		case Menuboard4:

			hTextLCD.printPos(0, 0, LCD_SETUPMENU_LINE0_TEMPLATE_4);
			hTextLCD.printPos(0, 1, LCD_SETUPMENU_LINE1_TEMPLATE_4);
			hTextLCD.printPos(0, 2, LCD_SETUPMENU_LINE2_TEMPLATE_4);
			hTextLCD.printPos(0, 3, LCD_SETUPMENU_LINE3_TEMPLATE_4);

		break;

		case Menuboard5:

			hTextLCD.printPos(0, 0, LCD_SETUPMENU_LINE0_TEMPLATE_5);
			hTextLCD.printPos(0, 1, LCD_SETUPMENU_LINE1_TEMPLATE_5);
			hTextLCD.printPos(0, 2, LCD_SETUPMENU_LINE2_TEMPLATE_5);
			hTextLCD.printPos(0, 3, LCD_SETUPMENU_LINE3_TEMPLATE_5);

		break;
	}

	_nPreviousPos = _nCurrentPos;
	_nPreviousLine = _nCurrentLine;

	hTextLCD.homeCursor();
}

void display_SetupMenuPageTitle(void)
{
	hTextLCD.clear();

	switch (_nCurrentMenupage)
	{
		case Menupage1:
		case Menupage2:

			hTextLCD.printPos(0, 0, LCD_PROGRAM_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_PROGRAM_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_PROGRAM_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_PROGRAM_LINE3_TEMPLATE);

			_nCurrentPos = PROGRAM_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= PROGRAM_MODE_VALUE_LINE;

		break;

		case Menupage3:

			hTextLCD.printPos(0, 0, LCD_MANUAL_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_MANUAL_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_MANUAL_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_MANUAL_LINE3_TEMPLATE);

			_nCurrentPos = MANUAL_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= SAVE_TO_PROG_NUM_VALUE_LINE;

		break;

		case Menupage4:

			hTextLCD.printPos(0, 0, LCD_INJECTION_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_INJECTION_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_INJECTION_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_INJECTION_LINE3_TEMPLATE);

			_nCurrentPos = INJECTION_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= INJECTION_MODE_VALUE_LINE;

		break;

		case Menupage5:
		case Menupage6:
		case Menupage7:

			if (_nCurrentMenupage == Menupage5) {
				hTextLCD.printPos(0, 0, LCD_GROUPTEMP_LINE0_TEMPLATE);				
			}
			else if (_nCurrentMenupage == Menupage6) {
				hTextLCD.printPos(0, 0, LCD_BREWTEMP_LINE0_TEMPLATE);
			}
			else {
				hTextLCD.printPos(0, 0, LCD_PRETEMP_LINE0_TEMPLATE);
			}

			hTextLCD.printPos(0, 1, LCD_GROUPTEMP_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_GROUPTEMP_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_GROUPTEMP_LINE3_TEMPLATE);

			_nCurrentPos = BOILER_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= SET_TEMP_VALUE_LINE;

		break;

		case Menupage8:

			hTextLCD.printPos(0, 0, LCD_MOTORPUMP_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_MOTORPUMP_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_MOTORPUMP_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_MOTORPUMP_LINE3_TEMPLATE);

			_nCurrentPos = PUMP_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= MOTOR_POWER_VALUE_LINE;

		break;

		case Menupage9:

			hTextLCD.printPos(0, 0, LCD_LAMP_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_LAMP_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_LAMP_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_LAMP_LINE3_TEMPLATE);

			_nCurrentPos = LAMP_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine= LAMP_MODE_VALUE_LINE;

		break;

		case Menupage10:

			hTextLCD.printPos(0, 0, LCD_LEDPIXEL_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_LEDPIXEL_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_LEDPIXEL_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_LEDPIXEL_LINE3_TEMPLATE);

			_nCurrentPos = LEDPIXEL_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine = PROGRAM1_INDICATOR_VALUE_LINE;

		break;

		case Menupage11:
			hTextLCD.printPos(0, 0, LCD_TIMEDATE_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_TIMEDATE_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_TIMEDATE_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_TIMEDATE_LINE3_TEMPLATE);

			_nCurrentPos = TIMEDATE_SETUP_SUBTITLE_DIGIT;
			_nCurrentLine = DATE_VALUE_LINE;

		break;

		case Menupage12:

			hTextLCD.printPos(0, 0, LCD_POWSAVE_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_POWSAVE_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_POWSAVE_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_POWSAVE_LINE3_TEMPLATE);

			_nCurrentPos = POWER_SAVING_MODE_SUBTITLE_DIGIT;
			_nCurrentLine = POWER_SAVING_MODE_VALUE_LINE;

		break;

		case Menupage13:

			hTextLCD.printPos(0, 0, LCD_HEATER_PID_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_HEATER_PID_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_HEATER_PID_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_HEATER_PID_LINE3_TEMPLATE);

			_nCurrentPos = HEATER_SUBTITLE_DIGIT;
			_nCurrentLine = HEATER_SET_VALUE_LINE;

		break;

		case Menupage14:

			hTextLCD.printPos(0, 0, LCD_SENSOR_ADJ_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_SENSOR_ADJ_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_SENSOR_ADJ_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_SENSOR_ADJ_LINE3_TEMPLATE);

			_nCurrentPos = SENSOR_SUBTITLE_DIGIT;
			_nCurrentLine = SENSOR_CHANNEL_VALUE_LINE;

		break;

		case Menupage15:

			hTextLCD.printPos(0, 0, LCD_MOTOR_ACC_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_MOTOR_ACC_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_MOTOR_ACC_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_MOTOR_ACC_LINE3_TEMPLATE);

			_nCurrentPos = MOTOR_ACC_SUBTITLE_DIGIT;
			_nCurrentLine = MOTOR_ACC_TIME_VALUE_LINE;

		break;

		case Menupage16:

#if AUTOMATION_SENSOR_ATTACHED
			hTextLCD.printPos(0, 0, LCD_AUTOMATION_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_AUTOMATION_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_AUTOMATION_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_AUTOMATION_LINE3_TEMPLATE);

			_nCurrentPos = AUTOMATION_MODE_SUBTITLE_DIGIT;
			_nCurrentLine = AUTOMATION_MODE_VALUE_LINE;
#else
			hTextLCD.printPos(0, 0, LCD_FLUSHING_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_FLUSHING_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_FLUSHING_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_FLUSHING_LINE3_TEMPLATE);

			_nCurrentPos = FLUSHING_DETECTION_SUBTITLE_DIGIT;
			_nCurrentLine = FLUSHING_DETECTION_MODE_VALUE_LINE;
#endif

		break;

		case Menupage17:

			hTextLCD.printPos(0, 0, LCD_MAINTENANCE_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_MAINTENANCE_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_MAINTENANCE_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_MAINTENANCE_LINE3_TEMPLATE);

			_nCurrentPos = MAINTENANCE_SUBTITLE_DIGIT;
			_nCurrentLine= MAINTENANCE_MODE_VALUE_LINE;

		break;

		case Menupage18:

			hTextLCD.printPos(0, 0, LCD_RESET_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_RESET_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_RESET_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_RESET_LINE3_TEMPLATE);

			_nCurrentPos = RESET_SUBTITLE_DIGIT;
			_nCurrentLine= DEFAUT_SET_VALUE_LINE;

		break;

		case Menupage19:

			hTextLCD.printPos(0, 0, LCD_SYSTEM_INFO_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_SYSTEM_INFO_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_SYSTEM_INFO_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_SYSTEM_INFO_LINE3_TEMPLATE);

			_nCurrentPos = INFORMATION_SUBTITLE_DIGIT;
			_nCurrentLine= HARDWARE_MODEL_NAME_VALUE_LINE;

			_bNoSelectionPage = true;

		break;

		case Menupage20:

			hTextLCD.printPos(0, 0, LCD_PASSWORD_LINE0_TEMPLATE);
			hTextLCD.printPos(0, 1, LCD_PASSWORD_LINE1_TEMPLATE);
			hTextLCD.printPos(0, 2, LCD_PASSWORD_LINE2_TEMPLATE);
			hTextLCD.printPos(0, 3, LCD_PASSWORD_LINE3_TEMPLATE);

			_nCurrentPos = PASSWORD_SUBTITLE_DIGIT;
			_nCurrentLine= PASSWORD_VALUE_LINE;

		break;
	}

	_nPreviousPos = _nCurrentPos;
	_nPreviousLine = _nCurrentLine;

	hTextLCD.homeCursor();
}

void display_SetupMenuPageContents(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate)
{
	if (_nCurrentView == MessageView) {

		return;

	}

	if (_bIsLeavingMenupage == true) {

		_bIsLeavingMenupage = false;

		return;

	}

	switch (_nCurrentMenupage)
	{
		case Menupage1:
		case Menupage2:
		{
			int nProgramNum = _nCurrentMenupage == Menupage1 ? PROGRAM_SET_ONE : PROGRAM_SET_TWO;

			print_String(PROGRAM_SETUP_SUBVALUE_POS, PROGRAM_MODE_VALUE_LINE, _strProgramMode[rDataSet.stSetup.ProgramMode[nProgramNum]]);

			if (rDataSet.stSetup.ProgramMode[nProgramNum]) {

				float fInjectionTime = rDataSet.stSetup.InjectionTime[nProgramNum];
				float fExtractionTime = rDataSet.stSetup.ExtractionTime[nProgramNum];

				print_FloatProgramShotTimeValue(PROGRAM_SETUP_SUBVALUE_POS, INJECTION_VALUE_LINE, fInjectionTime);
				print_FloatProgramShotTimeValue(PROGRAM_SETUP_SUBVALUE_POS, EXTRACTION_VALUE_LINE, fExtractionTime);
				print_FloatProgramShotTimeValue(PROGRAM_SETUP_SUBVALUE_POS, TOTAL_TIME_VALUE_LINE, (fInjectionTime+fExtractionTime));

			}
			else {

				float fWaterFlowQuantityFactor = get_FlowmeterFactor();
				float fInjectionFlow = float(rDataSet.stSetup.InjectionFlow[nProgramNum] * fWaterFlowQuantityFactor);/*WATERFLOW_QUANTITY_FACTOR);*/
				float fExtractionFlow = float(rDataSet.stSetup.ExtractionFlow[nProgramNum] * fWaterFlowQuantityFactor);/*WATERFLOW_QUANTITY_FACTOR);*/

				print_FloatProgramFlowQuantityValue(PROGRAM_SETUP_SUBVALUE_POS, INJECTION_VALUE_LINE, fInjectionFlow);
				print_FloatProgramFlowQuantityValue(PROGRAM_SETUP_SUBVALUE_POS, EXTRACTION_VALUE_LINE, fExtractionFlow);
				print_FloatProgramFlowQuantityValue(PROGRAM_SETUP_SUBVALUE_POS, TOTAL_TIME_VALUE_LINE, (fInjectionFlow+fExtractionFlow));

			}
		}
		break;

		case Menupage3:
		{
			unsigned int nTotFlowCount = rDataSet.stSetup.InjectionFlow[MANUAL_SET_ONE]+rDataSet.stSetup.ExtractionFlow[MANUAL_SET_ONE];
			float fTotShotTime = rDataSet.stSetup.InjectionTime[MANUAL_SET_ONE]+rDataSet.stSetup.ExtractionTime[MANUAL_SET_ONE];

			print_Char(MANUAL_SETUP_SUBVALUE_POS, SAVE_TO_PROG_NUM_VALUE_LINE, _strProgramNum[rDataSet.stSetup.ProgramSetNum]);
			print_FloatValue(MANUAL_SHOT_SUBVALUE_POS, MANUAL_SHOT_TIME_VALUE_LINE, fTotShotTime);
			print_FloatFlowQuantityValue(MANUAL_SHOT_SUBVALUE_POS, MANUAL_FLOW_QUANTITY_VALUE_LINE, float(nTotFlowCount * get_FlowmeterFactor()/*WATERFLOW_QUANTITY_FACTOR*/));

		}
		break;

		case Menupage4:

			print_String(INJECTION_SETUP_SUBVALUE_POS, INJECTION_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.InjectionMode]);
			print_IntegerValue(INJECTION_SETUP_SUBVALUE_POS, INJECTION_POWER_VALUE_LINE, _convert_float_to_decimal(rDataSet.stSetup.InjectionPower, 100ul));

		break;

		case Menupage5:

			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_TEMP_VALUE_LINE, rDataSet.stAcquisition.GroupTemp);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, SET_TEMP_VALUE_LINE, rDataSet.stSetup.SetValueTemp[TM4_GROUPHEAD_CHANNEL-1]);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_HEATING_MV_LINE, rDataSet.stAcquisition.GroupMV);

		break;

		case Menupage6:

			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_TEMP_VALUE_LINE, rDataSet.stAcquisition.BrewTemp);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, SET_TEMP_VALUE_LINE, rDataSet.stSetup.SetValueTemp[TM4_BREWING_CHANNEL-1]);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_HEATING_MV_LINE, rDataSet.stAcquisition.BrewMV);

		break;

		case Menupage7:

			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_TEMP_VALUE_LINE, rDataSet.stAcquisition.PreheatTemp);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, SET_TEMP_VALUE_LINE, rDataSet.stSetup.SetValueTemp[TM4_PREHEAT_CHANNEL-1]);
			print_FloatValue(BOILER_SETUP_SUBVALUE_POS, PRESENT_HEATING_MV_LINE, rDataSet.stAcquisition.PreheatMV);

		break;

		case Menupage8:

			print_IntegerValue(PUMP_SETUP_SUBVALUE_POS, MOTOR_POWER_VALUE_LINE, _convert_float_to_decimal(rDataSet.stSetup.MotorPower, 100ul));
#if AUTOMATION_SENSOR_ATTACHED
			print_String(PUMP_SETUP_SUBVALUE_POS, STANDBY_MODE_VALUE_LINE, _strStandbyMode[rDataSet.stSetup.MotorStandbyMode]);
#else
			print_String(PUMP_SETUP_SUBVALUE_POS, STANDBY_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.MotorStandbyMode]);
#endif
			print_String(PUMP_SETUP_SUBVALUE_POS, MOTOR_RUN_VALUE_LINE, _strRunStop[rDataSet.stSetup.MotorRun]);
			print_FloatPressureValue(PUMP_SETUP_SUBVALUE_POS, PUMP_PRESSURE_VALUE_LINE, rDataSet.stAcquisition.BrewPress);


		break;

		case Menupage9:

			print_String(LAMP_SETUP_SUBVALUE_POS, LAMP_MODE_VALUE_LINE, _strLampMode[rDataSet.stSetup.LampMode]);
			print_String(LAMP_SETUP_SUBVALUE_POS, BRIGHTNESS_VALUE_LINE, _strLowHigh[rDataSet.stSetup.LampIntensity]);

		break;

		case Menupage10:

			print_String(LEDPIXEL_SETUP_SUBVALUE_POS, PROGRAM1_INDICATOR_VALUE_LINE, _strLEDMode[rDataSet.stSetup.LedPixelMode[PROGRAM_SET_ONE]]);
			print_String(LEDPIXEL_SETUP_SUBVALUE_POS, PROGRAM2_INDICATOR_VALUE_LINE, _strLEDMode[rDataSet.stSetup.LedPixelMode[PROGRAM_SET_TWO]]);
			print_String(LEDPIXEL_SETUP_SUBVALUE_POS, MANUAL1_INDICATOR_VALUE_LINE, _strLEDMode[rDataSet.stSetup.LedPixelMode[MANUAL_SET_ONE]]);

		break;

		case Menupage11:

			if (_bSelectSetupTimeDateContents == false) {

				print_TimeDateValue(TIMEDATE_SETUP_YEAR_POS, DATE_VALUE_LINE, rTimeDate.year);
				print_TimeDateValue(TIMEDATE_SETUP_MONTH_POS, DATE_VALUE_LINE, rTimeDate.month);
				print_TimeDateValue(TIMEDATE_SETUP_DAY_POS, DATE_VALUE_LINE, rTimeDate.day);
				print_TimeDateValue(TIMEDATE_SETUP_HOUR_POS, TIME_VALUE_LINE, rTimeDate.hour);
				print_TimeDateValue(TIMEDATE_SETUP_MINUTE_POS, TIME_VALUE_LINE, rTimeDate.minute);
				print_TimeDateValue(TIMEDATE_SETUP_SECOND_POS, TIME_VALUE_LINE, rTimeDate.second);
				print_String(TIMEDATE_SETUP_WEEKDAY_POS, WDAY_VALUE_LINE, _strWeekday[rTimeDate.weekday]);

			}
			else {

				print_TimeDateValue(TIMEDATE_SETUP_YEAR_POS, DATE_VALUE_LINE, rDataSet.stSetup.Year);
				print_TimeDateValue(TIMEDATE_SETUP_MONTH_POS, DATE_VALUE_LINE, rDataSet.stSetup.Month);
				print_TimeDateValue(TIMEDATE_SETUP_DAY_POS, DATE_VALUE_LINE, rDataSet.stSetup.Day);
				print_TimeDateValue(TIMEDATE_SETUP_HOUR_POS, TIME_VALUE_LINE, rDataSet.stSetup.Hour);
				print_TimeDateValue(TIMEDATE_SETUP_MINUTE_POS, TIME_VALUE_LINE, rDataSet.stSetup.Minute);
				print_TimeDateValue(TIMEDATE_SETUP_SECOND_POS, TIME_VALUE_LINE, rDataSet.stSetup.Second);
				print_String(TIMEDATE_SETUP_WEEKDAY_POS, WDAY_VALUE_LINE, _strWeekday[rDataSet.stSetup.Weekday]);

			}

		break;

		case Menupage12:

			print_String(POWER_SAVING_MODE_SUBVALUE_POS, POWER_SAVING_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.PowerSaveMode]);
			print_TimeDateValue(POWER_SAVING_TIME_SETUP_HOUR_POS, BEGIN_TIME_VALUE_LINE, rDataSet.stSetup.StartHour);
			print_TimeDateValue(POWER_SAVING_TIME_SETUP_MINUTE_POS, BEGIN_TIME_VALUE_LINE, rDataSet.stSetup.StartMinute);
			print_TimeDateValue(POWER_SAVING_TIME_SETUP_HOUR_POS, END_TIME_VALUE_LINE, rDataSet.stSetup.EndHour);
			print_TimeDateValue(POWER_SAVING_TIME_SETUP_MINUTE_POS, END_TIME_VALUE_LINE, rDataSet.stSetup.EndMinute);

		break;

		case Menupage13:

			print_String(HEATER_SUBVALUE_POS, HEATER_SET_VALUE_LINE, _strHeater[rDataSet.stSetup.SetHeaterChannel]);

			if (rDataSet.stSetup.SetHeaterChannel <= TM4_GROUPHEAD_CHANNEL-1) {

				int nHeaterChannel = rDataSet.stSetup.SetHeaterChannel;

				print_FloatValue(HEATER_SUBVALUE_POS, P_BAND_VALUE_LINE, rDataSet.stSetup.ProportionalBandTemp[nHeaterChannel]);
				print_IntegralDerivativeValue(HEATER_SUBVALUE_POS, I_TIME_VALUE_LINE, rDataSet.stSetup.IntegralTime[nHeaterChannel]);
				print_IntegralDerivativeValue(HEATER_SUBVALUE_POS, D_TIME_VALUE_LINE, rDataSet.stSetup.DerivativeTime[nHeaterChannel]);

			}
			else {

				if (rDataSet.stSetup.SetHeaterChannel == TM4_MVHIGH_SETUP_CHANNEL-1) {

					print_FloatValue(HEATER_SUBVALUE_POS, PREHEAT_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_PREHEAT_CHANNEL-1]);
					print_FloatValue(HEATER_SUBVALUE_POS, BREW_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_BREWING_CHANNEL-1]);
					print_FloatValue(HEATER_SUBVALUE_POS, GROUPHEAD_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_GROUPHEAD_CHANNEL-1]);

				}
				else {

					print_String(HEATER_SUBVALUE_POS, INPUT_SENSOR_CH_LINE, _strTSensor[rDataSet.stSetup.TempSensorChannel]);
					print_FloatInputBiasValue(HEATER_SUBVALUE_POS, INPUT_BIAS_VALUE_LINE, rDataSet.stSetup.SensorInputBias[rDataSet.stSetup.TempSensorChannel]);

				}

			}

		break;

		case Menupage14:

			print_String(SENSOR_CHANNEL_SUBVALUE_POS, SENSOR_CHANNEL_VALUE_LINE, _strPSensor[rDataSet.stSetup.SetSensorChannel]);
			print_IntegerADCDigit(SENSOR_ADC_SUBVALUE_POS, SENSOR_CHANNEL_VALUE_LINE, rDataSet.stSetup.SensorADCDigit);

		break;

		case Menupage15:

			print_IntegerMotorAccTime(MOTOR_ACC_SUBVALUE_POS, MOTOR_ACC_TIME_VALUE_LINE, rDataSet.stSetup.MotorAccTime);
			print_IntegerValue(MOTOR_ACC_SUBVALUE_POS, MOTOR_POWER_CUR_VALUE_LINE, _convert_float_to_decimal(rDataSet.stSetup.MotorPower, 100ul));
			print_IntegerMotorAccTime(MOTOR_ACC_SUBVALUE_POS, MOTOR_ACC_TIME_EST_VALUE_LINE, rDataSet.stSetup.MotorAccTimeEST);

		break;

		case Menupage16:

#if AUTOMATION_SENSOR_ATTACHED
			print_String(AUTOMATION_MODE_SUBVALUE_POS, AUTOMATION_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.AutomationMode]);
			print_FloatAutoFlushingModeTimeValue(AUTOMATION_MODE_SUBVALUE_POS, AUTO_FLUSHING_TIME_VALUE_LINE, rDataSet.stSetup.AutoFlushingTime);
			if (rDataSet.stSetup.AutoBrewWaitTime > 0.0F)
				print_FloatAutoFlushingModeTimeValue(AUTOMATION_MODE_SUBVALUE_POS, AUTO_BREW_WAIT_TIME_VALUE_LINE, rDataSet.stSetup.AutoBrewWaitTime);
			else
				print_String(AUTOMATION_MODE_SUBVALUE_POS, AUTO_BREW_WAIT_TIME_VALUE_LINE, LCD_OFF_STRING);
#else
			print_String(FLUSHING_DETECTION_SUBVALUE_POS, FLUSHING_DETECTION_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.FlushingDetectionMode]);
			print_FloatFlushingDetectionTimeValue(FLUSHING_DETECTION_SUBVALUE_POS, FLUSHING_DETECTION_TIME_VALUE_LINE, rDataSet.stSetup.FlushingDetectionTime);
			print_FloatPressureValue(FLUSHING_DETECTION_SUBVALUE_POS, FLUSHING_DETECTION_PRES_VALUE_LINE, rDataSet.stSetup.FlushingDetectionPress);
#endif

		break;

		case Menupage17:

			print_String(MAINTENANCE_SUBVALUE_POS, MAINTENANCE_MODE_VALUE_LINE, _strOnOff[rDataSet.stSetup.MaintenanceMode]);
			print_LongValueUnsigned(MAINTENANCE_SUBVALUE_POS, TOTAL_FLOW_VALUE_LINE, rDataSet.stMaintenance.TotFlowQuantity);
			print_LongValueUnsigned(MAINTENANCE_SUBVALUE_POS, TOTAL_BREW_VALUE_LINE, rDataSet.stMaintenance.TotBrewCount);

			if (rDataSet.stMaintenance.RecentErrorCode) {

				print_ExceptionCode(MAINTENANCE_SUBVALUE_POS, ERROR_CODE_VALUE_LINE, rDataSet.stMaintenance.RecentErrorCode+0xE000);

			}
			else {

				print_String(MAINTENANCE_SUBVALUE_POS, ERROR_CODE_VALUE_LINE, LCD_NONE_STRING);

			}

		break;

		case Menupage18:

			print_String(RESET_SUBVALUE_POS, DEFAUT_SET_VALUE_LINE, _strYesNo[rDataSet.stSetup.DefaultSet]);

		break;

		case Menupage19:

			print_String(IFNORMATION_SUBVALUE_POS, HARDWARE_MODEL_NAME_VALUE_LINE, HARDWARE_MODELNAME);
			print_VersionCode(IFNORMATION_SUBVALUE_POS, HARDWARE_VERSION_VALUE_LINE, HARDWARE_VERSION, 0, HARDWARE_TYPEDEF);
			print_VersionCode(IFNORMATION_SUBVALUE_POS, SOFTWARE_VERSION_VALUE_LINE, FIRMWARE_VERSION, FIRMWARE_PATCH, FIRMWARE_TYPEDEF);

		break;

		case Menupage20:

			print_Char(PASSWORD0_SUBVALUE_POS, PASSWORD_VALUE_LINE, _strNumbers[rDataSet.stSetup.PasswordInput[0]]);
			print_Char(PASSWORD1_SUBVALUE_POS, PASSWORD_VALUE_LINE, _strNumbers[rDataSet.stSetup.PasswordInput[1]]);
			print_Char(PASSWORD2_SUBVALUE_POS, PASSWORD_VALUE_LINE, _strNumbers[rDataSet.stSetup.PasswordInput[2]]);
			print_Char(PASSWORD3_SUBVALUE_POS, PASSWORD_VALUE_LINE, _strNumbers[rDataSet.stSetup.PasswordInput[3]]);

		break;
	}

	hTextLCD.homeCursor();
}

void change_HeaterSetupMenupage(const bool bDisplayContents, const stDataSet_t& rDataSet)
{
	if (rDataSet.stSetup.SetHeaterChannel <= TM4_GROUPHEAD_CHANNEL-1) {

		print_String(0, 1, LCD_HEATER_PID_LINE1_TEMPLATE);
		print_String(0, 2, LCD_HEATER_PID_LINE2_TEMPLATE);
		print_String(0, 3, LCD_HEATER_PID_LINE3_TEMPLATE);

		if (bDisplayContents) {

			int nHeaterChannel = rDataSet.stSetup.SetHeaterChannel;

			print_FloatValue(HEATER_SUBVALUE_POS, P_BAND_VALUE_LINE, rDataSet.stSetup.ProportionalBandTemp[nHeaterChannel]);
			print_IntegralDerivativeValue(HEATER_SUBVALUE_POS, I_TIME_VALUE_LINE, rDataSet.stSetup.IntegralTime[nHeaterChannel]);
			print_IntegralDerivativeValue(HEATER_SUBVALUE_POS, D_TIME_VALUE_LINE, rDataSet.stSetup.DerivativeTime[nHeaterChannel]);

		}

	}
	else {

		if (rDataSet.stSetup.SetHeaterChannel == TM4_MVHIGH_SETUP_CHANNEL-1) {
		
			print_String(0, 1, LCD_HEATER_MV_LINE1_TEMPLATE);
			print_String(0, 2, LCD_HEATER_MV_LINE2_TEMPLATE);
			print_String(0, 3, LCD_HEATER_MV_LINE3_TEMPLATE);

			if (bDisplayContents) {

				print_FloatValue(HEATER_SUBVALUE_POS, PREHEAT_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_PREHEAT_CHANNEL-1]);
				print_FloatValue(HEATER_SUBVALUE_POS, BREW_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_BREWING_CHANNEL-1]);
				print_FloatValue(HEATER_SUBVALUE_POS, GROUPHEAD_MV_VALUE_LINE, rDataSet.stSetup.MVHighLimit[TM4_GROUPHEAD_CHANNEL-1]);

			}

		}
		else { //rDataSet.stSetup.SetHeaterChannel == TM4_INPUT_BIAS_SETUP_CHANNEL-1

			print_String(0, 1, LCD_HEATER_BIAS_LINE1_TEMPLATE);
			print_String(0, 2, LCD_HEATER_BIAS_LINE2_TEMPLATE);
			print_String(0, 3, LCD_HEATER_BIAS_LINE3_TEMPLATE);

			if (bDisplayContents) {

				int nTempSensorChannel = rDataSet.stSetup.TempSensorChannel;

				print_String(HEATER_SUBVALUE_POS, INPUT_SENSOR_CH_LINE, _strTSensor[nTempSensorChannel]);
				print_FloatInputBiasValue(HEATER_SUBVALUE_POS, INPUT_BIAS_VALUE_LINE, rDataSet.stSetup.SensorInputBias[nTempSensorChannel]);

			}

		}

	}
}

