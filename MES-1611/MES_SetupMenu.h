#ifndef _MES_SETUPMENU_H_
#define _MES_SETUPMENU_H_


#include "MES_Arduino.h"
#include "MES_Peripheral_Interface.h"
#include "MES_DataSet_Definition.h"


////////////////////////////////////////////////////////////
// TYPE-DEFINITION FOR SETUP MENU FRAMEWORKS

typedef enum eLCDView {DashboardView=0, MenuboardView, MenupageView, MessageView, AlarmView, CleaningView};

typedef enum eDashboard {DashboardA=0, DashboardB, DashboardC};

typedef enum eMenuBoard {
	Menuboard1=0,
	Menuboard2,
	Menuboard3,
	Menuboard4,
	Menuboard5
};

typedef enum eMenuPage {
	Menupage1=0, Menupage2, Menupage3, Menupage4,
	Menupage5, Menupage6, Menupage7, Menupage8,
	Menupage9, Menupage10, Menupage11, Menupage12,
	Menupage13, Menupage14, Menupage15, Menupage16,
	Menupage17, Menupage18, Menupage19, Menupage20
};


////////////////////////////////////////////////////////////
// DEFINITION OF TEXT LCD DISPLAY

#define TEXT_LCD_INVALIDATE_PERIOD1					(50UL)								// < 100 msec
#define TEXT_LCD_INVALIDATE_PERIOD2					(500UL)								// 500 msec
#define TEXT_LCD_INVALIDATE_PERIOD3					(1000UL)							// 1 sec
#define TEXT_LCD_INVALIDATE_PERIOD4					(2000UL)							// 2 sec

#define LCDVIEW_NUM									(4)
#define DASHBOARD_NUM								(5)
#define MENUBOARD_NUM								(5)
#define MENUPAGE_NUM								(20)

#define DEFAULT_LCDVIEW 							(DashboardView)
#define DEFAULT_DASHBOARD							(DashboardA)
#define DEFAULT_MENUBOARD							(Menuboard1)
#define DEFAULT_MENUPAGE							(Menupage1)


////////////////////////////////////////////////////////////
// INTERFACES FOR TEXT LCD DISPLAY

void initialize_TextLCDParameters(void);

void set_TextLCDView(const eLCDView nView);
void set_TextLCDDashboard(const eDashboard nDashboard);
void set_TextLCDMenuboard(const eMenuBoard nMenuboard);
void set_TextLCDMenupage(const eMenuPage nMenupage);
eLCDView get_TextLCDView(void);
eDashboard get_TextLCDDashboard(void);
eMenuBoard get_TextLCDMenuboard(void);
eMenuPage get_TextLCDMenupage(void);
void set_SelectCursorPosition(const unsigned char nPos, const unsigned char nLine);
void get_SelectCursorPosition(unsigned char& nPos, unsigned char& nLine);
void set_SelectCursorPos(const unsigned char nPos);
unsigned char get_SelectCursorPos(void);
void set_SelectCursorLine(const unsigned char nLine);
unsigned char get_SelectCursorLine(void);
void set_ChangingTextLCD(const bool bChanging);
bool get_ChangingTextLCD(void);
void set_LeavingMenupage(const bool bLeaving);
bool get_LeavingMenupage(void);
void set_SelectedCursor(const bool bSelected);
bool get_SelectedCursor(void);
void set_DefaultResetString(const bool bResetString);
void set_AuthorizedString(const bool bAuthorizedString);
bool get_AuthorizedString(void);
void set_NoSelectionPage(const bool bNoSelectCursor);
void select_SetupTimeDateContents(const bool bSetupContents);

void display_SystemInitialization(void);
void display_SystemFactoryReset(void);
void display_SystemInformation(void);
void display_TM4CommunicationSettingPass(void);
void display_TM4CommunicationSettingFail(void);
void display_SensorAdjFailNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void display_SensorZeroShiftNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void display_SensorSpanNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void display_SensorInitNoticeMessage(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void display_ErrorMessage(const stMaintenanceDataSet_t& rMaintenance);
void display_NoticeMessage(const stMaintenanceDataSet_t& rMaintenance);
void display_EntryMessage(const bool bDashboard);
void display_MenupageExitMessage(const bool bSaveMsg);
void display_CleaningEntryMessage(const bool bCondition);
void display_CleaningReadyMessage(void);
void display_CleaningModeMessage(void);
void display_SystemDashboard(const bool bDisplayContents, const stDataSet_t& rDataSet);
void display_SystemMenuboard(void);
void display_SystemMenupage(const bool bDisplayContents, const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void invalidate_TextLCD(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);

void display_SelectCursor(void);

void display_DashboardTitle(void);
void display_DashboardContents(const stDataSet_t& rDataSet);

void display_SetupMenuBoard(void);

void display_SetupMenuPageTitle(void);
void display_SetupMenuPageContents(const stDataSet_t& rDataSet, const stTimeDate_t& rTimeDate);
void change_HeaterSetupMenupage(const bool bDisplayContents, const stDataSet_t& rDataSet);


#endif//_MES_SETUPMENU_H_

