#ifndef __LINUX_SSD253X_TS_H__
#define __LINUX_SSD253X_TS_H__

/**************************************************************
使用前注意通道数，驱动默认使用通道是sense 
大于drive否则需要将使用到的DRIVENO与SENSENO调换
此情况包括0x66和0x67寄存器，但不必修改。
***************************************************************/

#define DRIVENO	15
#define SENSENO	10
#define ENABLE_INT		2	// 0->Polling, 1->Interupt, 2->Hybrid
#define EdgeDisable		1	// if Edge Disable, set it to 1, else reset to 0, OR  SSD2533 set 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	25000000// 100Hz - 10,000,000us
#define FINGERNO		5
#define MAX_TOUCH_MAJOR		10		//Charles added
#define MAX_WIDTH_MAJOR		15		//Charles added
#define MAX_TRACKID_ITEM		10	//Charles added

#define REPORT_TOUCH_MAJOR		3		//Charles added
#define REPORT_WIDTH_MAJOR		4		//Charles added
#define SSDS53X_SCREEN_MAX_X    CONFIG_FB_OSD1_DEFAULT_WIDTH
#define SSDS53X_SCREEN_MAX_Y    CONFIG_FB_OSD1_DEFAULT_HEIGHT
#define SSD253x_REPORT_COORD_ORIGIN		0x06
#define SSD253x_REPORT_RATE_TIME		0X0F

#define SSD253x_TOUCH_KEY
#undef SSD253x_TOUCH_KEY

struct ts_platform_data{
	int irq_no;
	u32 irq_gpio_no;
	u32 reset_gpio_no;
	u32 power_gpio_no;
};

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

#define RST_PIN_RESUME		//if define, use Reset Pin to Resume the TP
#ifdef SSD253x_TOUCH_KEY
	static uint32_t key_code[4] = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH }; 
#endif


#define SSD253x_CUT_EDGE    //0x8b must be 0x00;  EdgeDisable set 0
//#undef  SSD253x_CUT_EDGE

#ifdef SSD253x_CUT_EDGE
		#define XPOS_MAX (DRIVENO -EdgeDisable) *64
		#define YPOS_MAX (SENSENO -EdgeDisable) *64
#endif



#endif
