/* * * * * * * * * * * * * * * * * * * * * * * * * *
 * ProjectionBall Firmware						   *
 * Copyright (c) 2017  							   *
 * K.Watanabe,Crescent 							   *
 * Released under the MIT license 				   *
 * http://opensource.org/licenses/mit-license.php  *
 * 17/06/16 v1.0 Initial Release                   *
 * 												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f3xx_hal.h"
#include "data.h"
//#include "ff.h"
#include "main.h"

//#define Enable_POS_DEBUG

//#define FORCE_MOTOR_DIS
//#define FORCE_MOTOR1_DIS
//#define FORCE_MOTOR2_DIS

//#define Enable_CODE_PROTECT	//Block to read the bin data from STLINK

#define VR_READ_FROM_SDADC

//#define TLE5012B				//Not Worked
//#define PCEN0	2650
//#define PCEN1	5550
//#define CENOFFSET	1024
//#define EnableCurrentControl	//Not Worked

#define Enable_TSC //Touch Sensing

/*Error Stop Function*/
#define ERROR_Stop_EN

//#define V3	//AS5047D V3 Only Test
#define V2	//AS5048A V2&V5

#ifdef EnableCurrentControl
	#define KCP	1.0f
	#define KCI	0.01f
	#define ADC2DUTY	1
#endif

#ifndef TLE5012B
	#define KP	4
	#define KD	100
	#define KI  0
#else
	#define KP	0.1
	#define KD	3
	#define KI  0
#endif


#ifdef V3
	#define AS5047D
	#define PCEN0	10600
	#define PCEN1	6500
	#define CENOFFSET	682
#endif
#ifdef V2
	#define AS5048A
	#define PCEN0	6200
	#define PCEN1	10300
	#define CENOFFSET	1024
#endif

#ifdef TLE5012B
	#define PCEN0	1950
	#define PCEN1	6350
	#define CENOFFSET	680
#endif

#define PMAX0	PCEN0+2000
#define PMIN0	PCEN0-2000
#define PMAX1	PCEN1+2000
#define PMIN1	PCEN1-2000


#define SaveStrData


//#define EN_PID_I
#define PseudoD
//#define DiffD

#define omega 1
#define w	1.4f//1.4->AS5047D
#define st	1

#define AMP				80

#define CMPRATE			3	//compression rate
#define MAXCMD			(380/CMPRATE)	//Max Amplitude
#define LSRMAXCMD		50

#define MAXDUTY 0x0400
#define LSD_Offset 4//5
#define TRIFUNCLEN		120

#define POS_IN_MAX	60//60-->50
#define POS_IN_MIN	-POS_IN_MAX



#define SWCOUNT		1500
#define DataStrLen	32

/* TSC Setting*/
#ifdef Enable_TSC
	#define TSC_TH	100//80
	enum{TSC_Free,TSC_Touched};

	extern int16_t TscBuf[2][10];
	extern int16_t TscAve[2][2];
	extern int8_t TscFlag[2];
#endif

enum{	TSC_DATANONE,//No RTCMem Data-> First Power ON
		TSC_ON,
		TSC_OFF};


void MainControl(void);

void StatusLoop(void);
uint16_t GetEncPos(uint8_t ch);

void SetTauRef(int16_t cmd0, int16_t cmd1);
uint16_t GetADCVal(uint8_t ch);
void InitSDADC();
void ChkSWStatus(uint8_t *ptsw, uint8_t *mdsw, uint16_t *ptcnt, uint16_t *mdcnt);



int8_t ChkUart();
void StartControl();
void InitParam();
void InitCenPos(void);
void GetSDData();

void setpath(struct path* pR, char* str, unsigned int num);
void setpath2(struct fpath* pR, char* str, unsigned int num);
void PosCmdChk(int8_t *cmd);
void LsrCmdChk(int8_t *cmd, unsigned int num);
void InitSDIO();
void SetEncParam(uint8_t ch);
void CurrentControl();
void ChkTSCStatus(uint8_t *ptsw, uint8_t *mdsw);
int16_t GetTSCVal(uint8_t ch);

void LoadStrRTCMem();
void SaveStrRTCMem();
void LoadDegRTCMem();
void SaveDegRTCMem();
void SetTSCRTCMem();
void EnableCodeProtect();
void DisableCodeProtect();


#define MODE_MAXNUM		8
#pragma once
enum{
	enum_ANIMATION,	//0
	enum_ROTATION,	//1
	enum_ONESTROKE,	//2
	enum_ALWAYS,	//3
	enum_AWATCH,	//4
	enum_DWATCH,	//5
	enum_DATE,		//6
	enum_STRING		//7
};

#define PTTRN_MAXNUM	9
#pragma once
enum{
	enum_STAR,
	enum_ARROW,
	enum_MAIL,
	enum_SMILE,
	enum_HEART,
	enum_SUN,
	enum_CLOUD,
	enum_RAIN,
	enum_SNOW,
	enum_THUNDER
};

#pragma once
enum{
	enum_RTCMEM_MODE,
	enum_RTCMEM_PTTRN,
	enum_RTCMEM_StrLen,
	enum_RTCMEM_Angle,
	enum_RTCMEM_TSC,
	enum_RTCMEM_StrTop
//RTC_BKP_DR0 (uint32_t)0x00000000
};

/*  Uart String Definition  */
#define Str_Switch_Mode		"swm"
#define Str_Uart_Mode		"urm"

#define Str_Start			"srt"
#define Str_Stop			"stp"

#define Str_Frame_Mode		"frm"
#define Str_SW0				"sw0"
#define Str_SW1				"sw1"
#define Str_SW2				"sw2"
#define Str_SW3				"sw3"
#define Str_SW4				"sw4"
#define Str_SW5				"sw5"
#define Str_SW6				"sw6"
#define Str_SW7				"sw7"
#define Str_SW8				"sw8"
#define Str_SW9				"sw9"

#define Str_Rotation		"rot"
#define Str_Stroke			"str"
#define Str_Always			"alw"
#define Str_AnalogWatch		"awt"
#define Str_DigitalWatch	"dwt"
#define Str_Day				"dym"
#define Str_StringMode		"stm"
#define Str_TSC_Change		"tsc"

#define Str_SD_Mode			"sdm"
#define Str_Internal_Mode	"inm"

#define Str_Center_Pos		"cen="
#define Str_Angle_Deg		"deg="
#define Str_Debug_Mode		"dbg"
#define Str_Restart			"rst"
#define Str_FrameNo			"fno="
#define Str_FramePause		"fpa"
#define Str_GetString		"stg="

#define Str_SetTime			"tim="
#define Str_GetTime			"tim?"
#define Str_SetDate			"day="
#define Str_GetDate			"day?"
#define Find_String			"=,"

#define Str_SetDData		"sdd="	//Set Draw Data
#define Str_SetDNum			"sdn="	//Set Draw Num
#define Str_SetFData		"sfd="  //Set Frame Data
#define Str_SetFNum			"sfn="  //Set Frame Num

#define Str_GetDData		"gdd="	//Get Draw Data
#define Str_GetDNum			"gdn="	//Get Draw Num
#define Str_GetFData		"gff="  //Get Frame Data
#define Str_GetFNum			"gfn="  //Get Frame Num
#define Str_ForceLaserON	"flo"

#define GetStringMaxLen	6
#define ScrollDispLen	5




	extern RTC_TimeTypeDef gTime;
	extern RTC_DateTypeDef gDate;


	//UART Display
	extern uint8_t  uart_out;
	extern uint8_t SW_Status;
	extern uint8_t PAT_Status;
	extern uint16_t Init_Count;

	//extern int16_t duty0, duty1,duty2,duty3;
	extern int16_t OUT0, OUT1;
	extern uint16_t x_bit0,x_bit1,x_bit2,x_bit3,x_bit4,x_bit5,x_bit6,x_bit7;

	extern int16_t x_res0,dx_res0,x_err0,x_sum0,x_cmd0,x_cmd2,dx_cmd0;
	extern int16_t x_res1,dx_res1,x_err1,x_sum1,x_cmd1,x_cmd3,dx_cmd1;
	extern int16_t dx_tmp0, dx_tmp1;

	extern int16_t cnt0,cnt1,cnt2,cnt3;
	extern int16_t tau_cmd0, tau_cmd1;
	extern int16_t x_cen0, x_cen1;


	extern char RxWptr,RxRptr;
	extern char RxWbuf[40],RxRbuf[40];


	extern uint8_t allp_num[DATA_NUM];

	extern uint8_t allf_num[FRAME_NUM];

	extern uint8_t  LSD_num;

	extern int16_t deg0, deg1;
	extern int16_t rdeg;
	extern int8_t rflag;
	extern int16_t angle;
	extern int16_t u_angle;
	extern int8_t SD_mode;//0:Internal memory 1:SD memory
	extern int8_t UART_mode;//0:SW Mode 1:Uart Mode
	extern int8_t Start_EN;//0:OFF 1:ON
	extern int8_t Debug_mode;//0:OFF,1:ON
	extern int8_t Frame_EN;//0:OFF,1:ON
	extern int8_t Force_LSR_ON;

	extern uint8_t frameNum;
	extern uint8_t NextFlag;
	extern int16_t p_cmd0, p_cmd1, p_cmd2, p_cmd3;
	extern uint16_t Err_cnt0, Err_cnt1;

	extern char allstr[DataStrLen];
	extern uint8_t allstrlen;
	extern uint8_t allstrcnt;

	extern char GetAllStr[DataStrLen];
	extern uint8_t GetAllStrLen;
	extern uint8_t GetAllStrShft;
	extern int8_t AllStrShftFlag;

	extern int8_t dwatchstr[8];
	extern uint8_t dwatchstrlen;
	extern uint8_t dwatchstrcnt;


	extern uint8_t allwatcnt;

	extern int8_t watflg;
	extern int8_t wat_h;
	extern int8_t wat_m;

	extern int8_t time_h;
	extern int8_t time_m;
	extern int8_t time_s;
	extern int8_t date_d;
	extern int8_t date_m;
	extern int8_t date_y;
	extern int16_t time_us;//10000

	extern volatile int8_t UART1_Data[8];
	extern volatile int8_t UART2_Data[8];

	extern int16_t I_sum0,I_sum1;
	extern int16_t outA,outB;
	extern int16_t IsenA,IsenB;
	extern int16_t IerrA,IerrB;

	extern uint32_t InjChannel;
	extern int16_t InjectedConvData[2];
	extern uint8_t InjCount;

	extern uint8_t TSC_Enable;
