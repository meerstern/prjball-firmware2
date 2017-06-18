/* * * * * * * * * * * * * * * * * * * * * * * * * *
 * ProjectionBall Firmware						   *
 * Copyright (c) 2017  							   *
 * K.Watanabe,Crescent 							   *
 * Released under the MIT license 				   *
 * http://opensource.org/licenses/mit-license.php  *
 * 17/06/16 v1.0 Initial Release                   *
 * 												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "control.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include "data.h"
#include "font.h"
#include "fontcontrol.h"
#include "string.h"
#include "stdlib.h"
#include "ff.h"

FATFS FatFs;		/* FatFs work area  */
FIL Fil;			/* File object */


	RTC_TimeTypeDef gTime;
	RTC_DateTypeDef gDate;

	//UART Display
	uint8_t  uart_out;
	uint8_t SW_Status;
	uint8_t PAT_Status;
	uint16_t Init_Count;

	//int16_t duty0, duty1,duty2,duty3;
	int16_t OUT0, OUT1;
	uint16_t x_bit0,x_bit1,x_bit2,x_bit3,x_bit4,x_bit5,x_bit6,x_bit7;

	int16_t x_res0,dx_res0,x_err0,x_sum0,x_cmd0,x_cmd2,dx_cmd0;
	int16_t x_res1,dx_res1,x_err1,x_sum1,x_cmd1,x_cmd3,dx_cmd1;
	int16_t dx_tmp0, dx_tmp1;


	int16_t cnt0,cnt1,cnt2,cnt3;
	int16_t tau_cmd0, tau_cmd1;
	int16_t x_cen0, x_cen1;


	char RxWptr,RxRptr;
	char RxWbuf[40],RxRbuf[40];


	uint8_t allp_num[DATA_NUM];

	uint8_t allf_num[FRAME_NUM];

	uint8_t  LSD_num;


	int16_t deg0, deg1;
	int16_t rdeg;
	int8_t rflag;
	int16_t angle;
	int16_t u_angle;
	int8_t SD_mode;//0:Internal memory 1:SD memory
	int8_t UART_mode;//0:SW Mode 1:Uart Mode
	int8_t Start_EN;//0:OFF 1:ON
	int8_t Debug_mode;//0:OFF,1:ON
	int8_t Frame_EN;//0:OFF,1:ON
	int8_t Force_LSR_ON;

	uint8_t frameNum;
	uint8_t NextFlag;

	int16_t p_cmd0, p_cmd1, p_cmd2, p_cmd3;

	uint16_t Err_cnt0, Err_cnt1;
	uint8_t LsrPort;

	char allstr[DataStrLen];
	uint8_t allstrlen;
	uint8_t allstrcnt;

	char GetAllStr[DataStrLen];
	uint8_t GetAllStrLen;
	uint8_t GetAllStrShft;
	int8_t AllStrShftFlag;

	int8_t dwatchstr[8];
	uint8_t dwatchstrlen;
	uint8_t dwatchstrcnt;


	uint8_t allwatcnt;

	int8_t watflg;
	int8_t wat_h;
	int8_t wat_m;

	int8_t time_h;
	int8_t time_m;
	int8_t time_s;
	int8_t date_d;
	int8_t date_m;
	int8_t date_y;
	int16_t time_us;//10000

	volatile int8_t UART1_Data[8];
	volatile int8_t UART2_Data[8];

	enum{ ADC_BUFFER_LENGTH = 2 };
	int16_t ADCBuffer[ADC_BUFFER_LENGTH];
	int16_t I_sum0,I_sum1;
	int16_t outA,outB;
	int16_t IsenA,IsenB;
	int16_t IerrA,IerrB;

	int16_t TscBuf[2][10];
	int16_t TscAve[2][2];
	int8_t TscFlag[2];

	uint32_t InjChannel = 0;
	int16_t InjectedConvData[2];
	uint8_t InjCount;

	uint8_t TSC_Enable;

void MainControl3(void){

	// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0x100);
}


void MainControl(void){


	HAL_RTC_GetTime(&hrtc, &gTime, FORMAT_BIN);

	time_s=gTime.Seconds;
	time_m=gTime.Minutes;
	time_h=gTime.Hours;

	int sum0;
	int sum1;


	//dx buffer
	x_bit6=x_bit4; x_bit7=x_bit5;
	x_bit4=x_bit2; x_bit5=x_bit3;
	x_bit2=x_bit0; x_bit3=x_bit1;

	x_bit0=(GetEncPos(1));


	x_bit1=(GetEncPos(2));


	sum0=(x_bit0+x_bit2+x_bit4+x_bit6);
	sum1=(x_bit1+x_bit3+x_bit5+x_bit7);
#ifndef TLE5012B
	x_res0=sum0/4;
	x_res1=sum1/4;
#else
	x_res0=sum0/8;
	x_res1=sum1/8;
#endif





	GetCmdPos(&p_cmd0,&p_cmd1 ,&LsrPort);

	HAL_GPIO_WritePin(LSRCTR_GPIO_Port, LSRCTR_Pin,LsrPort);


	p_cmd2=p_cmd0;
	p_cmd3=p_cmd1;

	p_cmd0=p_cmd2*cose[u_angle]-p_cmd3*sine[u_angle];
	p_cmd1=p_cmd2*sine[u_angle]+p_cmd3*cose[u_angle];

	//PAT_Status==1 -> Rotate Mode
	if(PAT_Status==1){
#ifdef AS5048A
		x_cmd0=(int)(x_cen0-p_cmd0*cose[rdeg]);
		x_cmd1=(int)(x_cen1-p_cmd1);
#else
		x_cmd0=(int)(x_cen0+p_cmd0*cose[rdeg]);
		x_cmd1=(int)(x_cen1+p_cmd1);
#endif

	}
	else{
#ifdef AS5048A
		x_cmd0=x_cen0-p_cmd0;
		x_cmd1=x_cen1-p_cmd1;
#else
		x_cmd0=x_cen0+p_cmd0;
		x_cmd1=x_cen1+p_cmd1;
#endif


	}

	dx_cmd0=0;
	dx_cmd1=0;

	x_err0=x_cmd0-x_res0;
	x_err1=x_cmd1-x_res1;

#ifdef ERROR_Stop_EN
	if(Start_EN==1){
		if(x_err0>300||x_err0<-300){
			Err_cnt0++;
		}
		else{Err_cnt0=0;}
		if(x_err1>300||x_err1<-300){
			Err_cnt1++;
		}
		else{Err_cnt1=0;}
	}else{
		Err_cnt0=0;
		Err_cnt1=0;
	}
#endif
	tau_cmd0=KP*x_err0+KI*x_sum0+KD*(dx_cmd0-dx_res0);
	tau_cmd1=KP*x_err1+KI*x_sum1+KD*(dx_cmd1-dx_res1);

	OUT0=tau_cmd0;
	OUT1=tau_cmd1;

	//Repeat Count
	if(POS_IN_MIN<x_err0&&x_err0<POS_IN_MAX &&POS_IN_MIN<x_err1&&x_err1<POS_IN_MAX){
		cnt1++;cnt3++;
	}
	RepeatChk();

	SetTauRef(OUT0,OUT1);

#ifdef DiffD
	dx_res0=(x_bit0-x_bit2);
	dx_res1=(x_bit1-x_bit3);
#endif

#ifdef PseudoD
	dx_tmp0 += dx_res0*st;
	dx_res0 = w*(x_res0 - dx_tmp0);

	dx_tmp1 += dx_res1*st;
	dx_res1 = w*(x_res1 - dx_tmp1);
#endif


	if(Init_Count<1000)Init_Count++;

}

/*						*
 * 		Status LOOP		*
 * 						*/
void StatusLoop(void){
	//Display
		uart_out=1;
		NextFlag=1;

		HAL_RTC_GetDate(&hrtc, &gDate, FORMAT_BIN);
		date_d=gDate.Date;
		date_m=gDate.Month;
		date_y=gDate.Year;

		if(SD_mode==1){

		}else{

			if(PAT_Status==0){
				if(rdeg>12){
					//printf("Lissajous orbit X:sin(%d/72), Y: cos(%d/72)\n",cnt0,cnt2);
				}
			}
		}



		if(GetStringMaxLen<GetAllStrLen){

			for(int a=0;a<DataStrLen;a++)allstr[a]=' ';

			if(0<=GetAllStrShft && GetAllStrShft<GetStringMaxLen){
				strncpy(	&allstr[GetStringMaxLen-GetAllStrShft-1],
							&GetAllStr[0],
							GetAllStrShft+1);
			}
			else if(GetStringMaxLen<=GetAllStrShft && GetAllStrShft<(GetAllStrLen+1) ){
				strncpy(	allstr,
							&GetAllStr[GetAllStrShft-GetStringMaxLen+1],
							GetStringMaxLen);
			}
			else if((GetAllStrLen+1)<=GetAllStrShft && GetAllStrShft<=(GetAllStrLen+GetStringMaxLen-1) ){
				strncpy(	allstr,
							&GetAllStr[GetAllStrShft-GetStringMaxLen+1],
							GetAllStrLen+GetStringMaxLen-GetAllStrShft-1);
			}
			//printf("Shift IN:%d,%d\n",GetAllStrLen,GetAllStrShft);
			if(AllStrShftFlag==0)GetAllStrShft++;
			if((GetAllStrLen+GetStringMaxLen-1)<GetAllStrShft)GetAllStrShft=0;
		}

		AllStrShftFlag++;
		if(AllStrShftFlag>1)AllStrShftFlag=0;



}

void InitCenPos(void){
#if defined	AS5047D || defined AS5048A
	#ifndef VR_READ_FROM_SDADC
		x_cen0=PCEN0+(GetADCVal(1)/3-682);
		x_cen1=PCEN1+(GetADCVal(0)/3-682);
		printf("Center Pos X:%d, Y:%d\n",x_cen0,x_cen1);
	#else
		//ADC Center Position Read
		x_cen0=PCEN0;
		x_cen1=PCEN1;
		InitSDADC();
	#endif

#endif


#ifdef	TLE5012B
		//ADC Center Position Read
		x_cen0=PCEN0;
		x_cen1=PCEN1;
		printf("Center Pos X:%d, Y:%d\n",x_cen0,x_cen1);
#endif



}

/*						*
 * 		GET Enc POS 	*
 * 						*/
uint16_t GetEncPos(uint8_t ch){

	uint8_t	sData[2];
	uint8_t rData[2];
	uint16_t res;

#if defined(AS5048A) || defined(AS5047D)

	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,0);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS2_GPIO_Port, ENC_CS2_Pin,0);

	sData[0]=0xFF;sData[1]=0xFF;
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);

	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,1);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS2_Pin,1);

	res =(rData[0]<<8)+rData[1];
	return  0x3FFF&res;
#endif

#ifdef TLE5012B

	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,0);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS2_GPIO_Port, ENC_CS2_Pin,0);

	sData[0]=0x80;sData[1]=0x21;
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0x00;sData[1]=0x00;
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);

	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,1);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS2_Pin,1);

	//if(rData[0]&0x80==0)printf("Old Data! \n");
	res =(rData[0]<<8)+rData[1];

	return  0x7FFF&res;

#endif


}




/*						*
 * 		GET ADC Value 	*
 * 						*/
uint16_t GetADCVal(uint8_t ch){

	ADC_ChannelConfTypeDef sConfig;
	uint16_t adcValue=0;

	if(ch==0){
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	}
	else if(ch==1){
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 2;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	}


	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		//Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		adcValue = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	return adcValue;

}

void InitSDADC(){

	InjCount=0;
	 if (HAL_SDADC_AssociateChannelConfig(	&hsdadc2,
			 	 	 	 	 	 	 	 	SDADC_CHANNEL_7|SDADC_CHANNEL_8,
											SDADC_CONF_INDEX_0) != HAL_OK){
		 	 printf("ERROR:  HAL_SDADC_AssociateChannelConfig");
	 }

     if (HAL_SDADC_InjectedConfigChannel(	&hsdadc2,
    		  	  	  	  	  	  	  	  	SDADC_CHANNEL_7|SDADC_CHANNEL_8,
											SDADC_CONTINUOUS_CONV_ON) != HAL_OK){
	         printf("ERROR:  HAL_SDADC_InjectedConfigChannel");
	 }

     if (HAL_SDADC_SelectInjectedTrigger(	&hsdadc2,
    		 	 	 	 	 	 	 	 	SDADC_SOFTWARE_TRIGGER) != HAL_OK){
    	   	 printf("ERROR:  HAL_SDADC_SelectInjectedTrigger");
	 }

	 if (HAL_SDADC_CalibrationStart(		&hsdadc2,
			 	 	 	 	 	 	 	 	SDADC_CALIBRATION_SEQ_2) != HAL_OK){
		     printf("ERROR:  HAL_SDADC_CalibrationStart");
	 }

	 if (HAL_SDADC_PollForCalibEvent(		&hsdadc2,
			 	 	 	 	 	 	 	 	HAL_MAX_DELAY) != HAL_OK){
	 	   	 printf("ERROR:  HAL_SDADC_PollForCalibEvent");
	 }

	 if (HAL_SDADC_InjectedStart_IT(		&hsdadc2) != HAL_OK){
		    printf("ERROR:  HAL_SDADC_InjectedStart_IT");
	 }


}



#ifdef EnableCurrentControl

void StartCurrentControl(){

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCBuffer, ADC_BUFFER_LENGTH);
	HAL_TIM_Base_Start_IT(&htim19);
	//printf("Tim19 Init\n");
}

void CurrentControl(){

	IsenA=(ADCBuffer[0])/ADC2DUTY/1;
	IsenB=(ADCBuffer[1])/ADC2DUTY/1;
	//IsenA=IsenA+1;
	if(OUT0>=0){
		IerrA=OUT0-IsenA;
		I_sum0=I_sum0+IerrA;
		outA=KCP*(IerrA+KCI*I_sum0);

	}else{
		IerrA=-OUT0-IsenA;
		I_sum0=I_sum0+IerrA;
		outA=-KCP*(IerrA+KCI*I_sum0);
	}

	if(OUT1>=0){
		IerrB=OUT1-IsenB;
		I_sum1=I_sum1+IerrB;
		outB=KCP*(IerrB+KCI*I_sum1);

	}else{
		IerrB=-OUT1-IsenB;
		I_sum1=I_sum1+IerrB;
		outB=-KCP*(IerrB+KCI*I_sum1);

	}

	SetTauRef(outA,outB);
}
#endif



void SetTauRef(int16_t cmd0, int16_t cmd1){

	int16_t duty0,duty1;
	//M1
	if(cmd0>0){
		duty0=cmd0;
#ifdef AS5048A
		GPIOB->BRR= A1_Pin;
		GPIOB->BSRR= A2_Pin;
#endif
#ifdef AS5047D
		GPIOB->BSRR= A1_Pin;
		GPIOB->BRR= A2_Pin;
#endif
#ifdef TLE5012B
		GPIOB->BRR= A1_Pin;
		GPIOB->BSRR= A2_Pin;
#endif

	}else if(cmd0<0){
		duty0=-cmd0;
#ifdef AS5048A
		GPIOB->BSRR= A1_Pin;
		GPIOB->BRR= A2_Pin;
#endif
#ifdef AS5047D
		GPIOB->BRR= A1_Pin;
		GPIOB->BSRR= A2_Pin;
#endif
#ifdef TLE5012B
		GPIOB->BSRR= A1_Pin;
		GPIOB->BRR= A2_Pin;
#endif

	}else{
		duty0=0;
		GPIOB->BRR= A1_Pin;
		GPIOB->BRR= A2_Pin;
	}

	if(duty0>MAXDUTY)duty0=MAXDUTY;
	if(Start_EN!=1)duty0=0;

	//M2
	if(cmd1>0){
		duty1=cmd1;
#ifdef AS5048A
		GPIOB->BSRR= B1_Pin;
		GPIOB->BRR= B2_Pin;
#endif
#ifdef AS5047D
		GPIOB->BRR= B1_Pin;
		GPIOB->BSRR= B2_Pin;
#endif
#ifdef TLE5012B
		GPIOB->BSRR= B1_Pin;
		GPIOB->BRR= B2_Pin;
#endif


	}else if(cmd1<0){
		duty1=-cmd1;
#ifdef AS5048A
		GPIOB->BRR= B1_Pin;
		GPIOB->BSRR= B2_Pin;
#endif
#ifdef AS5047D
		GPIOB->BSRR= B1_Pin;
		GPIOB->BRR= B2_Pin;
#endif
#ifdef TLE5012B
		GPIOB->BRR= B1_Pin;
		GPIOB->BSRR= B2_Pin;
#endif

	}else{
		duty1=0;
		GPIOB->BRR= B1_Pin;
		GPIOB->BRR= B2_Pin;
	}

	if(duty1>MAXDUTY)duty1=MAXDUTY;
	if(Start_EN!=1)duty1=0;


	//Check Res
	if(PMIN0>x_res0|| x_res0>PMAX0){
		duty0=duty0/3;
		Err_cnt0++;
	}

	if(PMIN1>x_res1|| x_res1>PMAX1){
		duty1=duty1/3;
		Err_cnt1++;
	}

#ifdef FORCE_MOTOR1_DIS
	duty0=0;
#endif
#ifdef FORCE_MOTOR2_DIS
	duty1=0;
#endif

	if(Init_Count<200){
		duty0=duty0/5;
		duty1=0;
		Err_cnt0=0;
		Err_cnt1=0;
	}else if(Init_Count<500){
		duty0=duty0/3;
		duty1=duty1/5;
		Err_cnt0=0;
		Err_cnt1=0;
	}


	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,duty0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,duty1);

}


void ChkSWStatus(uint8_t *ptsw, uint8_t *mdsw, uint16_t *ptcnt, uint16_t *mdcnt){

	uint8_t now_ptsw;
	uint8_t now_mdsw;
	uint8_t status_ptsw;
	uint8_t status_mdsw;

	status_ptsw=*ptsw;
	status_mdsw=*mdsw;

	//Change GPIO
	now_mdsw=HAL_GPIO_ReadPin(PTSW_GPIO_Port, PTSW_Pin);
	now_ptsw=HAL_GPIO_ReadPin(MDSW_GPIO_Port, MDSW_Pin);


	if(now_mdsw==0 && now_ptsw==0)Start_EN=(Start_EN==0)?1:0;


	if(now_ptsw==0){
		*ptcnt=*ptcnt+1;
		if(*ptcnt>SWCOUNT){
			if(status_ptsw>=PTTRN_MAXNUM){
				status_ptsw=0;
				*ptcnt=0;
			}
			else{
				status_ptsw=status_ptsw+1;
				*ptcnt=0;
			}
		}
		//SAVE MEM
		Init_Count=0;
		*ptsw=status_ptsw;
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_PTTRN,status_ptsw);

	}
	else{
		*ptcnt=0;
	}


	if(now_mdsw==0){
		*mdcnt=*mdcnt+1;
		if(*mdcnt>SWCOUNT){
			if(status_mdsw>=MODE_MAXNUM){
				status_mdsw=0;
				*mdcnt=0;
			}
			else{
				status_mdsw=status_mdsw+1;
				*mdcnt=0;
			}
		}
		//SAVE MEM
		Init_Count=0;
		*mdsw=status_mdsw;
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_MODE,status_mdsw);
	}
	else{
		*mdcnt=0;
	}

}








int8_t ChkUart(){

	int8_t res=0;

	if(RxRptr!=0){
		RxRptr=0;
		printf(" %s ",RxRbuf);

		if(strcmp(RxRbuf,Str_Switch_Mode) == 0){
			UART_mode=0;
			printf("**Switch Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_Uart_Mode) == 0){
			UART_mode=1;
			printf("**UART Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_Start) == 0){
			Start_EN=1;
			Init_Count=0;
			printf("**Start** \n");
		}
		else if(strcmp(RxRbuf,Str_Stop) == 0){
			Start_EN=0;
			Init_Count=0;
			printf("**STOP** \n");
		}
		else if(strcmp(RxRbuf,Str_Frame_Mode) == 0){
			PAT_Status=enum_ANIMATION;
			printf("**Frame Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_SW0) == 0){
			SW_Status=enum_STAR;
			printf("**SW 0** \n");
		}
		else if(strcmp(RxRbuf,Str_SW1) == 0){
			SW_Status=enum_ARROW;
			printf("**SW 1** \n");
		}
		else if(strcmp(RxRbuf,Str_SW2) == 0){
			SW_Status=enum_MAIL;
			printf("**SW 2** \n");
		}
		else if(strcmp(RxRbuf,Str_SW3) == 0){
			SW_Status=enum_SMILE;
			printf("**SW 3** \n");
		}
		else if(strcmp(RxRbuf,Str_SW4) == 0){
			SW_Status=enum_HEART;
			printf("**SW 4** \n");
		}
		else if(strcmp(RxRbuf,Str_SW5) == 0){
			SW_Status=enum_SUN;
			printf("**SW 5** \n");
		}
		else if(strcmp(RxRbuf,Str_SW6) == 0){
			SW_Status=enum_CLOUD;
			printf("**SW 6** \n");
		}
		else if(strcmp(RxRbuf,Str_SW7) == 0){
			SW_Status=enum_RAIN;
			printf("**SW 7** \n");
		}
		else if(strcmp(RxRbuf,Str_SW8) == 0){
			SW_Status=enum_SNOW;
			printf("**SW 8** \n");
		}
		else if(strcmp(RxRbuf,Str_SW9) == 0){
			SW_Status=enum_THUNDER;
			printf("**SW 9** \n");
		}
		else if(strcmp(RxRbuf,Str_Rotation) == 0){
			PAT_Status=enum_ROTATION;
			printf("**Rotation** \n");
		}
		else if(strcmp(RxRbuf,Str_Stroke) == 0){
			PAT_Status=enum_ONESTROKE;
			printf("**Stroke** \n");
		}
		else if(strcmp(RxRbuf,Str_Always) == 0){
			PAT_Status=enum_ALWAYS;
			printf("**Always** \n");
		}
		else if(strcmp(RxRbuf,Str_AnalogWatch) == 0){
			PAT_Status=enum_AWATCH;
			printf("**AnalogWatch** \n");
		}
		else if(strcmp(RxRbuf,Str_DigitalWatch) == 0){
			PAT_Status=enum_DWATCH;
			printf("**DigitalWatch** \n");
		}
		else if(strcmp(RxRbuf,Str_Day) == 0){
			PAT_Status=enum_DATE;
			printf("**Day** \n");
		}
		else if(strcmp(RxRbuf,Str_StringMode) == 0){
			PAT_Status=enum_STRING;
			printf("**String** \n");
		}
		else if(strcmp(RxRbuf,Str_SD_Mode) == 0){
			SD_mode=1;
			printf("**SD Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_Internal_Mode) == 0){
			SD_mode=0;
			printf("**Internal Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_Debug_Mode) == 0){
			if(Debug_mode==0)Debug_mode=1;
			else Debug_mode=0;
			printf("**Debug Mode** \n");
		}
		else if(strcmp(RxRbuf,Str_ForceLaserON) == 0){
			if(Force_LSR_ON==0)Force_LSR_ON=1;
			else Force_LSR_ON=0;
			printf("**Force_LSR_ON Mode: %d ** \n",Force_LSR_ON);
		}
		else if(strcmp(RxRbuf,Str_FramePause) == 0){
			if(Frame_EN==0)Frame_EN=1;
			else Frame_EN=0;
			printf("**Frame Pause  Status: %d ** \n",Frame_EN);
		}
		else if(strcmp(RxRbuf,Str_Restart) == 0){
			Start_EN=0;
			printf("**Restart** \n");
			for(int k=0;k<20;k++){
				RxRbuf[k]=0;
			}
			RxRptr = 0;                //  送信終了を明示
			RxWptr = 0;
			//goto restart;
			res=-1;
		}
		else if(strncmp(RxRbuf,Str_Center_Pos,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			//printf("tok1: %s\n",tok);
			printf("**Center Pos X:%d, Y:%d ->",x_cen0,x_cen1);
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)x_cen0=atoi(tok)+x_cen0;
				if(find_num==1)x_cen1=atoi(tok)+x_cen1;
				find_num++;
			}
			printf(" X:%d, Y:%d **\n",x_cen0,x_cen1);
		}
		else if(strncmp(RxRbuf,Str_FrameNo,4) == 0){
			char *p;
			int tmp_no=0;
			p=strpbrk( RxRbuf, "1234567890" );
			printf("**Frame No : %d -> ",frameNum);
			if ( p != NULL ) tmp_no = atoi( p );

			if( tmp_no < 0 )frameNum=0;
			else if(FRAME_NUM<=frameNum)frameNum=0;
			else frameNum=tmp_no;
			printf("%d  **\n",frameNum);

		}
		else if(strncmp(RxRbuf,Str_Angle_Deg,4) == 0){
			char *p;
			int tmp_ang=0;
			p=strpbrk( RxRbuf, "1234567890" );

			printf("**Angle : %d degree -> ",angle);
			if ( p != NULL ) tmp_ang = atoi( p );

			if(0<=tmp_ang && tmp_ang<360){
				angle=tmp_ang/((int)360/TRIFUNCLEN);
				angle=angle*((int)360/TRIFUNCLEN);
			}
			else angle=0;

			u_angle=(int)angle/((int)360/TRIFUNCLEN);
			SaveDegRTCMem();

			printf("%d degree **\n",angle);
		}
		else if(strncmp(RxRbuf,Str_GetString,4) == 0){

			for(int k=0;k<32;k++){
				allstr[k]=0;
				GetAllStr[k]=0;
			}
			char tmplen=strlen(RxRbuf)-4;//p);
			if(tmplen>0)strncpy(GetAllStr,&RxRbuf[4],tmplen);

			//Over -> Scroll mode
			if(GetStringMaxLen<tmplen){
				allstrlen=GetStringMaxLen;
				GetAllStrLen=tmplen;
				//strncpy(allstr,GetAllStr,allstrlen);
			}
			else{//Under -> No scroll mode
				allstrlen=tmplen;
				GetAllStrLen=tmplen;
				strncpy(allstr,GetAllStr,tmplen);
			}
			printf("**GetString Len:%d, STR:\"%s\" **",tmplen,GetAllStr);
#ifdef SaveStrData
			SaveStrRTCMem();
#endif
		}
		else if(strncmp(RxRbuf,Str_SetTime,4) == 0){
			char *p;
			long time=0;
			uint8_t h,m,s;
			p=strpbrk( RxRbuf, "1234567890" );
			if ( p != NULL ) time = atol( p );
			//printf("TIME: %ld",time);
			h=(uint8_t)(time/10000);
			m=(uint8_t)((time/100)%100);
			s=(uint8_t)(time%100);
			RTC_TimeTypeDef sTime;
			if(0<=h && h <24)sTime.Hours =(uint8_t)h;
			if(0<=m && m <60)sTime.Minutes=(uint8_t)m;
			if(0<=s && s <60)sTime.Seconds=(uint8_t)s;
			 sTime.SubSeconds = 0;
			 sTime.TimeFormat = RTC_HOURFORMAT_24;
			 sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			 sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			 HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);
			 printf("**Get: %02d:%02d:%02d NOW: %02d:%02d:%02d **\n\r",
					h,m,s,sTime.Hours,sTime.Minutes,sTime.Seconds);


		}
		else if(strncmp(RxRbuf,Str_SetDate,4) == 0){
			char *p;
			long date=0;
			unsigned int y,m,d;
			p=strpbrk( RxRbuf, "1234567890" );
			if ( p != NULL ) date = atol( p );
			y=date/10000;
			m=(date/100)%100;
			d=date%100;
			if(0<=y && y <99)date_y=y;
			if(0<=m && m <13)date_m=m;
			if(0<=d && d <32)date_d=d;
			RTC_DateTypeDef sDate;
		    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		    sDate.Month = (uint8_t)date_m;
		    sDate.Date = (uint8_t)date_d;
		    sDate.Year = (uint8_t)date_y;
		    HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);
			printf("**Get: %02d/%02d/%02d NOW: %02d/%02d/%02d **\n\r",
					y,m,d,date_y,date_m,date_d);
		}
		else if(strncmp(RxRbuf,Str_GetTime,4) == 0){
			printf("**NOW: %02d:%02d:%02d **\n\r",time_h,time_m,time_s);
		}
		else if(strncmp(RxRbuf,Str_GetDate,4) == 0){
			printf("**NOW: 20%02d/%02d/%02d **\n\r",date_y,date_m,date_d);
		}
		else if(strncmp(RxRbuf,Str_SetDData,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//No.
				if(find_num==2)tmp[2]=atoi(tok);//Laser
				if(find_num==3)tmp[3]=atoi(tok);//X
				if(find_num==4)tmp[4]=atoi(tok);//Y
				find_num++;
			}
			if(find_num>4){
				p[tmp[0]][tmp[1]].pow=tmp[2];
				p[tmp[0]][tmp[1]].x=tmp[3];
				p[tmp[0]][tmp[1]].y=tmp[4];
			}
			printf("**Set Data: Data%d, No.%d, Lsr:%d, X:%d, Y:%d **\n",
					tmp[0],tmp[1],tmp[2],tmp[3],tmp[4]);
		}
		else if(strncmp(RxRbuf,Str_SetDNum,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//All Num
				find_num++;
			}
			if(find_num>1){
				allp_num[tmp[0]]=tmp[1];
			}
			printf("**Set All Data Num : Data%d, AllNum:%d **\n",
					tmp[0],tmp[1]);
		}
		else if(strncmp(RxRbuf,Str_GetDData,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//No.
				find_num++;
			}
			if(find_num>1){
				printf("**Get Data: Data%d, No.%d, Lsr:%d, X:%d, Y:%d **\n",
						tmp[0],tmp[1],p[tmp[0]][tmp[1]].pow,p[tmp[0]][tmp[1]].x,p[tmp[0]][tmp[1]].y);
			}

		}
		else if(strncmp(RxRbuf,Str_GetDNum,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				find_num++;
			}
			if(find_num>0){
				printf("**Get All Data Num : Data%d, AllNum:%d **\n",
									tmp[0],allp_num[tmp[0]]);
			}

		}
		/*Frame */
		else if(strncmp(RxRbuf,Str_SetFData,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//No.
				if(find_num==2)tmp[2]=atoi(tok);//Laser
				if(find_num==3)tmp[3]=atoi(tok);//X
				if(find_num==4)tmp[4]=atoi(tok);//Y
				find_num++;
			}
			if(find_num>4){
				f[tmp[0]][tmp[1]].pow=tmp[2];
				f[tmp[0]][tmp[1]].x=tmp[3];
				f[tmp[0]][tmp[1]].y=tmp[4];
			}
			printf("**Set Frame Data: Data%d, No.%d, Lsr:%d, X:%d, Y:%d **\n",
					tmp[0],tmp[1],tmp[2],tmp[3],tmp[4]);
		}
		else if(strncmp(RxRbuf,Str_SetFNum,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//All Num
				find_num++;
			}
			if(find_num>1){
				allf_num[tmp[0]]=tmp[1];
			}
			printf("**Set Frame All Data Num : Data%d, AllNum:%d **\n",
					tmp[0],tmp[1]);
		}
		else if(strncmp(RxRbuf,Str_GetFData,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				if(find_num==1)tmp[1]=atoi(tok);//No.
				find_num++;
			}
			if(find_num>1){
				printf("**Get Frame Data: Data%d, No.%d, Lsr:%d, X:%d, Y:%d **\n",
						tmp[0],tmp[1],f[tmp[0]][tmp[1]].pow,f[tmp[0]][tmp[1]].x,f[tmp[0]][tmp[1]].y);
			}

		}
		else if(strncmp(RxRbuf,Str_GetFNum,4) == 0){
			char *tok;
			char find_num=0;
			tok = strtok( RxRbuf, Find_String);
			int tmp[5]={0,0,0,0,0};
			while( tok != NULL ){
				//printf( "%s\n", tok );
				tok = strtok( NULL, Find_String);
				//printf("tok2: %s\n",tok);
				if(find_num==0)tmp[0]=atoi(tok);//data0,1,2,
				find_num++;
			}
			if(find_num>0){
				printf("**Get Frame All Data Num : Data%d, AllNum:%d **\n",
									tmp[0],allf_num[tmp[0]]);
			}

		}
		else if(strcmp(RxRbuf,Str_TSC_Change) == 0){
			TSC_Enable=(TSC_Enable==0)?1:0;
			HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_TSC,TSC_Enable);
			if(TSC_Enable==1)	printf("**TSC Enabled** \n");
			else				printf("**TSC Disabled** \n");
		}



		for(int k=0;k<20;k++){
			RxRbuf[k]=0;
		}
	 	RxRptr = 0;
		RxWptr = 0;
        printf("\r\n");
	}

	return res;
}


void StartControl(){

	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


	 HAL_TIM_Base_Start_IT(&htim3);
	 HAL_TIM_Base_Start_IT(&htim4);

#ifndef FORCE_MOTOR_DIS
	 HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, 1);
#else
	 HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, 0);
#endif

#ifdef  EnableCurrentControl
	 StartCurrentControl();
#endif


}

void InitParam(){
	UART_mode=0;
	Start_EN=1;
	Frame_EN=1;
	Init_Count=0;
	Debug_mode=0;

	NextFlag=0;
	AllStrShftFlag=0;
	LSD_num=0;

	rdeg=0;
	rflag=0;
	angle=0;
	//u_angle=0;

	Err_cnt0=0;
	Err_cnt1=0;


	watflg=1;
	wat_m=0;
	wat_h=0;
	date_d=1;
	date_m=1;
	date_y=17;




}



void GetSDData(){

	uint16_t k,j;
	char str[20];

	if(f_mount(&FatFs, "", 1)==FR_OK){
		SD_mode=1;//0:Internal memory 1:SD memory

	}
	else{
		printf(" No SD card. \n");
		printf("Draw from internal memory. \n");
		SD_mode=0;//0:Internal memory 1:SD memory
		return;
	}

	//!!!!!!!!!!!Load SD Data!!!!!!!!!!!!!
	for(k=0;k<DATA_NUM;k++){
		j=0;
		char Name_tmp[15];
		sprintf(Name_tmp,"data%d.csv",k);

		if(f_open(&Fil, Name_tmp, FA_READ) == FR_OK){//f_open(&Fil, Name_tmp, FA_READ | FA_OPEN_ALWAYS)

			while (f_gets(str,16, &Fil) != NULL) {

				setpath(&p[k][j], str, j);
				//printf("Data :%d %d %d\n",p[k][j].pow,p[k][j].x,p[k][j].y);
				j++;
				if(j>=ALL_DATA_NUM)break;
			}
			allp_num[k]=j;
			f_close(&Fil);

		}
		printf(" data%d num:%d \n",k,j);
	}

	//return;


	//!!!!!!!!!!!Load Frame Data!!!!!!!!!!!!!
	for(k=0;k<FRAME_NUM;k++){
		j=0;

		char Name_tmp[15];
		sprintf(Name_tmp,"frame%d.csv",k);

		if(f_open(&Fil, Name_tmp, FA_READ) == FR_OK){//f_open(&Fil, Name_tmp, FA_READ | FA_OPEN_ALWAYS)

			while (f_gets(str,16, &Fil) != NULL) {

				setpath2(&f[k][j], str, j);
				j++;
				if(j>=ALL_FRAME_NUM)break;
			}
			allf_num[k]=j;
			f_close(&Fil);
		}
		printf(" Frame%d num:%d \n",k,j);
	}

}




void setpath(struct path* pR, char* str, unsigned int num) {
	unsigned char i = 0;
	int8_t temp=0;

	char* pStr = strtok(&str[0], "\",\"");
	while (pStr) {
		switch (i) {
		case 0:

			temp = atoi(pStr);
			LsrCmdChk(&temp, num);
			pR->pow = temp;
			break;
		case 1:

			temp = atoi(pStr)/CMPRATE;
			PosCmdChk(&temp);
			pR->x = temp;
			break;
		case 2:

			temp = atoi(pStr)/CMPRATE;
			PosCmdChk(&temp);
			pR->y = temp;
			break;
		default:
			break;
		}
		i++;

		pStr = strtok(NULL, "\",\"");
		if (pStr == NULL ) {

			pStr = strtok(NULL, "\"\n");
		}
	}

}

void setpath2(struct fpath* pR, char* str, unsigned int num) {
	unsigned char i = 0;
	int temp=0;

	char* pStr = strtok(&str[0], "\",\"");
	while (pStr) {
		switch (i) {
		case 0:

			temp = atoi(pStr);
			LsrCmdChk((int8_t*)&temp, num);
			pR->pow = temp;
			break;
		case 1:

			temp = atoi(pStr)/CMPRATE;
			PosCmdChk((int8_t*)&temp);
			pR->x = temp;
			break;
		case 2:

			temp = atoi(pStr)/CMPRATE;
			PosCmdChk((int8_t*)&temp);
			pR->y = temp;
			break;
		default:
			break;
		}
		i++;

		pStr = strtok(NULL, "\",\"");
		if (pStr == NULL ) {

			pStr = strtok(NULL, "\"\n");
		}
	}

}

void PosCmdChk(int8_t *cmd){

	if(*cmd>MAXCMD)*cmd=MAXCMD;
	if(*cmd<-MAXCMD)*cmd=-MAXCMD;
}

void LsrCmdChk(int8_t *cmd, unsigned int num){

	if(num==0){
		if(*cmd>LSRMAXCMD)*cmd=LSRMAXCMD;
		if(*cmd<1)*cmd=1;

	}
	else{

		if(*cmd>2)*cmd=1;
		if(*cmd<0)*cmd=0;

	}


}

#ifdef Enable_TSC

void ChkTSCStatus(uint8_t *ptsw, uint8_t *mdsw){

	uint8_t now_ptsw,now_mdsw;
	now_ptsw=*ptsw;
	now_mdsw=*mdsw;

	uint8_t pt_touched=0,md_touched=0;

	for (int i=10-1 ; 0<i ;i--){
		TscBuf[0][i]=TscBuf[0][i-1];
		TscBuf[1][i]=TscBuf[1][i-1];
    }

	TscBuf[0][0]=GetTSCVal(4)*1.25f;
	TscBuf[1][0]=GetTSCVal(3)*1.00f;

	TscAve[0][0]=(TscBuf[0][0]+TscBuf[0][1]+TscBuf[0][2]+TscBuf[0][3])/4;
	TscAve[0][1]=(TscBuf[0][6]+TscBuf[0][7]+TscBuf[0][8]+TscBuf[0][9])/4;

	TscAve[1][0]=(TscBuf[1][0]+TscBuf[1][1]+TscBuf[1][2]+TscBuf[1][3])/4;
	TscAve[1][1]=(TscBuf[1][6]+TscBuf[1][7]+TscBuf[1][8]+TscBuf[1][9])/4;

	//printf(" Ave 1: %d, 2: %d ",TscAve[0][0],TscAve[0][1]);
	//printf("Value Sen1: %d, Sen2: %d \n\r",Value1,Value2);
	//printf("NOW STASTUS= %d, %d \n",TscBuf[0][0],TscBuf[1][0]);

	/* Mode SW */
	if(TscAve[0][1]-TscAve[0][0]>TSC_TH){

		if(TscFlag[0]==TSC_Free){

			TscFlag[0]=TSC_Touched;
			md_touched=1;
		}
		//else TscFlag[0]=TSC_Free;
	 }
	 else TscFlag[0]=TSC_Free;

	 /* Pattern SW */
	 if(TscAve[1][1]-TscAve[1][0]>TSC_TH){
		if(TscFlag[1]==TSC_Free){

			TscFlag[1]=TSC_Touched;
			pt_touched=1;

		}
		//else TscFlag[1]=TSC_Free;

	 }
	 else TscFlag[1]=TSC_Free;


	 if(md_touched==1 && pt_touched==1){
		 printf("Mode & Pattern SW Touched! \n\r");
			if(Start_EN==0)Start_EN=1;
			else Start_EN=0;
			Init_Count=0;

	 }
	 else if(Start_EN==1 && md_touched==1 && pt_touched==0){

		 printf("Mode SW Touched! \n\r");
		if(now_mdsw>=MODE_MAXNUM)now_mdsw=0;
		else now_mdsw=now_mdsw+1;
		*mdsw=now_mdsw;
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_MODE,now_mdsw);

	 }
	 else if(Start_EN==1 && md_touched==0 && pt_touched==1){

		 printf("Pattern SW Touched! \n\r");
		if(now_ptsw>=PTTRN_MAXNUM)now_ptsw=0;
		else now_ptsw=now_ptsw+1;
		*ptsw=now_ptsw;
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_PTTRN,now_ptsw);
	 }



}

int16_t GetTSCVal(uint8_t ch){

	uint16_t val;
	val=0;
	if(ch==3){
		htsc.Init.ChannelIOs = TSC_GROUP1_IO3;
	}
	else if(ch==4){
		htsc.Init.ChannelIOs = TSC_GROUP1_IO4;
	}else return -1;

	HAL_TSC_Init(&htsc);
	HAL_TSC_IODischarge(&htsc, ENABLE);
	asm("nop");asm("nop");asm("nop");
	//HAL_Delay(1);
	if (HAL_TSC_Start(&htsc) != HAL_OK)Error_Handler();
	while (HAL_TSC_GetState(&htsc) == HAL_TSC_STATE_BUSY);
	__HAL_TSC_CLEAR_FLAG(&htsc, (TSC_FLAG_EOA | TSC_FLAG_MCE));
	if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP1_IDX) == TSC_GROUP_COMPLETED){
		val = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP1_IDX);
	}
	return val;
}

#endif


void InitSDIO(){

	 GPIO_InitTypeDef GPIO_InitStruct;
	 /*Configure GPIO pin : OUTPUT */
	 GPIO_InitStruct.Pin = SD_CS_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = SPI2MOSI_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(SPI2MOSI_GPIO_Port, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = SPI2SCK_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(SPI2SCK_GPIO_Port, &GPIO_InitStruct);

	 /*Configure GPIO pins : Input_Pin */
	 GPIO_InitStruct.Pin = SPI2MISO_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 HAL_GPIO_Init(SPI2MISO_GPIO_Port, &GPIO_InitStruct);

}

void StopControl(){

	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, 0);//STBY_OFF
	HAL_GPIO_WritePin(LSRCTR_GPIO_Port, LSRCTR_Pin,0);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin,0);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin,0);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin,0);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin,0);
	HAL_GPIO_WritePin(ERRLED_GPIO_Port, ERRLED_Pin,1);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim4);

	printf("\n Safety Stop!! Check Device!!\n");
	Err_cnt0=0;
	Err_cnt1=0;
}

void SaveDegRTCMem(){

	HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_Angle,u_angle);
}

void LoadDegRTCMem(){

	HAL_Delay(1);
	u_angle=(int16_t)HAL_RTCEx_BKUPRead(&hrtc, enum_RTCMEM_Angle);
}

void SetTSCRTCMem(){

	uint8_t now_ptsw,now_mdsw;
	HAL_Delay(1);
	now_mdsw=HAL_GPIO_ReadPin(PTSW_GPIO_Port, PTSW_Pin);
	HAL_Delay(1);
	now_ptsw=HAL_GPIO_ReadPin(MDSW_GPIO_Port, MDSW_Pin);

	if(now_mdsw==0 && now_ptsw==0 ){

		HAL_Delay(1);
		TSC_Enable=(int16_t)HAL_RTCEx_BKUPRead(&hrtc, enum_RTCMEM_TSC);
		TSC_Enable=(TSC_Enable==TSC_ON)?TSC_OFF:TSC_ON;
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_TSC,TSC_Enable);
		//printf("TSC write: %d \n",TSC_Enable);
	}
	else{
		HAL_Delay(1);
		TSC_Enable=(int16_t)HAL_RTCEx_BKUPRead(&hrtc, enum_RTCMEM_TSC);
		//printf("TSC read: %d \n",TSC_Enable);
	}


	if(TSC_Enable==TSC_DATANONE){//No RTCMem Data-> First Power ON
		TSC_Enable=TSC_ON;
		SW_Status=enum_STAR;
		PAT_Status=enum_ALWAYS;
		HAL_Delay(1);
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_TSC,TSC_Enable);
	}

}


void SaveStrRTCMem(){

	uint8_t tmp[4];
	uint32_t data;

	if(GetStringMaxLen<GetAllStrLen){
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_StrLen,GetAllStrLen);

	}else{
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_StrLen,allstrlen);
	}

	for(int i=0; i<DataStrLen/4;i++){
		tmp[0]=(uint8_t)GetAllStr[4*i+0];
		tmp[1]=(uint8_t)GetAllStr[4*i+1];
		tmp[2]=(uint8_t)GetAllStr[4*i+2];
		tmp[3]=(uint8_t)GetAllStr[4*i+3];
		data=tmp[0]+(tmp[1]<<8)+(tmp[2]<<16)+(tmp[3]<<24);
		HAL_RTCEx_BKUPWrite(&hrtc, enum_RTCMEM_StrTop+i,data);
		//printf("Write:%c%c%c%c \n",tmp[0],tmp[1],tmp[2],tmp[3]);
	}

}



void LoadStrRTCMem(){

	uint8_t Len=HAL_RTCEx_BKUPRead(&hrtc, enum_RTCMEM_StrLen);
	HAL_Delay(1);
	char TmpStr[DataStrLen];

	for(int i=0; i<DataStrLen/4;i++){

		HAL_Delay(1);
		uint32_t TmpData=HAL_RTCEx_BKUPRead(&hrtc, enum_RTCMEM_StrTop+i);

		TmpStr[4*i+0]=(char)TmpData&0xFF;
		TmpStr[4*i+1]=(char)((TmpData>>8)&0xFF);
		TmpStr[4*i+2]=(char)((TmpData>>16)&0xFF);
		TmpStr[4*i+3]=(char)((TmpData>>24)&0xFF);

	}


	//Over -> Scroll mode
	if(GetStringMaxLen<Len){
		allstrlen=GetStringMaxLen;
		GetAllStrLen=Len;
		strncpy(GetAllStr,TmpStr,Len);
	}
	else{//Under -> No scroll mode
		allstrlen=Len;
		GetAllStrLen=Len;
		strncpy(allstr,TmpStr,Len);
	}
	//printf("\n**Load String Len:%d, STR:\"%s\" **\n",Len,TmpStr);



}



void SetEncParam(uint8_t ch){


#ifdef TLE5012B

	uint8_t	sData[2];
	uint8_t rData[2];
	uint16_t res;


	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,0);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS2_GPIO_Port, ENC_CS2_Pin,0);
	for(int i=0; i<1000; i++)asm("nop");
	sData[0]=0x50;sData[1]=0x61;//MOD_1
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0b11000000;sData[1]=0b00000000;
	for(int i=0; i<1000; i++)asm("nop");
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);

	sData[0]=0x50;sData[1]=0x71;//SIL
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0b00000000;sData[1]=0b00000000;
	for(int i=0; i<1000; i++)asm("nop");
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);


	sData[0]=0x50;sData[1]=0x81;//MOD_2
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0x08;sData[1]=0b00001101;
	for(int i=0; i<1000; i++)asm("nop");
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);

	sData[0]=0x50;sData[1]=0x91;//MOD_3
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0x00;sData[1]=0b00001000;
	for(int i=0; i<1000; i++)asm("nop");
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);


	sData[0]=0x50;sData[1]=0xD1;//MOD_3
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	sData[0]=0x00;sData[1]=0b00001011;
	for(int i=0; i<1000; i++)asm("nop");
	HAL_SPI_TransmitReceive(&hspi1,sData,rData,2,100);
	for(int i=0; i<1000; i++)asm("nop");

	if(ch==1)		HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS1_Pin,1);
	else if(ch==2)	HAL_GPIO_WritePin(ENC_CS1_GPIO_Port, ENC_CS2_Pin,1);
	printf("Encoder Setting Fin! \n");

#endif
}


void EnableCodeProtect(void){

	/*Variable used to handle the Options Bytes*/
	static FLASH_OBProgramInitTypeDef OptionsBytesStruct;
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	HAL_FLASH_OB_Unlock();

	/* Get pages write protection status ****************************************/
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

	/* Check if readoutprtection is enabled ***********************/
	if((OptionsBytesStruct.RDPLevel) == OB_RDP_LEVEL_0)
	{
		OptionsBytesStruct.OptionType= OPTIONBYTE_RDP;
		OptionsBytesStruct.RDPLevel   = OB_RDP_LEVEL_1;
		if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
		{
		  /* Error occurred while options bytes programming. **********************/
		  while (1)printf("Error in bytes programming!");
		}

		/* Generate System Reset to load the new option byte values ***************/
		HAL_FLASH_OB_Launch();
	}

	/* Lock the Options Bytes *************************************************/
	HAL_FLASH_OB_Lock();
}

void DisableCodeProtect(void){

	/*Variable used to handle the Options Bytes*/
	static FLASH_OBProgramInitTypeDef OptionsBytesStruct;
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	HAL_FLASH_OB_Unlock();

	/* Get pages write protection status ****************************************/
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

	/* Check if readoutprtection is enabled ***********************/
	if((OptionsBytesStruct.RDPLevel) == OB_RDP_LEVEL_1)
	{
		OptionsBytesStruct.OptionType= OPTIONBYTE_RDP;
		OptionsBytesStruct.RDPLevel   = OB_RDP_LEVEL_0;
		if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
		{
		  /* Error occurred while options bytes programming. **********************/
		  while (1)printf("Error in bytes programming!");
		}

		/* Generate System Reset to load the new option byte values ***************/
		HAL_FLASH_OB_Launch();
	}

	/* Lock the Options Bytes *************************************************/
	HAL_FLASH_OB_Lock();

}
