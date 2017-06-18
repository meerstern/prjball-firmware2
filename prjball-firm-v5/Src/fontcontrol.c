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
#include "font.h"
#include "control.h"
#include "data.h"
#include "stdlib.h"
#include "fontcontrol.h"

/*					*
 * 		GET POS 	*
 * 					*/
void GetCmdPos(int16_t *cmd0, int16_t *cmd1, uint8_t *lsr){

		//Laser ON/OFF Setting
		//XYCommand
		uint8_t LSD=0;
		int16_t x0=0;
		int16_t x1=0;


		if(PAT_Status==enum_STRING){

			if(allstrlen!=0){
				char lsrtmp;
				GetStrPos(	(char*)&allstr,
							allstrlen,
							allstrcnt,
							cnt0,
							&lsrtmp,
							(int16_t*)&x0,
							(int16_t*)&x1	);
				LSD=lsrtmp;
			}
		}
		else if(PAT_Status==enum_AWATCH){
			if(0< time_h && time_h <=12)wat_h=120-10*time_h;
			else if(12< time_h && time_h <24)wat_h=240-10*time_h;
			else wat_h=0;
			if(0< time_m && time_m <30)wat_m=120-(time_m*2);
			else if(30<= time_m && time_m <60){
				wat_m=120-(time_m*2);
				if(wat_h>5)wat_h=wat_h-5;
				else wat_h=wat_h+115;
			}
			else wat_m=0;

			if(allwatcnt==0){
				x0=awatch_frame_x[cnt0];
				x1=awatch_frame_y[cnt0];
				LSD=awatch_frame_l[cnt0];
			}
			else if(allwatcnt==1){
				x0=awatch_min_x[cnt0]*cose[(int)wat_m]
						-awatch_min_y[cnt0]*sine[(int)wat_m];
				x1=awatch_min_x[cnt0]*sine[(int)wat_m]
						+awatch_min_y[cnt0]*cose[(int)wat_m];
				LSD=awatch_min_l[cnt0];
			}else {
				x0=awatch_hour_x[cnt0]*cose[(int)wat_h]
						-awatch_hour_y[cnt0]*sine[(int)wat_h];
				x1=awatch_hour_x[cnt0]*sine[(int)wat_h]
						+awatch_hour_y[cnt0]*cose[(int)wat_h];
				LSD=awatch_hour_l[cnt0];
			}
		}
		else if(PAT_Status==enum_DWATCH){
			if(time_h<10){
				dwatchstrlen=4;
				dwatchstr[0]='0'+(int8_t)time_h;
				dwatchstr[1]=':';
				dwatchstr[2]='0'+(int8_t)time_m/10;
				dwatchstr[3]='0'+(int8_t)time_m%10;
			}
			else{
				dwatchstrlen=5;
				dwatchstr[0]='0'+(int8_t)time_h/10;
				dwatchstr[1]='0'+(int8_t)time_h%10;
				dwatchstr[2]=':';
				dwatchstr[3]='0'+(int8_t)time_m/10;
				dwatchstr[4]='0'+(int8_t)time_m%10;
			}
			if(dwatchstrlen!=0){
				char lsrtmp;
				GetStrPos(	(char*)&dwatchstr,
							dwatchstrlen,
							dwatchstrcnt,
							cnt0,
							&lsrtmp,
							(int16_t*)&x0,
							(int16_t*)&x1	);
				LSD=lsrtmp;
			}
		}
		else if(PAT_Status==enum_DATE){
			if(date_m<10 && date_d<10){
				dwatchstrlen=3;
				dwatchstr[0]='0'+(char)date_m;
				dwatchstr[1]='/';
				dwatchstr[2]='0'+(char)date_d;
			}
			else if(date_m>=10 && date_d<10){
				dwatchstrlen=4;
				dwatchstr[0]='0'+(char)date_m/10;
				dwatchstr[1]='0'+(char)date_m%10;
				dwatchstr[2]='/';
				dwatchstr[3]='0'+(char)date_d;
			}else if(date_m<10 && date_d>=10){
				dwatchstrlen=4;
				dwatchstr[0]='0'+(char)date_m;
				dwatchstr[1]='/';
				dwatchstr[2]='0'+(char)date_d/10;
				dwatchstr[3]='0'+(char)date_d%10;
			}else{
				dwatchstrlen=5;
				dwatchstr[0]='0'+(char)date_m/10;
				dwatchstr[1]='0'+(char)date_m%10;
				dwatchstr[2]='/';
				dwatchstr[3]='0'+(char)date_d/10;
				dwatchstr[4]='0'+(char)date_d%10;
			}
			if(dwatchstrlen!=0){
				char lsrtmp;
				GetStrPos(	(char*)&dwatchstr,
							dwatchstrlen,
							dwatchstrcnt,
							cnt0,
							&lsrtmp,
							(int16_t*)&x0,
							(int16_t*)&x1	);
				LSD=lsrtmp;
			}
		}
		else if(SD_mode==1){

			if(PAT_Status==enum_ANIMATION){

				x0=f[(int)frameNum][cnt0].x*CMPRATE;
				x1=f[(int)frameNum][cnt0].y*CMPRATE;

				if(0<(cnt0-LSD_Offset)){
					LSD=f[(int)frameNum][cnt0-LSD_Offset].pow;
				}
				else if(0<cnt0-LSD_Offset+allf_num[(int)frameNum]-1){
					LSD=f[(int)frameNum][cnt0-LSD_Offset+allf_num[(int)frameNum]-1].pow;
				}
				else{
					 LSD=f[(int)frameNum][cnt0].pow;
				}
			}
			else{

				x0=p[SW_Status][cnt0].x*CMPRATE;
				x1=p[SW_Status][cnt0].y*CMPRATE;

				if(0<(cnt0-LSD_Offset)){
					LSD=p[SW_Status][cnt0-LSD_Offset].pow;
				}
				else if(0<(cnt0-LSD_Offset+allp_num[SW_Status]-1)){
					LSD=p[SW_Status][cnt0-LSD_Offset+allp_num[SW_Status]-1].pow;
				}
				else{
					LSD=p[SW_Status][cnt0].pow;
				}
			}
		}
		else{//Internal

			if(PAT_Status==enum_ANIMATION){

				if(cnt0>TRIFUNCLEN)cnt0=cnt0-TRIFUNCLEN;
				if(cnt2>TRIFUNCLEN)cnt2=cnt2-TRIFUNCLEN;
				x0=AMP*sine[cnt0];
				x1=AMP*cose[cnt2];
				LSD=1;
			}
			else if(SW_Status==enum_STAR){
				x0=(int16_t)(starx[cnt0]/3);
				x1=(int16_t)(stary[cnt0]/3);
				LSD=1;
			}
			else if(SW_Status==enum_ARROW){
				x0=(((int16_t)arrowx[cnt0]));//+x_cmd2)/2;
				x1=(((int16_t)arrowy[cnt0]));//+x_cmd3)/2;
				LSD=1;
			}
			else if(SW_Status==enum_MAIL){
				x0=(((int16_t)mailx[cnt0]));//+x_cmd2)/2;
				x1=(((int16_t)maily[cnt0]));//+x_cmd3)/2;
				LSD=maill[cnt0];
			}
			else if(SW_Status==enum_SMILE){
				x0=((int)nikox[cnt0])/2;
				x1=((int)nikoy[cnt0])/2;
				LSD=nikol[cnt0];
			}
			else if(SW_Status==enum_SUN){
				x0=mk_sun_x[cnt0];
				x1=mk_sun_y[cnt0];
				LSD=mk_sun_l[cnt0];
			}
			else if(SW_Status==enum_CLOUD){
				x0=(int)mk_cloud_x[cnt0];
				x1=(int)mk_cloud_y[cnt0];
				LSD=mk_cloud_l[cnt0];
			}
			else if(SW_Status==enum_RAIN){
				x0=mk_rain_x[cnt0];
				x1=mk_rain_y[cnt0];
				LSD=mk_rain_l[cnt0];
			}
			else if(SW_Status==enum_SNOW){
				x0=mk_snow_x[cnt0];
				x1=mk_snow_y[cnt0];
				LSD=mk_snow_l[cnt0];
			}
			else if(SW_Status==enum_THUNDER){
				x0=mk_thunder_x[cnt0];
				x1=mk_thunder_y[cnt0];
				LSD=mk_thunder_l[cnt0];
			}
			else if(SW_Status==enum_HEART){
					x0=mk_heart_x[cnt0];
					x1=mk_heart_y[cnt0];
					LSD=mk_heart_l[cnt0];
				}

		}//Internal

		if(Start_EN!=1)LSD=0;

		//PAT_Status==2 -> LSR Drowing Mode
		if(PAT_Status==2&&LSD==1&&(LSD_num>cnt0)){
			//LSD=LSD;
		}
		else if(PAT_Status==2&&LSD==1&&(LSD_num<=cnt0)){
			LSD=0;
		}

		if(Init_Count<800)LSD=0;
		if(Force_LSR_ON==1)LSD=1;

		*lsr=LSD;
		*cmd0=x0;
		*cmd1=x1;
}


void RepeatChk(void){
	//Repeat Check -> Reset or not
	//Point Count RESET


	if(PAT_Status==enum_STRING){

		if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
		ChkStrCnt(	( char*)&allstr,
					allstrlen,
					&allstrcnt,
					(int16_t*)&cnt0,
					(int16_t*)&cnt1);
	}
	else if(PAT_Status==enum_AWATCH){

		if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
		if(allwatcnt==0){
			if(cnt0>=(sizeof(awatch_frame_l)/sizeof(int))){allwatcnt++;cnt0=0;cnt1=-8;}
		}
		else if(allwatcnt==1){
			if(cnt0>=(sizeof(awatch_min_x)/sizeof(int))){allwatcnt++;cnt0=0;cnt1=-8;}

		}else{
			if(cnt0>=(sizeof(awatch_hour_x)/sizeof(int))){allwatcnt=0;cnt0=0;cnt1=-8;}
		}

			//if(allstrlen<=allstrcnt){allstrcnt=0;cnt1=-10;}
	}
	else if(PAT_Status==enum_DWATCH){

			if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
			ChkStrCnt(	( char*)&dwatchstr,
						dwatchstrlen,
						&dwatchstrcnt,
						(int16_t*)&cnt0,
						(int16_t*)&cnt1);
	}
	else if(PAT_Status==enum_DATE){
		if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
			ChkStrCnt(	( char*)&dwatchstr,
						dwatchstrlen,
						&dwatchstrcnt,
						(int16_t*)&cnt0,
						(int16_t*)&cnt1);
	}
	else if(SD_mode==1){

			if(PAT_Status==enum_ANIMATION){

				if(NextFlag==1){
					if(Frame_EN==1){
						frameNum++;
					}
					if(FRAME_NUM<=frameNum){
						frameNum=0;
					}else if(allf_num[(int)frameNum]==0){
						frameNum=0;
					}
					NextFlag=0;
				}

				if(cnt1>=f[(int)frameNum][0].pow){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
				if(cnt0>=allf_num[(int)frameNum])cnt0=1;
			}
			else{

				if(NextFlag==1){
					if(PAT_Status==1){
						rdeg=rdeg+3;
						if(rdeg>=TRIFUNCLEN-10)rdeg=0;
					}
					else if(PAT_Status==2){
						LSD_num=LSD_num+5;
						if(LSD_num>allp_num[SW_Status]+20)LSD_num=0;

					}else{
						rdeg=0;
					}
					NextFlag=0;
				}
				if(cnt1>=p[SW_Status][0].pow){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
				if(cnt0>=allp_num[SW_Status])cnt0=1;
			}

		}//SD MODE
		else{//Internal Mode

			if(PAT_Status==enum_ANIMATION){

				if(NextFlag==1){

					if(rdeg>12){
						deg0=(rand()%36)*2;
						deg1=(rand()%18)*3;
						rdeg=0;
					}
					else{
						rdeg++;
					}
					NextFlag=0;
				}


				if(cnt1>=6){cnt0=cnt0+1+deg0;cnt1=0;x_sum0=0;x_sum1=0;}
				if(cnt3>=6){cnt2=cnt2+1+deg1;cnt3=0;x_sum0=0;x_sum1=0;}

				if(cnt0>TRIFUNCLEN-10)cnt0=cnt0-TRIFUNCLEN+10;
				if(cnt2>TRIFUNCLEN-10)cnt2=cnt2-TRIFUNCLEN+10;

				if(cnt0>=TRIFUNCLEN-10)cnt0=0;
				if(cnt2>=TRIFUNCLEN-10)cnt2=0;
			}
			else{

				if(NextFlag==1){
					if(PAT_Status==enum_ROTATION){
						rdeg=rdeg+3;
						if(rdeg>=TRIFUNCLEN-10)rdeg=0;
					}
				else if(PAT_Status==enum_ONESTROKE){

					if(SW_Status==enum_STAR){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(starx)/sizeof(starx[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_ARROW){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(arrowx)/sizeof(arrowx[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_MAIL){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mailx)/sizeof(mailx[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_SMILE){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(nikox)/sizeof(nikox[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_SUN){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_sun_l)/sizeof(mk_sun_l[0])+20)LSD_num=0;

					}
					else if(SW_Status==enum_CLOUD){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_cloud_l)/sizeof(mk_cloud_l[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_RAIN){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_rain_l)/sizeof(mk_rain_l[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_SNOW){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_snow_l)/sizeof(mk_snow_l[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_THUNDER){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_thunder_l)/sizeof(mk_thunder_l[0])+20)LSD_num=0;
					}
					else if(SW_Status==enum_HEART){
						LSD_num=LSD_num+5;
						if(LSD_num>sizeof(mk_heart_l)/sizeof(mk_heart_l[0])+20)LSD_num=0;
					}
				}
				else{
						LSD_num=0;
						rdeg=0;
					}
					NextFlag=0;
				}


				if(SW_Status==enum_STAR){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}
					if(cnt0>=sizeof(starx)/sizeof(starx[0]))cnt0=0;
				}
				else if(SW_Status==enum_ARROW){
					if(cnt1>=5){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//5
					if(cnt0>=sizeof(arrowx)/sizeof(arrowx[0]))cnt0=0;
				}
				else if(SW_Status==enum_MAIL){
					if(cnt1>=5){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//5
					if(cnt0>=sizeof(mailx)/sizeof(mailx[0]))cnt0=0;
				}
				else if(SW_Status==enum_SMILE){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(nikox)/sizeof(nikox[0])-1)cnt0=0;
				}
				else if(SW_Status==enum_SUN){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					//if(cnt0>=197)cnt0=0;
					if(cnt0>=sizeof(mk_sun_x)/sizeof(mk_sun_x[0])){cnt0=0;}
				}
				else if(SW_Status==enum_CLOUD){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(mk_cloud_x)/sizeof(mk_cloud_x[0]))cnt0=0;
					//if(cnt0>=52)cnt0=0;
				}
				else if(SW_Status==enum_RAIN){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(mk_rain_l)/sizeof(mk_rain_l[0]))cnt0=0;
				}
				else if(SW_Status==enum_SNOW){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(mk_snow_l)/sizeof(mk_snow_l[0]))cnt0=0;
				}
				else if(SW_Status==enum_THUNDER){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(mk_thunder_l)/sizeof(mk_thunder_l[0]))cnt0=0;
				}
				else if(SW_Status==enum_HEART){
					if(cnt1>=4){cnt0++;cnt1=0;x_sum0=0;x_sum1=0;}//4
					if(cnt0>=sizeof(mk_heart_l)/sizeof(mk_heart_l[0]))cnt0=0;
				}
			}//if(PAT_Status==enum_ANIMATION){
		}//Internal Mode

}




void GetStrPos(	char *str,
				uint8_t strlen,
				uint8_t strcnt,
				int cnt,
				char *lsr,
				int16_t *xpos,
				int16_t *ypos	){

	char strng=*(str+strcnt);
	int16_t l=0,x=0,y=0;

	/*NUMBER*/
	if(strng=='0'){
		l=num_zero_l[cnt];		x=(int16_t)num_zero_x[cnt];		y=(int16_t)num_zero_y[cnt];
	}
	else if(strng=='1'){
		l=num_one_l[cnt];		x=(int16_t)num_one_x[cnt];		y=(int16_t)num_one_y[cnt];
	}
	else if(strng=='2'){
		l=num_two_l[cnt];		x=(int16_t)num_two_x[cnt];		y=(int16_t)num_two_y[cnt];
	}
	else if(strng=='3'){
		l=num_three_l[cnt];		x=(int16_t)num_three_x[cnt];	y=(int16_t)num_three_y[cnt];
	}
	else if(strng=='4'){
		l=num_four_l[cnt];		x=(int16_t)num_four_x[cnt];		y=(int16_t)num_four_y[cnt];
	}
	else if(strng=='5'){
		l=num_five_l[cnt];		x=(int16_t)num_five_x[cnt];		y=(int16_t)num_five_y[cnt];
	}
	else if(strng=='6'){
		l=num_six_l[cnt];		x=(int16_t)num_six_x[cnt];		y=(int16_t)num_six_y[cnt];
	}
	else if(strng=='7'){
		l=num_seven_l[cnt];		x=(int16_t)num_seven_x[cnt];	y=(int16_t)num_seven_y[cnt];
	}
	else if(strng=='8'){
		l=num_eight_l[cnt];		x=(int16_t)num_eight_x[cnt];	y=(int16_t)num_eight_y[cnt];
	}
	else if(strng=='9'){
		l=num_nine_l[cnt];		x=(int16_t)num_nine_x[cnt];		y=(int16_t)num_nine_y[cnt];
	}

	/*MARK*/
	if(strng==':'){
		l=mk_colon_l[cnt];		x=(int16_t)mk_colon_x[cnt];		y=(int16_t)mk_colon_y[cnt];
	}
	else if(strng=='/'){
		l=mk_over_l[cnt];		x=mk_over_x[cnt];		y=mk_over_y[cnt];
	}
	else if(strng=='!'){
		l=mk_exclamation_l[cnt];x=mk_exclamation_x[cnt];y=mk_exclamation_y[cnt]-15;
	}
	else if(strng=='#'){
		l=mk_hash_l[cnt];		x=mk_hash_x[cnt];		y=mk_hash_y[cnt];
	}
	else if(strng=='$'){
		l=mk_dollars_l[cnt];		x=mk_dollars_x[cnt];		y=mk_dollars_y[cnt];
	}
	else if(strng=='%'){
		l=mk_percent_l[cnt];		x=mk_percent_x[cnt];		y=mk_percent_y[cnt];
	}
	else if(strng=='&'){
		l=mk_ampersand_l[cnt];		x=mk_ampersand_x[cnt];		y=mk_ampersand_y[cnt];
	}
	else if(strng=='('){
		l=mk_parenl_l[cnt];		x=mk_parenl_x[cnt];		y=mk_parenl_y[cnt];
	}
	else if(strng==')'){
		l=mk_parenr_l[cnt];		x=mk_parenr_x[cnt];		y=mk_parenr_y[cnt];
	}
	else if(strng=='*'){
		l=mk_asterisk_l[cnt];		x=mk_asterisk_x[cnt];		y=mk_asterisk_y[cnt];
	}
	else if(strng==','){
		l=mk_comma_l[cnt];		x=mk_comma_x[cnt];		y=mk_comma_y[cnt];
	}
	else if(strng=='.'){
		l=mk_period_l[cnt];		x=mk_period_x[cnt];		y=mk_period_y[cnt];
	}
	else if(strng=='?'){
		l=mk_question_l[cnt];		x=mk_question_x[cnt];		y=mk_question_y[cnt];
	}
	else if(strng=='@'){
		l=mk_at_l[cnt];		x=mk_at_x[cnt];		y=mk_at_y[cnt];
	}
	else if(strng=='['){
		l=mk_bracketl_l[cnt];		x=mk_bracketl_x[cnt];		y=mk_bracketl_y[cnt];
	}
	else if(strng==']'){
		l=mk_bracketr_l[cnt];		x=mk_bracketr_x[cnt];		y=mk_bracketr_y[cnt];
	}
	else if(strng=='^'){
		l=mk_hat_l[cnt];		x=mk_hat_x[cnt];		y=mk_hat_y[cnt];
	}
	else if(strng=='_'){
		l=mk_underscore_l[cnt];		x=mk_underscore_x[cnt];		y=mk_underscore_y[cnt];
	}
	else if(strng=='|'){
		l=mk_verticalbar_l[cnt];	x=mk_verticalbar_x[cnt];	y=mk_verticalbar_y[cnt];
	}
	else if(strng=='~'){
		l=mk_tilde_l[cnt];		x=mk_tilde_x[cnt];		y=mk_tilde_y[cnt];
	}
	else if(strng=='\''){
		l=mk_singlequotation_l[cnt];	x=mk_singlequotation_x[cnt];	y=mk_singlequotation_y[cnt];
	}
	else if(strng=='\"'){
		l=mk_doublequotation_l[cnt];	x=mk_doublequotation_x[cnt];	y=mk_doublequotation_y[cnt];
	}
	else if(strng=='\\'){
		l=mk_yen_l[cnt];		x=mk_yen_x[cnt];		y=mk_yen_y[cnt];
	}
	else if(strng=='+'){
		l=mk_plus_l[cnt];		x=mk_plus_x[cnt];		y=mk_plus_y[cnt];
	}
	else if(strng=='<'){
		l=mk_lessthan_l[cnt];		x=mk_lessthan_x[cnt];		y=mk_lessthan_y[cnt];
	}
	else if(strng=='='){
		l=mk_equal_l[cnt];		x=mk_equal_x[cnt];		y=mk_equal_y[cnt];
	}
	else if(strng=='>'){
		l=mk_graterthan_l[cnt];		x=mk_graterthan_x[cnt];		y=mk_graterthan_y[cnt];
	}
	else if(strng=='-'){
		l=mk_dash_l[cnt];		x=mk_dash_x[cnt];		y=mk_dash_y[cnt];
	}

	/*STRING*/
	if(strng==' '){
		l=str_space_l[cnt];		x=str_space_x[cnt];		y=str_space_y[cnt];
	}
	else if(strng=='a'){
		l=str_a_l[cnt];		x=str_a_x[cnt];		y=str_a_y[cnt]-10;
	}
	else if(strng=='b'){
		l=str_b_l[cnt];		x=str_b_x[cnt];		y=str_b_y[cnt];
	}
	else if(strng=='c'){
		l=str_c_l[cnt];		x=str_c_x[cnt];		y=str_c_y[cnt];
	}
	else if(strng=='d'){
		l=str_d_l[cnt];		x=str_d_x[cnt];		y=str_d_y[cnt];
	}
	else if(strng=='e'){
		l=str_e_l[cnt];		x=str_e_x[cnt];		y=str_e_y[cnt];
	}
	else if(strng=='f'){
		l=str_f_l[cnt];		x=str_f_x[cnt];		y=str_f_y[cnt];
	}
	else if(strng=='g'){
		l=str_g_l[cnt];		x=str_g_x[cnt];		y=str_g_y[cnt];
	}
	else if(strng=='h'){
		l=str_h_l[cnt];		x=str_h_x[cnt];		y=str_h_y[cnt];
	}
	else if(strng=='i'){
		l=str_i_l[cnt];		x=str_i_x[cnt];		y=str_i_y[cnt];
	}
	else if(strng=='j'){
		l=str_j_l[cnt];		x=str_j_x[cnt];		y=str_j_y[cnt]*0.9f;
	}
	else if(strng=='k'){
		l=str_k_l[cnt];		x=str_k_x[cnt];		y=str_k_y[cnt];
	}
	else if(strng=='l'){
		l=str_l_l[cnt];		x=str_l_x[cnt];		y=str_l_y[cnt];
	}
	else if(strng=='m'){
		l=str_m_l[cnt];		x=str_m_x[cnt];		y=str_m_y[cnt];
	}
	else if(strng=='n'){
		l=str_n_l[cnt];		x=str_n_x[cnt];		y=str_n_y[cnt];
	}
	else if(strng=='o'){
		l=str_o_l[cnt];		x=str_o_x[cnt];		y=str_o_y[cnt]-8;
	}
	else if(strng=='p'){
		l=str_p_l[cnt];		x=str_p_x[cnt];		y=str_p_y[cnt]-10;
	}
	else if(strng=='q'){
		l=str_q_l[cnt];		x=str_q_x[cnt];		y=str_q_y[cnt];
	}
	else if(strng=='r'){
		l=str_r_l[cnt];		x=str_r_x[cnt];		y=str_r_y[cnt];
	}
	else if(strng=='s'){
		l=str_s_l[cnt];		x=str_s_x[cnt];		y=str_s_y[cnt]-10;
	}
	else if(strng=='t'){
		l=str_t_l[cnt];		x=str_t_x[cnt];		y=str_t_y[cnt];
	}
	else if(strng=='u'){
		l=str_u_l[cnt];		x=str_u_x[cnt];		y=str_u_y[cnt];
	}
	else if(strng=='v'){
		l=str_v_l[cnt];		x=str_v_x[cnt];		y=str_v_y[cnt];
	}
	else if(strng=='w'){
		l=str_w_l[cnt];		x=str_w_x[cnt];		y=str_w_y[cnt];
	}
	else if(strng=='x'){
		l=str_x_l[cnt];		x=str_x_x[cnt];		y=str_x_y[cnt];
	}
	else if(strng=='y'){
		l=str_y_l[cnt];		x=str_y_x[cnt];		y=str_y_y[cnt];
	}
	else if(strng=='z'){
		l=str_z_l[cnt];		x=str_z_x[cnt];		y=str_z_y[cnt];
	}

	/*Large STRING*/
	if(strng=='A'){
		l=str_A_l[cnt];		x=str_A_x[cnt];		y=str_A_y[cnt];
	}
	else if(strng=='B'){
		l=str_B_l[cnt];		x=str_B_x[cnt]*1.1f;		y=str_B_y[cnt]*1.1f;
	}
	else if(strng=='C'){
		l=str_C_l[cnt];		x=str_C_x[cnt];		y=str_C_y[cnt];
	}
	else if(strng=='D'){
		l=str_D_l[cnt];		x=str_D_x[cnt];		y=str_D_y[cnt];
	}
	else if(strng=='E'){
		l=str_E_l[cnt];		x=str_E_x[cnt];		y=str_E_y[cnt];
	}
	else if(strng=='F'){
		l=str_F_l[cnt];		x=str_F_x[cnt];		y=str_F_y[cnt];
	}
	else if(strng=='G'){
		l=str_G_l[cnt];		x=str_G_x[cnt];		y=str_G_y[cnt];
	}
	else if(strng=='H'){
		l=str_H_l[cnt];		x=str_H_x[cnt];		y=str_H_y[cnt];
	}
	else if(strng=='I'){
		l=str_I_l[cnt];		x=str_I_x[cnt];		y=str_I_y[cnt];
	}
	else if(strng=='J'){
		l=str_J_l[cnt];		x=str_J_x[cnt];		y=str_J_y[cnt];
	}
	else if(strng=='K'){
		l=str_K_l[cnt];		x=str_K_x[cnt];		y=str_K_y[cnt];
	}
	else if(strng=='L'){
		l=str_L_l[cnt];		x=str_L_x[cnt];		y=str_L_y[cnt];
	}
	else if(strng=='M'){
		l=str_M_l[cnt];		x=str_M_x[cnt]*0.95f;		y=str_M_y[cnt];
	}
	else if(strng=='N'){
		l=str_N_l[cnt];		x=str_N_x[cnt];		y=str_N_y[cnt];
	}
	else if(strng=='O'){
		l=str_O_l[cnt];		x=str_O_x[cnt];		y=str_O_y[cnt];
	}
	else if(strng=='P'){
		l=str_P_l[cnt];		x=str_P_x[cnt]*1.1f;		y=str_P_y[cnt]*1.1f;
	}
	else if(strng=='Q'){
		l=str_Q_l[cnt];		x=str_Q_x[cnt];		y=str_Q_y[cnt];
	}
	else if(strng=='R'){
		l=str_R_l[cnt];		x=str_R_x[cnt];		y=str_R_y[cnt];
	}
	else if(strng=='S'){
		l=str_S_l[cnt];		x=str_S_x[cnt];		y=str_S_y[cnt];
	}
	else if(strng=='T'){
		l=str_T_l[cnt];		x=str_T_x[cnt];		y=str_T_y[cnt];
	}
	else if(strng=='U'){
		l=str_U_l[cnt];		x=str_U_x[cnt];		y=str_U_y[cnt]-10;
	}
	else if(strng=='V'){
		l=str_V_l[cnt];		x=str_V_x[cnt];		y=str_V_y[cnt];
	}
	else if(strng=='W'){
		l=str_W_l[cnt];		x=str_W_x[cnt];		y=str_W_y[cnt];
	}
	else if(strng=='X'){
		l=str_X_l[cnt];		x=str_X_x[cnt];		y=str_X_y[cnt];
	}
	else if(strng=='Y'){
		l=str_Y_l[cnt];		x=str_Y_x[cnt];		y=str_Y_y[cnt];
	}
	else if(strng=='Z'){
		l=str_Z_l[cnt];		x=str_Z_x[cnt];		y=str_Z_y[cnt];
	}

	*lsr=l;
	*xpos=x+StrOffset[(int)strlen][(int)strcnt];
	*ypos=y;


}

void ChkStrCnt(	char *STR,
				uint8_t ALLSTRLEN,
				uint8_t *ALLSTRCNT,
				int16_t *CNT0,
				int16_t *CNT1){


	char strng=*(STR+*ALLSTRCNT);
	int length=0;
	/*NUMBER*/
	if(strng=='0'){
		length=sizeof(num_zero_l)/sizeof(num_zero_l[0]);
	}
	else if(strng=='1'){
		length=sizeof(num_one_l)/sizeof(num_one_l[0]);
	}
	else if(strng=='2'){
		length=sizeof(num_two_l)/sizeof(num_two_l[0]);
	}
	else if(strng=='3'){
		length=sizeof(num_three_l)/sizeof(num_three_l[0]);
	}
	else if(strng=='4'){
		length=sizeof(num_four_l)/sizeof(num_four_l[0]);
	}
	else if(strng=='5'){
		length=sizeof(num_five_l)/sizeof(num_five_l[0]);
	}
	else if(strng=='6'){
		length=sizeof(num_six_l)/sizeof(num_six_l[0]);
	}
	else if(strng=='7'){
		length=sizeof(num_seven_l)/sizeof(num_seven_l[0]);
	}
	else if(strng=='8'){
		length=sizeof(num_eight_l)/sizeof(num_eight_l[0]);
	}
	else if(strng=='9'){
		length=sizeof(num_nine_l)/sizeof(num_nine_l[0]);
	}

	/*MARK*/
	if(strng==':'){
		length=sizeof(mk_colon_l)/sizeof(mk_colon_l[0]);
	}
	else if(strng=='/'){
		length=sizeof(mk_over_l)/sizeof(mk_over_l[0]);
	}
	else if(strng=='!'){
		length=sizeof(mk_exclamation_l)/sizeof(mk_exclamation_l[0]);
	}
	else if(strng=='#'){
		length=sizeof(mk_hash_l)/sizeof(mk_hash_l[0]);
	}
	else if(strng=='$'){
		length=sizeof(mk_dollars_l)/sizeof(mk_dollars_l[0]);
	}
	else if(strng=='%'){
		length=sizeof(mk_percent_l)/sizeof(mk_percent_l[0]);
	}
	else if(strng=='&'){
		length=sizeof(mk_ampersand_l)/sizeof(mk_ampersand_l[0]);
	}
	else if(strng=='('){
		length=sizeof(mk_parenl_l)/sizeof(mk_parenl_l[0]);
	}
	else if(strng==')'){
		length=sizeof(mk_parenr_l)/sizeof(mk_parenr_l[0]);
	}
	else if(strng=='*'){
		length=sizeof(mk_asterisk_l)/sizeof(mk_asterisk_l[0]);
	}
	else if(strng==','){
		length=sizeof(mk_comma_l)/sizeof(char);
	}
	else if(strng=='.'){
		length=sizeof(mk_period_l)/sizeof(char);
	}
	else if(strng=='?'){
		length=sizeof(mk_question_l)/sizeof(char);
	}
	else if(strng=='@'){
		length=sizeof(mk_at_l)/sizeof(char);
	}
	else if(strng=='['){
		length=sizeof(mk_bracketl_l)/sizeof(char);
	}
	else if(strng==']'){
		length=sizeof(mk_bracketr_l)/sizeof(char);
	}
	else if(strng=='^'){
		length=sizeof(mk_hat_l)/sizeof(char);
	}
	else if(strng=='_'){
		length=sizeof(mk_underscore_l)/sizeof(char);
	}
	else if(strng=='|'){
		length=sizeof(mk_verticalbar_l)/sizeof(char);
	}
	else if(strng=='~'){
		length=sizeof(mk_tilde_l)/sizeof(char);
	}
	else if(strng=='\''){
		length=sizeof(mk_singlequotation_l)/sizeof(char);
	}
	else if(strng=='\"'){
		length=sizeof(mk_doublequotation_l)/sizeof(char);
	}
	else if(strng=='\\'){
		length=sizeof(mk_yen_l)/sizeof(char);
	}
	else if(strng=='+'){
		length=sizeof(mk_plus_l)/sizeof(char);
	}
	else if(strng=='<'){
		length=sizeof(mk_lessthan_l)/sizeof(char);
	}
	else if(strng=='='){
		length=sizeof(mk_equal_l)/sizeof(char);
	}
	else if(strng=='>'){
		length=sizeof(mk_graterthan_l)/sizeof(char);
	}
	else if(strng=='-'){
		length=sizeof(mk_dash_l)/sizeof(char);
	}
	/*STRING*/
	if(strng==' '){
		length=sizeof(str_space_l)/sizeof(char);
	}
	else if(strng=='a'){
		length=sizeof(str_a_l)/sizeof(char);
	}
	else if(strng=='b'){
		length=sizeof(str_b_l)/sizeof(char);
	}
	else if(strng=='c'){
		length=sizeof(str_c_l)/sizeof(char);
	}
	else if(strng=='d'){
		length=sizeof(str_d_l)/sizeof(char);
	}
	else if(strng=='e'){
		length=sizeof(str_e_l)/sizeof(char);
	}
	else if(strng=='f'){
		length=sizeof(str_f_l)/sizeof(char);
	}
	else if(strng=='g'){
		length=sizeof(str_g_l)/sizeof(char);
	}
	else if(strng=='h'){
		length=sizeof(str_h_l)/sizeof(char);
	}
	else if(strng=='i'){
		length=sizeof(str_i_l)/sizeof(char);
	}
	else if(strng=='j'){
		length=sizeof(str_j_l)/sizeof(char);
	}
	else if(strng=='k'){
		length=sizeof(str_k_l)/sizeof(char);
	}
	else if(strng=='l'){
		length=sizeof(str_l_l)/sizeof(char);
	}
	else if(strng=='m'){
		length=sizeof(str_m_l)/sizeof(char);
	}
	else if(strng=='n'){
		length=sizeof(str_n_l)/sizeof(char);
	}
	else if(strng=='o'){
		length=sizeof(str_o_l)/sizeof(char);
	}
	else if(strng=='p'){
		length=sizeof(str_p_l)/sizeof(char);
	}
	else if(strng=='q'){
		length=sizeof(str_q_l)/sizeof(char);
	}
	else if(strng=='r'){
		length=sizeof(str_r_l)/sizeof(char);
	}
	else if(strng=='s'){
		length=sizeof(str_s_l)/sizeof(char);
	}
	else if(strng=='t'){
		length=sizeof(str_t_l)/sizeof(char);
	}
	else if(strng=='u'){
		length=sizeof(str_u_l)/sizeof(char);
	}
	else if(strng=='v'){
		length=sizeof(str_v_l)/sizeof(char);
	}
	else if(strng=='w'){
		length=sizeof(str_w_l)/sizeof(char);
	}
	else if(strng=='x'){
		length=sizeof(str_x_l)/sizeof(char);
	}
	else if(strng=='y'){
		length=sizeof(str_y_l)/sizeof(char);
	}
	else if(strng=='z'){
		length=sizeof(str_z_l)/sizeof(char);
	}

	/*Large STRING*/
	if(strng=='A'){
		length=sizeof(str_A_l)/sizeof(char);
	}
	else if(strng=='B'){
		length=sizeof(str_B_l)/sizeof(char);
	}
	else if(strng=='C'){
		length=sizeof(str_C_l)/sizeof(char);
	}
	else if(strng=='D'){
		length=sizeof(str_D_l)/sizeof(char);
	}
	else if(strng=='E'){
		length=sizeof(str_E_l)/sizeof(char);
	}
	else if(strng=='F'){
		length=sizeof(str_F_l)/sizeof(char);
	}
	else if(strng=='G'){
		length=sizeof(str_G_l)/sizeof(char);
	}
	else if(strng=='H'){
		length=sizeof(str_H_l)/sizeof(char);
	}
	else if(strng=='I'){
		length=sizeof(str_I_l)/sizeof(char);
	}
	else if(strng=='J'){
		length=sizeof(str_J_l)/sizeof(char);
	}
	else if(strng=='K'){
		length=sizeof(str_K_l)/sizeof(char);
	}
	else if(strng=='L'){
		length=sizeof(str_L_l)/sizeof(char);
	}
	else if(strng=='M'){
		length=sizeof(str_M_l)/sizeof(char);
	}
	else if(strng=='N'){
		length=sizeof(str_N_l)/sizeof(char);
	}
	else if(strng=='O'){
		length=sizeof(str_O_l)/sizeof(char);
	}
	else if(strng=='P'){
		length=sizeof(str_P_l)/sizeof(char);
	}
	else if(strng=='Q'){
		length=sizeof(str_Q_l)/sizeof(char);
	}
	else if(strng=='R'){
		length=sizeof(str_R_l)/sizeof(char);
	}
	else if(strng=='S'){
		length=sizeof(str_S_l)/sizeof(char);
	}
	else if(strng=='T'){
		length=sizeof(str_T_l)/sizeof(char);
	}
	else if(strng=='U'){
		length=sizeof(str_U_l)/sizeof(char);
	}
	else if(strng=='V'){
		length=sizeof(str_V_l)/sizeof(char);
	}
	else if(strng=='W'){
		length=sizeof(str_W_l)/sizeof(char);
	}
	else if(strng=='X'){
		length=sizeof(str_X_l)/sizeof(char);
	}
	else if(strng=='Y'){
		length=sizeof(str_Y_l)/sizeof(char);
	}
	else if(strng=='Z'){
		length=sizeof(str_Z_l)/sizeof(char);
	}

	if(*CNT0>=length){
		*ALLSTRCNT=*ALLSTRCNT+1;
		*CNT0=0;
		*CNT1=-8;
	}
	if(ALLSTRLEN<=*ALLSTRCNT){
		*ALLSTRCNT=0;
		*CNT1=-16;
	}

}
