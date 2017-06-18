/* * * * * * * * * * * * * * * * * * * * * * * * * *
 * ProjectionBall Firmware						   *
 * Copyright (c) 2017  							   *
 * K.Watanabe,Crescent 							   *
 * Released under the MIT license 				   *
 * http://opensource.org/licenses/mit-license.php  *
 * 17/06/16 v1.0 Initial Release                   *
 * 												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * */



void GetCmdPos(int16_t *cmd0, int16_t *cmd1, uint8_t *lsr);
void RepeatChk(void);


void GetStrPos(	 char *str,
				uint8_t len,
				uint8_t strcnt,
				int cnt,
				char *lsr,
				int16_t *xpos,
				int16_t *ypos	);
void ChkStrCnt(	 char *STR,
				uint8_t ALLSTRLEN,
				uint8_t *ALLSTRCNT,
				int16_t *CNT0,
				int16_t *CNT1);
