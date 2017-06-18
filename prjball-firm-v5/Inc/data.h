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

#define DATA_NUM		10
#define ALL_DATA_NUM	202

#define FRAME_NUM		20
#define ALL_FRAME_NUM	202//202



extern const float sine[120];
extern const float cose[120];
	
	
	
extern const int starx[168];					
extern const int stary[168];

extern const int maill[164];
extern const int mailx[164];
extern const int maily[164];

extern const int nikox[125];
extern const int nikoy[125];
extern const char nikol[125];

extern const int arrowx[108];					
extern const int arrowy[108];


extern const int StrOffset[7][6];

#pragma once				   
struct path{
	//レーザーON/OFF
	int8_t pow;
	//パスX
	int8_t x;
	//パスY
	int8_t y;
};
#pragma once
struct fpath{
	//レーザーON/OFF
	int8_t pow;
	//パスX
	int8_t x;
	//パスY
	int8_t y;
}; 

extern struct path p[DATA_NUM][ALL_DATA_NUM];
extern struct fpath f[FRAME_NUM][ALL_FRAME_NUM];

