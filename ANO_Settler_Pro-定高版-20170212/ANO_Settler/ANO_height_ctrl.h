#ifndef __ANO_HEIGHT_CTRL_H
#define __ANO_HEIGHT_CTRL_H

#include "stm32f10x.h"
#include "ANO_PID.h"

typedef struct
{
	float speed_ei;
	float speed;
	float speed_fix;
	float height;

} _height_fusion_st;
extern _height_fusion_st baro_fus;

typedef struct
{
	float exp_speed;
	float exp_disp;
	
	float fb_speed;
	float fb_disp;
	
	float pid_exp_speed;
	float pid_exp_acc;
	
	s16 thr_out;

} _height_ctrl_st;
extern _height_ctrl_st h_c;
	
void height_get(float dT);
void baro_ctrl(float dT,u8 en,_height_ctrl_st *h_c,PID_arg_t *arg_0_hs,PID_val_t *val_0_hs);
	
extern s32 baro_height,baro_speed,baro_speed_o;
extern float baro_speed_av;
#endif

