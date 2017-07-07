

#include "ANO_height_ctrl.h"
#include "spl06_01.h"
#include "mymath.h"
#include "filter.h"
#include "ANO_IMU.h"
#include "ANO_CTRL.h"

/*
基础单位：cm

*/
s32 baro_height,baro_height_old;
s32 baro_speed_o,baro_speed;

_height_fusion_st baro_fus;

#define MONUM 10
float baro_av_arr[MONUM];
float baro_speed_av;
u16 baro_av_cnt;

float baro_speed_delta;
float baro_speed_delta_lpf;

void height_get(float dT)
{
	s32 acc;
	acc = (s32)(imu_data.w_acc.z)/10;
	//////////////////////////////////////////////////////////////
	baro_height_old = baro_height;
	baro_height = (user_spl0601_get());
	baro_speed_o = safe_div(baro_height - baro_height_old,dT,0);
	
	Moving_Average(baro_av_arr,MONUM ,&baro_av_cnt,baro_speed_o,&baro_speed_av);
	
	baro_speed_delta = LIMIT(baro_speed_av - baro_speed,-2000*dT,2000*dT);
	
	LPF_1_(0.5f,dT,baro_speed_delta,baro_speed_delta_lpf);

	baro_speed += baro_speed_delta *LIMIT((ABS(baro_speed_delta_lpf)/(2000*dT)),0,1);
	
	///////////////////////////////////////////////////////////////
	
	baro_fus.speed += 1.05f *my_deadzone(acc,0,10) *dT;
	baro_fus.speed += (baro_speed - baro_fus.speed) *0.2f *dT;
	
	baro_fus.speed_ei += (baro_speed - baro_fus.speed_fix) *dT;
	
	baro_fus.speed_fix = baro_fus.speed + 0.2f *baro_fus.speed_ei;
	
	baro_fus.height += baro_fus.speed_fix *dT;
	baro_fus.height += (baro_height - baro_fus.height) *0.2f *dT;
}


void baro_fusion()
{


}

#define DISP_MAX_ERR 200  //cm

u8 ctrl_0_cnt;

PID_arg_t arg_0_2_hs ;
PID_val_t val_0_2_hs ;

void CTRL_0_2_PID_Init()
{
	arg_0_2_hs.kp = 1.2f ;//*ANO_Param.PID_hs.kp *0.001f;
	arg_0_2_hs.ki = 0.0f ;//*ANO_Param.PID_hs.ki *0.001f;
	arg_0_2_hs.kd = 0.6f;
	arg_0_2_hs.k_pre_d = 0.0f;// *ANO_Param.PID_hs.kd *0.001f;
	arg_0_2_hs.k_ff = 0.0f;
}


void baro_ctrl(float dT,u8 en,_height_ctrl_st *h_c,PID_arg_t *arg_0_hs,PID_val_t *val_0_hs)
{
	static float dT2;
	static float thr_inte_lim;
	thr_inte_lim = safe_div(THR_INTEGRA_LIM *20,arg_0_hs->ki,0);
	CTRL_0_2_PID_Init();
	
	switch(en)
	{
		case 0:
		{
			h_c->fb_speed = h_c->exp_speed = 0;
			h_c->fb_disp = h_c->exp_disp = 0;
			val_0_hs->err_i = safe_div(THR_INIT *20,arg_0_hs->ki,0);
		}	
		break;
		default:
		{
			
		}
		break;
	}
	
	ctrl_0_cnt++;
	ctrl_0_cnt%=5;
	dT2 += dT;
	if(ctrl_0_cnt==0)//10ms
	{	
	
		h_c->fb_disp += h_c->fb_speed *dT2;
		h_c->exp_disp += h_c->exp_speed *dT2;
		h_c->exp_disp = LIMIT(h_c->exp_disp,(h_c->fb_disp - DISP_MAX_ERR),(h_c->fb_disp + DISP_MAX_ERR));
		
		PID_calculate( dT2,										//周期（单位：秒）
									0,											//前馈值
									h_c->exp_disp,					//期望值（设定值）
									h_c->fb_disp,						//反馈值（）
									&arg_0_2_hs,							//PID参数结构体
									&val_0_2_hs,							//PID数据结构体
									0,											//integration limit，积分限幅
									&(h_c->pid_exp_speed)  );				//输出
			
		/////
		dT2 = 0;
	}
	
	PID_calculate( dT,									//周期（单位：秒）
									0,									//前馈值
									h_c->pid_exp_speed,	//期望值（设定值）
									h_c->fb_speed,			//反馈值（）
									arg_0_hs,						//PID参数结构体
									val_0_hs,						//PID数据结构体
									thr_inte_lim,									//integration limit，积分限幅
									&(h_c->pid_exp_acc)  );				//输出
	
	h_c->pid_exp_acc = LIMIT(h_c->pid_exp_acc,-3000,3000);//3g 3000cm/ss
	h_c->thr_out = h_c->pid_exp_acc/2;
	h_c->thr_out = LIMIT(h_c->thr_out,0,1000);
	
}


