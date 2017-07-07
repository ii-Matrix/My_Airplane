#ifndef __ANO_Init_H
#define __ANO_Init_H

#include "stm32f10x.h"
#include "include.h"
#include "bsp_SysTick.h"

void sys_init(void);


enum
{
	ORG=0,
	LPF,
	LPF_2,
	
};

typedef struct
{
	u8 finish;
	u8 ready;
	u16 cnt;
	
	float gyro_lenght[3];
	float ng_acc_lenght[3];
	float speed_lenght[3];
	
}_psac_st;
extern _psac_st psac;

#endif

