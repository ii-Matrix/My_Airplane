/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Init.c
 * 描述    ：初始化函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "ANO_Init.h"
#include "ANO_Drv_MPU6050.h"
#include "i2c_soft.h"
#include "ANO_Drv_Hid.h"
#include "ANO_Param.h"
#include "ANO_Motor.h"
#include "ANO_CTRL.h"
#include "filter.h"
#include "spl06_01.h"

u8 NRF_ENABLE;
void sys_init()
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化USB
	USB_HID_Init();
	
	//初始化系统滴答定时器
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	
	
	//led初始化
	ANO_LED_Init();
	//i2c初始化
	I2c_Soft_Init();
	
	//初始化SPI
	ANO_SPI_Init();
	//检查NRF连接是否正常
	NRF_ENABLE = ANO_NRF_Check();
	//如果连接正常，则将NRF初始化为TX2模式（高级发送）
	if(NRF_ENABLE)
	{
		ANO_NRF_Init(MODEL_TX2,80);
	}
	
	#ifdef ANO_DT_USE_WIFI
	ANO_UART3_Init(500000);
	Esp8266_Init();
	Esp8266_SetGpio0(1);
	Esp8266_SetEnable(1);
	flag.espDownloadMode = 0;
	#endif
	
	Delay_ms(100);
	
	MPU6050_Init(20);
	
	Delay_ms(200);
	
		spl0601_init();	
	
	//参数初始化
	ANO_Param_Read();
	
	//电机输出初始化
	pwm_out_init();
	
	//pid参数初始化
	pid_init();
	
	//电压检测初始化
	ADC1_Init();	
}

_psac_st psac;
void post_sensor_auto_cal(float dT,_psac_st *data)
{
	LPF_1_(5.0f,dT,data->gyro_lenght[0],data->gyro_lenght[1]);
	LPF_1_(5.0f,dT,data->ng_acc_lenght[0],data->ng_acc_lenght[1]);
	LPF_1_(5.0f,dT,data->speed_lenght[0],data->speed_lenght[1]);
	
	if(ABS(data->gyro_lenght[1])<0.1f)
	{
		if(ABS(data->ng_acc_lenght[1])<50)
		{
			if(ABS(data->speed_lenght[1])<100)
			{
				data->ready = 1;
			}
			else
			{
				data->ready = 0;
			}
		}
		else
		{
			data->ready = 0;
		}
	}
	else
	{
		data->ready = 0;
	}
/////////////////////////////////		
	if(data->finish ==0)
	{
		if(data->ready)
		{
			if(data->cnt<1000)/////超时
			{
				data->cnt += 1000*dT;
			}
			else
			{
				data->finish = 1;
			}		
		}
		else
		{
			data->cnt = 0;
		}
	}
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


