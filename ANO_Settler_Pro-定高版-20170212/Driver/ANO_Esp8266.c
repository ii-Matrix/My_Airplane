#include "ANO_Esp8266.h"

void Esp8266_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Esp8266_SetGpio0(uint8_t enable)
{
	if(enable)
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);	
}

void Esp8266_SetEnable(uint8_t enable)
{
	if(enable)
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);	
}
