#include "Key.h"
#include "stm32f10x.h"                  // Device header
#include "Delay.h"                  // Device header
#include "abstractInterfaceFunction.h"

void KEY_Init(void)
{
	ABS_RCC_GPIO_ClockCmd(KEYX, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin= KEY0 | KEY1;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;		//Work for Output
	GPIO_Init(KEYX, &GPIO_InitStructure);
}

uint8_t KEY_GetNum(void)
{
	uint8_t KeyNum = 255;
	if(GPIO_ReadInputDataBit(KEYX, KEY0) == 0)
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(KEYX, KEY0) == 0);
		delay_ms(20);
		KeyNum = 0;
	}
	
	else if(GPIO_ReadInputDataBit(KEYX, KEY1) == 0)
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(KEYX, KEY1) == 0);
		delay_ms(20);
		KeyNum = 1;
	}

	// else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0)
	// {
	// 	delay_ms(20);
	// 	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0);
	// 	delay_ms(20);
	// 	KeyNum = 3;
	// }

	return KeyNum;
}
