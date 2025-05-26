#include "pwm.h"
#include "abstractInterfaceFunction.h"
#include "timer.h"


PWMVARS pwmv = 
{
	.PERCENT_MAX = 100,

	// 10KHz，超高频率定时器，极限
};

void PWMVARS_Constructor(void)
{
	pwmv.SOFT_compare[0] = 0;
	pwmv.SOFT_compare[1] = 0;
	pwmv.SOFT_compare[2] = 0;
	pwmv.SOFT_compare[3] = 0;
}

void PWM_Init(uint16_t Psc, uint16_t Per)		// TIM5, 电机PWM，A0, A1, A2, A3
{
	PWMVARS_Constructor();

	pwmv.PWM_MAX = Per - 1;		// 0 - 35999
	
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	// Clock
	ABS_RCC_GPIO_ClockCmd(MPX, ENABLE);
	ABS_RCC_TIM_ClockCmd(MPT, ENABLE);

	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = MP1 | MP2 | MP3 | MP4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPX, &GPIO_InitStruct);
	
	// TIM 初始化定时器
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = Per - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = Psc - 1;
	TIM_TimeBaseInit(MPT, &TIM_TimeBaseInitStruct);
	
	// Output Compare 初始化输出比较
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;		// CCR
	TIM_OC1Init(MPT, &TIM_OCInitStruct);
	TIM_OC2Init(MPT, &TIM_OCInitStruct);
	TIM_OC3Init(MPT, &TIM_OCInitStruct);
	TIM_OC4Init(MPT, &TIM_OCInitStruct);
	
	// Preload 预装载器使能
	TIM_ARRPreloadConfig(MPT, ENABLE);	// ARR Preload
	TIM_OC1PreloadConfig(MPT, ENABLE);	// ARR Preload
	TIM_OC2PreloadConfig(MPT, ENABLE);	// ARR Preload
	TIM_OC3PreloadConfig(MPT, ENABLE);	// ARR Preload
	TIM_OC4PreloadConfig(MPT, ENABLE);	// ARR Preload

	TIM_Cmd(MPT, ENABLE);


	/*	软PWM
	*/	
	// GPIO_InitTypeDef GPIO_InitStruct;	// 已定义
	
	// Clock
	ABS_RCC_GPIO_ClockCmd(MDX, ENABLE);
	
	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = MD1 | MD2 | MD3 | MD4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MDX, &GPIO_InitStruct);

	// Timer
	TIMER_Init(MDT, MDT_SOFT_PSC, MDT_SOFT_PER, 1, 1);
}

void PWM_HARD_SetPWM_actual(uint8_t motor, uint16_t value)		// 0 - 39
{
	value *= (pwmv.PWM_MAX + 1) / (MDT_SOFT_MAX);		// patch
	// value *= 900;		// patch

	switch (motor)
	{
		case 1:
			TIM_SetCompare1(MPT, value);
			break;
		
		case 2:
			TIM_SetCompare2(MPT, value);
			break;
		
		case 3:
			TIM_SetCompare3(MPT, value);
			break;
		
		case 4:
			TIM_SetCompare4(MPT, value);
			break;
	}
}

void PWM_HARD_SetPWM_percent(uint8_t motor, uint8_t percent)	// motor: id;  percent: 0~100
{
	uint16_t value;
		
	value = (double)percent * ((double)MDT_SOFT_MAX / (double)pwmv.PERCENT_MAX);
	
	PWM_HARD_SetPWM_actual(motor, value);
}

void PWM_SOFT_SetPWM_actual(uint8_t motor, uint16_t value)		// 0 - 39
{
	if (value > MDT_SOFT_MAX)	// > 39 则 39
		value=  MDT_SOFT_MAX;

	switch (motor)
	{
		case 1:
			pwmv.SOFT_compare[0] = value;
			break;
		
		case 2:
			pwmv.SOFT_compare[1] = value;
			break;
		
		case 3:
			pwmv.SOFT_compare[2] = value;
			break;
		
		case 4:
			pwmv.SOFT_compare[3] = value;
			break;
	}
}

void PWM_SOFT_SetPWM_percent(uint8_t motor, uint8_t percent)	// motor: id;  percent: 0~100
{
	uint16_t value;
		
	value = (double)percent * ((double)MDT_SOFT_MAX / (double)pwmv.PERCENT_MAX);
	
	PWM_SOFT_SetPWM_actual(motor, value);
}

uint32_t PWM_GetPWMVal(uint8_t motor)
{
	switch (motor)
	{
	case 1:
		return TIM_GetCapture1(MPT);
		
	case 2:
		return TIM_GetCapture2(MPT);
		
	case 3:
		return TIM_GetCapture3(MPT);
		
	case 4:
		return TIM_GetCapture4(MPT);
	}
}

// 720KHz中断，需优化速度
void MDT_IRQHandler(void)
{
	if(TIM_GetITStatus(MDT, TIM_IT_Update) == SET)
	{
		pwmv.SOFT_counter++;
		pwmv.SOFT_counter %= (MDT_SOFT_MAX + 1);		// 最大值39
		GPIO_WriteBit(MDX, MD1, (BitAction)(pwmv.SOFT_counter <= pwmv.SOFT_compare[0]));
		GPIO_WriteBit(MDX, MD2, (BitAction)(pwmv.SOFT_counter <= pwmv.SOFT_compare[1]));
		GPIO_WriteBit(MDX, MD3, (BitAction)(pwmv.SOFT_counter <= pwmv.SOFT_compare[2]));
		GPIO_WriteBit(MDX, MD4, (BitAction)(pwmv.SOFT_counter <= pwmv.SOFT_compare[3]));
		
		TIM_ClearITPendingBit(MDT, TIM_IT_Update);
	}
}










