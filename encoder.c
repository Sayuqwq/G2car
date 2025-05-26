#include "stm32f10x.h"                  // Device header
#include "alias.h"
#include "encoder.h"
#include "timer.h"
#include "abstractInterfaceFunction.h"

#define TRUE 1
#define FALSE 0

// static uint16_t encoder_period;
// const uint16_t Tolerance = 200;
// const uint16_t Limit = 3000;

// uint16_t last_capture1 = 0, last_capture2 = 0, last_capture3 = 0;		// 上一次CCR的值
// uint16_t debug_speed1 = 0, debug_speed2 = 0, debug_speed3 = 0;
// uint16_t speedBuf_1[4], speedBuf_2[4], speedBuf_3[4];
// uint16_t pspeedBuf_1, pspeedBuf_2, pspeedBuf_3;

// DEBUG
		// uint16_t counter1 = 0;	// 计数器，用以判断20Hz以下的速度失效
		// uint16_t counter2 = 0;
		// uint16_t counter3 = 0;

		// uint16_t cap1 = 0, cap2 = 0, cap3 = 0;
		// uint16_t interval_1 = 0, interval_2 = 0, interval_3 = 0;


//

ENCODERVARS encv;		// const初始化

void ENCODERVARS_Constructor(void)
{
	// encv.speedM1;
	// encv.speedM2;
}


void ENC_Init(void)		// 约定Psc = 100, Per = 36000, TIM_1 通道123编码器测速，A8, A9, A10
{
	ENCODERVARS_Constructor();		// ENC成员变量构造函数
	
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// ME1
	
	// Clock
	ABS_RCC_GPIO_ClockCmd(ME1X, ENABLE);
	ABS_RCC_TIM_ClockCmd(ME1T, ENABLE);
	
	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = ME1A | ME1B;
	GPIO_Init(ME1X, &GPIO_InitStruct);
	
	// TIM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(ME1T, &TIM_TimeBaseInitStruct);
	
	// Input Capture
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(ME1T, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(ME1T, &TIM_ICInitStructure);
	
	// Encoder Interface
	TIM_EncoderInterfaceConfig(ME1T, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	// // NVIC
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	// NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	// NVIC_Init(&NVIC_InitStructure);

	// // Other operations
	// TIM_ClearFlag(ME1T, TIM_FLAG_Update);
	// TIM_ITConfig(ME1T, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(ME1T, 0);
	TIM_Cmd(ME1T, ENABLE);


	// ME2

	// Clock
	ABS_RCC_GPIO_ClockCmd(ME2X, ENABLE);
	ABS_RCC_TIM_ClockCmd(ME2T, ENABLE);
	
	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = ME2A | ME2B;
	GPIO_Init(ME2X, &GPIO_InitStruct);
	
	// TIM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(ME2T, &TIM_TimeBaseInitStruct);
	
	// Input Capture
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(ME2T, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(ME2T, &TIM_ICInitStructure);
	
	// Encoder Interface
	TIM_EncoderInterfaceConfig(ME2T, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	// // NVIC
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	// NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	// NVIC_Init(&NVIC_InitStructure);

	// // Other operations
	// TIM_ClearFlag(ME2T, TIM_FLAG_Update);
	// TIM_ITConfig(ME2T, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(ME2T, 0);
	TIM_Cmd(ME2T, ENABLE);
}

void ENC_GetSpeed(void)
{
	// 带方向修正，鉴于ME2方向正确，故MDIR为1则调转方向
	encv.speedM1 = (int16_t)TIM_GetCounter(ME1T);
	if (MDIR1 == 1)
		encv.speedM1 = - encv.speedM1;
	encv.pulsesM1 += encv.speedM1;
	TIM_SetCounter(ME1T, 0);

	encv.speedM2 = (int16_t)TIM_GetCounter(ME2T);
	if (MDIR2 == 1)
		encv.speedM2 = - encv.speedM2;
	encv.pulsesM2 += encv.speedM2;
	TIM_SetCounter(ME2T, 0);
}

void ENC_ResetSpeed(void)
{
	TIM_SetCounter(ME1T, 0);
	TIM_SetCounter(ME2T, 0);
}

int ENC_ReadOnlySpeed(uint8_t motor)
{
	if (motor == 1)
		return encv.speedM1;
	else if (motor == 2)
		return encv.speedM2;
}


// void speedFilter(void)
// {
// 	uint8_t i;
// 	uint16_t temp1 = 0, temp2 = 0, temp3 = 0, temp = 0;
	
// 	if (speedBuf_1[0] - speedBuf_1[1] > Tolerance)	temp1 += 2;
// 	if (speedBuf_1[0] - speedBuf_1[2] > Tolerance)	temp1 += 1;
	
// 	if (speedBuf_2[0] - speedBuf_2[1] > Tolerance)	temp2 += 2;
// 	if (speedBuf_2[0] - speedBuf_2[2] > Tolerance)	temp2 += 1;
	
// 	if (speedBuf_3[0] - speedBuf_3[1] > Tolerance)	temp3 += 2;
// 	if (speedBuf_3[0] - speedBuf_3[2] > Tolerance)	temp3 += 1;
	
// 	for (i = 0; i < 4; i++)
// 	{
// 		if ((3 - i) != temp1)
// 			debug_speed1 += speedBuf_1[i];
// 		if ((3 - i) != temp2)
// 			debug_speed2 += speedBuf_2[i];
// 		if ((3 - i) != temp3)
// 			debug_speed3 += speedBuf_3[i];
// 	}
	
// 	debug_speed1 /= 3;
// 	debug_speed2 /= 3;
// 	debug_speed3 /= 3;
// }

// // 720KHz读取脉冲间隔(50~2000+Hz, 即14400~360个CNT / 36000)
// int Read_Speed(uint16_t *pdebug_speed1, uint16_t *pdebug_speed2, uint16_t *pdebug_speed3)	// 测周法测速度（频率），cycle最大为0.05s, 即T = 20Hz / 20Hz = 1
// {
// 	debug_speed1 = debug_speed2 = debug_speed3 = 0;
	
// 	speedFilter();
	
// 	*pdebug_speed1 = debug_speed1;
// 	*pdebug_speed2 = debug_speed2;
// 	*pdebug_speed3 = debug_speed3;
	
// 	return TRUE;
// }


// void TIM2_IRQHandler(void)
// {
// 	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
// 	{
		
		
// 		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
// 	}
// }

// void TIM3_IRQHandler(void)
// {
// 	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
// 	{
		
		
// 		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
// 	}
// }















