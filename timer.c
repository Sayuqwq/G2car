#include "timer.h"

void TIMER_Init(TIM_TypeDef* TIMx, uint16_t psc, uint16_t per, uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority)
{		// 含中断优先级配置，其中一个255则不开启中断
//Turn on RCC
	ABS_RCC_TIM_ClockCmd(TIMx, ENABLE);
	
//Select Clock
	TIM_InternalClockConfig(TIMx);
	
//Config Time Base Unit
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = per - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);		//It inits by generate an update event,so ITFlag will set 1;
	
	
	//Clear ITFlag
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	
	if(NVIC_PreemptionPriority < 4 && NVIC_SubPriority < 4)
	{
	//EXTI Output Control
		TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
		
	//NVIC
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitTypeDef NVIC_InitStrutcure;
		if(TIMx == TIM1)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM1_UP_IRQn;
		else if(TIMx == TIM2)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM2_IRQn;
		else if(TIMx == TIM3)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM3_IRQn;
		else if(TIMx == TIM4)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM4_IRQn;
		else if(TIMx == TIM5)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM5_IRQn;
		else if(TIMx == TIM6)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM6_IRQn;
		else if(TIMx == TIM7)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM7_IRQn;
		else if(TIMx == TIM8)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM8_UP_IRQn;
		NVIC_InitStrutcure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStrutcure.NVIC_IRQChannelPreemptionPriority = NVIC_PreemptionPriority;
		NVIC_InitStrutcure.NVIC_IRQChannelSubPriority = NVIC_SubPriority;
		NVIC_Init(&NVIC_InitStrutcure);
	}
	
//Turn on TIMx
	TIM_Cmd(TIMx, ENABLE);
}

void TIMER_EnableInterrupt(TIM_TypeDef* TIMx, uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority)
{
	//Clear ITFlag
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	
	if(NVIC_PreemptionPriority < 4 && NVIC_SubPriority < 4)
	{
	//EXTI Output Control
		TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
		
	//NVIC
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitTypeDef NVIC_InitStrutcure;
		if(TIMx == TIM1)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM1_UP_IRQn;
		else if(TIMx == TIM2)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM2_IRQn;
		else if(TIMx == TIM3)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM3_IRQn;
		else if(TIMx == TIM4)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM4_IRQn;
		else if(TIMx == TIM5)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM5_IRQn;
		else if(TIMx == TIM6)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM6_IRQn;
		else if(TIMx == TIM7)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM7_IRQn;
		else if(TIMx == TIM8)	NVIC_InitStrutcure.NVIC_IRQChannel = TIM8_UP_IRQn;
		NVIC_InitStrutcure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStrutcure.NVIC_IRQChannelPreemptionPriority = NVIC_PreemptionPriority;
		NVIC_InitStrutcure.NVIC_IRQChannelSubPriority = NVIC_SubPriority;
		NVIC_Init(&NVIC_InitStrutcure);
	}
}

/*
void TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM, TIM_IT_Update) == SET)
	{
		
		

		TIM_ClearITPendingBit(TIM, TIM_IT_Update);
	}
}
*/



