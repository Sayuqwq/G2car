#include "abstractInterfaceFunction.h"

void ABS_RCC_TIM_ClockCmd(TIM_TypeDef *TIM_ID, FunctionalState NewState)
{
	void (*pfunc)(uint32_t, FunctionalState);
	uint32_t TIM_Periph;

	switch ((uint32_t)TIM_ID)
	{
		// advanced		APB2
	case (uint32_t)TIM1:
		pfunc = &RCC_APB2PeriphClockCmd;
		TIM_Periph = RCC_APB2Periph_TIM1;
		break;
	case (uint32_t)TIM8:
		pfunc = &RCC_APB2PeriphClockCmd;
		TIM_Periph = RCC_APB2Periph_TIM8;
		break;

		// general and basic	APB1
	case (uint32_t)TIM2:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM2;
		break;
	case (uint32_t)TIM3:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM3;
		break;
	case (uint32_t)TIM4:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM4;
		break;
	case (uint32_t)TIM5:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM5;
		break;
	case (uint32_t)TIM6:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM6;
		break;
	case (uint32_t)TIM7:
		pfunc = &RCC_APB1PeriphClockCmd;
		TIM_Periph = RCC_APB1Periph_TIM7;
		break;
	}

	pfunc(TIM_Periph, NewState);
}

void ABS_RCC_USART_ClockCmd(USART_TypeDef *USART_ID, FunctionalState NewState)
{
	void (*pfunc)(uint32_t, FunctionalState);
	uint32_t USART_Periph;

	switch ((uint32_t)USART_ID)
	{
		// APB2
	case (uint32_t)USART1:
		pfunc = &RCC_APB2PeriphClockCmd;
		USART_Periph = RCC_APB2Periph_USART1;
		break;

		// APB1
	case (uint32_t)USART2:
		pfunc = &RCC_APB1PeriphClockCmd;
		USART_Periph = RCC_APB1Periph_USART2;
		break;
	case (uint32_t)USART3:
		pfunc = &RCC_APB1PeriphClockCmd;
		USART_Periph = RCC_APB1Periph_USART3;
		break;
	case (uint32_t)UART4:
		pfunc = &RCC_APB1PeriphClockCmd;
		USART_Periph = RCC_APB1Periph_UART4;
		break;
	case (uint32_t)UART5:
		pfunc = &RCC_APB1PeriphClockCmd;
		USART_Periph = RCC_APB1Periph_UART5;
		break;
	}

	pfunc(USART_Periph, NewState);
}

void ABS_RCC_GPIO_ClockCmd(GPIO_TypeDef *GPIO_ID, FunctionalState NewState)
{
	uint32_t GPIO_Periph;

	switch ((uint32_t)GPIO_ID)
	{
	case (uint32_t)GPIOA:
		GPIO_Periph = RCC_APB2Periph_GPIOA;
		break;
	case (uint32_t)GPIOB:
		GPIO_Periph = RCC_APB2Periph_GPIOB;
		break;
	case (uint32_t)GPIOC:
		GPIO_Periph = RCC_APB2Periph_GPIOC;
		break;
	case (uint32_t)GPIOD:
		GPIO_Periph = RCC_APB2Periph_GPIOD;
		break;
	case (uint32_t)GPIOE:
		GPIO_Periph = RCC_APB2Periph_GPIOE;
		break;
	case (uint32_t)GPIOF:
		GPIO_Periph = RCC_APB2Periph_GPIOF;
		break;
	case (uint32_t)GPIOG:
		GPIO_Periph = RCC_APB2Periph_GPIOG;
		break;
	}
	
	RCC_APB2PeriphClockCmd(GPIO_Periph, NewState);
}

void ABS_RCC_DMA_ClockCmd(DMA_TypeDef *DMA_ID, FunctionalState NewState)
{
	uint32_t DMA_Periph;

	switch ((uint32_t)DMA_ID)
	{
	case (uint32_t)DMA1:
		DMA_Periph = RCC_AHBPeriph_DMA1;
		break;
	case (uint32_t)DMA2:
		DMA_Periph = RCC_AHBPeriph_DMA2;
		break;
	}
	
	RCC_AHBPeriphClockCmd(DMA_Periph, ENABLE);		// DMA Clock
}

inline uint16_t ABS_CycleAddPos(uint16_t start, uint16_t step, uint16_t max)        // # 发生过错误：数据类型大小不足
{
	return (start + step) % max;
}

inline uint16_t ABS_CycleSubPos(uint16_t end, uint16_t start, uint16_t max)
{
	return (end - start + max) % max;
}

void ABS_AbsoluteLimiter(int32_t *operand, int32_t Limit)	// 绝对值限幅函数，[-Limit, +Limit]
{
	if (*operand > Limit)
	{
		*operand = Limit;
		return;
	}
	else if (*operand < -Limit)
	{
		*operand = -Limit;
		return;
	}
}

int32_t ABS_AbsoluteAdd(int32_t operand, int32_t addAbsoluteNum)
{
	if (operand >= 0)
	{
		operand += addAbsoluteNum;
		if (operand < 0)		// 保护，如果operand为正，则限幅为非负
			operand = 0;
	}
	else 
	{
		operand -= addAbsoluteNum;
		if (operand > 0)
			operand = 0;
	}

    return operand;
}


