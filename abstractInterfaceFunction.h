#ifndef _ABS_INTERFACE_FUNC_H
#define _ABS_INTERFACE_FUNC_H

#include "stm32f10x.h"

// RCC时钟
void ABS_RCC_TIM_ClockCmd(TIM_TypeDef *TIM_ID, FunctionalState NewState);
void ABS_RCC_USART_ClockCmd(USART_TypeDef *USART_ID, FunctionalState NewState);
void ABS_RCC_GPIO_ClockCmd(GPIO_TypeDef *GPIO_ID, FunctionalState NewState);
void ABS_RCC_DMA_ClockCmd(DMA_TypeDef *DMA_ID, FunctionalState NewState);

// 循环队列指针计算
uint16_t ABS_CycleAddPos(uint16_t start, uint16_t step, uint16_t max);  // 环形队列指针相加函数
uint16_t ABS_CycleSubPos(uint16_t end, uint16_t start, uint16_t max);   // 环形队列指针相减函数(大 - 小)

// 数值变换函数
void ABS_AbsoluteLimiter(int32_t *operand, int32_t Limit);          // 绝对值限幅函数
int32_t ABS_AbsoluteAdd(int32_t operand, int32_t addAbsoluteNum);     // 绝对值相加函数

#endif

