#ifndef _ABS_INTERFACE_FUNC_H
#define _ABS_INTERFACE_FUNC_H

#include "stm32f10x.h"

// RCCʱ��
void ABS_RCC_TIM_ClockCmd(TIM_TypeDef *TIM_ID, FunctionalState NewState);
void ABS_RCC_USART_ClockCmd(USART_TypeDef *USART_ID, FunctionalState NewState);
void ABS_RCC_GPIO_ClockCmd(GPIO_TypeDef *GPIO_ID, FunctionalState NewState);
void ABS_RCC_DMA_ClockCmd(DMA_TypeDef *DMA_ID, FunctionalState NewState);

// ѭ������ָ�����
uint16_t ABS_CycleAddPos(uint16_t start, uint16_t step, uint16_t max);  // ���ζ���ָ����Ӻ���
uint16_t ABS_CycleSubPos(uint16_t end, uint16_t start, uint16_t max);   // ���ζ���ָ���������(�� - С)

// ��ֵ�任����
void ABS_AbsoluteLimiter(int32_t *operand, int32_t Limit);          // ����ֵ�޷�����
int32_t ABS_AbsoluteAdd(int32_t operand, int32_t addAbsoluteNum);     // ����ֵ��Ӻ���

#endif

