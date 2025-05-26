#ifndef __TIMER_H
#define __TIMER_H			

#include "stm32f10x.h"                  // Device header
#include "abstractInterfaceFunction.h"

void TIMER_Init(TIM_TypeDef* TIMx, uint16_t psc, uint16_t per, uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority);	// ���ȼ����ã�����һ��255��ر��ж�
void TIMER_EnableInterrupt(TIM_TypeDef* TIMx, uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority);


#endif





























