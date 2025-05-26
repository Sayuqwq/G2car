#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x.h"
#include "alias.h"

void PWM_Init(uint16_t Psc, uint16_t Per);		// TIM4, µç»úPWM£¬B6, B7, B8
void PWM_HARD_SetPWM_percent(uint8_t motor, uint8_t percent);	// motor: id;  percent: -100~100
void PWM_HARD_SetPWM_actual(uint8_t motor, uint16_t value);	// motor: id;  value: 0~39??
void PWM_SOFT_SetPWM_percent(uint8_t motor, uint8_t percent);	// motor: id;  percent: -100~100
void PWM_SOFT_SetPWM_actual(uint8_t motor, uint16_t value);	// motor: id;  value: 0~39??

uint32_t PWM_GetPWMVal(uint8_t motor);

typedef struct 
{
    const uint8_t PERCENT_MAX;	// ????????????
    uint16_t PWM_MAX;

    /*  ?pwm
    */
    uint16_t SOFT_counter;
    uint16_t SOFT_compare[4];

} PWMVARS;

#endif
