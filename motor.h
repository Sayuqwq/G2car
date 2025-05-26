#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "encoder.h"
#include "pwm.h"

void MOTOR_Init(uint16_t Psc, uint16_t Per);	//	电机初始化函数，封装
void MOTOR_Direction_Init(void);	// 电机方向引脚 B12, B13, B14
void MOTOR_SetDuty_percent(uint8_t motor, int8_t percent);
void MOTOR_SetDuty_actual(uint8_t motor, int32_t value);
void MOTOR_SetDuty_Zero(void);

uint32_t MOTOR_GetPWMVal(uint8_t motor);

//void Limit(int *motoA, int *motoB);	// 限幅函数
//void Load(int moto1, int moto2);	// 电机速度装载（PWM运算完成后的最终PWM值）
//void Stop(float *Med_Angle, float *Angle);	// 在角度偏差过大，电机速度过高时停止电机
//FunctionalState Get_control_state(void);
//void Control_Enable(FunctionalState state);


#endif
