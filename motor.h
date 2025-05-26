#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "encoder.h"
#include "pwm.h"

void MOTOR_Init(uint16_t Psc, uint16_t Per);	//	�����ʼ����������װ
void MOTOR_Direction_Init(void);	// ����������� B12, B13, B14
void MOTOR_SetDuty_percent(uint8_t motor, int8_t percent);
void MOTOR_SetDuty_actual(uint8_t motor, int32_t value);
void MOTOR_SetDuty_Zero(void);

uint32_t MOTOR_GetPWMVal(uint8_t motor);

//void Limit(int *motoA, int *motoB);	// �޷�����
//void Load(int moto1, int moto2);	// ����ٶ�װ�أ�PWM������ɺ������PWMֵ��
//void Stop(float *Med_Angle, float *Angle);	// �ڽǶ�ƫ����󣬵���ٶȹ���ʱֹͣ���
//FunctionalState Get_control_state(void);
//void Control_Enable(FunctionalState state);


#endif
