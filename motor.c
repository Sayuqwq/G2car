#include "stm32f10x.h"                  // Device header
#include "alias.h"
#include "motor.h"
#include "pwm.h"
#include "abstractInterfaceFunction.h"


//////// friend variables
// pwm.c
extern PWMVARS pwmv;

// void MOTOR_Direction_Init(void);
int32_t _MOTOR_Set_Direction_PWMCal(uint8_t motor, _Bool isPositive, int32_t value, int32_t limit);


void MOTOR_Init(uint16_t Psc, uint16_t Per)
{
	PWM_Init(Psc, Per);
}

void MOTOR_SetDuty_percent(uint8_t motor, int8_t percent)
{
	_Bool dir = 1;		// 先充当正负，再充当方向，0是电机默认方向，1则反转
	
	// 输入检查
	if (percent < 0)
	{
		dir = 0;
		percent = - percent;		// 已回正的value值
	}
	
	if (percent > 100)
		percent = 100;

	switch (motor)
	{
	case 1:
		dir ^= MDIR1;
		break;
	case 2:
		dir ^= MDIR2;
		break;
	case 3:
		dir ^= MDIR3;
		break;
	case 4:
		dir ^= MDIR4;
		break;
	default:
		return;
	}

	if (dir == 0)		// 电机默认方向
	{
		PWM_HARD_SetPWM_percent(motor, percent);
		PWM_SOFT_SetPWM_percent(motor, 0);
	}
	else		// 电机默认方向的反转
	{
		PWM_HARD_SetPWM_percent(motor, 0);
		PWM_SOFT_SetPWM_percent(motor, percent);
	}
}

void MOTOR_SetDuty_actual(uint8_t motor, int32_t value)		// 0 - 40
{
	_Bool dir = 1;		// 先充当正负(1正)，再充当方向，0是电机默认方向，1则反转
	
	// 输入检查
	if (value < 0)
	{
		dir = 0;
		value = - value;		// 已回正的value值
	}
	
	if (value > MDT_SOFT_MAX)
		value = MDT_SOFT_MAX;

	switch (motor)
	{
	case 1:
		dir ^= MDIR1;
		break;
	case 2:
		dir ^= MDIR2;
		break;
	case 3:
		dir ^= MDIR3;
		break;
	case 4:
		dir ^= MDIR4;
		break;
	default:
		return;
	}

	if (dir == 0)		// 电机默认方向
	{
		PWM_HARD_SetPWM_actual(motor, value);
		PWM_SOFT_SetPWM_actual(motor, 0);
	}
	else		// 电机默认方向的反转
	{
		PWM_HARD_SetPWM_actual(motor, 0);
		PWM_SOFT_SetPWM_actual(motor, value);
	}
}

// int32_t _MOTOR_Set_Direction_PWMCal(uint8_t motor, _Bool isPositive, int32_t value, int32_t limit)
// {
// 	_Bool dirPin;

// 	// 调整指定电机
// 	switch (motor)
// 	{
// 		case 1:
// 			dirPin = isPositive ^ MDIR1;
// 			GPIO_WriteBit(MDX, MD1, (BitAction)dirPin);	// 11=0, 10 01 = 1, 00=1.  0 is forward
// 			break;
		
// 		case 2:
// 			dirPin = isPositive ^ MDIR2;
// 			GPIO_WriteBit(MDX, MD2, (BitAction)dirPin);
// 			break;
		
// 		case 3:
// 			dirPin = isPositive ^ MDIR3;
// 			GPIO_WriteBit(MDX, MD3, (BitAction)dirPin);
// 			break;
		
// 		case 4:
// 			dirPin = isPositive ^ MDIR4;
// 			GPIO_WriteBit(MDX, MD4, (BitAction)dirPin);
// 			break;
// 	}

// 	if (dirPin == 0)	// 正向运转
// 		return value;
// 	else
// 		return limit - value;

// }

void MOTOR_SetDuty_Zero(void)
{
    PWM_HARD_SetPWM_percent(1, 0);
    PWM_SOFT_SetPWM_percent(1, 0);
    PWM_HARD_SetPWM_percent(3, 0);
    PWM_SOFT_SetPWM_percent(3, 0);
    
    PWM_HARD_SetPWM_percent(2, 0);
    PWM_SOFT_SetPWM_percent(2, 0);
    PWM_HARD_SetPWM_percent(4, 0);
    PWM_SOFT_SetPWM_percent(4, 0);
}

uint32_t MOTOR_GetPWMVal(uint8_t motor)
{
	return PWM_GetPWMVal(motor);
}

/*
void Limit(int *motoA, int *motoB)	// 限幅函数
{
	pwmv.PWM_MAX = PWM_LIMIT;
	PWM_MIN = -PWM_LIMIT;
	
	if(*motoA > pwmv.PWM_MAX) *motoA = pwmv.PWM_MAX;
	else if(*motoA < PWM_MIN) *motoA = PWM_MIN;
	
	if(*motoB > pwmv.PWM_MAX) *motoB = pwmv.PWM_MAX;
	else if(*motoB < PWM_MIN) *motoB = PWM_MIN;
}

void Load(int moto1, int moto2)	// 电机速度装载（PWM运算完成后的最终PWM值）
{
	if(moto1 > 0)
	{
		Ain1 = 1, Ain2 = 0;
	}
	else
	{
		Ain1 = 0, Ain2 = 1;
		moto1 = -moto1;
	}

	if(moto2 > 0)
	{
		Bin1 = 1, Bin2 = 0;
	}
	else
	{
		Bin1 = 0, Bin2 = 1;
		moto2 = -moto2;
	}
	
	Limit(&moto1, &moto2);
	
	TIM_SetCompare1(TIM1, moto1);
	TIM_SetCompare4(TIM1, moto2);
}

char PWM_Zero = 0;
FunctionalState control_state = ENABLE;

FunctionalState Get_control_state(void)
{
	return control_state;
}

void Control_Enable(FunctionalState state)
{
	if (state == ENABLE)
		control_state = ENABLE;
	else
		control_state = DISABLE;
}

void Stop(float *Med_Angle, float *Angle)	// 在角度偏差过大，电机速度过高时停止电机
{
	float diff;
	
	diff = *Angle - *Med_Angle;
	
	if (diff < -50 || diff > 50)
	{
		Control_Enable(DISABLE);
	}
}
*/


	
	
	
	
	







