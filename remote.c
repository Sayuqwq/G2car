#include "remote.h"
#include "communication.h"

#include "motor.h"

/*  friend visit
*/
// 互斥量
#include "pid.h"
#include "Track.h"
extern PIDVARS pv;
extern TRACKVARS trv;
////

REMOTEVARS rmv;

void REMOTEVARS_Constructor(void)
{
    rmv.forwardSpeed = 30;
    rmv.backwardSpeed = 30;
    rmv.rotateSpeed = 25;
    rmv.tendencySpeed = 20;

    rmv.isEnabled = 0;
}

void REMOTE_Init(void)
{
    REMOTEVARS_Constructor();
}

void _REMOTE_Reset(void);


void REMOTE_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t uint8Num, first;
    int8_t int8Num;
	uint8_t *pointer = data;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(uint8Num));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
	switch (first)
	{
	case (uint8_t)REMOTE_E_UNITPROTOCAL_ENABLE:
        PID_StatusVars_Reset();     // pid归零
        MOTOR_SetDuty_Zero();       // 电机pwm归零
        _REMOTE_Reset();
        
		rmv.isEnabled = uint8Num;       // 互斥量, pidDebug, track, remote
		if (uint8Num == 1)
		{
			trv.isEnabled = 0;
			pv.isDebugEnabled = 0;
		}
		break;
    case REMOTE_E_UNITPROTOCAL_ACTION:
        rmv.mergedBits = uint8Num;
        break;
    case REMOTE_E_UNITPROTOCOL_PARAMS:
        CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int8Num, &pointer, sizeof(int8Num));
        switch (uint8Num)
        {
        case REMOTE_E_UP_PARAMS_FORWARDSPEED:
            rmv.forwardSpeed = int8Num;
            break;
        case REMOTE_E_UP_PARAMS_BACKWARDSPEED:
            rmv.backwardSpeed = int8Num;
            break;
        case REMOTE_E_UP_PARAMS_ROTATESPEED:
            rmv.rotateSpeed = int8Num;
            break;
        case REMOTE_E_UP_PARAMS_TENDENCYSPEED:
            rmv.tendencySpeed = int8Num;
            break;
        }
        break;
	}
}

void REMOTE_Processor(void)
{
    // 功能使能
    if (rmv.isEnabled == 0)
        return;

    rmv.fbBits = rmv.mergedBits >> 2;
    rmv.lrBits = rmv.mergedBits & 0x03;

    switch (rmv.fbBits)
    {
    case 0:     // 停止，特殊：停止时特殊判断左右以自转, 判断完就返回
        if (rmv.lrBits == 1)        // 左自转
        {
            pv.velocityPID_targetSpeed[0] = -rmv.rotateSpeed;
            pv.velocityPID_targetSpeed[1] = +rmv.rotateSpeed;
            return;
        }
        else if (rmv.lrBits == 2)   // 右自转
        {
            pv.velocityPID_targetSpeed[0] = +rmv.rotateSpeed;
            pv.velocityPID_targetSpeed[1] = -rmv.rotateSpeed;
            return;
        }
        else
            pv.velocityPID_targetSpeed[0] = pv.velocityPID_targetSpeed[1] = 0;
            // PID_StatusVars_Reset();     // 此函数直接清零
            PID_VelocitySumDecline();   // 稳态积分衰减
        break;

    case 1:     // 后退
        pv.velocityPID_targetSpeed[0] = pv.velocityPID_targetSpeed[1] = -rmv.backwardSpeed;
        break;

    case 2:     // 前进
        pv.velocityPID_targetSpeed[0] = pv.velocityPID_targetSpeed[1] = rmv.forwardSpeed;
        break;
    }

    switch (rmv.lrBits)
    {
    case 0:     // 停止
        break;

    case 1:     // 左旋趋势, 绝对值--
        pv.velocityPID_targetSpeed[0] = ABS_AbsoluteAdd(pv.velocityPID_targetSpeed[0], -rmv.tendencySpeed);
        break;

    case 2:     // 右旋趋势
        pv.velocityPID_targetSpeed[1] = ABS_AbsoluteAdd(pv.velocityPID_targetSpeed[1], -rmv.tendencySpeed);
        break;
    }
}


/*  功能函数
*/
void _REMOTE_Reset(void)
{
    rmv.fbBits = 0;
    rmv.lrBits = 0;
    rmv.mergedBits = 0;
}



