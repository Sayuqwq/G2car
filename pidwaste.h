#ifndef _PID_H
#define _PID_H

#include "alias.h"

void PID_Init(void);
void PID_StatusVars_Reset(void);
int32_t PID_Processor(int16_t speed1, int16_t speed2, _Bool *trackBits);

typedef struct
{
        ////

        /*      --速度环--
        */
        float   velocityPid_Kp, 
                velocityPid_Ki, 
                velocityPid_Kd;
        int16_t	targetSpeed[ME_NUMBER];                 // 目标速度，关系复杂的变量，暂且放在pid.h中

        int32_t velocityPid_LastErr[ME_NUMBER],         // 上一次偏差

        int32_t velocityPid_Sum[ME_NUMBER];             // 积分
                velocityPid_SumLimit[ME_NUMBER];

            // EnC_Err_Lowout_last[ME_NUMBER],
                velocityPid_ErrDiff[ME_NUMBER];         // 微分
        float       filter;
        int32_t	LIMIT_constant,
                        LIMIT_linear;			// LIMIT = constant + linear * target


        int32_t Velocity_Output[ME_NUMBER],             // 速度环输出

        /*      --循迹环--
        */
        float   track_Kp,
                track_Ki,
                track_Kd;

        int16_t     motorLoadPWM[ME_NUMBER];        // 左右电机

} PIDVARS;

#endif

