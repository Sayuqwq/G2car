#ifndef _PID_H
#define _PID_H

#include "stm32f10x.h"
#include "alias.h"
#include "abstractInterfaceFunction.h"
#include "Track.h"

void PID_Init(void);
void PID_Processor(int16_t speed1, int16_t speed2, TRACK_enum_TRPID_ACTION trackAction, int16_t trackDeviationLevel, int16_t trackBasicTargetSpeed);
void PID_StatusVars_Reset(void);
void PID_TRACK_StatusVars_Reset(void);
void PID_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);

void PID_VelocitySumDecline(void);

// typedef struct 
// {


// } PID_struct_TRACKDeviationVal;		// 循迹偏差映射量化值



// typedef struct 
// {


// } PID_TRACK_struct_CrossingState;

typedef enum
{
	PID_E_UNITPROTOCOL_RESERVED = 0,

	PID_E_UNITPROTOCAL_ENABLE,
	PID_E_UNITPROTOCOL_DEBUG_ENABLE,
	PID_E_UNITPROTOCOL_PARAMS

} PID_enum_UNITPROTOCOL_SUBMODESELECT;

typedef enum
{
    PID_E_UP_PARAMS_RESERVED = 0,

    PID_E_UP_PARAMS_VKP = 1,
    PID_E_UP_PARAMS_VKI,
    PID_E_UP_PARAMS_VKD,
    PID_E_UP_PARAMS_VKILIMIT,
    PID_E_UP_PARAMS_VSUMDECLINE,

    PID_E_UP_PARAMS_TKP = 9,
    PID_E_UP_PARAMS_TKI,
    PID_E_UP_PARAMS_TKD,
    PID_E_UP_PARAMS_TKILIMIT,
    PID_E_UP_PARAMS_TSUMDECLINE,
    PID_E_UP_PARAMS_TTARGETSPEEDCO,
    PID_E_UP_PARAMS_TTORQUERATIO

} PID_enum_UNITPROTOCOL_PARAMS_ID;

typedef struct
{
	/*	速度环		输入:期望速度,编码器测速x2		输出:电机PWMx2
	*/
	// PID基本配置
	float velocityPID_Kp;
	float velocityPID_Ki;
	float velocityPID_Kd;

	int16_t	velocityPID_targetSpeed[ME_NUMBER];    	// 速度环目标速度，关系复杂的变量，暂且放在pid.h中
	int32_t velocityPID_err[ME_NUMBER];				// 偏差值，定义为成员变量以调试显示
	int32_t velocityPID_lastErr[ME_NUMBER];				// 上一次偏差

	// 积分
	int32_t velocityPID_sum[ME_NUMBER];             
	int32_t velocityPID_sumLimit;		    // 积分上限，要求是正值
    int32_t velocityPID_sumDecline;         // 积分衰减
	// int32_t	velocityPID_sumLimitConstant;				// 动态积分上限，LIMIT = constant + linear * target
	// int32_t velocityPID_sumLimitLinear;

	// 微分
	int32_t velocityPID_errDiff[ME_NUMBER];

	// 输出
	int32_t velocityPID_output[ME_NUMBER];		// 速度环输出

	// 其他
	// float velocityPID_lowoutFilter;				// 低通滤波因子



	/*	循迹环		输入:灰度传感器情况x5,期望灰度传感器情况x5(即循迹,默认为11011)		输出:速度偏差
	*/
	// PID基本配置
	float trackPID_Kp;
	float trackPID_Ki;
	float trackPID_Kd;
 
	float trackVelocityPID_targetSpeedCo;    	        // *循迹模式一般直行时 速度环目标速度
    float trackPID_TorqueRatio;             // “扭力”动作的加减比
	// int16_t	trackVelocityPID_turnTargetSpeed;    	        // *循迹模式自转时 速度环目标速度
    float trackPID_deviationLevelActionCoefficient[TRPID_E_TOTAL];  // 循迹"偏差等级"在不同"动作指导"下的映射系数

	int32_t trackPID_err;				// 偏差值，定义为成员变量以调试显示
	int32_t trackPID_lastErr;				// 上一次偏差

	// 积分
	int32_t trackPID_sum;             
	int32_t trackPID_sumLimit;			// 积分上限，要求是正值
    int32_t trackPID_sumDecline;        // 稳态积分衰减
	// int32_ttrackPID_sumLimitConstant;		// 动态积分上限调整，LIMIT = constant + linear * target
	// int32_t trackPID_sumLimitLinear;

	// 微分
	int32_t trackPID_errDiff;

	// 输出
	int32_t trackPID_output;			// 循迹环输出，由于速度环输入输出须整数，故整数

	// 其他
	// float trackPID_lowoutFilter;					// 低通滤波因子


	/*	其他
	*/
	int16_t motorLoadPWM[ME_NUMBER];	// 串级PID输出的PWM
	uint8_t isEnabled;
	uint8_t isDebugEnabled;			// 2号包PID调参使能

} PIDVARS;


#endif



