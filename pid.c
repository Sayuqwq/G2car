#include "pid.h"
#include "timer.h"
#include "encoder.h"
#include "motor.h"
#include "Serial.h"
#include "communication.h"


/* friend visit
*/
// pwm.c
extern uint16_t PWM_MAX;
// 互斥量涉及
#include "Track.h"
#include "remote.h"
extern TRACKVARS trv;
extern REMOTEVARS rmv;
////


PIDVARS pv;

void PIDVARS_Constructor(void)
{
	/*	速度环
	*/
	pv.velocityPID_Kp = 0.5;
	pv.velocityPID_Ki = 0.025;
	pv.velocityPID_Kd = 1;
	
	pv.velocityPID_targetSpeed[0] = 0;
	pv.velocityPID_targetSpeed[1] = 0;
	pv.velocityPID_sumLimit = 1000;
    pv.velocityPID_sumDecline = 20;
	// pv.velocityPID_sumLimitConstant;
	// pv.velocityPID_sumLimitLinear;

	/*	循迹环
	*/
	pv.trackPID_Kp = 0.2;
	pv.trackPID_Ki = 0.005;
	pv.trackPID_Kd = 0.02;

    pv.trackVelocityPID_targetSpeedCo = 1;
	pv.trackPID_TorqueRatio = 1;
    pv.trackPID_sumLimit = 1000;
    pv.trackPID_sumDecline = 20;
	// pv.trackPID_sumLimitConstant;
	// pv.trackPID_sumLimitLinear;

    /*  基本构想，Track模块内部计算状态，返回"动作指导"和"偏向等级(程度)"；PID模块无需了解循迹状态，而只需要根据"动作指导"进行循迹环计算
            , 特别动作(如岔路选择，多岔路选择，通知)与路径指令(全路程的执行"代码指令") 将在Track模块内部决定，并返回"动作指导"

                "偏向等级(程度)": PID模块是根据这个等级，计算出对应的循迹环"偏差值"，作为速度环输入
                "动作指导": 用于指导PID执行的动作，如"一般偏差调节(减弱一边速度/加减两边速度)", "自转"等,
                    PID模块根据返回的"动作指导"状态"基于"设置变量velocityPID_targetSpeed联合"偏差值"进行修改，以执行希望的动作
    */

    // 循迹"偏差等级"在不同"动作指导"下的映射系数
    pv.trackPID_deviationLevelActionCoefficient[TRPID_E_CORRECTION_TENDENCY] = 2;
    pv.trackPID_deviationLevelActionCoefficient[TRPID_E_CORRECTION_TORQUE] = 2;
    pv.trackPID_deviationLevelActionCoefficient[TRPID_E_ROTATE] = 5;
    pv.trackPID_deviationLevelActionCoefficient[TRPID_E_STRAIGHT] = 1;

	/*	其他
	*/
	pv.isEnabled = 1;
	pv.isDebugEnabled = 1;
}

int32_t _PID_TrackCorrectionVal(void);      // 保留返回值，实际上可以剔除
int32_t _PID_Velocity(uint8_t motor, int32_t Target, int32_t Speed);


void PID_Init(void)
{
	// 初始化全局变量
	PIDVARS_Constructor();
}

void PID_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t uint8Num, first;
	uint8_t *pointer = data;
    int32_t int32Num;
    float floatNum;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(first));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
	switch (first)
	{
	case (uint8_t)PID_E_UNITPROTOCAL_ENABLE:
		pv.isEnabled = uint8Num;
		break;
	case (uint8_t)PID_E_UNITPROTOCOL_DEBUG_ENABLE:
        MOTOR_SetDuty_Zero();
		pv.isDebugEnabled = uint8Num;		// 互斥量, pidDebug, track, remote
		if (uint8Num == 1)
		{
			trv.isEnabled = 0;
			rmv.isEnabled = 0;
		}
		break;
    case (uint8_t)PID_E_UNITPROTOCOL_PARAMS:
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &pointer, sizeof(floatNum));        // int32_t 与 float 大小一致
        switch (uint8Num)
        {
        case (uint8_t)PID_E_UP_PARAMS_VKP:
            pv.velocityPID_Kp = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_VKI:
            pv.velocityPID_Ki = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_VKD:
            pv.velocityPID_Kd = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_VKILIMIT:
            memcpy(&int32Num, &floatNum, sizeof(floatNum));
            pv.velocityPID_sumLimit = int32Num;
            break;
        case (uint8_t)PID_E_UP_PARAMS_VSUMDECLINE:
            memcpy(&int32Num, &floatNum, sizeof(floatNum));
            pv.velocityPID_sumDecline = int32Num;
            break;

        case (uint8_t)PID_E_UP_PARAMS_TKP:
            pv.trackPID_Kp = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TKI:
            pv.trackPID_Ki = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TKD:
            pv.trackPID_Kd = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TKILIMIT:
            memcpy(&int32Num, &floatNum, sizeof(floatNum));
            pv.trackPID_sumLimit = int32Num;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TSUMDECLINE:
            memcpy(&int32Num, &floatNum, sizeof(floatNum));
            pv.trackPID_sumDecline = int32Num;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TTARGETSPEEDCO:
            pv.trackVelocityPID_targetSpeedCo = floatNum;
            break;
        case (uint8_t)PID_E_UP_PARAMS_TTORQUERATIO:
            pv.trackPID_TorqueRatio = floatNum;
            break;
        }
        break;
	}
}

void PID_Processor(int16_t speed1, int16_t speed2, TRACK_enum_TRPID_ACTION trackAction, int16_t trackDeviationLevel, int16_t trackBasicTargetSpeed)    // 串级PID
{
	// 1.若选择左右轮一体计算，不稳定
	// 		X 无法使用串级PID，无法将track纳入速度环期望(track as speed)
	//		O 若单纯使用加法(track as PWM)，可行

	// 2.若选择左右轮独立计算
	// 		O 若使用串级PID，将track纳入速度环期望(track as speed)，则可根据循迹精确控制小车的速度
	//		X 若单纯使用加法(track as PWM)，积分的补偿会使循迹扭力会毫无作用，小车会“顽固速度环”

	// ----选择左右轮独立计算---- //


	// pid功能使能
	if (pv.isEnabled == 0)
    {
        PID_StatusVars_Reset();
        pv.motorLoadPWM[0] = pv.motorLoadPWM[1] = 0;    // 清除串级PID的结果装载PWM值

		return;
    }

    // 循迹功能使能
    if (trv.isEnabled == 1)     // 循迹 开
    {
        // 映射偏差值
        pv.trackPID_err = trackDeviationLevel * pv.trackPID_deviationLevelActionCoefficient[trackAction];

        // 进行循迹环计算
        pv.trackPID_output = _PID_TrackCorrectionVal();

        // 实现动作，并实现修正(修改速度环输入)
        switch (trackAction)
        {
        case TRPID_E_CORRECTION_TENDENCY:
            pv.velocityPID_targetSpeed[0] = trackBasicTargetSpeed * pv.trackVelocityPID_targetSpeedCo;
            pv.velocityPID_targetSpeed[1] = trackBasicTargetSpeed * pv.trackVelocityPID_targetSpeedCo;

            if (pv.trackPID_output < 0)        // 左偏，左轮减速
                pv.velocityPID_targetSpeed[0] += pv.trackPID_output;
            else                                // 右偏，右轮减速
                pv.velocityPID_targetSpeed[1] -= pv.trackPID_output;

            break;

        case TRPID_E_CORRECTION_TORQUE:
            pv.velocityPID_targetSpeed[0] = trackBasicTargetSpeed * pv.trackVelocityPID_targetSpeedCo;
            pv.velocityPID_targetSpeed[1] = trackBasicTargetSpeed * pv.trackVelocityPID_targetSpeedCo;

            int32_t speedPositive = pv.trackPID_output * pv.trackPID_TorqueRatio;

            if (pv.trackPID_output < 0)        // 左偏，左轮减速, 右轮加速
            {
                pv.velocityPID_targetSpeed[0] += pv.trackPID_output;
                pv.velocityPID_targetSpeed[1] -= speedPositive;
            }
            else                                // 右偏，右轮减速，左轮加速
            {
                pv.velocityPID_targetSpeed[0] += speedPositive;
                pv.velocityPID_targetSpeed[1] -= pv.trackPID_output;
            }

            break;

        case TRPID_E_ROTATE:
            pv.velocityPID_targetSpeed[0] = pv.trackPID_output;
            pv.velocityPID_targetSpeed[1] = -pv.trackPID_output;

            break;

        case TRPID_E_STRAIGHT:      // 直接操控运动
            pv.velocityPID_targetSpeed[0] = trackDeviationLevel * pv.trackVelocityPID_targetSpeedCo;
            pv.velocityPID_targetSpeed[1] = trackDeviationLevel * pv.trackVelocityPID_targetSpeedCo;

            PID_TRACK_StatusVars_Reset();   // 失能循迹环计算

            break;
        }
    }
    else                        // 循迹 关
    {
        PID_TRACK_StatusVars_Reset();   // track关闭时，清空所有状态与结果，不修改其他任何有关值，将不会产生偏差值（扭力）
    }

    // PID速度环统一接口，只接收 电机id，输入，期望 作为参数，其他不管
    pv.motorLoadPWM[0] = pv.velocityPID_output[0] = 
        _PID_Velocity(1, pv.velocityPID_targetSpeed[0], speed1);
    pv.motorLoadPWM[1] = pv.velocityPID_output[1] = 
        _PID_Velocity(2, pv.velocityPID_targetSpeed[1], speed2);

	// pv.trackPID_lastState = trackState;

	// pwm装载至电机
	MOTOR_SetDuty_actual(1, pv.motorLoadPWM[0]);
	MOTOR_SetDuty_actual(3, pv.motorLoadPWM[0]);

	MOTOR_SetDuty_actual(2, pv.motorLoadPWM[1]);
	MOTOR_SetDuty_actual(4, pv.motorLoadPWM[1]);
}

// 循迹环
int32_t _PID_TrackCorrectionVal(void)		// 字节内大端存储 -> 12345678 ->
{
	float output;
    int32_t err = pv.trackPID_err;    // 局部变量运行快

	pv.trackPID_errDiff = err - pv.trackPID_lastErr;		// 微分
	pv.trackPID_lastErr = err;

    if (err == 0)
        pv.trackPID_sum = ABS_AbsoluteAdd(pv.trackPID_sum, -pv.trackPID_sumDecline);     // 静态积分衰减
    else
	    pv.trackPID_sum += err;				// 积分
	ABS_AbsoluteLimiter(&pv.trackPID_sum, pv.trackPID_sumLimit);    // 限幅

	// PID计算
	output = pv.trackPID_Kp * err
			+ pv.trackPID_Ki * pv.trackPID_sum
			- pv.trackPID_Kd * pv.trackPID_errDiff;

	return (int32_t)output;
}

// 速度环 
int32_t _PID_Velocity(uint8_t motor, int32_t Target, int32_t Speed)		// 速度大约0 ~ 1800
{
	uint8_t index = motor - 1;
	int32_t Output;

	// 1.计算速度偏差
	pv.velocityPID_err[index] = Target - Speed;
	
		// 1a.对速度偏差进行低通滤波，是波形更加平滑；滤除高频信号（速度过大/速度突变），防止影响直立环的正常工作
//	// low_out = (1 - a) * Ek + a * low_out_last
//	EnC_Err_Lowout = (1 - pv.filter) * Encoder_Err + pv.filter * pv.EnC_Err_Lowout_last;
//	pv.EnC_Err_Lowout_last = EnC_Err_Lowout;

	// 2.计算微分
	pv.velocityPID_errDiff[index] = pv.velocityPID_err[index] - pv.velocityPID_lastErr[index];
	
		// 保存本次偏差
	pv.velocityPID_lastErr[index] = pv.velocityPID_err[index];
	
	// 3.对速度偏差积分，积分出位移
//	pv.velocityPID_sum += EnC_Err_Lowout;
	pv.velocityPID_sum[index] += pv.velocityPID_err[index];
	
	// 4.积分限幅
	// pv.velocityPID_sumLimit[index] = pv.velocityPID_sumLimitconstant +  Target * pv.LIMIT_linear;	// 动态上限
	ABS_AbsoluteLimiter(&pv.velocityPID_sum[index], pv.velocityPID_sumLimit);
		
	// 5.速度环PID计算
	Output = pv.velocityPID_Kp * pv.velocityPID_err[index] 
		+ pv.velocityPID_Ki * pv.velocityPID_sum[index] 
		- pv.velocityPID_Kd * pv.velocityPID_errDiff[index];
	
	return Output;
}

// 功能函数
void PID_StatusVars_Reset(void)
{
	for (uint8_t i = 0; i < ME_NUMBER; i++)
	{
        pv.velocityPID_targetSpeed[i] = 0;
		pv.velocityPID_err[i] = 0;
		pv.velocityPID_lastErr[i] = 0;
		pv.velocityPID_sum[i] = 0;
		pv.velocityPID_errDiff[i] = 0;
		pv.velocityPID_output[i] = 0;
	}
}

void PID_TRACK_StatusVars_Reset(void)
{
    pv.trackPID_err = 0;
    pv.trackPID_lastErr = 0;
    pv.trackPID_sum = 0;
    pv.trackPID_errDiff = 0;
    pv.trackPID_output = 0;
}

// 速度环积分量衰减
void PID_VelocitySumDecline(void)
{
    pv.velocityPID_sum[0] = ABS_AbsoluteAdd(pv.velocityPID_sum[0], -pv.velocityPID_sumDecline);
    pv.velocityPID_sum[1] = ABS_AbsoluteAdd(pv.velocityPID_sum[1], -pv.velocityPID_sumDecline);
}


	// pv.trackPID_deviationValMapping[TRS_DEVIATION_NO] 		= 0;
	// pv.trackPID_deviationValMapping[TRS_DEVIATION_LEFT_LITTLE] 		= -2;
	// pv.trackPID_deviationValMapping[TRS_DEVIATION_LEFT_BIG] 		= -5;
	// pv.trackPID_deviationValMapping[TRS_DEVIATION_RIGHT_LITTLE] 	= 2;
	// pv.trackPID_deviationValMapping[TRS_DEVIATION_RIGHT_BIG] 		= 5;
	// pv.trackPID_deviationValMapping[TRS_JUNCTION_Y_TURN_LEFT] 		= 2;
	// pv.trackPID_deviationValMapping[TRS_JUNCTION_Y_TURN_RIGHT] 		= -2;

	// pv.trackPID_crossingSelectTurnSpeed[0]		= -40;
	// pv.trackPID_crossingSelectTurnSpeed[1]		= 40;

	// 连续变化状态：普通单线循迹

	// 非连续状态：路线中断
	// pv.trackPID_deviationNormalScore[4] = -1;		// W W W W W	停止，检测不到路线
	// pv.trackPID_deviationNormalScore[5] = -2;		// ? B B B ?	停止，踩到停止标

	// 非连续状态：岔路与急转
	// pv.trackPID_deviationNormalScore[6] = -9;		// W ? B ? B
	// pv.trackPID_deviationNormalScore[7] = -10;	// W 



