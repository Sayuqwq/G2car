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


// } PID_struct_TRACKDeviationVal;		// ѭ��ƫ��ӳ������ֵ



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
	/*	�ٶȻ�		����:�����ٶ�,����������x2		���:���PWMx2
	*/
	// PID��������
	float velocityPID_Kp;
	float velocityPID_Ki;
	float velocityPID_Kd;

	int16_t	velocityPID_targetSpeed[ME_NUMBER];    	// �ٶȻ�Ŀ���ٶȣ���ϵ���ӵı��������ҷ���pid.h��
	int32_t velocityPID_err[ME_NUMBER];				// ƫ��ֵ������Ϊ��Ա�����Ե�����ʾ
	int32_t velocityPID_lastErr[ME_NUMBER];				// ��һ��ƫ��

	// ����
	int32_t velocityPID_sum[ME_NUMBER];             
	int32_t velocityPID_sumLimit;		    // �������ޣ�Ҫ������ֵ
    int32_t velocityPID_sumDecline;         // ����˥��
	// int32_t	velocityPID_sumLimitConstant;				// ��̬�������ޣ�LIMIT = constant + linear * target
	// int32_t velocityPID_sumLimitLinear;

	// ΢��
	int32_t velocityPID_errDiff[ME_NUMBER];

	// ���
	int32_t velocityPID_output[ME_NUMBER];		// �ٶȻ����

	// ����
	// float velocityPID_lowoutFilter;				// ��ͨ�˲�����



	/*	ѭ����		����:�Ҷȴ��������x5,�����Ҷȴ��������x5(��ѭ��,Ĭ��Ϊ11011)		���:�ٶ�ƫ��
	*/
	// PID��������
	float trackPID_Kp;
	float trackPID_Ki;
	float trackPID_Kd;
 
	float trackVelocityPID_targetSpeedCo;    	        // *ѭ��ģʽһ��ֱ��ʱ �ٶȻ�Ŀ���ٶ�
    float trackPID_TorqueRatio;             // ��Ť���������ļӼ���
	// int16_t	trackVelocityPID_turnTargetSpeed;    	        // *ѭ��ģʽ��תʱ �ٶȻ�Ŀ���ٶ�
    float trackPID_deviationLevelActionCoefficient[TRPID_E_TOTAL];  // ѭ��"ƫ��ȼ�"�ڲ�ͬ"����ָ��"�µ�ӳ��ϵ��

	int32_t trackPID_err;				// ƫ��ֵ������Ϊ��Ա�����Ե�����ʾ
	int32_t trackPID_lastErr;				// ��һ��ƫ��

	// ����
	int32_t trackPID_sum;             
	int32_t trackPID_sumLimit;			// �������ޣ�Ҫ������ֵ
    int32_t trackPID_sumDecline;        // ��̬����˥��
	// int32_ttrackPID_sumLimitConstant;		// ��̬�������޵�����LIMIT = constant + linear * target
	// int32_t trackPID_sumLimitLinear;

	// ΢��
	int32_t trackPID_errDiff;

	// ���
	int32_t trackPID_output;			// ѭ��������������ٶȻ����������������������

	// ����
	// float trackPID_lowoutFilter;					// ��ͨ�˲�����


	/*	����
	*/
	int16_t motorLoadPWM[ME_NUMBER];	// ����PID�����PWM
	uint8_t isEnabled;
	uint8_t isDebugEnabled;			// 2�Ű�PID����ʹ��

} PIDVARS;


#endif



