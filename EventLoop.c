#include "EventLoop.h"
#include "timer.h"
#include "COMM.h"
#include "ESP_01S.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "pwm.h"
#include "Track.h"
#include "main.h"
#include "remote.h"
#include "communication.h"

/*		�¼�ѭ�� EventLoop
	��Ҫϵͳ������ѯ��ͳһ������
	���ɳ����������йصı�����
	����ֱ��ʹ����������ı�����
	ֻ��ʹ�ù涨����������
	��Ϊϵͳ������Ľ�����
	����������Ҫ��ʾ�ķ���ֵ���ĵ���������������Ա�������������������������Ա����
*/

EVENTLOOPVARS elv = {
    .eventloop1SecMonitorPeriod = 100
};

void EVENTLOOPVARS_Constructor(void)
{
    elv.eventloop1SecMonitor = 0;
}


void EVENTLOOP_Init(void)
{
    EVENTLOOPVARS_Constructor();

	TIMER_Init(EVENTLOOP1T, 
			EVENTLOOP1_PSC, 
			EVENTLOOP1_PER, 
			EVENTLOOP1_NVIC_PRIORITY_PREEMPTION, 
			EVENTLOOP1_NVIC_PRIORITY_SUB);

	TIMER_EnableInterrupt(EVENTLOOP2T,
			EVENTLOOP2_NVIC_PRIORITY_PREEMPTION,
			EVENTLOOP2_NVIC_PRIORITY_SUB);
}

/* friend visit
*/
extern ENCODERVARS encv;
extern TRACKVARS trv;
extern PIDVARS pv;
extern MAINVARS mv;

// 100Hz, 10ms
void EVENTLOOP1_IRQHandler(void)
{
	if(TIM_GetITStatus(EVENTLOOP1T, TIM_IT_Update) == SET)
	{
		// ͨ��
		COMM_RXbuffer_Processor();
		ESP_RXbuffer_Processor();


		// ��Ϣ�������
		MAIN_MsgProcess();


		// ѭ��
		TRACK_GetPIDDeviationLevel();
		
		// ��ȡ�ٶ�
		ENC_GetSpeed();

		// �˶�ң��
		REMOTE_Processor();
		
		// ����PID
		PID_Processor(encv.speedM1, encv.speedM2, trv.PID_action, trv.PID_deviationLevel, trv.basicTargetLevel);

		

		// һЩ�������Ͷ�ʱ��
		CMNCT_timerRunning();


        // �����ñ���
        elv.eventloop1SecMonitor++;
        elv.eventloop1SecMonitor %= elv.eventloop1SecMonitorPeriod;


		TIM_ClearITPendingBit(EVENTLOOP1T, TIM_IT_Update);
	}
}

// 500Hz, 2ms
// ����Ӳpwm�ж�
void EVENTLOOP2_IRQHandler(void)
{
	if(TIM_GetITStatus(EVENTLOOP2T, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(EVENTLOOP2T, TIM_IT_Update);
	}
}




