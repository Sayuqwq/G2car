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

/*		事件循环 EventLoop
	主要系统任务轮询的统一管理处，
	不可持有与任务有关的变量，
	不可直接使用其他对象的变量，
	只能使用规定的任务函数，
	作为系统各任务的交流处
	遇到可能需要显示的返回值，改掉函数，返回至成员变量，再其他函数参数中填成员变量
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
		// 通信
		COMM_RXbuffer_Processor();
		ESP_RXbuffer_Processor();


		// 消息处理核心
		MAIN_MsgProcess();


		// 循迹
		TRACK_GetPIDDeviationLevel();
		
		// 获取速度
		ENC_GetSpeed();

		// 运动遥控
		REMOTE_Processor();
		
		// 串级PID
		PID_Processor(encv.speedM1, encv.speedM2, trv.PID_action, trv.PID_deviationLevel, trv.basicTargetLevel);

		

		// 一些计数器和定时器
		CMNCT_timerRunning();


        // 窥视用变量
        elv.eventloop1SecMonitor++;
        elv.eventloop1SecMonitor %= elv.eventloop1SecMonitorPeriod;


		TIM_ClearITPendingBit(EVENTLOOP1T, TIM_IT_Update);
	}
}

// 500Hz, 2ms
// 借用硬pwm中断
void EVENTLOOP2_IRQHandler(void)
{
	if(TIM_GetITStatus(EVENTLOOP2T, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(EVENTLOOP2T, TIM_IT_Update);
	}
}




