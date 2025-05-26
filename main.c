#include "main.h"
#include "EventLoop.h"
#include "motor.h"
#include "OLED.h"
#include "delay.h"
#include "encoder.h"
#include "pid.h"
#include "Serial.h"
// #include "COMM.h"
// #include "ESP_01S.h"
#include "OV7670.h"
#include "communication.h"
#include "Key.h"
#include "abstractInterfaceFunction.h"

#include <string.h>

#define TRUE 1
#define FALSE 0

// friend visit
////
#include "Track.h"
#include "remote.h"
extern COMMVARS cv;
extern ESPVARS ev;
extern PIDVARS pv;
extern ENCODERVARS encv;
extern PWMVARS pwmv;
extern TRACKVARS trv;
extern REMOTEVARS rmv;
extern CMNCTVARS cmnv;
extern OVVARS ovv;
extern EVENTLOOPVARS elv;
//

MAINVARS mv;

void MAINVARS_Constructor(void)
{
	mv.keyNum = 255;
    // mv.COMM_MsgFlag;
	// mv.COMM_RxLen;
    // mv.ESP_MsgFlag;
	// mv.ESP_RxLen;
    mv.Page = (uint8_t)OP_UNITPROTOCOL_RELATED_2;
    // mv.s[2];
	// mv.s1Max[2];
}


/*	main.c��������	*/
void _OLED_Parameters_Display(uint8_t page);
uint8_t _MAIN_CopyCOMMMsg(void);
uint8_t _MAIN_CopyESPMsg(void);
uint8_t *_MAIN_COMMGetMsg(COMM_struct_packInfo *msgInfo);
uint8_t *_MAIN_ESPGetMsg(ESP_struct_packInfo *msgInfo);
// void _MAIN_ESPMsgRestore(ESP_struct_packInfo *tempInfo, uint8_t *container);
	// ������Ѱ����Ϣ�����Ժ���
uint8_t MAIN_ESP_FindNewMsg(char *match);		// ����ƥ�����Ϣ���³̶�, 1������, 0��ʧ��
uint8_t MAIN_ESP_FindBufferedMsg(char *match);

void _MAIN_OLED_AddAndShowESPInitProgress(void);


int main(void)
{
/* 		main�ֲ�����
*/	
	// uint8_t COMMPackCount;
	// uint8_t ESPPackCount;

	// uint8_t tempLen;
	
/*		��ʼ��mainȫ�ֱ���
*/
	MAINVARS_Constructor();

/* 		�����ʼ��
*/
	delay_init();

	OLED_Init();
	KEY_Init();

	MOTOR_Init(MPT_PSC, MPT_PER);		// 500Hz, "Ƶ��̫�߻᲻ת��̫�Ͳⲻ׼"
	ENC_Init();
	PID_Init();
    TRACK_Init();

	EVENTLOOP_Init();
	COMM_Init(115200);
	ESP_Init(1000000);

	CMNCT_Init();
	REMOTE_Init();

	OV7670_Init();

		OLED_ShowString(1, 1, "Hello");		// ������

/*		��ʼ������
*/
	// һ��ͨ��
	// Serial_SendString_Patch("Hello!\n", 7, COMM_USART);

	// ���
	MOTOR_SetDuty_actual(1, 0);
	MOTOR_SetDuty_actual(2, 0);
	MOTOR_SetDuty_actual(3, 0);
	MOTOR_SetDuty_actual(4, 0);
	
	// ATָ�������
	// delay_ms(20);
	// CMNCT_ESP_ATSEND_toTransparent(1);

	// DMA����
	// Serial_DMACopySend("sent by DMA.", 12, COMM_USART);		// DMA���Ͳ��ԣ��ɹ�
	// delay_ms(10);

// ESP��ʼ��ATָ��(������)
	// ATָ��Ԥ���ͣ�ÿ�ζ�����ϵ���������Ҫ�������ֶ�/�Զ������������������ӷ����������ӷ���������Զ�����͸��ģʽ
	Serial_SendString("+++", ESP_USART);
	delay_ms(1000);
	_MAIN_OLED_AddAndShowESPInitProgress();
	
	Serial_SendString("\r\nAT+RST\r\n", ESP_USART);	// �ܲ����ˣ��ֶ�����
	while (MAIN_ESP_FindBufferedMsg("ready") == 0);		// �ȴ�ESP��ʼ�����
	_MAIN_OLED_AddAndShowESPInitProgress();

	Serial_SendString("ATE0\r\n", ESP_USART);
	while (MAIN_ESP_FindNewMsg("OK") == 0);
	_MAIN_OLED_AddAndShowESPInitProgress();
	
	// Serial_SendString("AT+CIPCLOSE\r\n", ESP_USART);
	// while (MAIN_ESP_FindNewMsg("OK") == 0 && MAIN_ESP_FindNewMsg("ERROR") == 0);
	// _MAIN_OLED_AddAndShowESPInitProgress();

	Serial_SendString("AT+CWMODE_DEF=1\r\n", ESP_USART);
	while (MAIN_ESP_FindNewMsg("OK") == 0);
	_MAIN_OLED_AddAndShowESPInitProgress();
	
	Serial_SendString("AT+CIPMODE=0\r\n", ESP_USART);
	while (MAIN_ESP_FindNewMsg("OK") == 0);
	_MAIN_OLED_AddAndShowESPInitProgress();
	
	OLED_ShowString(3, 2, "Wifi...");
	while (MAIN_ESP_FindNewMsg("WIFI GOT IP") == 0);		// ֻ��receiveCmd�Ŀ�while�Ῠ������֪�ǲ����Ż�����
	_MAIN_OLED_AddAndShowESPInitProgress();
////

/*		��ѭ��
*/
	while (1)
	{
		// Key
		mv.keyNum = KEY_GetNum();
		if (mv.keyNum == 0)		// ҳ��
		{
			mv.Page++;
			mv.Page %= OP_TOTAL;
		}
		else if (mv.keyNum == 1)	// ҳ��
		{
			if (mv.Page == 0)
				mv.Page = OP_TOTAL;
			mv.Page--;
		}

		// OV7670������,��֡����
		OV7670_Work();

		// OLEDҳ��ʾ������ֻ������ʾ����������в���
		_OLED_Parameters_Display(mv.Page);
	}
//	
//	Motor_SetSpeed(3, 0);
//	OLED_ShowNum(4, 1, counter, 5);
}

void MAIN_MsgProcess(void)		// ��ʱ����main������Ǩ��CMNCT
{
	COMM_struct_packInfo *tempCOMMInfo;
	ESP_struct_packInfo *tempESPInfo;

	// COMM
	mv.COMM_newMsgCount = _MAIN_CopyCOMMMsg();		// ��������#2��Ƕ��forѭ��ʹ��ͬһ i
	for (uint8_t i = 0; i < mv.COMM_newMsgCount; i++)
	{
		tempCOMMInfo = mv.COMM_MsgInfo + ABS_CycleSubPos(mv.COMM_pMsgInfo, mv.COMM_newMsgCount - i, MAIN_COMM_MSG_NUMBER);	// rewind
		mv.COMM_msgHandler[i] = _MAIN_COMMGetMsg(tempCOMMInfo);
		
		if (tempCOMMInfo->flag == 1)
		{
			Serial_SendString_Patch("COMM: ", 6, COMM_USART);
			Serial_SendString_Patch((char *)mv.COMM_msgHandler[i], tempCOMMInfo->len, COMM_USART);	// COMM_ESP����ת����ESP
			Serial_SendString_Patch("\n", 1, COMM_USART);
		}
		else if (tempCOMMInfo->flag == 2)
		{
			Serial_SendString_Patch("COMM: ", 6, COMM_USART);
			CMNCT_Parameters_Modify(mv.COMM_msgHandler[i], tempCOMMInfo->len);
			Serial_SendString_Patch("\n", 1, COMM_USART);
		}
		else if (tempCOMMInfo->flag == 3)
		{
			Serial_SendString_Patch((char *)mv.COMM_msgHandler[i], tempCOMMInfo->len, ESP_USART);	// COMM_ESP����ת����ESP
		}
		else if (tempCOMMInfo->flag == 4)
		{
			CMNCT_UnitProtocolResolve(mv.COMM_msgHandler[i], tempCOMMInfo->len);
		}
	}

	// ESP
	mv.ESP_newMsgCount = _MAIN_CopyESPMsg();
	for (uint8_t i = 0; i < mv.ESP_newMsgCount; i++)
	{
		Serial_SendString_Patch("ESP:  ", 6, COMM_USART);

		tempESPInfo = mv.ESP_MsgInfo + ABS_CycleSubPos(mv.ESP_pMsgInfo, mv.ESP_newMsgCount - i, MAIN_ESP_MSG_NUMBER);
		mv.ESP_msgHandler[i] = (uint8_t *)_MAIN_ESPGetMsg(tempESPInfo);
		
		if (tempESPInfo->flag == 1)
		{
			Serial_SendString_Patch((char *)mv.ESP_msgHandler[i], tempESPInfo->len - 1, COMM_USART);	// ESP����ת����COMM	// ��ȥ'\0'
			CMNCT_ESPATCommandProcess(mv.ESP_msgHandler[i], tempESPInfo->len);
		}
		
		Serial_SendString_Patch("\n", 1, COMM_USART);
	}
}


/*		OLED��ʾ����
*/
void _OLED_Parameters_Display(uint8_t page)
{
/*		�ֲ�����
*/
	// Page_1
	static uint8_t i_1 = 2;
	
	// Page 2
	uint8_t i_2 = 0;

	// Page 3
	int i_3;

	// OP_COMM_RX
	int i_4;
	COMM_struct_packInfo *tempCOMMInfo;
	uint8_t tempCOMMLen;

	// OP_ESP_RX
	int i_5;
	ESP_struct_packInfo *tempESPInfo;
	uint8_t tempESPLen;
	
	// Page 6
	int i_6;
	
	// Page 7
	int i_7;
	
	// Common
	static uint8_t Last_Page = 255;
	static OLED_enum_PAGE OP_page;
	uint8_t i, j;
	
/*		��ҳ����
*/
	if (Last_Page != page)
	{
		Last_Page = page;
		OLED_Clear();
	}

/*		ҳ��ʾ����
*/
	OP_page = (OLED_enum_PAGE)page;
	switch (OP_page)
	{
		case OP_PID_ENC:		// pid ������������״̬
			OLED_ShowString(1, 1, "S:");	// �ٶ�
			OLED_ShowString(2, 1, "O:");	// ���
			OLED_ShowString(3, 1, "I:");	// ����
			OLED_ShowString(4, 1, "D:");	// ΢��

			OLED_ShowSignedNum(1, 3, encv.speedM1, 2);
			OLED_ShowSignedNum(1, 10, encv.speedM2, 2);
			OLED_ShowSignedNum(1, 6, pv.velocityPID_err[0], 2);
			OLED_ShowSignedNum(1, 13, pv.velocityPID_err[1], 2);
			OLED_ShowSignedNum(2, 3, pv.velocityPID_output[0], 3);
			OLED_ShowSignedNum(2, 10, pv.velocityPID_output[1], 3);
			OLED_ShowSignedNum(3, 3, pv.velocityPID_sum[0], 5);
			OLED_ShowSignedNum(3, 10, pv.velocityPID_sum[1], 5);
			OLED_ShowSignedNum(4, 3, pv.velocityPID_errDiff[0], 3);
			OLED_ShowSignedNum(4, 10, pv.velocityPID_errDiff[1], 3);
			
			break;

		case OP_PID_CONFIG:		// PID ���ã�(���ð�ʹ��2�Ű�ͷβ)
			OLED_ShowFloatNum(1, 1, pv.velocityPID_Kp, 4, 2);
			OLED_ShowFloatNum(2, 1, pv.velocityPID_Ki, 4, 2);
			OLED_ShowFloatNum(3, 1, pv.velocityPID_Kd, 4, 2);
			OLED_ShowSignedNum(4, 1, pv.velocityPID_targetSpeed[0], 5);

			OLED_ShowFloatNum(1, 9, pv.velocityPID_Kp, 4, 2);
			OLED_ShowFloatNum(2, 9, pv.velocityPID_Ki, 4, 2);
			OLED_ShowFloatNum(3, 9, pv.velocityPID_Kd, 4, 2);
			OLED_ShowSignedNum(4, 9, pv.velocityPID_targetSpeed[1], 5);

			// OLED_ShowSignedNum(2, 10, pv.target3, 5);
			// OLED_ShowSignedNum(3, 10, pv.LIMIT_constant, 6);
			// OLED_ShowSignedNum(4, 10, pv.LIMIT_linear, 4);
			break;

		case OP_PID_STATUS:		// PID ״̬
			OLED_ShowFloatNum(1, 1, pv.velocityPID_Kp, 4, 2);
			OLED_ShowFloatNum(2, 1, pv.velocityPID_Ki, 4, 2);
			OLED_ShowFloatNum(3, 1, pv.velocityPID_Kd, 4, 2);
			OLED_ShowSignedNum(4, 1, pv.velocityPID_targetSpeed[0], 5);

			OLED_ShowFloatNum(1, 9, pv.velocityPID_Kp, 4, 2);
			OLED_ShowFloatNum(2, 9, pv.velocityPID_Ki, 4, 2);
			OLED_ShowFloatNum(3, 9, pv.velocityPID_Kd, 4, 2);
			OLED_ShowSignedNum(4, 9, pv.velocityPID_targetSpeed[1], 5);

			// OLED_ShowSignedNum(2, 10, pv.target3, 5);
			// OLED_ShowSignedNum(3, 10, pv.LIMIT_constant, 6);
			// OLED_ShowSignedNum(4, 10, pv.LIMIT_linear, 4);
			break;

		case OP_PWM_VAL:
			OLED_ShowString(1, 1, "P1:");
			OLED_ShowString(2, 1, "P2:");
			OLED_ShowString(3, 1, "P3:");
			OLED_ShowString(4, 1, "P4:");

			OLED_ShowNum(1, 4, MOTOR_GetPWMVal(1), 5);
			OLED_ShowNum(2, 4, MOTOR_GetPWMVal(2), 5);
			OLED_ShowNum(3, 4, MOTOR_GetPWMVal(3), 5);
			OLED_ShowNum(4, 4, MOTOR_GetPWMVal(4), 5);
			
			OLED_ShowNum(1, 10, pwmv.SOFT_compare[0], 5);
			OLED_ShowNum(2, 10, pwmv.SOFT_compare[1], 5);
			OLED_ShowNum(3, 10, pwmv.SOFT_compare[2], 5);
			OLED_ShowNum(4, 10, pwmv.SOFT_compare[3], 5);

			OLED_ShowNum(1, 16, GPIO_ReadInputDataBit(MDX, MD1), 1);
			OLED_ShowNum(2, 16, GPIO_ReadInputDataBit(MDX, MD2), 1);
			OLED_ShowNum(3, 16, GPIO_ReadInputDataBit(MDX, MD3), 1);
			OLED_ShowNum(4, 16, GPIO_ReadInputDataBit(MDX, MD4), 1);

			break;
			
		case OP_ENC_VAL:
			OLED_ShowString(1, 1, "E1:");
			OLED_ShowString(3, 1, "E2:");

			OLED_ShowSignedNum(1, 4, encv.speedM1, 5);
			OLED_ShowSignedNum(2, 5, encv.pulsesM1, 5);
			OLED_ShowSignedNum(3, 4, encv.speedM2, 5);
			OLED_ShowSignedNum(4, 5, encv.pulsesM2, 5);

			break;
			
		case OP_TRACK_PID:
			OLED_ShowString(1, 1, "TrackPID:");
			OLED_ShowString(1, 11, "o:");
            OLED_ShowSignedNum(1, 13, pv.trackPID_output, 3);
            
            OLED_ShowString(2, 1, "P:");
            OLED_ShowString(3, 1, "I:");
            OLED_ShowString(4, 1, "D:");
            OLED_ShowFloatNum(2, 3, pv.trackPID_Kp, 1, 4);
            OLED_ShowFloatNum(3, 3, pv.trackPID_Ki, 1, 4);
            OLED_ShowFloatNum(4, 3, pv.trackPID_Kd, 1, 4);

            OLED_ShowString(2, 11, "e:");
            OLED_ShowString(3, 10, "s:");
            OLED_ShowString(4, 11, "d:");
            OLED_ShowSignedNum(2, 13, pv.trackPID_err, 3);
            OLED_ShowSignedNum(3, 12, pv.trackPID_sum, 4);
            OLED_ShowSignedNum(4, 13, pv.trackPID_errDiff, 3);

			break;
			
		case OP_TRACK:
			OLED_ShowString(1, 1, "TR:");
            OLED_ShowNum(1, 4, GPIO_ReadInputDataBit(TR1X, TR1), 1);
            OLED_ShowNum(1, 6, GPIO_ReadInputDataBit(TR2X, TR2), 1);
            OLED_ShowNum(1, 8, GPIO_ReadInputDataBit(TR3X, TR3), 1);
            OLED_ShowNum(1, 10, GPIO_ReadInputDataBit(TR4X, TR4), 1);
            OLED_ShowNum(1, 12, GPIO_ReadInputDataBit(TR5X, TR5), 1);
            OLED_ShowNum(1, 15, trv.sensorBits, 2);

            OLED_ShowString(2, 1, "CMD:");
            OLED_ShowString(2, 5, "p:");
            OLED_ShowNum(2, 7, trv.pCmds, 2);
            OLED_ShowString(2, 10, "m:");
            OLED_ShowNum(2, 12, trv.stateListMonitering, 2);

            OLED_ShowString(3, 1, "dR:");
            OLED_ShowNum(3, 4, trv.delayRounds, 5);
            OLED_ShowNum(3, 10, trv.cmdDelayRounds, 3);
            OLED_ShowString(4, 1, "mR:");
            OLED_ShowNum(4, 4, trv.maintainRounds, 5);

			break;

        case OP_TRACK_CMDLIST:
			OLED_ShowString(1, 1, "TrackList:");
			OLED_ShowNum(1, 11, trv.cmdsTotal, 2);
            
            OLED_ShowNum(2, 2, trv.cmds[0].action, 2);
            OLED_ShowNum(2, 5, trv.cmds[0].levelOrData, 3);
            OLED_ShowNum(2, 10, trv.cmds[1].action, 2);
            OLED_ShowNum(2, 13, trv.cmds[1].levelOrData, 3);

            OLED_ShowNum(3, 2, trv.cmds[2].action, 2);
            OLED_ShowNum(3, 5, trv.cmds[2].levelOrData, 3);
            OLED_ShowNum(3, 10, trv.cmds[3].action, 2);
            OLED_ShowNum(3, 13, trv.cmds[3].levelOrData, 3);

            OLED_ShowNum(4, 2, trv.cmds[4].action, 2);
            OLED_ShowNum(4, 5, trv.cmds[4].levelOrData, 3);
            OLED_ShowNum(4, 10, trv.cmds[5].action, 2);
            OLED_ShowNum(4, 13, trv.cmds[5].levelOrData, 3);

            break;
			
		case OP_COMM_QUEUE:		// COMM ѭ������
			OLED_ShowString(1, 1, "COMM_Qp: ");
			
			for (i_2 = 0; i_2 < 32; i_2++)
			{
				OLED_ShowHexNum(2 + i_2 / 4, 2 + 3 * (i_2 % 4), cv.RxDataQueue[i_2], 2);
				if (i_2 >= 11)
					break;
			}
			
			OLED_ShowNum(1, 10, cv.queue_pos, 4);
		
			break;

		case OP_ESP_QUEUE:		// ESP ѭ������
			OLED_ShowString(1, 1, "ESP_Qp: ");
			
			for (i_3 = 0; i_3 < 32; i_3++)
			{
				OLED_ShowHexNum(2 + i_3 / 4, 2 + 3 * (i_3 % 4), ev.RxDataQueue[i_3], 2);
				if (i_3 >= 11)
					break;
			}
			
			OLED_ShowNum(1, 9, ev.queue_pos, 4);
		
			break;
			
		case OP_COMM_RX:		// COMM �������
			tempCOMMInfo = mv.COMM_MsgInfo;
			if (mv.COMM_newMsgCount > 0)
				for (uint8_t i = 0; i < MAIN_COMM_MSG_NUMBER; i++)
				{
					tempCOMMLen = tempCOMMInfo->len;
					if (tempCOMMLen >= 7)
						tempCOMMLen = 7;

					OLED_ShowString_Len(1 + i, 1, "          ", 10);
					OLED_ShowString_Len(1 + i, 1, _MAIN_COMMGetMsg(tempCOMMInfo), tempCOMMLen);
					OLED_ShowNum(1 + i, 9, tempCOMMInfo->flag, 1);

					tempCOMMInfo++;
				}

			OLED_ShowString(1, 11, "pP:");
			OLED_ShowNum(1, 14, cv.pRxPacket, 3);
			
			OLED_ShowString(2, 11, "Sta:");
			OLED_ShowNum(2, 15, (uint32_t)cv.RxState, 2);
		
			OLED_ShowString(3, 11, "Qpo:");
			OLED_ShowNum(3, 15, cv.queue_pos, 2);
		
			OLED_ShowString(4, 11, "-COMM-");
		
			break;
			
		case OP_ESP_RX:		// ESP �������
			tempESPInfo = mv.ESP_MsgInfo;
			// if (mv.ESP_newMsgCount > 0)
			// 	for (uint8_t i = 0; i < MAIN_ESP_MSG_NUMBER; i++)
			// 	{
			// 		tempESPLen = tempESPInfo->len;
			// 		if (tempESPLen >= 7)
			// 			tempESPLen = 7;

			// 		OLED_ShowString_Len(1 + i, 1, "          ", 10);
			// 		OLED_ShowString_Len(1 + i, 1, _MAIN_ESPGetMsg(tempESPInfo), tempESPLen);
			// 		OLED_ShowNum(1 + i, 9, tempESPInfo->flag, 1);

			// 		tempESPInfo++;
			// 	}

			OLED_ShowString(1, 11, "pP:");
			OLED_ShowNum(1, 14, ev.pRxPacket, 3);
			
			OLED_ShowString(2, 11, "Sta:");
			OLED_ShowNum(2, 15, (uint32_t)ev.RxState, 2);
		
			OLED_ShowString(3, 11, "Qpo:");
			OLED_ShowNum(3, 15, ev.queue_pos, 2);
		
			OLED_ShowString(4, 11, "-ESP-");
		
			break;
			
		case OP_COMM_VARS:		// COMM�����Ӵ�
			OLED_ShowString(4, 1, "cv");

			OLED_ShowString(1, 1, "IfP:");
			OLED_ShowNum(1, 5, cv.RxPackInfoPos, 3);
			
			OLED_ShowString(2, 1, "UnR:");
			OLED_ShowNum(2, 5, cv.RxPackInfoUnreadNum, 3);
						
			OLED_ShowString(1, 8, "I1:");
			OLED_ShowNum(1, 11, cv.RxPackInfo[0].head, 3);
			OLED_ShowNum(1, 14, cv.RxPackInfo[0].len, 3);
			OLED_ShowString(2, 8, "I2:");
			OLED_ShowNum(2, 11, cv.RxPackInfo[1].head, 3);
			OLED_ShowNum(2, 14, cv.RxPackInfo[1].len, 3);
			OLED_ShowString(3, 8, "I3:");
			OLED_ShowNum(3, 11, cv.RxPackInfo[2].head, 3);
			OLED_ShowNum(3, 14, cv.RxPackInfo[2].len, 3);
			OLED_ShowString(4, 8, "I4:");
			OLED_ShowNum(4, 11, cv.RxPackInfo[3].head, 3);
			OLED_ShowNum(4, 14, cv.RxPackInfo[3].len, 3);
			
			break;

		case OP_ESP_VARS:
			OLED_ShowString(4, 1, "ev");

			OLED_ShowString(1, 1, "IfP:");
			OLED_ShowNum(1, 5, ev.RxPackInfoPos, 3);
			
			OLED_ShowString(2, 1, "UnR:");
			OLED_ShowNum(2, 5, ev.RxPackInfoUnreadNum, 3);
												
			OLED_ShowString(1, 8, "I1:");
			OLED_ShowNum(1, 11, ev.RxPackInfo[0].head, 3);
			OLED_ShowNum(1, 14, ev.RxPackInfo[0].len, 3);
			OLED_ShowString(2, 8, "I2:");
			OLED_ShowNum(2, 11, ev.RxPackInfo[1].head, 3);
			OLED_ShowNum(2, 14, ev.RxPackInfo[1].len, 3);
			OLED_ShowString(3, 8, "I3:");
			OLED_ShowNum(3, 11, ev.RxPackInfo[2].head, 3);
			OLED_ShowNum(3, 14, ev.RxPackInfo[2].len, 3);
			OLED_ShowString(4, 8, "I4:");
			OLED_ShowNum(4, 11, ev.RxPackInfo[3].head, 3);
			OLED_ShowNum(4, 14, ev.RxPackInfo[3].len, 3);

			break;

		case OP_MAIN_COMM_VARS:		// MAIN_COMM�����Ӵ�
			OLED_ShowString(4, 1, "mv_cv");

			OLED_ShowString(1, 1, "IfP:");
			OLED_ShowNum(1, 5, mv.COMM_pMsgInfo, 3);
			
			OLED_ShowString(2, 1, "New:");
            if (mv.COMM_newMsgCount > 0)
			    OLED_ShowNum(2, 5, mv.COMM_newMsgCount, 3);
						
            OLED_ShowString(3, 1, "p:");
            OLED_ShowNum(3, 3, mv.COMM_pMsg, 3);
						
			OLED_ShowString(1, 8, "I1:");
			OLED_ShowNum(1, 11, mv.COMM_MsgInfo[0].head, 3);
			OLED_ShowNum(1, 14, mv.COMM_MsgInfo[0].len, 3);
			OLED_ShowString(2, 8, "I2:");
			OLED_ShowNum(2, 11, mv.COMM_MsgInfo[1].head, 3);
			OLED_ShowNum(2, 14, mv.COMM_MsgInfo[1].len, 3);
			OLED_ShowString(3, 8, "I3:");
			OLED_ShowNum(3, 11, mv.COMM_MsgInfo[2].head, 3);
			OLED_ShowNum(3, 14, mv.COMM_MsgInfo[2].len, 3);
			OLED_ShowString(4, 8, "I4:");
			OLED_ShowNum(4, 11, mv.COMM_MsgInfo[3].head, 3);
			OLED_ShowNum(4, 14, mv.COMM_MsgInfo[3].len, 3);

			break;
			
		case OP_MAIN_ESP_VARS:		// MAIN_ESP�����Ӵ�
			OLED_ShowString(4, 1, "mv_ev");

			OLED_ShowString(1, 1, "IfP:");
			OLED_ShowNum(1, 5, mv.ESP_pMsgInfo, 3);
			
            OLED_ShowString(2, 1, "New:");
            if (mv.ESP_newMsgCount > 0)
                OLED_ShowNum(2, 5, mv.ESP_newMsgCount, 3);

            OLED_ShowString(3, 1, "p:");
            OLED_ShowNum(3, 3, mv.ESP_pMsg, 3);
						
			OLED_ShowString(1, 8, "I1:");
			OLED_ShowNum(1, 11, mv.ESP_MsgInfo[0].head, 3);
			OLED_ShowNum(1, 14, mv.ESP_MsgInfo[0].len, 3);
			OLED_ShowString(2, 8, "I2:");
			OLED_ShowNum(2, 11, mv.ESP_MsgInfo[1].head, 3);
			OLED_ShowNum(2, 14, mv.ESP_MsgInfo[1].len, 3);
			OLED_ShowString(3, 8, "I3:");
			OLED_ShowNum(3, 11, mv.ESP_MsgInfo[2].head, 3);
			OLED_ShowNum(3, 14, mv.ESP_MsgInfo[2].len, 3);
			OLED_ShowString(4, 8, "I4:");
			OLED_ShowNum(4, 11, mv.ESP_MsgInfo[3].head, 3);
			OLED_ShowNum(4, 14, mv.ESP_MsgInfo[3].len, 3);

			break;
			
		case OP_MAIN_COMM_MSGBUFFER:		// mainCommMsg��������Ӵ�	
			OLED_ShowString(1, 1, "MvCv_MsgBuffer: ");
			
			for (uint8_t i = 0; i < 32; i++)
			{
				OLED_ShowHexNum(2 + i / 4, 2 + 3 * (i % 4), mv.COMM_Msg[i], 2);
				if (i >= 11)
					break;
			}
			break;
			
		case OP_COMM_MSGBUFFER:		// COMM����������Ӵ�	
			OLED_ShowString(1, 1, "COMM_MsgBuffer: ");
			
			for (uint8_t i = 0; i < 32; i++)
			{
				OLED_ShowHexNum(2 + i / 4, 2 + 3 * (i % 4), cv.RxPacket[i], 2);
				if (i >= 11)
					break;
			}
			break;
			
		case OP_ESP_MSGBUFFER:		// COMM����������Ӵ�	
			OLED_ShowString(1, 1, "ESP_MsgBuffer: ");
			
			for (uint8_t i = 0; i < 32; i++)
			{
				OLED_ShowHexNum(2 + i / 4, 2 + 3 * (i % 4), ev.RxPacket[i], 2);
				if (i >= 11)
					break;
			}
			break;

		case OP_UNITPROTOCOL_RELATED_1:
			OLED_ShowString(1, 1, "UNIT:");

			OLED_ShowString(2, 1, "p:");
			OLED_ShowString(3, 1, "t:");
			OLED_ShowString(4, 1, "r:");

			OLED_ShowNum(2, 3, pv.isEnabled, 2);	OLED_ShowNum(2, 6, pv.isDebugEnabled, 2);
			OLED_ShowNum(3, 3, trv.isEnabled, 2);

			OLED_ShowNum(4, 3, rmv.isEnabled, 2);
			OLED_ShowNum(4, 6, rmv.fbBits, 2);		
			OLED_ShowNum(4, 9, rmv.lrBits, 2);
			OLED_ShowNum(4, 12, rmv.mergedBits, 2);

			break;

        case OP_UNITPROTOCOL_RELATED_2:     // �ٶȻ���ң���ٶ�
            OLED_ShowFloatNum(1, 1, pv.velocityPID_Kp, 2, 4);
            OLED_ShowFloatNum(2, 1, pv.velocityPID_Ki, 2, 4);
            OLED_ShowFloatNum(3, 1, pv.velocityPID_Kd, 2, 4);
            OLED_ShowNum(4, 1, pv.velocityPID_sumLimit, 5);
            OLED_ShowNum(4, 7, pv.velocityPID_sumDecline, 3);

            OLED_ShowNum(1, 12, rmv.forwardSpeed, 5);
            OLED_ShowNum(2, 12, rmv.backwardSpeed, 5);
            OLED_ShowNum(3, 12, rmv.rotateSpeed, 5);
            OLED_ShowNum(4, 12, rmv.tendencySpeed, 5);
            break;
		
        case OP_UNITPROTOCOL_RELATED_3:     // ѭ����
            OLED_ShowFloatNum(1, 1, pv.trackPID_Kp, 2, 4);
            OLED_ShowFloatNum(2, 1, pv.trackPID_Ki, 2, 4);
            OLED_ShowFloatNum(3, 1, pv.trackPID_Kd, 2, 4);
            OLED_ShowNum(4, 1, pv.trackPID_sumLimit, 5);

            OLED_ShowNum(1, 10, pv.trackPID_sumDecline, 3);
            OLED_ShowFloatNum(2, 10, pv.trackVelocityPID_targetSpeedCo, 1, 3);
            OLED_ShowFloatNum(3, 10, pv.trackPID_TorqueRatio, 1, 3);
            break;
		
		case OP_AT_STATUS_CHECK:
			OLED_ShowString(1, 1, "AT:");

			OLED_ShowString(2, 1, "R:");
			OLED_ShowNum(2, 3, cmnv.ESPAT_receiveCmd, 3);

			OLED_ShowString(2, 9, "S:");
			OLED_ShowNum(2, 11, cmnv.ESPAT_sendCmd, 3);
			OLED_ShowNum(3, 11, cmnv.ESPAT_toTransparentState, 3);

			OLED_ShowString(4, 1, "ATMode:");
			OLED_ShowNum(4, 8, cmnv.ESP_Mode, 2);

			break;

		case OP_OV7670:
			OLED_ShowString(1, 1, "OV7670:");
			OLED_ShowNum(1, 15, GPIO_ReadInputDataBit(OV_VSYNC_GPIOX, OV_VSYNC), 2);
			
			OLED_ShowString(2, 1, "Md:");
			OLED_ShowNum(2, 4, ovv.mode, 1);
			OLED_ShowNum(2, 6, ovv.lastMode, 1);

			OLED_ShowString(2, 9, "IT:");
			OLED_ShowNum(2, 12, ovv.interruptIndicator, 3);
			
			OLED_ShowString(3, 1, "Prog:");		// 7~16Ϊ������
			uint8_t progress10 = OV7670_GetProgress10();
			for (uint8_t i; i < 10; i++)
			{
				if (i < progress10)
					OLED_ShowChar(3, 7 + i, '>');
				else 
					OLED_ShowChar(3, 7 + i, ' ');
			}

			OLED_ShowString(4, 1, "St:");
			OLED_ShowNum(4, 4, ovv.OV7670_state, 3);
			if (ovv.OV7670_state != 3)
				Serial_SendByte('0' + ovv.OV7670_state, COMM_USART);

			break;

		case OP_TIMER_MONITOR:
			OLED_ShowString(1, 1, "TIMER_MONITOR");
			OLED_ShowString(2, 1, "E1:");
			OLED_ShowString(3, 1, "E2/PT:");
			OLED_ShowString(4, 1, "DT:");

			OLED_ShowNum(2, 4, TIM_GetCounter(EVENTLOOP1T), 5);
			OLED_ShowNum(2, 10, elv.eventloop1SecMonitor, 3);
			OLED_ShowNum(3, 7, TIM_GetCounter(EVENTLOOP2T), 5);
			OLED_ShowNum(4, 4, TIM_GetCounter(MDT), 5);

			break;

		// case OP_ENC_PESUDO_SPEED:
		// 	OLED_ShowString(1, 1, "Speed(pseudo): ");
		// 	if (pv.velocityPID_targetSpeed[0] < 0)
		// 		OLED_ShowChar(2, 2, '-');
		// 	else
		// 		OLED_ShowChar(2, 2, '+');
		// 	if (pv.velocityPID_targetSpeed[1] < 0)
		// 		OLED_ShowChar(3, 2, '-');
		// 	else
		// 		OLED_ShowChar(3, 2, '+');
			
		// 	OLED_ShowNum(2, 3, mv.s[0], 4);
		// 	OLED_ShowNum(3, 3, mv.s[1], 4);
			
		// 	break;

		case OP_HOMEPAGE:
			OLED_ShowString(2, 3, "----OLED----");
			OLED_ShowString(3, 2, "totalpages: ");
			OLED_ShowNum(3, 13, (uint32_t)OP_TOTAL, 2);
			break;

		}
}	

uint8_t _MAIN_CopyCOMMMsg(void)
{
	uint8_t i;
	uint8_t count;
	uint8_t srcStart = 255, srcEnd = 255, /*srcLen = 255, */tempLen = 255;
	COMM_struct_packInfo *srcFirstInfo, *srcLastInfo;	// ������main�еİ���Ϣ
	COMM_struct_packInfo *tempMainInfo;

	count = COMM_GetPackInfo(mv.COMM_MsgInfo, mv.COMM_pMsgInfo, MAIN_COMM_MSG_NUMBER);
	if (count > 0 && count <= MAIN_COMM_MSG_NUMBER)		// ������������Ϣ������Ϣ/���������Ĭ
	{
		srcFirstInfo = &mv.COMM_MsgInfo[mv.COMM_pMsgInfo];
		srcLastInfo = &mv.COMM_MsgInfo
			[ABS_CycleAddPos(mv.COMM_pMsgInfo, count - 1, MAIN_COMM_MSG_NUMBER)];
		srcStart = srcFirstInfo->head;
		srcEnd = srcLastInfo->head + srcLastInfo->len;
		// srcLen = ABS_CycleSubPos(srcEnd, srcStart, COMM_MSG_NUMBER);	// ��Ϣ�ܳ���

		// ������Ϣ��info����Ϊmain�����head
		for (uint8_t i = 0; i < count; i++)
		{
			tempMainInfo = &mv.COMM_MsgInfo[mv.COMM_pMsgInfo];
			tempLen = tempMainInfo->len;
			if (mv.COMM_pMsg + tempLen >= MAIN_COMM_MSGBUFFER_SIZE)		// ����Patch���������0��ʼ����
				mv.COMM_pMsg = 0;
			tempMainInfo->head = mv.COMM_pMsg;		// ����head��len��flag�������

			for (uint8_t i = 0; i < tempLen; i++)		// �ֶο�����Ϣ��main����
			{
				mv.COMM_Msg[ABS_CycleAddPos(mv.COMM_pMsg, i, MAIN_COMM_MSGBUFFER_SIZE)] 
					= cv.RxPacket[ABS_CycleAddPos(srcStart, i, COMM_RXBUFFER_SIZE)];
			}

			mv.COMM_pMsg = ABS_CycleAddPos(mv.COMM_pMsg, tempLen, MAIN_COMM_MSGBUFFER_SIZE);		// ���»���ָ��
			mv.COMM_pMsgInfo = ABS_CycleAddPos(mv.COMM_pMsgInfo, 1, MAIN_COMM_MSG_NUMBER);		// ����Ϣ����ָ��++
			srcStart = ABS_CycleAddPos(srcStart, tempLen, COMM_RXBUFFER_SIZE);				// ����Դ���ݶ�ָ��
		}
	}

	return count;
}

uint8_t _MAIN_CopyESPMsg(void)
{
	uint8_t i;
	uint8_t count;
	uint8_t srcStart = 255, srcEnd = 255, /*srcLen = 255, */tempLen = 255;
	ESP_struct_packInfo *srcFirstInfo, *srcLastInfo;	// ������main�еİ���Ϣ
	ESP_struct_packInfo *tempMainInfo;

	count = ESP_GetPackInfo(mv.ESP_MsgInfo, mv.ESP_pMsgInfo, MAIN_ESP_MSG_NUMBER);
	if (count > 0 && count <= MAIN_ESP_MSG_NUMBER)		// ������������Ϣ������Ϣ/���������Ĭ
	{
		srcFirstInfo = &mv.ESP_MsgInfo[mv.ESP_pMsgInfo];
		srcLastInfo = &mv.ESP_MsgInfo
			[ABS_CycleAddPos(mv.ESP_pMsgInfo, count - 1, MAIN_ESP_MSG_NUMBER)];
		srcStart = srcFirstInfo->head;
		srcEnd = srcLastInfo->head + srcLastInfo->len;
		// srcLen = ABS_CycleSubPos(srcEnd, srcStart, ESP_MSG_NUMBER);	// ��Ϣ�ܳ���

		// ������Ϣ��info����Ϊmain�����head
		for (uint8_t i = 0; i < count; i++)
		{
			tempMainInfo = &mv.ESP_MsgInfo[mv.ESP_pMsgInfo];
			tempMainInfo->len++;						// Ϊ��β����'\0'��ESP���ֵ�tempLen����+1�����ý�Ӱ�쵽head�ĸ���
			tempLen = tempMainInfo->len;
			if (mv.ESP_pMsg + tempLen >= MAIN_ESP_MSGBUFFER_SIZE)		// ����Patch���������0��ʼ����
				mv.ESP_pMsg = 0;
			tempMainInfo->head = mv.ESP_pMsg;		// ����head��len��flag�������

			for (uint8_t i = 0; i < tempLen; i++)		// �ֶο�����Ϣ��main����
			{
				mv.ESP_Msg[mv.ESP_pMsg + i]         // �����Ѿ���֤��Ϣ���ڴ�����������ŵģ������ټ���ѭ������ָ�룬�ڸ���������������ʱ������벻����bug  # �����������������ʹ�С���㵼�µ�α���ϣ���ʹ��Ϣ�����ض���ͷ��
					= ev.RxPacket[ABS_CycleAddPos(srcStart, i, ESP_RXBUFFER_SIZE)];
				if (i == tempLen - 1)
					mv.ESP_Msg[mv.ESP_pMsg + i] 
						= '\0';
			}

			mv.ESP_pMsg = ABS_CycleAddPos(mv.ESP_pMsg, tempLen, MAIN_ESP_MSGBUFFER_SIZE);		// ���»���ָ��
			mv.ESP_pMsgInfo = ABS_CycleAddPos(mv.ESP_pMsgInfo, 1, MAIN_ESP_MSG_NUMBER);		// ����Ϣ����ָ��++
			srcStart = ABS_CycleAddPos(srcStart, tempLen - 1, ESP_RXBUFFER_SIZE);				// ����Դ���ݶ�ָ�룬Դ�β�+1
		}
	}

	return count;
}

uint8_t *_MAIN_COMMGetMsg(COMM_struct_packInfo *msgInfo)
{
	return mv.COMM_Msg + msgInfo->head;
}

uint8_t *_MAIN_ESPGetMsg(ESP_struct_packInfo *msgInfo)
{
	return mv.ESP_Msg + msgInfo->head;
}


// ��ʼ���������������ڲ���ESP��Ϣ�������Ƿ����ָ����Ϣ���ô˺�������ATָ���������¼�ѭ������AT�������޳�ʼ��ʹ��
// ��������Ϣ��Ѱ��, ����ƥ�����Ϣ���³̶�, 1������, 0��ʧ��
uint8_t MAIN_ESP_FindNewMsg(char *match)
{
	uint8_t count = mv.ESP_newMsgCount;
	char *msgBuff[MAIN_ESP_MSG_NUMBER];
	for (uint8_t i = 0; i < count; i++)
		msgBuff[i] = (char *)mv.ESP_msgHandler[i];

	for (uint8_t i = 0; i < count; i++)
	{
		if (strcmp(match, msgBuff[count - 1 - i]) == 0)
			return i + 1;
	}

	return 0;
}

// �ӻ�����Ϣ��Ѱ��, �����ҵ����
uint8_t MAIN_ESP_FindBufferedMsg(char *match)
{
	ESP_struct_packInfo *tempInfo = mv.ESP_MsgInfo;;

	for (uint8_t i = 0; i < ESP_MSG_NUMBER; i++)
	{
		if (strcmp(match, (char *)_MAIN_ESPGetMsg(tempInfo)) == 0)		// ESP���Դ�'\0'
			return 1;

		tempInfo++;
	}

	return 0;
}

void _MAIN_OLED_AddAndShowESPInitProgress(void)
{
	mv.ESP_initProgress++;
	OLED_ShowChar(2, mv.ESP_initProgress, '>');
}


// UNIT PROTOCOL
void MAIN_OLEDPAGE_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t first, second;
	uint8_t *pointer = data;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(first));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&second, &pointer, sizeof(second));
	switch (first)
	{
	case (uint8_t)MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_PAGE_ACTION:
        switch (second)
        {
        case MAIN_OLEDPAGE_E_TURNPAGEACTION__HOME:
            mv.Page = OP_HOMEPAGE;
            break;

        case MAIN_OLEDPAGE_E_TURNPAGEACTION__UP:
            if (mv.Page == OP_HOMEPAGE)
                mv.Page = OP_TOTAL;
            mv.Page--;
            break;

        case MAIN_OLEDPAGE_E_TURNPAGEACTION__DOWN:
            mv.Page++;
            mv.Page %= OP_TOTAL;
            break;
        }
		break;

	case (uint8_t)MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_TO_TARGET_PAGE:
        if (second >= OP_TOTAL)
            second = OP_TOTAL - 1;
        mv.Page = second;
		break;
	}
}





// uint8_t _MAIN_CopyESPMsg(void)
// {
// 	uint8_t i;
// 	uint8_t count;
// 	uint8_t srcStart = 255, srcEnd = 255, srcLen = 255, tempLen = 255;
// 	ESP_struct_packInfo *srcFirstInfo, *srcLastInfo, *tempSrcInfo, *tempMainInfo;

// 	count
// 		= ESP_GetPackInfo(mv.ESP_MsgInfo, mv.ESP_lastMsgInfo, MAIN_MSG_NUMBER);
// 	if (mv.ESP_RxPackReadNum != 0 || mv.ESP_RxPackReadNum != 255)
// 	{
// 		srcFirstInfo = &mv.ESP_MsgInfo[mv.ESP_lastMsgInfo];
// 		srcLastInfo = &mv.ESP_MsgInfo
// 			[ABS_CycleAddPos(mv.ESP_lastMsgInfo, mv.ESP_RxPackReadNum, MAIN_MSG_NUMBER)];
// 		srcStart = srcFirstInfo->head;
// 		srcEnd = srcLastInfo->head + srcLastInfo->len;
// 		srcLen = ABS_CycleSubPos(srcEnd, srcStart, ESP_MSG_NUMBER);
// 	}
// 	for (uint8_t i = 0; i < srcLen; i++)
// 	{
// 		mv.ESP_Msg[ABS_CycleAddPos(mv.ESP_lastMsgInfo, i, MAIN_MSG_NUMBER] 
// 			= ev.RxPacket[ABS_CycleAddPos(srcStart, i, ESP_MSG_NUMBER)];
// 	}

// 	// info�޸�Ϊmain�����
// 	for (uint8_t i = 0; i < count; i++)
// 	{
// 		tempMainInfo = &mv.ESP_MsgInfo[ABS_CycleAddPos(mv.ESP_lastMsgInfo, i, MAIN_MSG_NUMBER)];
// 		tempSrcInfo = &ev.MsgInfo[ABS_CycleSubPos(ev.lastMsgInfo, count-i, ESP_MSG_NUMBER)];
// 		tempLen += tempSrcInfo->len;
// 		tempMainInfo->head = ABS_CycleAddPos(mv.ESP_pMsg, tempLen, MAIN_MSG_NUMBER);
// 	}

// 	mv.ESP_pMsg = ABS_CycleAddPos(mv.ESP_pMsg, srcLen, MAIN_MSG_NUMBER);		// ���»���ָ��

// 	return count;
// }

// void _MAIN_ESPMsgRestore(ESP_struct_packInfo *tempInfo, uint8_t *container)
// {
// 	uint8_t tempInfoPos, *tempMsg;
// 	uint8_t i;

// 	for (uint8_t i = 0; i < tempInfo->len; i++)		// ������Ϣ����Ϊ���лᵼ����Ϣ����
// 	{
// 		*container = mv.ESP_Msg[ABS_CycleAddPos(tempInfo->head, i, MAIN_MSG_NUMBER)];

// 		container++;

// 		if (i >= 48)
// 			Serial_SendByte(0xCC, ESP_USART);		// ���ͱ�����Ϣ
// 	}

// }


