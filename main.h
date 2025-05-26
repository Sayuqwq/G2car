#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"
#include "alias.h"
#include "COMM.h"
#include "ESP_01S.h"

// ԤǨ�ƺ�������CNMCT����
void MAIN_MsgProcess(void);		// ��ʱ����main������ӦǨ��CMNCT

// OLED_PAGE_UNITPROTOCOL
void MAIN_OLEDPAGE_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);

/*  UNITPROTOCOL
*/
typedef enum
{ 
	MAIN_OLEDPAGE_E_UNITPROTOCOL__RESERVED = 0,

	MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_PAGE_ACTION,         // ��ҳ����
	MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_TO_TARGET_PAGE,      // ��ת��ָ��ҳ��

} MAIN_OLEDPAGE_enum_UNITPROTOCOL_SUBMODESELECT;

typedef enum 
{
    MAIN_OLEDPAGE_E_TURNPAGEACTION__HOME = 0,
    MAIN_OLEDPAGE_E_TURNPAGEACTION__UP,
    MAIN_OLEDPAGE_E_TURNPAGEACTION__DOWN,

    MAIN_OLEDPAGE_E_TURNPAGEACTION_TOTAL

} OV7670_enum_TURN_PAGE_ACTION;

////

typedef enum
{
    OP_HOMEPAGE = 0,

// PID�鿴��
    OP_PID_ENC,
    OP_PID_CONFIG,
    OP_PID_STATUS,

// PWM, ENC�鿴��
    OP_PWM_VAL,
    OP_ENC_VAL,

// Trackѭ����
    OP_TRACK_PID,
    OP_TRACK,
    OP_TRACK_CMDLIST,

// Remoteң����
    OP_REMOTE,

// ͨ��DMA���ζ�����+���Ͳ鿴��
    OP_COMM_QUEUE,
    OP_ESP_QUEUE,

// ͨ�Ž�����
    OP_COMM_RX,
    OP_ESP_RX,

// ��Ϣ����鿴�Ӵ�
    OP_MAIN_COMM_MSGBUFFER,
    OP_COMM_MSGBUFFER,
    OP_ESP_MSGBUFFER,

// ��λͨ��Э����Ʊ����鿴�Ӵ�
    OP_UNITPROTOCOL_RELATED_1,
    OP_UNITPROTOCOL_RELATED_2,
    OP_UNITPROTOCOL_RELATED_3,
    
// ATָ��״̬�鿴
    OP_AT_STATUS_CHECK,

// ģ������鿴��
    OP_COMM_VARS,
    OP_ESP_VARS,
    OP_MAIN_COMM_VARS,
    OP_MAIN_ESP_VARS,

// OV7670
    OP_OV7670,

// ��ʱ�������Ӵ�
    OP_TIMER_MONITOR,


    OP_TOTAL

} OLED_enum_PAGE;

typedef struct 
{
    uint8_t keyNum;

    // COMM
	#define MAIN_COMM_MSGBUFFER_SIZE 128
	#define MAIN_COMM_MSG_NUMBER (COMM_MSG_NUMBER + 4)
    uint8_t COMM_Rxflag;
    uint8_t COMM_Msg[MAIN_COMM_MSGBUFFER_SIZE];    // ��Ϣ����Patch���������壺DMA���� -> ��������� -> main�������
    uint16_t COMM_pMsg;
	COMM_struct_packInfo COMM_MsgInfo[MAIN_COMM_MSG_NUMBER];	// main�ݴ�İ���ʶ��Ϣ
    uint8_t *COMM_msgHandler[MAIN_COMM_MSG_NUMBER];      // ��¼����msg
	uint8_t COMM_pMsgInfo;		// ָʾ��ǰ�����ָ��
    uint8_t COMM_newMsgCount;

    // ESP
 	#define MAIN_ESP_MSGBUFFER_SIZE 384             // ������ 128 -> 384
	#define MAIN_ESP_MSG_NUMBER  (ESP_MSG_NUMBER + 10)
    uint8_t ESP_Rxflag;
    uint8_t ESP_Msg[MAIN_ESP_MSGBUFFER_SIZE];    // ��Ϣ����Patch���������壺DMA���� -> ��������� -> main�������
    uint16_t ESP_pMsg;
	ESP_struct_packInfo ESP_MsgInfo[MAIN_ESP_MSG_NUMBER];	// main�ݴ�İ���ʶ��Ϣ
    uint8_t *ESP_msgHandler[MAIN_ESP_MSG_NUMBER];      // ��¼����msg
	uint8_t ESP_pMsgInfo;		// ָʾ��ǰ�����ָ��
    uint8_t ESP_newMsgCount;

    // ENCODER
    int16_t ENC_speed1;
    int16_t ENC_speed2;


    uint8_t Page;
    uint8_t ESP_initProgress;

} MAINVARS;


#endif
