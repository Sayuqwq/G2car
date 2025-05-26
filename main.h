#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"
#include "alias.h"
#include "COMM.h"
#include "ESP_01S.h"

// 预迁移函数，由CNMCT控制
void MAIN_MsgProcess(void);		// 暂时放在main，最终应迁往CMNCT

// OLED_PAGE_UNITPROTOCOL
void MAIN_OLEDPAGE_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);

/*  UNITPROTOCOL
*/
typedef enum
{ 
	MAIN_OLEDPAGE_E_UNITPROTOCOL__RESERVED = 0,

	MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_PAGE_ACTION,         // 翻页动作
	MAIN_OLEDPAGE_E_UNITPROTOCAL__TURN_TO_TARGET_PAGE,      // 跳转到指定页数

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

// PID查看组
    OP_PID_ENC,
    OP_PID_CONFIG,
    OP_PID_STATUS,

// PWM, ENC查看组
    OP_PWM_VAL,
    OP_ENC_VAL,

// Track循迹组
    OP_TRACK_PID,
    OP_TRACK,
    OP_TRACK_CMDLIST,

// Remote遥控组
    OP_REMOTE,

// 通信DMA环形队列组+发送查看组
    OP_COMM_QUEUE,
    OP_ESP_QUEUE,

// 通信接收组
    OP_COMM_RX,
    OP_ESP_RX,

// 消息缓冲查看视窗
    OP_MAIN_COMM_MSGBUFFER,
    OP_COMM_MSGBUFFER,
    OP_ESP_MSGBUFFER,

// 单位通信协议设计变量查看视窗
    OP_UNITPROTOCOL_RELATED_1,
    OP_UNITPROTOCOL_RELATED_2,
    OP_UNITPROTOCOL_RELATED_3,
    
// AT指令状态查看
    OP_AT_STATUS_CHECK,

// 模块变量查看组
    OP_COMM_VARS,
    OP_ESP_VARS,
    OP_MAIN_COMM_VARS,
    OP_MAIN_ESP_VARS,

// OV7670
    OP_OV7670,

// 定时器监视视窗
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
    uint8_t COMM_Msg[MAIN_COMM_MSGBUFFER_SIZE];    // 消息队列Patch，三级缓冲：DMA队列 -> 包缓冲队列 -> main缓存队列
    uint16_t COMM_pMsg;
	COMM_struct_packInfo COMM_MsgInfo[MAIN_COMM_MSG_NUMBER];	// main暂存的包标识信息
    uint8_t *COMM_msgHandler[MAIN_COMM_MSG_NUMBER];      // 记录最新msg
	uint8_t COMM_pMsgInfo;		// 指示当前缓冲的指针
    uint8_t COMM_newMsgCount;

    // ESP
 	#define MAIN_ESP_MSGBUFFER_SIZE 384             // 已扩容 128 -> 384
	#define MAIN_ESP_MSG_NUMBER  (ESP_MSG_NUMBER + 10)
    uint8_t ESP_Rxflag;
    uint8_t ESP_Msg[MAIN_ESP_MSGBUFFER_SIZE];    // 消息队列Patch，三级缓冲：DMA队列 -> 包缓冲队列 -> main缓存队列
    uint16_t ESP_pMsg;
	ESP_struct_packInfo ESP_MsgInfo[MAIN_ESP_MSG_NUMBER];	// main暂存的包标识信息
    uint8_t *ESP_msgHandler[MAIN_ESP_MSG_NUMBER];      // 记录最新msg
	uint8_t ESP_pMsgInfo;		// 指示当前缓冲的指针
    uint8_t ESP_newMsgCount;

    // ENCODER
    int16_t ENC_speed1;
    int16_t ENC_speed2;


    uint8_t Page;
    uint8_t ESP_initProgress;

} MAINVARS;


#endif
