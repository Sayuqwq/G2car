#ifndef __COMM_H__
#define __COMM_H__

#include "stm32f10x.h"
#include "alias.h"

typedef struct 
{
	uint16_t head;
	uint16_t len;
	uint8_t flag;
} COMM_struct_packInfo;

typedef enum
{
	RXSTATE_WAITING_HEAD1 = 0,
	RXSTATE_WAITING_HEAD2 = 1,
	RXSTATE_RECEIVING_DATA = 8,
	RXSTATE_WAITING_TAIL2 = 9,
	RXSTATE_PACK_OVERFLOW = 14
} COMM_enum_RXSTATE;

void COMM_Init(uint32_t baudrate);
void COMM_RX_PackingReceiver(uint8_t Byte);
void COMM_RXbuffer_Processor(void);
uint8_t COMM_GetPackInfo(COMM_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax);		// 返回个数和包信息

uint8_t COMM_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len);		// 与Serial_DMADirectSend配合使用
void COMM_DMASetDirectSendEnabled(uint8_t newState);


typedef struct 
{
	#define COMM_RXBUFFER_SIZE 64
	#define COMM_PACK_MAXSIZE 47
	#define COMM_MSG_NUMBER 4
	#define COMM_RXQUEUE_SIZE 64
	#define COMM_TXQUEUE_SIZE 320	// 测试
		//	Comm
	// 协议，包头尾 in Serial.c
	const uint8_t PACK_HEAD;	// double same head and tail
	const uint8_t PACK_HEAD_2;
	const uint8_t PACK_HEAD_3;
	const uint8_t PACK_HEAD_4;	// 用于蓝牙遥控
	const uint8_t PACK_TAIL;
	const uint8_t PACK_TAIL_2;
	const uint8_t PACK_TAIL_3;
	
	uint8_t RxPacket[COMM_RXBUFFER_SIZE];	// COMM收到的包循环缓冲队列
	uint16_t pRxPacket;			// 包缓冲队列位置指针
	COMM_struct_packInfo RxPackInfo[COMM_MSG_NUMBER];	// ESP包标识信息
	uint8_t RxPackInfoPos;		// 包缓冲队列头指针数组位置指针
	uint8_t RxPackInfoUnreadNum;		// 包缓冲队列头指针数组读指针

	uint8_t RxDataQueue[COMM_RXQUEUE_SIZE];		// COMM_DMA循环队列
	uint16_t queue_pos;			// Comm循环队列位置指针, 255是错误代码，表示接收暂未开始
	uint16_t last_queue_pos;		// Comm循环队列上一次位置指针，接收从0开始，故初始化为0

	uint8_t TxDataQueue[COMM_TXQUEUE_SIZE];		// 测试：DMA发送缓存	不适用环形队列，因为计数器到0才会停，也不想使用了
	uint8_t isDMATxDirectSending;

	uint8_t test;
	uint8_t RxFlag;			// 接收打包完毕标志位
	uint8_t RxAck;
	uint8_t RxPacketLen_ERRVAL;
	COMM_enum_RXSTATE RxState;		// Comm接收状态机	
} COMMVARS;


#endif
