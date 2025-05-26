#ifndef __ESP_01S_H__
#define __ESP_01S_H__

#include "stm32f10x.h"
#include "alias.h"

typedef struct 
{
	uint16_t head;
	uint16_t len;
	uint8_t flag;
} ESP_struct_packInfo;

typedef enum
{
	// ESP_RXSTATE_START_RECEVING = 1,
	ESP_RXSTATE_RECEIVING_DATA = 8,
	ESP_RXSTATE_WAITING_TAIL2 = 9,

	ESP_RXSTATE_TIMEOUT = 20

} ESP_enum_RXSTATE;

// int8_t ESP_Parameters_Modify(uint8_t *datalink);
void ESP_Init(uint32_t baudrate);
void ESP_RX_PackingReceiver(uint8_t Byte);	// 返回当前包的长度
void ESP_RXbuffer_Processor(void);
uint8_t ESP_GetPackInfo(ESP_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax);		// 返回个数和包信息

uint8_t ESP_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len);		// 与Serial_DMADirectSend配合使用
void ESP_DMASetDirectSendEnabled(uint8_t newState);

typedef struct 
{
	#define ESP_RXBUFFER_SIZE 192								// 已扩容 64 -> 96 -> 192
	#define ESP_PACK_MAXSIZE 64		// 溢出则截断为两个包		  // 已扩容 48 -> 64
	#define ESP_MSG_NUMBER 30									// 已扩容 4 -> 6
	#define ESP_QUEUE_SIZE 256									// 已扩容 32 -> 80 -> 256
	#define ESP_TXQUEUE_SIZE 80	// 摄像头测试
		//	Comm
	// 协议，包头尾 in Serial.c
	const uint8_t PACK_HEAD;	// double same head and tail
	const uint8_t PACK_HEAD_2;
	const uint8_t PACK_TAIL;
	const uint8_t PACK_TAIL_2;
	
	uint8_t RxPacket[ESP_RXBUFFER_SIZE];	// ESP收到的包循环缓冲队列
	uint16_t pRxPacket;			// 包缓冲队列位置指针
	ESP_struct_packInfo RxPackInfo[ESP_MSG_NUMBER];	// ESP包标识信息
	uint8_t RxPackInfoPos;		// 包缓冲队列头指针数组位置指针
	uint8_t RxPackInfoUnreadNum;		// 包缓冲队列头指针数组读指针

	uint8_t RxDataQueue[ESP_QUEUE_SIZE];		// ESP_DMA循环队列
	uint16_t queue_pos;			// ESP循环队列位置指针, 255是错误代码，表示接收暂未开始
	uint16_t last_queue_pos;		// ESP循环队列上一次位置指针，接收从0开始，故初始化为0

	uint8_t TxDataQueue[ESP_TXQUEUE_SIZE];		// 测试：DMA发送缓存	不适用环形队列，因为计数器到0才会停，也不想使用了
	uint8_t isDMATxDirectSending;

	uint8_t test;
	uint8_t RxFlag;			// 接收打包完毕标志位
	uint8_t RxAck;
	uint8_t RxPacketLen_ERRVAL;
	ESP_enum_RXSTATE RxState;		// 接收状态机	

	uint8_t isIgnoreEmptyMsg;

} ESPVARS;

#endif
