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
uint8_t COMM_GetPackInfo(COMM_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax);		// ���ظ����Ͱ���Ϣ

uint8_t COMM_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len);		// ��Serial_DMADirectSend���ʹ��
void COMM_DMASetDirectSendEnabled(uint8_t newState);


typedef struct 
{
	#define COMM_RXBUFFER_SIZE 64
	#define COMM_PACK_MAXSIZE 47
	#define COMM_MSG_NUMBER 4
	#define COMM_RXQUEUE_SIZE 64
	#define COMM_TXQUEUE_SIZE 320	// ����
		//	Comm
	// Э�飬��ͷβ in Serial.c
	const uint8_t PACK_HEAD;	// double same head and tail
	const uint8_t PACK_HEAD_2;
	const uint8_t PACK_HEAD_3;
	const uint8_t PACK_HEAD_4;	// ��������ң��
	const uint8_t PACK_TAIL;
	const uint8_t PACK_TAIL_2;
	const uint8_t PACK_TAIL_3;
	
	uint8_t RxPacket[COMM_RXBUFFER_SIZE];	// COMM�յ��İ�ѭ���������
	uint16_t pRxPacket;			// ���������λ��ָ��
	COMM_struct_packInfo RxPackInfo[COMM_MSG_NUMBER];	// ESP����ʶ��Ϣ
	uint8_t RxPackInfoPos;		// ���������ͷָ������λ��ָ��
	uint8_t RxPackInfoUnreadNum;		// ���������ͷָ�������ָ��

	uint8_t RxDataQueue[COMM_RXQUEUE_SIZE];		// COMM_DMAѭ������
	uint16_t queue_pos;			// Commѭ������λ��ָ��, 255�Ǵ�����룬��ʾ������δ��ʼ
	uint16_t last_queue_pos;		// Commѭ��������һ��λ��ָ�룬���մ�0��ʼ���ʳ�ʼ��Ϊ0

	uint8_t TxDataQueue[COMM_TXQUEUE_SIZE];		// ���ԣ�DMA���ͻ���	�����û��ζ��У���Ϊ��������0�Ż�ͣ��Ҳ����ʹ����
	uint8_t isDMATxDirectSending;

	uint8_t test;
	uint8_t RxFlag;			// ���մ����ϱ�־λ
	uint8_t RxAck;
	uint8_t RxPacketLen_ERRVAL;
	COMM_enum_RXSTATE RxState;		// Comm����״̬��	
} COMMVARS;


#endif
