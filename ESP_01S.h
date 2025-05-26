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
void ESP_RX_PackingReceiver(uint8_t Byte);	// ���ص�ǰ���ĳ���
void ESP_RXbuffer_Processor(void);
uint8_t ESP_GetPackInfo(ESP_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax);		// ���ظ����Ͱ���Ϣ

uint8_t ESP_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len);		// ��Serial_DMADirectSend���ʹ��
void ESP_DMASetDirectSendEnabled(uint8_t newState);

typedef struct 
{
	#define ESP_RXBUFFER_SIZE 192								// ������ 64 -> 96 -> 192
	#define ESP_PACK_MAXSIZE 64		// �����ض�Ϊ������		  // ������ 48 -> 64
	#define ESP_MSG_NUMBER 30									// ������ 4 -> 6
	#define ESP_QUEUE_SIZE 256									// ������ 32 -> 80 -> 256
	#define ESP_TXQUEUE_SIZE 80	// ����ͷ����
		//	Comm
	// Э�飬��ͷβ in Serial.c
	const uint8_t PACK_HEAD;	// double same head and tail
	const uint8_t PACK_HEAD_2;
	const uint8_t PACK_TAIL;
	const uint8_t PACK_TAIL_2;
	
	uint8_t RxPacket[ESP_RXBUFFER_SIZE];	// ESP�յ��İ�ѭ���������
	uint16_t pRxPacket;			// ���������λ��ָ��
	ESP_struct_packInfo RxPackInfo[ESP_MSG_NUMBER];	// ESP����ʶ��Ϣ
	uint8_t RxPackInfoPos;		// ���������ͷָ������λ��ָ��
	uint8_t RxPackInfoUnreadNum;		// ���������ͷָ�������ָ��

	uint8_t RxDataQueue[ESP_QUEUE_SIZE];		// ESP_DMAѭ������
	uint16_t queue_pos;			// ESPѭ������λ��ָ��, 255�Ǵ�����룬��ʾ������δ��ʼ
	uint16_t last_queue_pos;		// ESPѭ��������һ��λ��ָ�룬���մ�0��ʼ���ʳ�ʼ��Ϊ0

	uint8_t TxDataQueue[ESP_TXQUEUE_SIZE];		// ���ԣ�DMA���ͻ���	�����û��ζ��У���Ϊ��������0�Ż�ͣ��Ҳ����ʹ����
	uint8_t isDMATxDirectSending;

	uint8_t test;
	uint8_t RxFlag;			// ���մ����ϱ�־λ
	uint8_t RxAck;
	uint8_t RxPacketLen_ERRVAL;
	ESP_enum_RXSTATE RxState;		// ����״̬��	

	uint8_t isIgnoreEmptyMsg;

} ESPVARS;

#endif
