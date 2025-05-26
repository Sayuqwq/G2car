#include "ESP_01S.h"
#include "pid.h"
#include "Serial.h"
#include "main.h"
#include "abstractInterfaceFunction.h"
#include <string.h>


////
// friend visit



// �������캯������const�Ƶ���ʼ���б�
ESPVARS ev = 
{
	.PACK_HEAD = 0xFF,
	.PACK_HEAD_2 = 0xEE,

	.PACK_TAIL = 0x0D,
	.PACK_TAIL_2 = 0x0A,
};

void ESPVARS_Constructor(void)
{
		//	ESP
	// ev.RxPacket;
	// ev.RxDataQueue;
	ev.queue_pos = 255;			// Commѭ������λ��ָ��, 255�Ǵ�����룬��ʾ������δ��ʼ
	ev.last_queue_pos = 0;		// Commѭ��������һ��λ��ָ�룬���մ�0��ʼ���ʳ�ʼ��Ϊ0

	ev.test = 0x43;
	// ev.RxFlag;
	// ev.RxAck;
	ev.pRxPacket = 0;			// ESP
	ev.RxPacketLen_ERRVAL = 255;
	ev.RxState = ESP_RXSTATE_RECEIVING_DATA;		// Comm����״̬��	

	ev.isIgnoreEmptyMsg = 1;
}

void _ESP_PackStart(void);
uint8_t _ESP_GetPackLen(void);
void _ESP_PackEncapsulate(uint8_t len);
_Bool _ESP_PackAppend(uint8_t Byte);


void ESP_RX_PackingReceiver(uint8_t Byte)		// �ְ���
{
	// static uint8_t HeadID = 0;
	static uint8_t isPackup = 0;	// 0����Ӧ��1���������2������������

	switch (ev.RxState)
	{
	case ESP_RXSTATE_RECEIVING_DATA:
		if (Byte == ev.PACK_TAIL)
		{
			ev.RxState = ESP_RXSTATE_WAITING_TAIL2;
		}
		else	// Data process
		{
			if (_ESP_PackAppend(Byte) == 0)
			{
				isPackup = 1;
				break;
			}
		}
		break;
		
	case ESP_RXSTATE_WAITING_TAIL2:
		if (Byte == ev.PACK_TAIL_2)								// �����ս���
		{
			isPackup = 1;
		}
		else 	// ��β2����
		{
			ev.RxState = ESP_RXSTATE_RECEIVING_DATA;
			if (_ESP_PackAppend(ev.PACK_TAIL) == 0)
			{
				isPackup = 2;       // ������������

				break;
			}
			if (_ESP_PackAppend(Byte) == 0)
			{
				isPackup = 1;
				break;
			}
		}
		break;
	case ESP_RXSTATE_TIMEOUT:		// ��ʱ����
		isPackup = 1;
		break;
	}

	if (isPackup > 0)		// �������
	{
		ev.RxState = ESP_RXSTATE_RECEIVING_DATA;
		ev.RxFlag = 1;
		_ESP_PackEncapsulate(_ESP_GetPackLen());		// ������

		_ESP_PackStart();		// ��־���տ�ʼ

		if (isPackup == 2)		// �����������β���մ���ʱ��������������
			ESP_RX_PackingReceiver(Byte);

		isPackup = 0;
	}
}

void ESP_RXbuffer_Processor(void)		// ��DMA���ζ��е���������ʵݸ��ְ���
{
	uint8_t currentByte;
	uint8_t temp;
	uint8_t count;
	static uint8_t timeoutCounter;
	
	// ������������ last_ESPvars.queue_pos ~ ev.queue_pos
	if (ev.queue_pos == 255)
	{
		temp = ESP_QUEUE_SIZE - DMA_GetCurrDataCounter(ESP_RX_DMA_CHANNEL);
		if (temp != 0)		// ��ʾ���տ�ʼ��
			ev.queue_pos = temp;	// �˳��������255
		else				// ����δ��ʼ������
			return ;
	}
	else
		ev.queue_pos = ESP_QUEUE_SIZE - DMA_GetCurrDataCounter(ESP_RX_DMA_CHANNEL);
	
	// ����ȡ��Byte����Ϊ�����Ʋ���
	for (count = 0; count < (ev.queue_pos + ESP_QUEUE_SIZE - ev.last_queue_pos) % ESP_QUEUE_SIZE; count++)
	{
		currentByte = ev.RxDataQueue[(ev.last_queue_pos + ESP_QUEUE_SIZE + count) % ESP_QUEUE_SIZE];
//		Serial_SendByte(currentByte, 2);	// DEBUG
		
		ESP_RX_PackingReceiver(currentByte);
	}

	if (count == 0 && (_ESP_GetPackLen() != 0))		// forѭ����else��	�����������ݣ���ǰ����Ϊ��
	{	// �������Ϊ�գ��ұ���û�д�DMA���ζ����յ��κ����ݣ����ж��Ƿ�ʱ��������ʱ״̬�����������÷ְ�������ʱ
		// �����¼�ѭ����10ms���趨20msΪ��ʱ�ߣ�����2�ν���ú��������ݣ�0(��) -> 1(��) -> 2(��) -> ��ʱ
		timeoutCounter++;
		if (timeoutCounter >= 2)	// ��ʱ
		{
			timeoutCounter = 0;
			ev.RxState = ESP_RXSTATE_TIMEOUT;
			ESP_RX_PackingReceiver(0x00);		// �������÷ְ���
		}
	}
	else	// forѭ��ִ�н�������ִ�����
		timeoutCounter = 0;
	
	ev.last_queue_pos = ev.queue_pos;		// ����˴�ָ��
}


void ESP_Init(uint32_t baudrate)
{
// ��ʼ��ȫ�ֱ���
	ESPVARS_Constructor();
//RCC
	ABS_RCC_GPIO_ClockCmd(ESP_GPIOX, ENABLE);
	ABS_RCC_USART_ClockCmd(ESP_USART, ENABLE);
	ABS_RCC_DMA_ClockCmd(ESP_RX_DMAX, ENABLE);		// DMA Clock
	
//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = ESP_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ESP_GPIOX, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = ESP_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ESP_GPIOX, &GPIO_InitStructure);
	
//USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(ESP_USART, &USART_InitStructure);
	
// //Interrupt
// 	USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);
	
// 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	NVIC_InitStructure.NVIC_IRQChannel = ESP_RX_IRQN;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// 	NVIC_Init(&NVIC_InitStructure);

//// DMA
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ESP_USART->DR;	// С�˴洢
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&test;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ev.RxDataQueue;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ESP_QUEUE_SIZE;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(ESP_RX_DMA_CHANNEL, &DMA_InitStructure);
	
	DMA_Cmd(ESP_RX_DMA_CHANNEL, ENABLE);

//Turn on DMA of USART
	USART_DMACmd(ESP_USART, USART_DMAReq_Rx, ENABLE);


// DMA����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ESP_USART->DR;	// С�˴洢
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ev.TxDataQueue;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		// ������Ŀ���ڴ�
	DMA_InitStructure.DMA_BufferSize = ESP_TXQUEUE_SIZE;	// �ڴ��С
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(ESP_TX_DMA_CHANNEL, &DMA_InitStructure);
	
	DMA_SetCurrDataCounter(ESP_TX_DMA_CHANNEL, 0);
	// DMA_Cmd(ESP_TX_DMA_CHANNEL, ENABLE);	// ��ʼʱ������

	//Turn on DMA of USART
	USART_DMACmd(ESP_USART, USART_DMAReq_Tx, ENABLE);



//Turn on USART
	USART_Cmd(ESP_USART, ENABLE);
}

uint8_t _ESP_GetPackLen(void)
{
	return ABS_CycleSubPos(ev.pRxPacket, ev.RxPackInfo[ev.RxPackInfoPos].head, ESP_RXBUFFER_SIZE);
}

uint8_t ESP_GetPackInfo(ESP_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax)		// ���ظ����Ͱ���Ϣ
{
	uint8_t ret;
	if (currentPos >= msgNumMax)
		return 0;
	for (uint8_t i = 0; i < ev.RxPackInfoUnreadNum; i++)		// currentPos��main�ľɰ�ָ�룬ev.RxPackInfoPos��ESP�µİ�ָ��
	{
		*(get + ABS_CycleAddPos(currentPos, i, msgNumMax)) = 
			*(ev.RxPackInfo + ABS_CycleSubPos(ev.RxPackInfoPos, ev.RxPackInfoUnreadNum - i, ESP_MSG_NUMBER));
	}
	ret = ev.RxPackInfoUnreadNum;
	ev.RxPackInfoUnreadNum = 0;		// ���δ����Ϣ��Ŀ

	return ret;
}

_Bool _ESP_PackAppend(uint8_t Byte)
{
	ev.RxPacket[ev.pRxPacket] = Byte;
	ev.pRxPacket ++;
	ev.pRxPacket %= ESP_RXBUFFER_SIZE;

	if (_ESP_GetPackLen() >= ESP_PACK_MAXSIZE)	// ����
	{
		return 0;	// ���뼰ʱ����˷���ֵ
	}
	return 1;
}

void _ESP_PackStart(void)
{
	// ��ʶ��ͷ
	ev.RxPackInfo[ev.RxPackInfoPos].head = ev.pRxPacket;
}

void _ESP_PackEncapsulate(uint8_t len)
{
	if (ev.isIgnoreEmptyMsg == 1 && len == 0)		// ���Կհ�
	{
		return;
	}

	// ��װ��Ϣ�����㳤�ȣ���עflag
	ev.RxPackInfo[ev.RxPackInfoPos].len = len;
	ev.RxPackInfo[ev.RxPackInfoPos].flag = ev.RxFlag;
	ev.RxFlag = 0;

	ev.RxPackInfoPos++;					// ��ָ��++
	ev.RxPackInfoPos %= ESP_MSG_NUMBER;
	if (++ev.RxPackInfoUnreadNum > ESP_MSG_NUMBER)				// δ����Ϣ++���޷�
		ev.RxPackInfoUnreadNum--;
}


// DMA��غ���
uint8_t ESP_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len)		// ��Serial_DMADirectSend���ʹ��
{
	if (DMA_GetCurrDataCounter(ESP_TX_DMA_CHANNEL) != 0)
		return 1;		// ���ڷ��ͣ�����д��

	if (destStart + len > ESP_TXQUEUE_SIZE)
		return 2;		// д���

	strncpy((char *)ev.TxDataQueue + destStart, (char *)data, len);
	return 0;
}

void ESP_DMASetDirectSendEnabled(uint8_t newState)
{
	ev.isDMATxDirectSending = newState;
}


