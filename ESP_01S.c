#include "ESP_01S.h"
#include "pid.h"
#include "Serial.h"
#include "main.h"
#include "abstractInterfaceFunction.h"
#include <string.h>


////
// friend visit



// 保留构造函数，将const移到初始化列表
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
	ev.queue_pos = 255;			// Comm循环队列位置指针, 255是错误代码，表示接收暂未开始
	ev.last_queue_pos = 0;		// Comm循环队列上一次位置指针，接收从0开始，故初始化为0

	ev.test = 0x43;
	// ev.RxFlag;
	// ev.RxAck;
	ev.pRxPacket = 0;			// ESP
	ev.RxPacketLen_ERRVAL = 255;
	ev.RxState = ESP_RXSTATE_RECEIVING_DATA;		// Comm接收状态机	

	ev.isIgnoreEmptyMsg = 1;
}

void _ESP_PackStart(void);
uint8_t _ESP_GetPackLen(void);
void _ESP_PackEncapsulate(uint8_t len);
_Bool _ESP_PackAppend(uint8_t Byte);


void ESP_RX_PackingReceiver(uint8_t Byte)		// 分包器
{
	// static uint8_t HeadID = 0;
	static uint8_t isPackup = 0;	// 0不响应，1正常封包，2处理遗留数据

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
		if (Byte == ev.PACK_TAIL_2)								// 包接收结束
		{
			isPackup = 1;
		}
		else 	// 包尾2错误
		{
			ev.RxState = ESP_RXSTATE_RECEIVING_DATA;
			if (_ESP_PackAppend(ev.PACK_TAIL) == 0)
			{
				isPackup = 2;       // 处理遗留数据

				break;
			}
			if (_ESP_PackAppend(Byte) == 0)
			{
				isPackup = 1;
				break;
			}
		}
		break;
	case ESP_RXSTATE_TIMEOUT:		// 超时操作
		isPackup = 1;
		break;
	}

	if (isPackup > 0)		// 封包操作
	{
		ev.RxState = ESP_RXSTATE_RECEIVING_DATA;
		ev.RxFlag = 1;
		_ESP_PackEncapsulate(_ESP_GetPackLen());		// 溢出打包

		_ESP_PackStart();		// 标志接收开始

		if (isPackup == 2)		// 特殊情况，包尾接收错误时，处理遗留数据
			ESP_RX_PackingReceiver(Byte);

		isPackup = 0;
	}
}

void ESP_RXbuffer_Processor(void)		// 将DMA环形队列的数据逐个呈递给分包器
{
	uint8_t currentByte;
	uint8_t temp;
	uint8_t count;
	static uint8_t timeoutCounter;
	
	// 计算数据区间 last_ESPvars.queue_pos ~ ev.queue_pos
	if (ev.queue_pos == 255)
	{
		temp = ESP_QUEUE_SIZE - DMA_GetCurrDataCounter(ESP_RX_DMA_CHANNEL);
		if (temp != 0)		// 表示接收开始了
			ev.queue_pos = temp;	// 退出错误代码255
		else				// 接收未开始，返回
			return ;
	}
	else
		ev.queue_pos = ESP_QUEUE_SIZE - DMA_GetCurrDataCounter(ESP_RX_DMA_CHANNEL);
	
	// 依次取出Byte，作为包机制参数
	for (count = 0; count < (ev.queue_pos + ESP_QUEUE_SIZE - ev.last_queue_pos) % ESP_QUEUE_SIZE; count++)
	{
		currentByte = ev.RxDataQueue[(ev.last_queue_pos + ESP_QUEUE_SIZE + count) % ESP_QUEUE_SIZE];
//		Serial_SendByte(currentByte, 2);	// DEBUG
		
		ESP_RX_PackingReceiver(currentByte);
	}

	if (count == 0 && (_ESP_GetPackLen() != 0))		// for循环的else；	本轮无新数据，当前包不为空
	{	// 如果包不为空，且本轮没有从DMA环形队列收到任何数据，则判断是否超时，改至超时状态，并立即调用分包器处理超时
		// 所在事件循环是10ms，设定20ms为超时线，即第2次进入该函数无数据：0(有) -> 1(无) -> 2(无) -> 超时
		timeoutCounter++;
		if (timeoutCounter >= 2)	// 超时
		{
			timeoutCounter = 0;
			ev.RxState = ESP_RXSTATE_TIMEOUT;
			ESP_RX_PackingReceiver(0x00);		// 立即调用分包器
		}
	}
	else	// for循环执行结束的再执行语句
		timeoutCounter = 0;
	
	ev.last_queue_pos = ev.queue_pos;		// 保存此次指针
}


void ESP_Init(uint32_t baudrate)
{
// 初始化全局变量
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
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ESP_USART->DR;	// 小端存储
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


// DMA发送
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ESP_USART->DR;	// 小端存储
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ev.TxDataQueue;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		// 外设是目标内存
	DMA_InitStructure.DMA_BufferSize = ESP_TXQUEUE_SIZE;	// 内存大小
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(ESP_TX_DMA_CHANNEL, &DMA_InitStructure);
	
	DMA_SetCurrDataCounter(ESP_TX_DMA_CHANNEL, 0);
	// DMA_Cmd(ESP_TX_DMA_CHANNEL, ENABLE);	// 开始时不开启

	//Turn on DMA of USART
	USART_DMACmd(ESP_USART, USART_DMAReq_Tx, ENABLE);



//Turn on USART
	USART_Cmd(ESP_USART, ENABLE);
}

uint8_t _ESP_GetPackLen(void)
{
	return ABS_CycleSubPos(ev.pRxPacket, ev.RxPackInfo[ev.RxPackInfoPos].head, ESP_RXBUFFER_SIZE);
}

uint8_t ESP_GetPackInfo(ESP_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax)		// 返回个数和包信息
{
	uint8_t ret;
	if (currentPos >= msgNumMax)
		return 0;
	for (uint8_t i = 0; i < ev.RxPackInfoUnreadNum; i++)		// currentPos是main的旧包指针，ev.RxPackInfoPos是ESP新的包指针
	{
		*(get + ABS_CycleAddPos(currentPos, i, msgNumMax)) = 
			*(ev.RxPackInfo + ABS_CycleSubPos(ev.RxPackInfoPos, ev.RxPackInfoUnreadNum - i, ESP_MSG_NUMBER));
	}
	ret = ev.RxPackInfoUnreadNum;
	ev.RxPackInfoUnreadNum = 0;		// 清除未读消息数目

	return ret;
}

_Bool _ESP_PackAppend(uint8_t Byte)
{
	ev.RxPacket[ev.pRxPacket] = Byte;
	ev.pRxPacket ++;
	ev.pRxPacket %= ESP_RXBUFFER_SIZE;

	if (_ESP_GetPackLen() >= ESP_PACK_MAXSIZE)	// 包满
	{
		return 0;	// 必须及时处理此返回值
	}
	return 1;
}

void _ESP_PackStart(void)
{
	// 标识包头
	ev.RxPackInfo[ev.RxPackInfoPos].head = ev.pRxPacket;
}

void _ESP_PackEncapsulate(uint8_t len)
{
	if (ev.isIgnoreEmptyMsg == 1 && len == 0)		// 忽略空包
	{
		return;
	}

	// 封装信息：计算长度，标注flag
	ev.RxPackInfo[ev.RxPackInfoPos].len = len;
	ev.RxPackInfo[ev.RxPackInfoPos].flag = ev.RxFlag;
	ev.RxFlag = 0;

	ev.RxPackInfoPos++;					// 包指针++
	ev.RxPackInfoPos %= ESP_MSG_NUMBER;
	if (++ev.RxPackInfoUnreadNum > ESP_MSG_NUMBER)				// 未读消息++，限幅
		ev.RxPackInfoUnreadNum--;
}


// DMA相关函数
uint8_t ESP_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len)		// 与Serial_DMADirectSend配合使用
{
	if (DMA_GetCurrDataCounter(ESP_TX_DMA_CHANNEL) != 0)
		return 1;		// 正在发送，不可写入

	if (destStart + len > ESP_TXQUEUE_SIZE)
		return 2;		// 写溢出

	strncpy((char *)ev.TxDataQueue + destStart, (char *)data, len);
	return 0;
}

void ESP_DMASetDirectSendEnabled(uint8_t newState)
{
	ev.isDMATxDirectSending = newState;
}


