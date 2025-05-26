#include "COMM.h"
#include "Serial.h"
#include "main.h"
#include "abstractInterfaceFunction.h"

#include "pid.h"
#include "motor.h"

#include <string.h>


/* friend visit
*/
extern PIDVARS pv;
extern MAINVARS mv;
////

// 保留构造函数，将const移到初始化列表，防止被优化掉
COMMVARS cv = 
{
	.PACK_HEAD = 0xFF,
	.PACK_HEAD_2 = 0xEE,
	.PACK_HEAD_3 = 0xFD,
	.PACK_HEAD_4 = 0xCC,

	.PACK_TAIL = 0xDD,
	.PACK_TAIL_2 = 0xDD,
	.PACK_TAIL_3 = 0xDD,
};

void COMMVARS_Constructor(void)
{
		//	COMM
	// cv.RxPacket;
	// cv.RxDataQueue;
	cv.queue_pos = 255;			// Comm循环队列位置指针, 255是错误代码，表示接收暂未开始
	cv.last_queue_pos = 0;		// Comm循环队列上一次位置指针，接收从0开始，故初始化为0

	cv.test = 0x43;
	// cv.RxFlag;
	// cv.RxAck;
	cv.pRxPacket = 0;			// COMM
	cv.RxPacketLen_ERRVAL = 255;
	cv.RxState = RXSTATE_WAITING_HEAD1;		// Comm接收状态机	

	cv.isDMATxDirectSending = 0;
}

void _COMM_PackStart(void);
uint8_t _COMM_GetPackLen(void);
void _COMM_PackEncapsulate(uint8_t len);
_Bool _COMM_PackAppend(uint8_t Byte);


void COMM_RX_PackingReceiver(uint8_t Byte)
{
	static uint8_t HeadID = 0;
	static uint8_t isPackup;

	switch (cv.RxState)
	{
	case RXSTATE_WAITING_HEAD1:
		if (cv.RxFlag == 0)
		{
			cv.RxState = RXSTATE_WAITING_HEAD2;
			
			if (Byte == cv.PACK_HEAD)
			{
				HeadID = 1;
			}
			else if (Byte == cv.PACK_HEAD_2)
			{
				HeadID = 2;
			}
			else if (Byte == cv.PACK_HEAD_3)
			{
				HeadID = 3;
			}
			else if (Byte == cv.PACK_HEAD_4)
			{
				HeadID = 4;
			}
		}
		break;
		
	case RXSTATE_WAITING_HEAD2:
		if ((HeadID == 1 && Byte == cv.PACK_HEAD)
			|| (HeadID == 2 && Byte == cv.PACK_HEAD_2)
			|| (HeadID == 3 && Byte == cv.PACK_HEAD_3)
			|| (HeadID == 4 && Byte == cv.PACK_HEAD_4)
			)		// 包接收开始
		{
			cv.RxState = RXSTATE_RECEIVING_DATA;
			_COMM_PackStart();
		}
		else
		{
			HeadID = 0;
			cv.RxState = RXSTATE_WAITING_HEAD1;
		}
		break;
		// if (HeadID == 1 && Byte == cv.PACK_HEAD)
		// {
		// 	cv.RxState = 8;
		// }
		// else if (HeadID == 2 && Byte == cv.PACK_HEAD_2)
		// {
		// 	cv.RxState = 8;
		// }
		// else if (HeadID == 3 && Byte == cv.PACK_HEAD_3)
		// {
		// 	cv.RxState = 8;
		// }
						
	case RXSTATE_RECEIVING_DATA:
		if (Byte == cv.PACK_TAIL)
		{
			cv.RxState = RXSTATE_WAITING_TAIL2;
		}
		else	// Data process
		{
			if (_COMM_PackAppend(Byte) == 0)		// 溢出操作
			{
				isPackup = 1;
				break;
			}
		}
		break;
		
	case RXSTATE_WAITING_TAIL2:
		if (Byte == cv.PACK_TAIL)	// 包接收结束
		{
			isPackup = 1;
			break;
		}
		else 	// save 0xFE and data
		{
			cv.RxState = RXSTATE_RECEIVING_DATA;
			if (_COMM_PackAppend(cv.PACK_TAIL) == 0)
			{
				isPackup = 2;
				break;
			}
			if (_COMM_PackAppend(Byte) == 0)
			{
				isPackup = 1;
				break;
			}
		}
			break;	
	}

	if (isPackup > 0)		// 封包操作
	{
		cv.RxState = RXSTATE_WAITING_HEAD1;
		cv.RxFlag = HeadID;
		_COMM_PackEncapsulate(_COMM_GetPackLen());
		HeadID = 0;

		if (isPackup == 2)		// 特殊情况，包尾接收错误时，处理遗留数据
			COMM_RX_PackingReceiver(Byte);

		isPackup = 0;
	}


}

void COMM_RXbuffer_Processor(void)
{
	uint8_t currentByte;
	uint8_t temp;
	uint8_t i;
	
	// 计算数据区间 last_COMMvars.queue_pos ~ cv.queue_pos
	if (cv.queue_pos == 255)
	{
		temp = COMM_RXQUEUE_SIZE - DMA_GetCurrDataCounter(COMM_RX_DMA_CHANNEL);
		if (temp != 0)		// 表示接收开始了
			cv.queue_pos = temp;	// 退出错误代码255
		else				// 接收未开始，返回
			return ;
	}
	else
		cv.queue_pos = COMM_RXQUEUE_SIZE - DMA_GetCurrDataCounter(COMM_RX_DMA_CHANNEL);
	
	// 依次取出Byte，作为包机制参数
	for (uint8_t i = 0; i < (cv.queue_pos + COMM_RXQUEUE_SIZE - cv.last_queue_pos) % COMM_RXQUEUE_SIZE; i++)
	{
		currentByte = cv.RxDataQueue[(cv.last_queue_pos + COMM_RXQUEUE_SIZE + i) % COMM_RXQUEUE_SIZE];
//		Serial_SendByte(currentByte, 2);	// DEBUG
		
		COMM_RX_PackingReceiver(currentByte);
	}
	
	cv.last_queue_pos = cv.queue_pos;		// 保存此次指针
}

void COMM_Init(uint32_t baudrate)
{
//// 初始化全局变量
	COMMVARS_Constructor();
//RCC
	ABS_RCC_GPIO_ClockCmd(COMM_GPIOX, ENABLE);
	ABS_RCC_USART_ClockCmd(COMM_USART, ENABLE);
	ABS_RCC_DMA_ClockCmd(COMM_RX_DMAX, ENABLE);		// DMA Clock
	
//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = COMM_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COMM_GPIOX, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = COMM_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COMM_GPIOX, &GPIO_InitStructure);
	
//USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(COMM_USART, &USART_InitStructure);
	
// //Interrupt
// 	USART_ITConfig(COMM_USART, USART_IT_RXNE, ENABLE);
	
// 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	NVIC_InitStructure.NVIC_IRQChannel = COMM_RX_IRQN;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// 	NVIC_Init(&NVIC_InitStructure);

//// DMA
	DMA_InitTypeDef DMA_InitStructure;
	// 接收
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&COMM_USART->DR;	// 小端存储
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&cv.RxDataQueue;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = COMM_RXQUEUE_SIZE;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(COMM_RX_DMA_CHANNEL, &DMA_InitStructure);
	
	DMA_Cmd(COMM_RX_DMA_CHANNEL, ENABLE);

	//Turn on DMA of USART
	USART_DMACmd(COMM_USART, USART_DMAReq_Rx, ENABLE);

	// 发送
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&COMM_USART->DR;	// 小端存储
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&cv.TxDataQueue;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		// 外设是目标内存
	DMA_InitStructure.DMA_BufferSize = COMM_TXQUEUE_SIZE;	// 内存大小
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(COMM_TX_DMA_CHANNEL, &DMA_InitStructure);
	
	DMA_SetCurrDataCounter(COMM_TX_DMA_CHANNEL, 0);
	// DMA_Cmd(COMM_TX_DMA_CHANNEL, ENABLE);	// 开始时不开启

	//Turn on DMA of USART
	USART_DMACmd(COMM_USART, USART_DMAReq_Tx, ENABLE);


//// Turn on USART
	USART_Cmd(COMM_USART, ENABLE);
}

uint8_t _COMM_GetPackLen(void)
{
	return ABS_CycleSubPos(cv.pRxPacket, cv.RxPackInfo[cv.RxPackInfoPos].head, COMM_RXBUFFER_SIZE);
}

uint8_t COMM_GetPackInfo(COMM_struct_packInfo *get, uint8_t currentPos, uint8_t msgNumMax)		// 返回个数和包信息
{
	uint8_t ret;
	if (currentPos >= msgNumMax)
		return 0;
	for (uint8_t i = 0; i < cv.RxPackInfoUnreadNum; i++)		// currentPos是main的旧包指针，cv.RxPackInfoPos是COMM新的包指针
	{
		*(get + ABS_CycleAddPos(currentPos, i, msgNumMax)) = 
			*(cv.RxPackInfo + ABS_CycleSubPos(cv.RxPackInfoPos, cv.RxPackInfoUnreadNum - i, COMM_MSG_NUMBER));
	}
	ret = cv.RxPackInfoUnreadNum;
	cv.RxPackInfoUnreadNum = 0;		// 清除未读消息数目

	return ret;
}

_Bool _COMM_PackAppend(uint8_t Byte)
{
	cv.RxPacket[cv.pRxPacket] = Byte;
	cv.pRxPacket ++;
	cv.pRxPacket %= COMM_RXBUFFER_SIZE;

	if (_COMM_GetPackLen() >= COMM_PACK_MAXSIZE)	// 包满
	{
		return 0;	// 必须及时处理此返回值
	}
	return 1;
}

void _COMM_PackStart(void)
{
	// 标识包头
	cv.RxPackInfo[cv.RxPackInfoPos].head = cv.pRxPacket;
}

void _COMM_PackEncapsulate(uint8_t len)
{
	// 封装信息：计算长度，标注flag
	cv.RxPackInfo[cv.RxPackInfoPos].len = len;
	cv.RxPackInfo[cv.RxPackInfoPos].flag = cv.RxFlag;
	cv.RxFlag = 0;

	cv.RxPackInfoPos++;					// 包指针++
	cv.RxPackInfoPos %= COMM_MSG_NUMBER;
	if (++cv.RxPackInfoUnreadNum > COMM_MSG_NUMBER)				// 未读消息++，限幅4
		cv.RxPackInfoUnreadNum--;
}


// DMA相关函数
uint8_t COMM_WriteDMASendQueue(uint8_t *data, uint16_t destStart, uint16_t len)		// 与Serial_DMADirectSend配合使用
{
	if (DMA_GetCurrDataCounter(COMM_TX_DMA_CHANNEL) != 0)
		return 1;		// 正在发送，不可写入

	if (destStart + len > COMM_TXQUEUE_SIZE)
		return 2;		// 写溢出

	strncpy((char *)cv.TxDataQueue + destStart, (char *)data, len);
	return 0;
}

void COMM_DMASetDirectSendEnabled(uint8_t newState)
{
	cv.isDMATxDirectSending = newState;
}

