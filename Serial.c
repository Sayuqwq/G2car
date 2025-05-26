#include "Serial.h"
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "alias.h"
#include "abstractInterfaceFunction.h"
#include "COMM.h"
#include "ESP_01S.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* friend visit
*/
extern COMMVARS cv;
extern ESPVARS ev;
////

// DMA 发送函数
uint8_t Serial_DMACopySend(uint8_t *data, uint16_t len, USART_TypeDef *USART_ID)
{
	DMA_Channel_TypeDef *DMAChannel;
	uint16_t DMATxQueueSize;
	uint8_t isDMADirectSendOn;

	if (USART_ID == COMM_USART)
	{
		DMAChannel = COMM_TX_DMA_CHANNEL;
		DMATxQueueSize = COMM_TXQUEUE_SIZE;
		isDMADirectSendOn = cv.isDMATxDirectSending;
	}
	else		// 测试
	{
		DMAChannel = ESP_TX_DMA_CHANNEL;
		DMATxQueueSize = ESP_TXQUEUE_SIZE;
		isDMADirectSendOn = ev.isDMATxDirectSending;
	}
	
	if (isDMADirectSendOn == 1)
		return 3;

	if (DMA_GetCurrDataCounter(DMAChannel) != 0)
		return 1;

	if (len == 0)
		return 2;
	if (len > DMATxQueueSize)
		len = DMATxQueueSize;

	
	strncpy((char *)cv.TxDataQueue, (char *)data, len);
	DMA_Cmd(DMAChannel, DISABLE);
	DMA_SetCurrDataCounter(DMAChannel, len);
	DMA_Cmd(DMAChannel, ENABLE);

	return 0;
}
	
uint8_t Serial_DMADirectSend(uint16_t len, USART_TypeDef *USART_ID)
{
	DMA_Channel_TypeDef *DMAChannel;
	uint16_t DMATxQueueSize;
	uint8_t isDMADirectSendOn;

	uint8_t temp;

	if (USART_ID == COMM_USART)
	{
		DMAChannel = COMM_TX_DMA_CHANNEL;
		DMATxQueueSize = COMM_TXQUEUE_SIZE;
		isDMADirectSendOn = cv.isDMATxDirectSending;
	}
	else		// 测试
	{
		DMAChannel = ESP_TX_DMA_CHANNEL;
		DMATxQueueSize = ESP_TXQUEUE_SIZE;
		isDMADirectSendOn = ev.isDMATxDirectSending;
	}

	if (isDMADirectSendOn == 0)		// 须切换到DMA直接发送模式
		return 3;

	if (temp = DMA_GetCurrDataCounter(DMAChannel) != 0)
		return temp;

	if (len == 0)		// 长度为0则默认发送成功
		return 0;

	if (len > DMATxQueueSize)
		len = DMATxQueueSize;

	DMA_Cmd(DMAChannel, DISABLE);
	DMA_SetCurrDataCounter(DMAChannel, len);
	DMA_Cmd(DMAChannel, ENABLE);

	return 0;
}
	
void Serial_SendByte(uint8_t Byte, USART_TypeDef *USART_ID)
{
	while (USART_GetFlagStatus(USART_ID, USART_FLAG_TXE) == RESET);			// 调换了顺序，先检查是否发送完毕
	USART_SendData(USART_ID, Byte);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length, USART_TypeDef *USART_ID)
{
	uint16_t i;
	for(i = 0; i < Length; i++)
	{
		Serial_SendByte(Array[i], USART_ID);
	}
}

void Serial_SendString(char *String, USART_TypeDef *USART_ID)
{
	uint8_t i;
	for(i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte(String[i], USART_ID);		
	}
}

void Serial_SendString_Patch(char *String, uint16_t length, USART_TypeDef *USART_ID)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		Serial_SendByte(String[i], USART_ID);		
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while(Y--)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length, USART_TypeDef *USART_ID)
{
	uint8_t i;
	for(i = Length; i > 0; i--)
	{
		Serial_SendByte(Number / Serial_Pow(10, i-1) % 10 + '0', USART_ID);
	}
}

//int fputc(int ch, FILE *f, USART_TypeDef *USART_ID)			//Redirect
//{
//	Serial_SendByte(ch, USART_ID);
//	return ch;
//}

void Serial_Printf(USART_TypeDef *USART_ID, char *format, ...)			//catch format string //
{
	char String[100];
	va_list arg;			//define a parameter list variable,,,//va_list:type name //arg:variable name
	va_start(arg, format);			//receive param list from position of format,and put into arg
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String, USART_ID);
}

void Serial_Printf_Pack(uint8_t PackID, USART_TypeDef *USART_ID, char *format, ...)			//catch format string //
{
	char String[100];
	va_list arg;			//define a parameter list variable,,,//va_list:type name //arg:variable name
	va_start(arg, format);			//receive param list from position of format,and put into arg
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String, USART_ID);
}

void Serial_SendString_Pack_uint8_t(uint8_t *String, uint16_t length, uint8_t PackID, USART_TypeDef *USART_ID)
{
	uint8_t i;
	uint8_t pack_head;
	uint8_t pack_tail;
	
	if (USART_ID == COMM_USART)
	{
		if (PackID == 1)
		{
			pack_head = cv.PACK_HEAD;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 2)
		{
			pack_head = cv.PACK_HEAD_2;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 3)
		{
			pack_head = cv.PACK_HEAD_3;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 4)
		{
			pack_head = cv.PACK_HEAD_4;
			pack_tail = cv.PACK_TAIL;
		}
	}
	else if (USART_ID == ESP_USART)
	{
		if (PackID == 1)
		{
			pack_head = ev.PACK_HEAD;
			pack_tail = ev.PACK_TAIL;
		}
		else if (PackID == 2)
		{
			pack_head = ev.PACK_HEAD_2;
			pack_tail = ev.PACK_TAIL;
		}
	}

	Serial_SendByte(pack_head, USART_ID);
	Serial_SendByte(pack_head, USART_ID);
	for(i = 0; i < length; i++)
	{
		Serial_SendByte(String[i], USART_ID);		
	}
	Serial_SendByte(pack_tail, USART_ID);
	Serial_SendByte(pack_tail, USART_ID);

}

void Serial_SendString_Pack_char(char *String, uint16_t length, uint8_t PackID, USART_TypeDef *USART_ID)
{
	uint8_t i;
	uint8_t pack_head;
	uint8_t pack_tail;
	
	if (USART_ID == COMM_USART)
	{
		if (PackID == 1)
		{
			pack_head = cv.PACK_HEAD;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 2)
		{
			pack_head = cv.PACK_HEAD_2;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 3)
		{
			pack_head = cv.PACK_HEAD_3;
			pack_tail = cv.PACK_TAIL;
		}
		else if (PackID == 4)
		{
			pack_head = cv.PACK_HEAD_4;
			pack_tail = cv.PACK_TAIL;
		}
	}
	else if (USART_ID == ESP_USART)
	{
		if (PackID == 1)
		{
			pack_head = ev.PACK_HEAD;
			pack_tail = ev.PACK_TAIL;
		}
		else if (PackID == 2)
		{
			pack_head = ev.PACK_HEAD_2;
			pack_tail = ev.PACK_TAIL;
		}
	}

	Serial_SendByte(pack_head, USART_ID);
	Serial_SendByte(pack_head, USART_ID);
	for(i = 0; i < length; i++)
	{
		Serial_SendByte(String[i], USART_ID);		
	}
	Serial_SendByte(pack_tail, USART_ID);
	Serial_SendByte(pack_tail, USART_ID);

}

// 暂时关闭这两个函数
// void Serial_Printf_Pack(USART_TypeDef *USART_ID, char *format, ...)			//catch format string //
// {
// 	char String[100];
// 	va_list arg;			//define a parameter list variable,,,//va_list:type name //arg:variable name
// 	va_start(arg, format);			//receive param list from position of format,and put into arg
// 	vsprintf(String,format,arg);
// 	va_end(arg);
// 	Serial_SendByte(PACK_HEAD, USART_ID);
// 	Serial_SendByte(PACK_HEAD, USART_ID);
// 	Serial_SendString(String, USART_ID);
// 	Serial_SendByte(PACK_TAIL, USART_ID);
// 	Serial_SendByte(PACK_TAIL, USART_ID);
// }

// void Serial_Printf_Pack_Patch(uint16_t length, USART_TypeDef *USART_ID, char *format, ...)			//catch format string //
// {
// 	char String[100];
// 	va_list arg;			//define a parameter list variable,,,//va_list:type name //arg:variable name
// 	va_start(arg, format);			//receive param list from position of format,and put into arg
// 	vsprintf(String,format,arg);
// 	va_end(arg);
// 	Serial_SendByte(PACK_HEAD, USART_ID);
// 	Serial_SendByte(PACK_HEAD, USART_ID);
// 	Serial_SendString_Patch(String, length, USART_ID);
// 	Serial_SendByte(PACK_TAIL, USART_ID);
// 	Serial_SendByte(PACK_TAIL, USART_ID);
// }


/*
== State Mechanism for Variable Packet Transmission ==
RxState:
	0 -> first Head
	1 -> second Head
	(reserved)
	8 -> Data / first Tail
	9 -> second Tail
	(reserved)
*/

//void USART1_IRQHandler(void)		//Transfer data in order to save multibyte data?
//{
//	static uint8_t RxState = 0;					//static init perform only once
//	static uint8_t HeadID = 0;
//	uint8_t RxData;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
//	{
//		RxData = USART_ReceiveData(USART1);
//		Serial_RxData = RxData;
//		Serial_SendByte(RxData, 1);
////		Serial_RxAck = 1;

//		switch (RxState)
//		{
//			case 0:
//				if (Serial_RxFlag == 0)
//				{
//					if (RxData == PACK_HEAD)
//					{
//						RxState	= 1;
//					}
//					else
//						RxState = 0;
//				}
//				break;
//				
//			case 1:
//				if (RxData == PACK_HEAD_2)
//				{
//					RxState = 8;
//					pRxPacket = 0;				
//				}
//				else
//				{
//					HeadID = 0;
//					RxState = 0;
//				}
//				break;
//								
//			case 8:
//				if (RxData == PACK_TAIL)
//				{
//					RxState = 9;
//				}
//				else	// Data process
//				{
//					Serial_RxPacket[pRxPacket] = RxData;
//					pRxPacket ++;
//				}
//				break;
//				
//			case 9:
//				if (RxData == PACK_TAIL_2)	// ??????
//				{
//					RxState = 0;
//					Serial_RxFlag = 1;
//					Serial_RxPacket[pRxPacket] = '\0';
//				}
//				else 	// save PACK_TAIL and data
//				{
//					RxState = 8;
//					Serial_RxPacket[pRxPacket] = PACK_TAIL;
//					pRxPacket ++;
//					Serial_RxPacket[pRxPacket] = RxData;
//					pRxPacket ++;
//				}
//				break;
//		}
//		
//		// Buffer-based receive
////		Serial_RxPacket[pRxPacket++] = RxData;
////		
////		if (pRxPacket > 64)
////			pRxPacket = 0;

//		
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//	}
//}


/*
GLOBAL: Serial_RxFlag Serial_RxFlag, Serial_RxAck, pRxPacket, RxState
*/
/*
== State Mechanism for Variable Packet Transmission ==
RxState:
	0 -> first Head
	1 -> second Head
	(reserved)
	8 -> Data / first Tail
	9 -> second Tail
	(reserved)
*/
