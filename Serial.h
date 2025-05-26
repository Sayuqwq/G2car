#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"


// Send
void Serial_SendByte(uint8_t Byte, USART_TypeDef *USART_ID);
void Serial_SendArray(uint8_t *Array, uint16_t Length, USART_TypeDef *USART_ID);
void Serial_SendString(char *String, USART_TypeDef *USART_ID);
void Serial_SendString_Patch(char *String, uint16_t length, USART_TypeDef *USART_ID);

void Serial_SendString_Pack_uint8_t(uint8_t *String, uint16_t length, uint8_t PackID, USART_TypeDef *USART_ID);
void Serial_SendNumber(uint32_t Number, uint8_t Length, USART_TypeDef *USART_ID);
void Serial_Printf(USART_TypeDef *USART_ID, char *format, ...);			//catch format string //
// void Serial_Printf_Pack(USART_TypeDef *USART_ID, char *format, ...);			//catch format string //
// void Serial_Printf_Pack_Patch(uint16_t length, USART_TypeDef *USART_ID, char *format, ...);			//catch format string //

uint8_t Serial_DMACopySend(uint8_t *data, uint16_t len, USART_TypeDef *USART_ID);
uint8_t Serial_DMADirectSend(uint16_t len, USART_TypeDef *USART_ID);

#endif

