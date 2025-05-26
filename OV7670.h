#ifndef __OV7670_H
#define __OV7670_H

#include "stm32f10x.h"                  // Device header
#include "alias.h"
#include "abstractInterfaceFunction.h"

void OV7670_Init(void);

void OV7670_STM32GetPic(void);
void OV7670_FIFOGetPic(void);

void OV7670_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t OV7670_ReadReg(uint8_t RegAddress);


void OV7670_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);
void OV7670_Work(void);
uint8_t OV7670_GetProgress10(void);
void OV7670_SetMode(uint8_t mode);

/*  UNITPROTOCOL
*/
typedef enum
{ 
	OV7670_E_UNITPROTOCOL_RESERVED = 0,

	OV7670_E_UNITPROTOCAL_MODE,
    OV7670_E_UNITPROTOCAL_WRITEREG,

} OV7670_enum_UNITPROTOCOL_SUBMODESELECT;

typedef enum 
{
    OV7670_E_MODE_OFF = 0,
    OV7670_E_MODE_SINGLESHOT,
    OV7670_E_MODE_LIVE,

    OV7670_E_MODE_TOTAL

} OV7670_enum_MODE;
////

typedef enum 
{
    OV7670_E_STATE_REQUESTING_NEW_FRAME = 0,        // 请求新帧
    OV7670_E_STATE_FIFO_BEING_WRITTEN,              // 帧正在写入FIFO
    OV7670_E_STATE_FRAME_READY_TO_READ,             // 帧就绪，应将帧读出
    OV7670_E_STATE_IDLE,                            // 摄像头空闲, 不执行读取帧操作

    OV7670_E_STATE_TOTAL

} OV7670_enum_STATE;

typedef struct 
{
    #define OV7670_ADDRESS 0x42         /* 0100 0010 --- 写地址   ***   0100 0011 ---读地址 */

    OV7670_enum_STATE OV7670_state;

    OV7670_enum_MODE mode;
    OV7670_enum_MODE lastMode;
    uint16_t progressHeight;

    const uint16_t OV_WIDTH;
    const uint16_t OV_HEIGHT;

    uint8_t interruptIndicator;
    
} OVVARS;

#endif

