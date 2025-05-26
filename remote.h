#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stm32f10x.h"
#include "alias.h"

void REMOTE_Init(void);
void REMOTE_Processor(void);
void REMOTE_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);

typedef enum
{
	REMOTE_E_UNITPROTOCOL_RESERVED = 0,

	REMOTE_E_UNITPROTOCAL_ENABLE,
    REMOTE_E_UNITPROTOCAL_ACTION,
    REMOTE_E_UNITPROTOCOL_PARAMS

} REMOTE_enum_UNITPROTOCOL_SUBMODESELECT;

typedef enum
{
    REMOTE_E_UP_PARAMS_RESERVED = 0,

    REMOTE_E_UP_PARAMS_FORWARDSPEED = 1,
    REMOTE_E_UP_PARAMS_BACKWARDSPEED,
    REMOTE_E_UP_PARAMS_ROTATESPEED,
    REMOTE_E_UP_PARAMS_TENDENCYSPEED

} REMOTE_enum_UNITPROTOCOL_PARAMS_ID;

typedef struct 
{
// 状态变量
    uint8_t fbBits;     // 前后位                    2
    uint8_t lrBits;     // 左右位                 1  0  2
    uint8_t mergedBits;     // 合并位(接收)          1

    int8_t forwardSpeed;
    int8_t backwardSpeed;
    int8_t rotateSpeed;
    int8_t tendencySpeed;

    uint8_t isEnabled;

} REMOTEVARS;

#endif
