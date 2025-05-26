#ifndef __ENCODER_H__
#define __ENCODER_H__

void ENC_Init(void);		// TIM_1 通道123编码器测速，A8, A9, A10
void ENC_GetSpeed(void);
void ENC_ResetSpeed(void);
int ENC_ReadOnlySpeed(uint8_t motor);

typedef struct 
{
    int16_t speedM1;
    int16_t speedM2;

    int32_t pulsesM1;
    int32_t pulsesM2;


} ENCODERVARS;

#endif
