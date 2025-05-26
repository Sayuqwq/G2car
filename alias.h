// 瀹忓畾涔夊強甯搁噺鍒?鍚?

#pragma once
#include "stm32f10x.h"                  // Device header

/*      EventLoop
*/
#define EVENTLOOP1T                             TIM2
#define EVENTLOOP1_PSC                          100
#define EVENTLOOP1_PER                          7200
#define EVENTLOOP1_NVIC_PRIORITY_PREEMPTION     2
#define EVENTLOOP1_NVIC_PRIORITY_SUB            2
#define EVENTLOOP1_IRQHandler                   TIM2_IRQHandler

// 借用硬pwm中断
#define EVENTLOOP2T                 MPT
#define EVENTLOOP2_NVIC_PRIORITY_PREEMPTION     2
#define EVENTLOOP2_NVIC_PRIORITY_SUB            2
#define EVENTLOOP2_IRQHandler       MPT_IRQHandler


/*		Motor
--MD: Motor_Direction
--MP: Motor_PWM
--ME: Motor_Encoder
*/
// softpwm(direction)
#define MD1     GPIO_Pin_1
#define MD2     GPIO_Pin_2
#define MD3     GPIO_Pin_3
#define MD4     GPIO_Pin_4
#define MDX     GPIOF
#define MDT     TIM6
#define MDT_IRQN    TIM6_IRQn
#define MDT_IRQHandler  TIM6_IRQHandler
#define MDT_SOFT_PSC    2               // 20KHz
#define MDT_SOFT_PER    1800
#define MDT_SOFT_MAX    ((uint32_t)72000000 / MDT_SOFT_PSC / MDT_SOFT_PER / PWM_FREQ - 1)     // 就是 40 - 1 = 39
    // 电机方向修正 reverse motor direction, 1 is forward
static const _Bool MDIR1 = 1;
static const _Bool MDIR2 = 0;
static const _Bool MDIR3 = 1;
static const _Bool MDIR4 = 0;

// pwm
#define MP1     GPIO_Pin_0
#define MP2     GPIO_Pin_1
#define MP3     GPIO_Pin_2
#define MP4     GPIO_Pin_3
#define MPX     GPIOA
#define MPT         TIM5
#define MPT_IRQN        TIM5_IRQn
#define MPT_IRQHandler  TIM5_IRQHandler
#define MPT_PSC     4
#define MPT_PER     36000
#define PWM_FREQ    ((uint32_t)72000000 / MPT_PSC / MPT_PER)      // PWM频率设定：500Hz
// #define MPX_CLOCK   RCC_APB2Periph_GPIOA
// #define MPT_CLOCK   RCC_APB1Periph_TIM5
// // clock_config_func
// #define MPX_RCC_CONFIG_FUNC     RCC_APB2PeriphClockCmd
// #define MPT_RCC_CONFIG_FUNC     RCC_APB1PeriphClockCmd

#define ME_NUMBER 2
// encoder 1
#define ME1A    GPIO_Pin_6
#define ME1B    GPIO_Pin_7
#define ME1X    GPIOB
#define ME1T    TIM4

// encoder 2
#define ME2A    GPIO_Pin_6
#define ME2B    GPIO_Pin_7
#define ME2X    GPIOC
#define ME2T    TIM8

/*      OLED
*/
#define OLED_SCL    GPIO_Pin_2
#define OLED_SDA    GPIO_Pin_3
#define OLED_GPIOX  GPIOG

/*      Serial
*/
#define COMM_TX     GPIO_Pin_9
#define COMM_RX     GPIO_Pin_10
#define COMM_GPIOX  GPIOA
#define COMM_USART  USART1
#define COMM_RX_IRQN        USART1_IRQn
#define COMM_RX_DMAX            DMA1
#define COMM_RX_DMA_CHANNEL     DMA1_Channel5
#define COMM_TX_DMA_CHANNEL     DMA1_Channel4
// clock_config

/*      ESP_01S
*/
#define ESP_TX     GPIO_Pin_10
#define ESP_RX     GPIO_Pin_11
#define ESP_GPIOX  GPIOB
#define ESP_USART  USART3
#define ESP_RX_IRQN            USART3_IRQn
#define ESP_RX_DMAX            DMA1
#define ESP_RX_DMA_CHANNEL     DMA1_Channel3
#define ESP_TX_DMA_CHANNEL     DMA1_Channel2

/*       communication config
	// 鍗忚??锛屽寘澶村熬
const uint8_t PACK_HEAD = 0xFF;	// double same head and tail
const uint8_t PACK_HEAD_2 = 0xFD;
const uint8_t PACK_TAIL = 0xFE;
const uint8_t PACK_TAIL_2 = 0xFE;

*/

/*		PID
*/
#define PID_IRQN    TIM2_IRQn

/*      Track
*/
#define TR_ACTIVATED_COUNT  5
#define TR_ROADID   1
#define TR_FIELDID  0

#define TR1     GPIO_Pin_6
#define TR2     GPIO_Pin_7
#define TR3     GPIO_Pin_9
#define TR4     GPIO_Pin_10
#define TR5     GPIO_Pin_11
// reserved
#define TR6     GPIO_Pin_12
#define TR7     GPIO_Pin_13
#define TR8     GPIO_Pin_14


#define TR1X    GPIOD
#define TR2X    GPIOD

#define TR3X    GPIOG
#define TR4X    GPIOG
#define TR5X    GPIOG
// reserved
#define TR6X    GPIOG
#define TR7X    GPIOG
#define TR8X    GPIOG


/*      OV7670
*/
#define OV_SEND_USART   ESP_USART
#define OV_VSYNC_AFIOEXTI_PORTSOURCE    GPIO_PortSourceGPIOF
#define OV_VSYNC_AFIOEXTI_PINSOURCE     GPIO_PinSource11
#define OV_VSYNC_EXTI_LINEX             EXTI_Line11
#define OV_VSYNC_EXTI_TRIIGGER          EXTI_Trigger_Rising
#define OV_VSYNC_NVIC_PRIORITY_PREEMPTION       1
#define OV_VSYNC_NVIC_PRIORITY_SUB              2
#define OV_VSYNC_EXTI_IRQHandler        EXTI15_10_IRQHandler
#define OV_VSYNC_EXTI_IRQN              EXTI15_10_IRQn

// SCCB
#define OV_SIOC     GPIO_Pin_0
#define OV_SIOD     GPIO_Pin_1

#define OV_SIOC_GPIOX     GPIOB
#define OV_SIOD_GPIOX     GPIOB

// data
#define OV_D7       GPIO_Pin_13
#define OV_D6       GPIO_Pin_14
#define OV_D5       GPIO_Pin_15
#define OV_D4       GPIO_Pin_0
#define OV_D3       GPIO_Pin_1
#define OV_D2       GPIO_Pin_7
#define OV_D1       GPIO_Pin_8
#define OV_D0       GPIO_Pin_9

#define OV_D7_GPIOX       GPIOF
#define OV_D6_GPIOX       GPIOF
#define OV_D5_GPIOX       GPIOF
#define OV_D4_GPIOX       GPIOG
#define OV_D3_GPIOX       GPIOG
#define OV_D2_GPIOX       GPIOE
#define OV_D1_GPIOX       GPIOE
#define OV_D0_GPIOX       GPIOE

// control
#define OV_VSYNC    GPIO_Pin_11
#define OV_RCK      GPIO_Pin_11
#define OV_WR       GPIO_Pin_12
#define OV_OE       GPIO_Pin_13
#define OV_WRST     GPIO_Pin_14
#define OV_RRST     GPIO_Pin_15

#define OV_VSYNC_GPIOX    GPIOF
#define OV_RCK_GPIOX      GPIOE
#define OV_WR_GPIOX       GPIOE
#define OV_OE_GPIOX       GPIOE
#define OV_WRST_GPIOX     GPIOE
#define OV_RRST_GPIOX     GPIOE

/*      Key
*/
#define KEY0    GPIO_Pin_4
#define KEY1    GPIO_Pin_3
#define KEYX    GPIOE


//// Reserved periphrals

/*      Servo
*/
#define SG90_PWM        GPIO_Pin_1
#define SG90_PWM_GPIOX  GPIOD

/*      Sonar
*/
#define HC_TRIG     GPIO_Pin_4
#define HC_ECHO     GPIO_Pin_5
#define HC_GPIOX    GPIOG
