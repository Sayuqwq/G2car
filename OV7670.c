#include "OV7670.h"
#include "SCCB.h"
#include "OLED.h"
#include "Delay.h"
#include "Serial.h"
// #include "ESP.h"
#include "ESP_01S.h"
#include "communication.h"

OVVARS ovv = 
{
	.OV_WIDTH = 320,
	.OV_HEIGHT = 240
};

void OVVARS_Costructor(void)
{
	ovv.OV7670_state = OV7670_E_STATE_IDLE;
	ovv.mode = OV7670_E_MODE_OFF;
	ovv.lastMode = ovv.mode;
}

void OV7670_Configure(void);



void OV7670_Serial_SendByte(uint8_t Byte)
{
	Serial_SendByte(Byte, OV_SEND_USART);
}


/* @brief ���漸��������Ҫ�����ڱ���޸Ĵ��ڵ�ƽ */
uint8_t OV7670_VS(void)	
{
	return GPIO_ReadInputDataBit(OV_VSYNC_GPIOX, OV_VSYNC);
}
void OV7670_W_RRST(uint8_t BitValue)	//
{
	GPIO_WriteBit(OV_RRST_GPIOX, OV_RRST, (BitAction)BitValue);
	delay_us(10);
}void OV7670_W_WRST(uint8_t BitValue)	//
{
	GPIO_WriteBit(OV_WRST_GPIOX, OV_WRST, (BitAction)BitValue);
	delay_us(10);
}
void OV7670_W_WEN(uint8_t BitValue)		//
{
	GPIO_WriteBit(OV_WR_GPIOX, OV_WR, (BitAction)BitValue);
	delay_us(10);
}
void OV7670_W_OE(uint8_t BitValue)	//
{
	GPIO_WriteBit(OV_OE_GPIOX, OV_OE, (BitAction)BitValue);
	delay_us(10);
}
void OV7670_W_RCLK(uint8_t BitValue)	//
{
	GPIO_WriteBit(OV_RCK_GPIOX, OV_RCK, (BitAction)BitValue);
	delay_us(10);
}
/*
  * @brief	OV7670���ų�ʼ����������OV7670_Init() �ﴥ���������ⲿ����
  * @param  ��
  * @retval ��
*/
void OV7670_Pin_Init()
{	
	/* WRST WEN OE RRST �� RCK�� IO�ڳ�ʼ�� */
	ABS_RCC_GPIO_ClockCmd(OV_WRST_GPIOX, ENABLE);        /* GPIOEʹ�� */	
	GPIO_InitTypeDef GPIO_InitStruct;                   /* �ṹ�嶨�� */
    GPIO_InitStruct.GPIO_Pin = OV_RCK | OV_WR | OV_OE | OV_WRST | OV_RRST;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;                   /* ������� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                   /*  50MHz*/
    GPIO_Init(OV_WRST_GPIOX, &GPIO_InitStruct);
	
	GPIO_SetBits(OV_WRST_GPIOX, OV_WRST | OV_OE | OV_RRST | OV_RCK);/* ��WRST RCK OE �� RRST��ʼ��ֵ */
	GPIO_ResetBits(OV_WRST_GPIOX, OV_WR);/* �� WEN ����ֵ */

	
	/* VS IO�ڳ�ʼ�� */
	ABS_RCC_GPIO_ClockCmd(OV_VSYNC_GPIOX, ENABLE);        /* VSYNC_GPIOXʹ�� */	
    GPIO_InitStruct.GPIO_Pin = OV_VSYNC;                   /* PF11 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;                   /* �������� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                   /*  50MHz*/
    GPIO_Init(OV_VSYNC_GPIOX, &GPIO_InitStruct); 

	/* D0-D7 IO�ڳ�ʼ�� */
	ABS_RCC_GPIO_ClockCmd(OV_D0_GPIOX, ENABLE);        /* GPIOʹ�� */	
    GPIO_InitStruct.GPIO_Pin = OV_D0 | OV_D1 | OV_D2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                   /* �������� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                   /*  50MHz*/
    GPIO_Init(OV_D0_GPIOX, &GPIO_InitStruct); 

	ABS_RCC_GPIO_ClockCmd(OV_D3_GPIOX, ENABLE);        /* GPIOʹ�� */
    GPIO_InitStruct.GPIO_Pin = OV_D3 | OV_D4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                   /* �������� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                   /*  50MHz*/
    GPIO_Init(OV_D3_GPIOX, &GPIO_InitStruct); 

	ABS_RCC_GPIO_ClockCmd(OV_D5_GPIOX, ENABLE);        /* GPIOʹ�� */
    GPIO_InitStruct.GPIO_Pin = OV_D5 | OV_D6 | OV_D7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                   /* �������� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                   /*  50MHz*/
    GPIO_Init(OV_D5_GPIOX, &GPIO_InitStruct); 
}

void OV7670_VSYNC_EXTI_Init(void)
{
	// ����VSYNC�ⲿ�ж�
//RCC
	ABS_RCC_GPIO_ClockCmd(OV_VSYNC_GPIOX, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTI, ENABLE);		EXTI,NVIC is Opened by default

//GPIO
	GPIO_InitTypeDef GPIO_InitStructrue;
	GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructrue.GPIO_Pin = OV_VSYNC;
	GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OV_VSYNC_GPIOX, &GPIO_InitStructrue);
	
//AFIO
	GPIO_EXTILineConfig(OV_VSYNC_AFIOEXTI_PORTSOURCE, OV_VSYNC_AFIOEXTI_PINSOURCE);
	
//EXTI
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = OV_VSYNC_EXTI_LINEX;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = OV_VSYNC_EXTI_TRIIGGER;
	EXTI_Init(&EXTI_InitStructure);
	
//NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = OV_VSYNC_EXTI_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = OV_VSYNC_NVIC_PRIORITY_PREEMPTION;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = OV_VSYNC_NVIC_PRIORITY_SUB;
	NVIC_Init(&NVIC_InitStructure);
}

void OV7670_Init(void)
{
	OVVARS_Costructor();

	SCCB_Init();				// SCCB��ʼ��
	OV7670_Pin_Init();			// ���ų�ʼ��
	OV7670_VSYNC_EXTI_Init();	// VSYNC�ⲿ�жϳ�ʼ��

	OV7670_Configure();			// �Ĵ���Ԥ��
}
/*
  * @brief	OV7670д�Ĵ���
  * @param  д��Ĵ����ĵ�ַ
  * @param  д������ݣ�һ���ֽ����ݣ�
  * @retval ��
*/
void OV7670_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	SCCB_Start();
	
	SCCB_SendByte(OV7670_ADDRESS);
	SCCB_ReceiveAck();			//SCCB_ReceiveAck()==0˵����һ��ִ�гɹ������������Ҳ��
	
	SCCB_SendByte(RegAddress);
	SCCB_ReceiveAck();
	
	SCCB_SendByte(Data);
	SCCB_ReceiveAck();
	
	SCCB_Stop();
}
/*
  * @brief	OV7670��ȡ�Ĵ���
  * @param  ��ȡ�Ĵ����ĵ�ַ
  * @retval ��Ӧ��ַ�ļĴ���������
*/
uint8_t OV7670_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	SCCB_Start();
	SCCB_SendByte(OV7670_ADDRESS);
	SCCB_ReceiveAck();			//SCCB_ReceiveAck()==0˵����һ��ִ�гɹ������������Ҳ��
	SCCB_SendByte(RegAddress);
	SCCB_ReceiveAck();
	SCCB_Stop();		//�����STOP�ܱ�Ҫ����������SCCB��ͬ��I2C�ĵط�������
	
	SCCB_Start();
	SCCB_SendByte(OV7670_ADDRESS | 0x01);
	SCCB_ReceiveAck();
	Data = SCCB_ReceiveByte();
	SCCB_SendNA();
	SCCB_Stop();
	
	return Data;
}
/* 
  * @brief ���������̵̳ļĴ�������
*/
void OV7670_RegExample(void)
{
    OV7670_WriteReg(0x3a, 0x04);
    OV7670_WriteReg(0x40, 0xd0);
    OV7670_WriteReg(0x12, 0x14);

    OV7670_WriteReg(0x32, 0x80);
    OV7670_WriteReg(0x17, 0x16);
    OV7670_WriteReg(0x18, 0x04);
    OV7670_WriteReg(0x19, 0x02);
    OV7670_WriteReg(0x1a, 0x7b);
    OV7670_WriteReg(0x03, 0x06);

    OV7670_WriteReg(0x0c, 0x00);
    OV7670_WriteReg(0x15, 0x00);
    OV7670_WriteReg(0x3e, 0x00);
    OV7670_WriteReg(0x70, 0x3a);
    OV7670_WriteReg(0x71, 0x35);
    OV7670_WriteReg(0x72, 0x11);
    OV7670_WriteReg(0x73, 0x00);

    OV7670_WriteReg(0xa2, 0x02);
    OV7670_WriteReg(0x11, 0x81);
    OV7670_WriteReg(0x7a, 0x20);
    OV7670_WriteReg(0x7b, 0x1c);
    OV7670_WriteReg(0x7c, 0x28);

    OV7670_WriteReg(0x7d, 0x3c);
    OV7670_WriteReg(0x7e, 0x55);
    OV7670_WriteReg(0x7f, 0x68);
    OV7670_WriteReg(0x80, 0x76);
    OV7670_WriteReg(0x81, 0x80);

    OV7670_WriteReg(0x82, 0x88);
    OV7670_WriteReg(0x83, 0x8f);
    OV7670_WriteReg(0x84, 0x96);
    OV7670_WriteReg(0x85, 0xa3);
    OV7670_WriteReg(0x86, 0xaf);

    OV7670_WriteReg(0x87, 0xc4);
    OV7670_WriteReg(0x88, 0xd7);
    OV7670_WriteReg(0x89, 0xe8);
    OV7670_WriteReg(0x13, 0xe0);
    OV7670_WriteReg(0x00, 0x00);

    OV7670_WriteReg(0x10, 0x00);
    OV7670_WriteReg(0x0d, 0x00);
    OV7670_WriteReg(0x14, 0x28);
    OV7670_WriteReg(0xa5, 0x05);
    OV7670_WriteReg(0xab, 0x07);

    OV7670_WriteReg(0x24, 0x75);
    OV7670_WriteReg(0x25, 0x63);
    OV7670_WriteReg(0x26, 0xA5);
    OV7670_WriteReg(0x9f, 0x78);
    OV7670_WriteReg(0xa0, 0x68);

    OV7670_WriteReg(0xa1, 0x03);
    OV7670_WriteReg(0xa6, 0xdf);
    OV7670_WriteReg(0xa7, 0xdf);
    OV7670_WriteReg(0xa8, 0xf0);
    OV7670_WriteReg(0xa9, 0x90);

    OV7670_WriteReg(0xaa, 0x94);
    OV7670_WriteReg(0x13, 0xe5);
    OV7670_WriteReg(0x0e, 0x61);
    OV7670_WriteReg(0x0f, 0x4b);
    OV7670_WriteReg(0x16, 0x02);

    OV7670_WriteReg(0x1e, 0x37);
    OV7670_WriteReg(0x21, 0x02);
    OV7670_WriteReg(0x22, 0x91);
    OV7670_WriteReg(0x29, 0x07);
    OV7670_WriteReg(0x33, 0x0b);

    OV7670_WriteReg(0x35, 0x0b);
    OV7670_WriteReg(0x37, 0x1d);
    OV7670_WriteReg(0x38, 0x71);
    OV7670_WriteReg(0x39, 0x2a);
    OV7670_WriteReg(0x3c, 0x78);

    OV7670_WriteReg(0x4d, 0x40);
    OV7670_WriteReg(0x4e, 0x20);
    OV7670_WriteReg(0x69, 0x00);
    OV7670_WriteReg(0x6b, 0x40);
    OV7670_WriteReg(0x74, 0x19);
    OV7670_WriteReg(0x8d, 0x4f);

    OV7670_WriteReg(0x8e, 0x00);
    OV7670_WriteReg(0x8f, 0x00);
    OV7670_WriteReg(0x90, 0x00);
    OV7670_WriteReg(0x91, 0x00);
    OV7670_WriteReg(0x92, 0x00);

    OV7670_WriteReg(0x96, 0x00);
    OV7670_WriteReg(0x9a, 0x80);
    OV7670_WriteReg(0xb0, 0x84);
    OV7670_WriteReg(0xb1, 0x0c);
    OV7670_WriteReg(0xb2, 0x0e);

    OV7670_WriteReg(0xb3, 0x82);
    OV7670_WriteReg(0xb8, 0x0a);
    OV7670_WriteReg(0x43, 0x14);
    OV7670_WriteReg(0x44, 0xf0);
    OV7670_WriteReg(0x45, 0x34);

    OV7670_WriteReg(0x46, 0x58);
    OV7670_WriteReg(0x47, 0x28);
    OV7670_WriteReg(0x48, 0x3a);
    OV7670_WriteReg(0x59, 0x88);
    OV7670_WriteReg(0x5a, 0x88);

    OV7670_WriteReg(0x5b, 0x44);
    OV7670_WriteReg(0x5c, 0x67);
    OV7670_WriteReg(0x5d, 0x49);
    OV7670_WriteReg(0x5e, 0x0e);
    OV7670_WriteReg(0x64, 0x04);
    OV7670_WriteReg(0x65, 0x20);

    OV7670_WriteReg(0x66, 0x05);
    OV7670_WriteReg(0x94, 0x04);
    OV7670_WriteReg(0x95, 0x08);
    OV7670_WriteReg(0x6c, 0x0a);
    OV7670_WriteReg(0x6d, 0x55);


    OV7670_WriteReg(0x4f, 0x80);
    OV7670_WriteReg(0x50, 0x80);
    OV7670_WriteReg(0x51, 0x00);
    OV7670_WriteReg(0x52, 0x22);
    OV7670_WriteReg(0x53, 0x5e);
    OV7670_WriteReg(0x54, 0x80);

    OV7670_WriteReg(0x09, 0x03);

    OV7670_WriteReg(0x6e, 0x11);
    OV7670_WriteReg(0x6f, 0x9f);
    OV7670_WriteReg(0x55, 0x00);
    OV7670_WriteReg(0x56, 0x40);
    OV7670_WriteReg(0x57, 0x40);
    OV7670_WriteReg(0x6a, 0x40);
    OV7670_WriteReg(0x01, 0x40);
    OV7670_WriteReg(0x02, 0x40);
    OV7670_WriteReg(0x13, 0xe7);
    OV7670_WriteReg(0x15, 0x00);  
    
        
    OV7670_WriteReg(0x58, 0x9e);
    
    OV7670_WriteReg(0x41, 0x08);
    OV7670_WriteReg(0x3f, 0x00);
    OV7670_WriteReg(0x75, 0x05);
    OV7670_WriteReg(0x76, 0xe1);
    OV7670_WriteReg(0x4c, 0x00);
    OV7670_WriteReg(0x77, 0x01);
    OV7670_WriteReg(0x3d, 0xc2);    
    OV7670_WriteReg(0x4b, 0x09);
    OV7670_WriteReg(0xc9, 0x60);
    OV7670_WriteReg(0x41, 0x38);
    
    OV7670_WriteReg(0x34, 0x11);
    OV7670_WriteReg(0x3b, 0x02); 

    OV7670_WriteReg(0xa4, 0x89);
    OV7670_WriteReg(0x96, 0x00);
    OV7670_WriteReg(0x97, 0x30);
    OV7670_WriteReg(0x98, 0x20);
    OV7670_WriteReg(0x99, 0x30);
    OV7670_WriteReg(0x9a, 0x84);
    OV7670_WriteReg(0x9b, 0x29);
    OV7670_WriteReg(0x9c, 0x03);
    OV7670_WriteReg(0x9d, 0x4c);
    OV7670_WriteReg(0x9e, 0x3f);
    OV7670_WriteReg(0x78, 0x04);
    
    OV7670_WriteReg(0x79, 0x01);
    OV7670_WriteReg(0xc8, 0xf0);
    OV7670_WriteReg(0x79, 0x0f);
    OV7670_WriteReg(0xc8, 0x00);
    OV7670_WriteReg(0x79, 0x10);
    OV7670_WriteReg(0xc8, 0x7e);
    OV7670_WriteReg(0x79, 0x0a);
    OV7670_WriteReg(0xc8, 0x80);
    OV7670_WriteReg(0x79, 0x0b);
    OV7670_WriteReg(0xc8, 0x01);
    OV7670_WriteReg(0x79, 0x0c);
    OV7670_WriteReg(0xc8, 0x0f);
    OV7670_WriteReg(0x79, 0x0d);
    OV7670_WriteReg(0xc8, 0x20);
    OV7670_WriteReg(0x79, 0x09);
    OV7670_WriteReg(0xc8, 0x80);
    OV7670_WriteReg(0x79, 0x02);
    OV7670_WriteReg(0xc8, 0xc0);
    OV7670_WriteReg(0x79, 0x03);
    OV7670_WriteReg(0xc8, 0x40);
    OV7670_WriteReg(0x79, 0x05);
    OV7670_WriteReg(0xc8, 0x30);
    OV7670_WriteReg(0x79, 0x26); 
    OV7670_WriteReg(0x09, 0x00);
//  //Frame Rate Adjustment for 24Mhz input clock
//  //30fps PCLK=24MHz
//  {0x11, 0x80},//���Ӧ���ֲ������õ���0x80���������õ���0x00
//  {0x6b, 0x0a},//PLL����,���Ӧ���ֲ������õ���0x0a,�������õ���0x40,��PLL���ߵĻ��ͻ��������
//  {0x2a, 0x00},
//  {0x2b, 0x00},
//  {0x92, 0x00},
//  {0x93, 0x00},
//  {0x3b, 0x0a},

//  //Output format
//  {0x12, 0x14},//QVGA(ovv.OV_WIDTH * ovv.OV_HEIGHT)��RGB

//  //RGB555/565 option(must set COM7[2] = 1 and COM7[0] = 0)
//  {0x40, 0x10},//RGB565,effective only when RGB444[1] is low
//  {0x8c, 0x00},

//  //Special effects - ��Ч 
//  //normal
//  {0x3a, 0x04},
//  {0x67, 0xc0},
//  {0x68, 0x80},

//  //Mirror/VFlip Enable - ˮƽ����/��ֱ��תʹ��
//  {0x1e, 0x37},//�޸�����ֵ������ͼ����ʾ���»����ҵߵ�
//  
////ע����Щ���õĻ�������б��ʾ������ʾ��飬�⵽���ǿ���ʲô�ģ���ʱ��ͼ�йأ�
//  {0x17, 0x16},//��ƵHorizontal Frame��ʼ�߰�λ(����λ��HREF[2��0])       
//  {0x18, 0x04},//��ƵHorizontal Frame�����߰�λ(����λ��HREF[5��3])
//  {0x19, 0x02},//��ƵVertical Frame��ʼ�߰�λ(�Ͷ�λ��VREF[1��0])
//  {0x1a, 0x7b},//��ƵVertical Frame�����߰�λ(�Ͷ�λ��VREF[3��2])
//  {0x32, 0x80},//HREF
//  {0x03, 0x06},//VREF

//  //ע��������õĻ�������ʾ������
//  {0x15, 0x02},//����PCLK��HREF��VSYNC���
}
/*
  * @brief	�Ĵ�����ʼ��
*/
void OV7670_Configure(void)
{
	OV7670_WriteReg(0x12,0x80);//�Ĵ�����λ,���мĴ�����λΪ��ʼĬ��ֵ
	delay_ms(1000);
	OLED_Clear();
	
	OV7670_RegExample();/* ���������̵̳ļĴ������ã�����Ĵ�����Ӧ�Ĺ�����δŪ�� */
	
	/*���ò���ͼ�����  �������õ��������ɫ����*/
	// OV7670_WriteReg(0x70, 0x3A);
	// OV7670_WriteReg(0x71, 0xB5);
	
	
	/*Ӧ���������ֲ����������ֲ���д���ǣ�
	(0x70[7],0x71[7])=(1,0)������ǰ�ɫ����		��
	(0x70[7],0x71[7])=(0,1)�������Shifting ��1��	��
	��ʵ�����Ƿ�������
	(0x70[7],0x71[7])=(1,0)�������Shifting ��1��	?
	(0x70[7],0x71[7])=(0,1)������ǰ�ɫ����		?
	*/
}
/*
  * @brief	FIFO��ȡͼ����Ϣ
  * @param  ��
  * @retval ��
*/
void OV7670_FIFOGetPic(void)
{
	switch (ovv.OV7670_state)
	{	
	case OV7670_E_STATE_REQUESTING_NEW_FRAME:
		OV7670_W_WRST(0);	//дָ�븴λ
		OV7670_W_WRST(1);
		OV7670_W_WEN(1);	//дʹ��
		ovv.OV7670_state = OV7670_E_STATE_FIFO_BEING_WRITTEN;		//��һ�ν����ж�OV7670_FIFOGetPic()����else
		break;

	case OV7670_E_STATE_FIFO_BEING_WRITTEN:
		// OV7670_W_WRST(0);   //дָ�븴λ
		// OV7670_W_WRST(1);
		OV7670_W_WEN(0);	//дʧ��
		ovv.OV7670_state = OV7670_E_STATE_FRAME_READY_TO_READ;
		break;
		
	case OV7670_E_STATE_FRAME_READY_TO_READ:
		OV7670_W_WEN(0);	//дʧ��
		break;
		
	case OV7670_E_STATE_IDLE:
		OV7670_W_WEN(0);	//дʧ��
		switch (ovv.mode)
		{
		case OV7670_E_MODE_OFF:
			break;

		case OV7670_E_MODE_SINGLESHOT:
			ovv.OV7670_state = OV7670_E_STATE_REQUESTING_NEW_FRAME;
			OV7670_SetMode((uint8_t)OV7670_E_MODE_OFF);		// �����ص�OFFģʽ
			break;

		case OV7670_E_MODE_LIVE:
			ovv.OV7670_state = OV7670_E_STATE_REQUESTING_NEW_FRAME;
			break;
		}
		break;
	}}
/*
  * @brief	STM32��OV7670��ȡͼ����Ϣ
  * @param  ��
  * @retval ��
*/
void OV7670_STM32GetPic(void)
{
	if (ovv.OV7670_state != OV7670_E_STATE_FRAME_READY_TO_READ)
		return;

	uint16_t row, column;
	uint16_t DMACounter = 0;
	uint8_t byte = 0x00;

	uint8_t temp;

    // ����ԭ�Ӷ�ָ�븴λ����, �������ȷ��
	OV7670_W_RRST(0);//��ָ�븴λ
    OV7670_W_RCLK(0);
    OV7670_W_RCLK(1);
    OV7670_W_RCLK(0);
	OV7670_W_RRST(1);//��λ��ָ�ԭ״̬
    OV7670_W_RCLK(1);

	OV7670_W_OE(0);//���ʹ��


	ESP_DMASetDirectSendEnabled(1);		// ��DMAֱ�ӷ���ģʽ
	
	ovv.progressHeight = 0;

	delay_ms(500);

	for (row = 0; row < ovv.OV_HEIGHT; row++)		// �߶��߷�
	{
		for (column = 0; column < ovv.OV_WIDTH * 2; column++)
		{
			byte = 0x00;

			OV7670_W_RCLK(0);

			byte |= GPIO_ReadInputDataBit(OV_D0_GPIOX, OV_D0);
			byte |= GPIO_ReadInputDataBit(OV_D1_GPIOX, OV_D1) << 1;
			byte |= GPIO_ReadInputDataBit(OV_D2_GPIOX, OV_D2) << 2;
			byte |= GPIO_ReadInputDataBit(OV_D3_GPIOX, OV_D3) << 3;
			byte |= GPIO_ReadInputDataBit(OV_D4_GPIOX, OV_D4) << 4;
			byte |= GPIO_ReadInputDataBit(OV_D5_GPIOX, OV_D5) << 5;
			byte |= GPIO_ReadInputDataBit(OV_D6_GPIOX, OV_D6) << 6;
			byte |= GPIO_ReadInputDataBit(OV_D7_GPIOX, OV_D7) << 7;

			OV7670_W_RCLK(1);

			while (ESP_WriteDMASendQueue(&byte, DMACounter, 1) != 0);
			DMACounter++;
			if (DMACounter == ESP_TXQUEUE_SIZE)	// ����DMAֱ�ӷ���
			{
				while (Serial_DMADirectSend(ESP_TXQUEUE_SIZE, ESP_USART) != 0);
				DMACounter = 0;
                delay_ms(1);
			}

            // OV7670_Serial_SendByte(byte);
		}

		ovv.progressHeight++;
	}

	while (Serial_DMADirectSend(DMACounter, ESP_USART) != 0);	// ����ٵ��÷���


	ESP_DMASetDirectSendEnabled(0);		// �ر�DMAֱ�ӷ���ģʽ

	
	OV7670_W_OE(1);     //���ʧ��

	switch (ovv.mode)
	{
	case OV7670_E_MODE_OFF:
		ovv.OV7670_state = OV7670_E_STATE_IDLE;
		break;

	case OV7670_E_MODE_SINGLESHOT:			// �����ܽ���, SINGLESHOTģʽ���뼴ת��OFF
		ovv.OV7670_state = OV7670_E_STATE_IDLE;
		break;

	case OV7670_E_MODE_LIVE:
		ovv.OV7670_state = OV7670_E_STATE_REQUESTING_NEW_FRAME;
		break;
	}
}

// UNIT PROTOCOL
void OV7670_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t first, second;
	uint8_t *pointer = data;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(first));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&second, &pointer, sizeof(second));
	switch (first)
	{
	case (uint8_t)OV7670_E_UNITPROTOCAL_MODE:
		OV7670_SetMode(second);
		break;
	case (uint8_t)OV7670_E_UNITPROTOCAL_WRITEREG:
		break;
	}
}


// �����װ����, ��������ʽ����main.cѭ����
void OV7670_Work(void)
{
	OV7670_STM32GetPic();
}

uint8_t OV7670_GetProgress10(void)
{
	return ovv.progressHeight * 10 / ovv.OV_HEIGHT;
}

void OV7670_SetMode(uint8_t mode)
{
	if (mode >= OV7670_E_MODE_TOTAL)
		return;

	ovv.lastMode = ovv.mode;
	ovv.mode = (OV7670_enum_MODE)mode;
}


void OV_VSYNC_EXTI_IRQHandler(void)		//Interrupt Function
{
	if(EXTI_GetITStatus(OV_VSYNC_EXTI_LINEX) == SET)
	{
			ovv.interruptIndicator++;
			ovv.interruptIndicator %= 128;
		if(GPIO_ReadInputDataBit(OV_VSYNC_GPIOX, OV_VSYNC) == 1)
		{
			OV7670_FIFOGetPic();		// VSYNC�жϴ���

		}
		EXTI_ClearITPendingBit(OV_VSYNC_EXTI_LINEX);
	}
}



