#include "SCCB.h"
#include "Delay.h"

/*
 * @brief  修改SCL的电平
 * @param  0或者1
 * @retval 无
*/
void SCCB_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(OV_SIOC_GPIOX, OV_SIOC, (BitAction)BitValue);
	delay_us(10);
}
/*
 * @brief  修改SDA的电平
 * @param  0或者1
 * @retval 无
*/
void SCCB_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(OV_SIOD_GPIOX, OV_SIOD, (BitAction)BitValue);
	delay_us(10);
}
/*
 * @brief  读取SDA的电平
 * @param  无
 * @retval 0或者1
*/
uint8_t SCCB_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(OV_SIOD_GPIOX, OV_SIOD);
	delay_us(10);
	return BitValue;
}
/*
 * @brief  SCCB初始化
 * @param  无
 * @retval 无
*/
void SCCB_Init(void)
{
	ABS_RCC_GPIO_ClockCmd(OV_SIOC_GPIOX, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = OV_SIOC | OV_SIOD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OV_SIOC_GPIOX, &GPIO_InitStructure);
	
	GPIO_SetBits(OV_SIOC_GPIOX, OV_SIOC | OV_SIOD);
}
/*
 * @brief  产生SCCB开始信号
 * @param  无
 * @retval 无
*/
void SCCB_Start(void)
{
	SCCB_W_SDA(1);
	delay_us(10);
	SCCB_W_SCL(1);
	delay_us(2);
	SCCB_W_SDA(0);
	delay_us(2);
	SCCB_W_SCL(0);
}
/*
 * @brief  产生SCCB结束信号
 * @param  无
 * @retval 无
*/
void SCCB_Stop(void)
{
	SCCB_W_SDA(0);
	SCCB_W_SCL(1);
	SCCB_W_SDA(1);
}
/*
 * @brief	SCCB发送一个字节
 * @param  一个字节数据
 * @retval 无
*/
void SCCB_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{	
		delay_us(2);
		SCCB_W_SDA(Byte & (0x80 >> i));
		delay_us(2);
		SCCB_W_SCL(1);
		delay_us(2);
		SCCB_W_SCL(0);
		delay_us(2);
	}
}
/*
 * @brief	SCCB接收一个字节
 * @param  无
 * @retval 接收到的字节
*/
uint8_t SCCB_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	SCCB_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		SCCB_W_SCL(1);
		if (SCCB_R_SDA() == 1){Byte |= (0x80 >> i);}
		SCCB_W_SCL(0);
		delay_us(1);
	}
	return Byte;
}
/*
 * @brief	SCCB发送NA信号
 * @param  无
 * @retval 无
*/
void SCCB_SendNA()
{
	SCCB_W_SDA(1);
	SCCB_W_SCL(1);
	SCCB_W_SCL(0);
	SCCB_W_SDA(0);//new
}
/*
 * @brief	SCCB接收Ack应答
 * @param  无
 * @retval 接收到的应答，若数据成功发送，应答为0，反之为1
*/
uint8_t SCCB_ReceiveAck(void)
{
	uint8_t AckBit;
	SCCB_W_SDA(1);
	SCCB_W_SCL(1);
	AckBit = SCCB_R_SDA();
	SCCB_W_SCL(0);
	return AckBit;
}




// /*
//   * @brief  修改SCL的电平
//   * @param  0或者1
//   * @retval 无
// */
// void SCCB_W_SCL(uint8_t BitValue)
// {
//     GPIO_WriteBit(OV_SIOC_GPIOX, OV_SIOC, (BitAction)BitValue);
//     delay_us(10);
// }
// /*
//   * @brief  修改SDA的电平
//   * @param  0或者1
//   * @retval 无
// */
// void SCCB_W_SDA(uint8_t BitValue)
// {
//     GPIO_WriteBit(OV_SIOD_GPIOX, OV_SIOD, (BitAction)BitValue);
//     delay_us(10);
// }
// /*
//   * @brief  读取SDA的电平
//   * @param  无
//   * @retval 0或者1
// */
// uint8_t SCCB_R_SDA(void)
// {
//     uint8_t BitValue;
//     BitValue = GPIO_ReadInputDataBit(OV_SIOD_GPIOX, OV_SIOD);
//     delay_us(10);
//     return BitValue;
// }
// /*
//   * @brief    设置SDA为输入模式
//   * @param  无
//   * @retval 无
// */
// void SCCB_SDA_IN(void)
// {
//     ABS_RCC_GPIO_ClockCmd(OV_SIOC_GPIOX, ENABLE);
    
//     GPIO_InitTypeDef GPIO_InitStructure;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//     GPIO_InitStructure.GPIO_Pin = OV_SIOD;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//     GPIO_SetBits(GPIOB, OV_SIOD);
// }
// /*
//   * @brief    设置SDA为输出模式
//   * @param  无
//   * @retval 无
// */
// void SCCB_SDA_OUT(void)
// {
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
//     GPIO_InitTypeDef GPIO_InitStructure;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//     GPIO_InitStructure.GPIO_Pin = OV_SIOD;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//     GPIO_SetBits(GPIOB, OV_SIOD);
// }
// /*
//   * @brief  SCCB初始化，初始时设置SCL和SDA都为输出，只有主机在控制SCL，因此设置成推挽输出即可
//             主机（STM32）和从机（OV7670）都有在控制SDA，但是大部分时间都是主机控制SDA，因此设置
//             成推挽输出，需要时再将SDA设置成上拉输入，从机控制完再换回推挽输出模式。
//   * @param  无
//   * @retval 无
// */
// void SCCB_Init(void)
// {
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
//     GPIO_InitTypeDef GPIO_InitStructure;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;/* 推挽输出 */
//     GPIO_InitStructure.GPIO_Pin = OV_SIOC | OV_SIOD;/* SCL和SDA */
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//     GPIO_SetBits(GPIOB, OV_SIOC | OV_SIOD);
// }
// /*
//   * @brief  产生SCCB开始信号
//   * @param  无
//   * @retval 无
// */
// void SCCB_Start(void)
// {
//     SCCB_W_SDA(1);
//     delay_us(10);
//     SCCB_W_SCL(1);
//     delay_us(2);
//     SCCB_W_SDA(0);
//     delay_us(2);
//     SCCB_W_SCL(0);
// }
// /*
//   * @brief  产生SCCB结束信号
//   * @param  无
//   * @retval 无
// */
// void SCCB_Stop(void)
// {
//     SCCB_W_SDA(0);
//     SCCB_W_SCL(1);
//     SCCB_W_SDA(1);
// }
// /*
//   * @brief    SCCB发送一个字节
//   * @param  一个字节数据
//   * @retval 无
// */
// void SCCB_SendByte(uint8_t Byte)
// {
//     uint8_t i;
//     for (i = 0; i < 8; i ++)
//     {    
//         delay_us(2);
//         SCCB_W_SDA(Byte & (0x80 >> i));
//         delay_us(2);
//         SCCB_W_SCL(1);
//         delay_us(2);
//         SCCB_W_SCL(0);
//         delay_us(2);
//     }
// }
// /*
//   * @brief    SCCB接收一个字节
//   * @param  无
//   * @retval 接收到的字节
// */
// uint8_t SCCB_ReceiveByte(void)
// {
    
//     uint8_t i, Byte = 0x00;
//     SCCB_SDA_IN();/* 转换SDA为输入模式 */
//     for (i = 0; i < 8; i ++)
//     {
//         SCCB_W_SCL(1);
//         if (SCCB_R_SDA() == 1){Byte |= (0x80 >> i);}
//         SCCB_W_SCL(0);
//         delay_us(1);
//     }
//     SCCB_SDA_OUT();/* 从机控制完SDA了，便转换SDA为输出模式 */
//     return Byte;
// }
// /*
//   * @brief    SCCB发送NA信号
//   * @param  无
//   * @retval 无
// */
// void SCCB_SendNA()
// {
//     SCCB_W_SDA(1);
//     SCCB_W_SCL(1);
//     SCCB_W_SCL(0);
//     SCCB_W_SDA(0);//new
// }
// /*
//   * @brief    SCCB接收Ack应答
//   * @param  无
//   * @retval 接收到的应答，若数据成功发送，应答为0，反之为1
// */
// uint8_t SCCB_ReceiveAck(void)
// {
//     uint8_t AckBit;
//     SCCB_SDA_IN();/* 转换SDA为输入模式 */
//     SCCB_W_SCL(1);
//     AckBit = SCCB_R_SDA();
//     SCCB_W_SCL(0);
//     SCCB_SDA_OUT();/* 转换SDA为输入模式 */
//     return AckBit;
// }
