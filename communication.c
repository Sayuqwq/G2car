#include "communication.h"
// #include <string.h>
#include "stdlib.h"

#include "Serial.h"
#include "motor.h"
// 需参与配置的
#include "main.h"
#include "pid.h"
#include "Track.h"
#include "remote.h"
#include "OV7670.h"

/*      消息通信总控 CMNCT
    建立于硬件通信层上的抽象层
    基于消息流，处理COMM和ESP的分包后抽象为统一接口的消息，再进行相应操作
    独立一体，将从main.c逐步迁移至此
*/

/*  friend visit
*/
extern MAINVARS mv;
extern PIDVARS pv;
extern TRACKVARS trv;
extern REMOTEVARS rmv;
////

CMNCTVARS cmnv = 
{
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_OK] = "OK",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_ERROR] = "ERROR",

	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFICONNECTED] = "WIFI CONNECTED",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFIDISCONNECTED] = "WIFI DISCONNECT",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFIGOTIP] = "WIFI GOT IP",

	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_CONNECT] = "CONNECT",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_CLOSED] = "CLOSED",

	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD] = "+IPD,",				// 复合
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_SENDNOW] = ">",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_BUSYSEND] = "busy s...",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_RECVXXBYTES] = "Recv ",		// 复合
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_SENDOK] = "SEND OK"
};


void CMNCTVARS_Constructor(void)
{
	cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_NO;
    
}

void CMNCT_ESPATCommandProcess(uint8_t *message, uint8_t msgLen);
CMNCT_enum_ESPAT_SENDCOMMAND CMNCT_ESPATCommandSendProcess(CMNCT_enum_ESPAT_RECECOMMAND receCmd);
CMNCT_enum_ESPAT_RECECOMMAND CMNCT_ESPATCommandReceProcess(uint8_t *message, uint8_t msgLen);

void CMNCT_timerSetEnabled(_Bool state);
uint16_t CMNCT_timerCheck(void);



/*  内部函数声明
*/


void CMNCT_Init(void)
{
    CMNCTVARS_Constructor();
}


/*      包规则:
    字节1：     功能选择
    字节2~N：   要设置的值，归各模块自己管
*/
// 指定参数设置/功能使用/遥控，使用4号包头
uint8_t CMNCT_UnitProtocolResolve(uint8_t *message, uint8_t msgLen)     // 返回模式
{
	uint8_t *pointer = message;
    uint8_t uint8Num;

    CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));

    // 返回
    // Serial_SendString_Pack_uint8_t(message, msgLen, 4, COMM_USART);

    // 模式解析
    switch (uint8Num)   // 字节1：Mode
    {
    case (uint8_t)CMNCT_MODE_PID:
        PID_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
        break;
        
    case (uint8_t)CMNCT_MODE_TRACK:
        TRACK_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
        break;
        
    case (uint8_t)CMNCT_MODE_REMOTE:
        REMOTE_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
        break;

	case (uint8_t)CMNCT_MODE_ESP:
		CMNCT_ESP_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
		break;

	case (uint8_t)CMNCT_MODE_OV7670:
		OV7670_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
		break;

	case (uint8_t)CMNCT_MODE_OLEDPAGE:
		MAIN_OLEDPAGE_UNITPROTOCOL_CONFIG(pointer, msgLen - 1);
		break;


    case (uint8_t)CMNCT_TCP_POLLING:
        break;
    }

    return uint8Num;
}

// 批量参数设置，使用2号包头
int8_t CMNCT_Parameters_Modify(uint8_t *message, uint8_t msgLen)
{
	uint8_t *ptr_uint8 = (uint8_t *)message;
	int8_t *ptr_int8 = (uint8_t *)message;

	// patch
	int8_t int8Num;
	int16_t int16Num;	// 自动对齐地址patch
	int32_t int32Num;
	float floatNum;

	int i = 0;			// 数据个数
	int length = 0;		// 数据长度
	uint8_t isRefresh = 0;		// 是否重置pid状态		
	uint8_t isReturn = 0;		// 是否返回接收到的数据？1:裸发，2:包发

	// main 和 一般配置
		ptr_int8 = (int8_t *)ptr_int8;
	mv.Page = *(ptr_int8++);	i++;	length += 1;
	isRefresh = *(ptr_int8++);	i++;	length += 1;
	isReturn = *(ptr_int8++);	i++;	length += 1;
	pv.isEnabled = *(ptr_int8++);	i++;	length += 1;

	ptr_uint8 = ptr_int8;

	// 内存对齐错误，又卡死硬件错误，真狗
	// 内存迁移至自动对齐地址

	if (pv.isEnabled == 0)		// 不开启pid，则直接设置pwm真值
	{
		// 设置duty真值
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(1, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(2, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(3, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(4, int32Num);
	}
	else		// 开启pid，则设置目标速度和pid参数
	{
		if (pv.isDebugEnabled == 1)		// 是否开启pidDebug
		{
			// 设置targetSpeed
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int16Num, &ptr_uint8, sizeof(int16Num));	i++;	length += sizeof(int16Num);
			pv.velocityPID_targetSpeed[0] = int16Num;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int16Num, &ptr_uint8, sizeof(int16Num));	i++;	length += sizeof(int16Num);
			pv.velocityPID_targetSpeed[1] = int16Num;

			// 设置 速度环pid 参数:		Kp, Ki, Kd, LIMIT
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Kp = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Ki = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Kd = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
			pv.velocityPID_sumLimit = int32Num;
		}
		
		// 设置 循迹环pid 参数
	}
	
	// pid_ConfigVars
	// 	ptr_float = (float *)ptr_int8;	
	// pv.velocityPID_Kp = *(ptr_float++);	i++;	length += 4;
	// pv.velocityPID_Ki = *(ptr_float++);	i++;	length += 4;
	// pv.velocityPID_Kd = *(ptr_float++);	i++;	length += 4;
	
	// 	ptr_int32 = (int32_t *)ptr_float;
	// pv.LIMIT_constant = *(ptr_int32++);	i++;	length += 4;
	// pv.LIMIT_linear = *(ptr_int32++);	i++;	length += 4;
	
	// 	ptr_int16 = (int16_t *)ptr_int32;
	// pv.target1 = *(ptr_int16++);	i++;	length += 2;
	// pv.target2 = *(ptr_int16++);	i++;	length += 2;
	
	if (isRefresh == 1)
		PID_StatusVars_Reset();
	
	if (isReturn == 1)
		Serial_SendString_Patch((char*)message, length, COMM_USART);
	else if (isReturn == 2)
		Serial_SendString_Pack_uint8_t(message, length, 2, COMM_USART);

	
	if (i > 32)	// Length Limit
		i = 32;

	return i;
}
/*
// G2小车调参
mv.Page = //b[1]
isRefresh = //b[1]
isReturn = //b[1]
pidEnabled = //b[1]

targetSpeed1 = //h[10]
targetSpeed1 = //h[10]

velocity_Kp = //f[0.5]
velocity_Ki = //f[0.025]
velocity_Kd = //f[1]

velocity_sumLimit = //l[1000]
*/

/*
// 实验室手抓驱动板-pid调参
velocity_Kp = 	//f[150],
velocity_Ki = 		//f[0.3],
velocity_Kd = 	//f[200],
LIMIT_constant = 	//i[5000], LIMIT = const + target * linear
LIMIT_linear = //i[64],
target1 = 	//h[1400],
target2 = 	//h[800],

Page = //B[5]
Refresh? = //B[1], 重置所有pid状态
Return? = //B[2], 0:不返回, 1:返回解包数据, 2:返回原始数据
*/

void CMNCT_ESPATCommandProcess(uint8_t *message, uint8_t msgLen)
{
	if (cmnv.ESP_Mode == 0)		// AT模式
	{
		CMNCT_ESPATCommandReceProcess(message, msgLen);
		if (CMNCT_ESPATCommandSendProcess(cmnv.ESPAT_receiveCmd) != CMNCT_E_ESPAT_SEND_NO)
			cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_NO;
	}
	else						// 透传模式
	{
		// 调用单位协议
		CMNCT_UnitProtocolResolve(message, msgLen);
	}
}

CMNCT_enum_ESPAT_SENDCOMMAND CMNCT_ESPATCommandSendProcess(CMNCT_enum_ESPAT_RECECOMMAND receCmd)
{
	switch (cmnv.ESPAT_sendCmd)
	{
	case CMNCT_E_ESPAT_SEND_NO:
		break;

	case CMNCT_E_ESPAT_SEND_TOTRANSPARENT:          // 流程已简化，从而修复不进入AT模式的bug
		switch (cmnv.ESPAT_toTransparentState)
		{
		case 0:
			Serial_SendString("AT+CIPMODE=1\r\n", ESP_USART);
			cmnv.ESPAT_toTransparentState++;
			break;
		case 1:
			if (receCmd == CMNCT_E_ESPAT_RECE_OK)
			{
				Serial_SendString("AT+CIPSEND\r\n", ESP_USART);
				cmnv.ESPAT_toTransparentState++;
			}
			else
				cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_NO;
			break;
		case 2:
			if (receCmd == CMNCT_E_ESPAT_RECE_SENDNOW)		// AT发送命令执行成功
			{
				cmnv.ESPAT_toTransparentState = 255;
				cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_NO;
				cmnv.ESP_Mode = 1;
				return cmnv.ESPAT_sendCmd;
			}
			break;
		}
		break;
	}
	
	return CMNCT_E_ESPAT_SEND_NO;
}

// AT接收解析，解析结果传给AT发送处理
CMNCT_enum_ESPAT_RECECOMMAND CMNCT_ESPATCommandReceProcess(uint8_t *message, uint8_t msgLen)
{	// 解析指令分包后的来自esp的AT指令信息，属于抽象层中间解析，解析完毕后，使用通用协议处理信息
	CMNCT_enum_ESPAT_RECECOMMAND cmd;

	// 复合指令解析
	if (strncmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD], strlen(cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD])) == 0)
	{	// +IPD
	
		cmd = CMNCT_E_ESPAT_RECE_IPD;

		uint8_t numLen = 0;
		uint8_t carriedMsgLen = 0;

		char *prefixRemovedMsg = (char *)message + strlen(cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD]);
		char *pointer = prefixRemovedMsg;
		while (numLen + strlen(cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD]) < msgLen)
		{
			if (*pointer == ':')
				break;
			numLen++;		// 数字域长度++
			pointer++;		// 指针后移
		}
		pointer++;	// 指针移至数据域

		carriedMsgLen = atoi(prefixRemovedMsg);

		// 调用通用接口方法
		Serial_SendString_Patch(pointer, carriedMsgLen, COMM_USART);
	}
	
	// 单一指令解析
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_OK]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_OK;
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_ERROR]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_ERROR;

	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFICONNECTED]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_WIFICONNECTED;
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFIDISCONNECTED]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_WIFIDISCONNECTED;
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_WIFIGOTIP]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_WIFIGOTIP;

	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_CONNECT]) == 0)
	{	// 只要建立了传输层连接，则发起“切换为透明传输”请求
		cmd = CMNCT_E_ESPAT_RECE_CONNECT;
		CMNCT_ESP_ATSEND_toTransparent(1);
	}
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_CLOSED]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_CLOSED;

	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_SENDNOW]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_SENDNOW;
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_BUSYSEND]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_BUSYSEND;
	else if (strcmp((char *)message, cmnv.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_SENDOK]) == 0)
		cmd = CMNCT_E_ESPAT_RECE_SENDOK;

	cmnv.ESPAT_receiveCmd = cmd;		// 同时设置全局变量

	return cmd;
}

// 将需要对齐的信息拷贝进安全(自动对齐)地址
void CMNCT_TypeConversion_MemoryCopyAndForwardPatched(void *dest, uint8_t **ptrToSrcBytePtr, size_t size)
{
	memcpy(dest, *ptrToSrcBytePtr, size);
	*ptrToSrcBytePtr += size;
}

void CMNCT_timerSetEnabled(_Bool state)
{
	cmnv.timerCounter = 0;
	cmnv.timerEnabled = state;
}

uint16_t CMNCT_timerCheck(void)
{
	return cmnv.timerCounter;
}

void CMNCT_timerRunning(void)
{
	if (cmnv.timerEnabled == 1)
		cmnv.timerCounter++;
}



// 非阻塞调用AT发送指令的函数(能改变模式)，阻塞的使用AT的便利函数在main.c
uint8_t CMNCT_ESP_ATSEND_toTransparent(uint8_t mode)	// 0:成功，1:失败
{
	if (cmnv.ESPAT_sendCmd != CMNCT_E_ESPAT_SEND_NO)	// 正在执行某条AT发送命令
	{
		return 1;
	}

	if (cmnv.ESP_Mode == mode)
	{
		return 0;
	}

	if (mode == 0)		// AT
	{
		Serial_SendString("+++", ESP_USART);	// 解锁AT
		cmnv.ESP_Mode = mode;
		return 0;
	}
	else				// 透传
	{
		// 改mode交由事件循环
		cmnv.ESPAT_toTransparentState = 0;		// 重置状态，255是成功，0是开始或停止
		cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_TOTRANSPARENT;
		return 0;
	}
}

// UNITPROTOCOL配置ESP接口，通过UNITPROTOCOL配置能同时更改一些配置变量，仅开放部分配置，主要配置通过包头协议3号包头直接配置
void CMNCT_ESP_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t uint8Num, first;
	uint8_t *pointer = data;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(uint8Num));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
	switch (first)
	{
	case (uint8_t)CMNCT_ESP_E_UNITPROTOCAL_ATMODE:
		CMNCT_ESP_ATSEND_toTransparent(uint8Num);
	}
}


