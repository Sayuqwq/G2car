#include "communication.h"
// #include <string.h>
#include "stdlib.h"

#include "Serial.h"
#include "motor.h"
// ��������õ�
#include "main.h"
#include "pid.h"
#include "Track.h"
#include "remote.h"
#include "OV7670.h"

/*      ��Ϣͨ���ܿ� CMNCT
    ������Ӳ��ͨ�Ų��ϵĳ����
    ������Ϣ��������COMM��ESP�ķְ������Ϊͳһ�ӿڵ���Ϣ���ٽ�����Ӧ����
    ����һ�壬����main.c��Ǩ������
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

	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_IPD] = "+IPD,",				// ����
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_SENDNOW] = ">",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_BUSYSEND] = "busy s...",
	.ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_RECVXXBYTES] = "Recv ",		// ����
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



/*  �ڲ���������
*/


void CMNCT_Init(void)
{
    CMNCTVARS_Constructor();
}


/*      ������:
    �ֽ�1��     ����ѡ��
    �ֽ�2~N��   Ҫ���õ�ֵ�����ģ���Լ���
*/
// ָ����������/����ʹ��/ң�أ�ʹ��4�Ű�ͷ
uint8_t CMNCT_UnitProtocolResolve(uint8_t *message, uint8_t msgLen)     // ����ģʽ
{
	uint8_t *pointer = message;
    uint8_t uint8Num;

    CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));

    // ����
    // Serial_SendString_Pack_uint8_t(message, msgLen, 4, COMM_USART);

    // ģʽ����
    switch (uint8Num)   // �ֽ�1��Mode
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

// �����������ã�ʹ��2�Ű�ͷ
int8_t CMNCT_Parameters_Modify(uint8_t *message, uint8_t msgLen)
{
	uint8_t *ptr_uint8 = (uint8_t *)message;
	int8_t *ptr_int8 = (uint8_t *)message;

	// patch
	int8_t int8Num;
	int16_t int16Num;	// �Զ������ַpatch
	int32_t int32Num;
	float floatNum;

	int i = 0;			// ���ݸ���
	int length = 0;		// ���ݳ���
	uint8_t isRefresh = 0;		// �Ƿ�����pid״̬		
	uint8_t isReturn = 0;		// �Ƿ񷵻ؽ��յ������ݣ�1:�㷢��2:����

	// main �� һ������
		ptr_int8 = (int8_t *)ptr_int8;
	mv.Page = *(ptr_int8++);	i++;	length += 1;
	isRefresh = *(ptr_int8++);	i++;	length += 1;
	isReturn = *(ptr_int8++);	i++;	length += 1;
	pv.isEnabled = *(ptr_int8++);	i++;	length += 1;

	ptr_uint8 = ptr_int8;

	// �ڴ��������ֿ���Ӳ�������湷
	// �ڴ�Ǩ�����Զ������ַ

	if (pv.isEnabled == 0)		// ������pid����ֱ������pwm��ֵ
	{
		// ����duty��ֵ
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(1, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(2, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(3, int32Num);
		CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
		MOTOR_SetDuty_actual(4, int32Num);
	}
	else		// ����pid��������Ŀ���ٶȺ�pid����
	{
		if (pv.isDebugEnabled == 1)		// �Ƿ���pidDebug
		{
			// ����targetSpeed
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int16Num, &ptr_uint8, sizeof(int16Num));	i++;	length += sizeof(int16Num);
			pv.velocityPID_targetSpeed[0] = int16Num;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int16Num, &ptr_uint8, sizeof(int16Num));	i++;	length += sizeof(int16Num);
			pv.velocityPID_targetSpeed[1] = int16Num;

			// ���� �ٶȻ�pid ����:		Kp, Ki, Kd, LIMIT
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Kp = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Ki = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&floatNum, &ptr_uint8, sizeof(floatNum));	i++;	length += sizeof(floatNum);
			pv.velocityPID_Kd = floatNum;
			CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int32Num, &ptr_uint8, sizeof(int32Num));	i++;	length += sizeof(int32Num);
			pv.velocityPID_sumLimit = int32Num;
		}
		
		// ���� ѭ����pid ����
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
// G2С������
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
// ʵ������ץ������-pid����
velocity_Kp = 	//f[150],
velocity_Ki = 		//f[0.3],
velocity_Kd = 	//f[200],
LIMIT_constant = 	//i[5000], LIMIT = const + target * linear
LIMIT_linear = //i[64],
target1 = 	//h[1400],
target2 = 	//h[800],

Page = //B[5]
Refresh? = //B[1], ��������pid״̬
Return? = //B[2], 0:������, 1:���ؽ������, 2:����ԭʼ����
*/

void CMNCT_ESPATCommandProcess(uint8_t *message, uint8_t msgLen)
{
	if (cmnv.ESP_Mode == 0)		// ATģʽ
	{
		CMNCT_ESPATCommandReceProcess(message, msgLen);
		if (CMNCT_ESPATCommandSendProcess(cmnv.ESPAT_receiveCmd) != CMNCT_E_ESPAT_SEND_NO)
			cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_NO;
	}
	else						// ͸��ģʽ
	{
		// ���õ�λЭ��
		CMNCT_UnitProtocolResolve(message, msgLen);
	}
}

CMNCT_enum_ESPAT_SENDCOMMAND CMNCT_ESPATCommandSendProcess(CMNCT_enum_ESPAT_RECECOMMAND receCmd)
{
	switch (cmnv.ESPAT_sendCmd)
	{
	case CMNCT_E_ESPAT_SEND_NO:
		break;

	case CMNCT_E_ESPAT_SEND_TOTRANSPARENT:          // �����Ѽ򻯣��Ӷ��޸�������ATģʽ��bug
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
			if (receCmd == CMNCT_E_ESPAT_RECE_SENDNOW)		// AT��������ִ�гɹ�
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

// AT���ս����������������AT���ʹ���
CMNCT_enum_ESPAT_RECECOMMAND CMNCT_ESPATCommandReceProcess(uint8_t *message, uint8_t msgLen)
{	// ����ָ��ְ��������esp��ATָ����Ϣ�����ڳ�����м������������Ϻ�ʹ��ͨ��Э�鴦����Ϣ
	CMNCT_enum_ESPAT_RECECOMMAND cmd;

	// ����ָ�����
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
			numLen++;		// �����򳤶�++
			pointer++;		// ָ�����
		}
		pointer++;	// ָ������������

		carriedMsgLen = atoi(prefixRemovedMsg);

		// ����ͨ�ýӿڷ���
		Serial_SendString_Patch(pointer, carriedMsgLen, COMM_USART);
	}
	
	// ��һָ�����
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
	{	// ֻҪ�����˴�������ӣ������л�Ϊ͸�����䡱����
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

	cmnv.ESPAT_receiveCmd = cmd;		// ͬʱ����ȫ�ֱ���

	return cmd;
}

// ����Ҫ�������Ϣ��������ȫ(�Զ�����)��ַ
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



// ����������AT����ָ��ĺ���(�ܸı�ģʽ)��������ʹ��AT�ı���������main.c
uint8_t CMNCT_ESP_ATSEND_toTransparent(uint8_t mode)	// 0:�ɹ���1:ʧ��
{
	if (cmnv.ESPAT_sendCmd != CMNCT_E_ESPAT_SEND_NO)	// ����ִ��ĳ��AT��������
	{
		return 1;
	}

	if (cmnv.ESP_Mode == mode)
	{
		return 0;
	}

	if (mode == 0)		// AT
	{
		Serial_SendString("+++", ESP_USART);	// ����AT
		cmnv.ESP_Mode = mode;
		return 0;
	}
	else				// ͸��
	{
		// ��mode�����¼�ѭ��
		cmnv.ESPAT_toTransparentState = 0;		// ����״̬��255�ǳɹ���0�ǿ�ʼ��ֹͣ
		cmnv.ESPAT_sendCmd = CMNCT_E_ESPAT_SEND_TOTRANSPARENT;
		return 0;
	}
}

// UNITPROTOCOL����ESP�ӿڣ�ͨ��UNITPROTOCOL������ͬʱ����һЩ���ñ����������Ų������ã���Ҫ����ͨ����ͷЭ��3�Ű�ͷֱ������
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


