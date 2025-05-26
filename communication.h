#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "stm32f10x.h"
#include "alias.h"

#include <string.h>

typedef enum 
{
    CMNCT_MODE_LOOPBACK = 0,    // �ػ�ģʽ

    // ģʽѡ�񣬿����л�����
    CMNCT_MODE_PID,         // "PID"���ƣ�����ʹ��pid���ƣ��ر���ʹ��pwm����
    CMNCT_MODE_TRACK,       // "ѭ������"��"PID"��򿪣���"�˶�ң��"��ͻ���ر���"�˶�ң��"�Զ���
    CMNCT_MODE_REMOTE,      // "�˶�ң��"����"ѭ������"��ͻ
    CMNCT_MODE_ESP,         // "ESP_01S"����
    CMNCT_MODE_OV7670,      // "OV7670"����
    CMNCT_MODE_OLEDPAGE,        // "OLED"���ã���main������


    CMNCT_TCP_POLLING = 0x20,  // "TCP��ѯ"��ֻ���ղ�����ά��TCP����


    CMNCT_MODE_TOTAL

} CMNCT_enum_UNITPORTOCOL_MODESELECT;       // ��λ����Э��_ģʽѡ��

typedef enum
{
    CMNCT_E_ESPAT_RECE_RESERVED = 0,

// ����
    CMNCT_E_ESPAT_RECE_OK = 1,
    CMNCT_E_ESPAT_RECE_ERROR,

// WIFI
    CMNCT_E_ESPAT_RECE_WIFICONNECTED = 8,
    CMNCT_E_ESPAT_RECE_WIFIDISCONNECTED,
    CMNCT_E_ESPAT_RECE_WIFIGOTIP,

// �����TCP/UDP
    CMNCT_E_ESPAT_RECE_CONNECT = 16,             // ���ӳɹ�
    CMNCT_E_ESPAT_RECE_CLOSED,                   // ����ʧ�ܻ�Ͽ�

// �����,����
    // ����
    CMNCT_E_ESPAT_RECE_IPD = 24,
    // ����
    CMNCT_E_ESPAT_RECE_SENDNOW,
    CMNCT_E_ESPAT_RECE_BUSYSEND,
    CMNCT_E_ESPAT_RECE_RECVXXBYTES,
    CMNCT_E_ESPAT_RECE_SENDOK,


    CMNCT_E_ESPAT_RECE_TOTAL

} CMNCT_enum_ESPAT_RECECOMMAND;      
// ESP-AT����ָ�� ö��
////

typedef enum 
{
    CMNCT_E_ESPAT_SEND_RESERVED = 0,
    CMNCT_E_ESPAT_SEND_NO = 1,

// ����,�ֶ�����

// WIFI,�ֶ�����

// ͸��/AT �л�
    CMNCT_E_ESPAT_SEND_TOTRANSPARENT = 16,
    CMNCT_E_ESPAT_SEND_TOAT,


    CMNCT_E_ESPAT_SEND_TOTAL

} CMNCT_enum_ESPAT_SENDCOMMAND;      
// ESP-AT����ָ��/״̬��ȷ��ESP-AT����ʲô����״̬�������·ŵĽ���ָ����н���
////

typedef enum
{
	CMNCT_ESP_E_UNITPROTOCOL_RESERVED = 0,

	CMNCT_ESP_E_UNITPROTOCAL_ATMODE

} CMNCT_ESP_enum_UNITPROTOCOL_SUBMODESELECT;


void CMNCT_Init(void);
int8_t CMNCT_Parameters_Modify(uint8_t *message, uint8_t msgLen);
void CMNCT_TypeConversion_MemoryCopyAndForwardPatched(void *dest, uint8_t **ptrToSrcBytePtr, size_t size);      // ȡ����+��ַ���� ���ܺ���
uint8_t CMNCT_UnitProtocolResolve(uint8_t *message, uint8_t msgLen);     // ����ģʽ
void CMNCT_timerRunning(void);

// ESP-AT Related
void CMNCT_ESPATCommandProcess(uint8_t *message, uint8_t msgLen);   // ESP-AT�м����
uint8_t CMNCT_ESP_ATSEND_toTransparent(uint8_t mode);
void CMNCT_ESP_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);


typedef struct 
{
    const char *ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_TOTAL];     // ESP-ATָ����ձ�

    CMNCT_enum_ESPAT_RECECOMMAND ESPAT_receiveCmd;
    CMNCT_enum_ESPAT_SENDCOMMAND ESPAT_sendCmd;

    uint8_t ESPAT_toTransparentState;
    // uint8_t ESPAT_toAtState;
    

    uint8_t ESP_Mode;      // 0: ATģʽ��  1:͸��ģʽ

    uint16_t timerCounter;
    uint8_t timerEnabled;

} CMNCTVARS;

#endif
