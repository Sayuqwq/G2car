#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "stm32f10x.h"
#include "alias.h"

#include <string.h>

typedef enum 
{
    CMNCT_MODE_LOOPBACK = 0,    // 回环模式

    // 模式选择，可能有互斥量
    CMNCT_MODE_PID,         // "PID"控制，打开则使用pid控制，关闭则使用pwm控制
    CMNCT_MODE_TRACK,       // "循迹控制"，"PID"须打开，与"运动遥控"冲突，关闭则"运动遥控"自动打开
    CMNCT_MODE_REMOTE,      // "运动遥控"，与"循迹功能"冲突
    CMNCT_MODE_ESP,         // "ESP_01S"配置
    CMNCT_MODE_OV7670,      // "OV7670"配置
    CMNCT_MODE_OLEDPAGE,        // "OLED"配置，在main中配置


    CMNCT_TCP_POLLING = 0x20,  // "TCP轮询"，只接收不处理，维持TCP连接


    CMNCT_MODE_TOTAL

} CMNCT_enum_UNITPORTOCOL_MODESELECT;       // 单位传输协议_模式选择

typedef enum
{
    CMNCT_E_ESPAT_RECE_RESERVED = 0,

// 基本
    CMNCT_E_ESPAT_RECE_OK = 1,
    CMNCT_E_ESPAT_RECE_ERROR,

// WIFI
    CMNCT_E_ESPAT_RECE_WIFICONNECTED = 8,
    CMNCT_E_ESPAT_RECE_WIFIDISCONNECTED,
    CMNCT_E_ESPAT_RECE_WIFIGOTIP,

// 传输层TCP/UDP
    CMNCT_E_ESPAT_RECE_CONNECT = 16,             // 连接成功
    CMNCT_E_ESPAT_RECE_CLOSED,                   // 连接失败或断开

// 传输层,数据
    // 接收
    CMNCT_E_ESPAT_RECE_IPD = 24,
    // 发送
    CMNCT_E_ESPAT_RECE_SENDNOW,
    CMNCT_E_ESPAT_RECE_BUSYSEND,
    CMNCT_E_ESPAT_RECE_RECVXXBYTES,
    CMNCT_E_ESPAT_RECE_SENDOK,


    CMNCT_E_ESPAT_RECE_TOTAL

} CMNCT_enum_ESPAT_RECECOMMAND;      
// ESP-AT接收指令 枚举
////

typedef enum 
{
    CMNCT_E_ESPAT_SEND_RESERVED = 0,
    CMNCT_E_ESPAT_SEND_NO = 1,

// 基本,手动设置

// WIFI,手动设置

// 透传/AT 切换
    CMNCT_E_ESPAT_SEND_TOTRANSPARENT = 16,
    CMNCT_E_ESPAT_SEND_TOAT,


    CMNCT_E_ESPAT_SEND_TOTAL

} CMNCT_enum_ESPAT_SENDCOMMAND;      
// ESP-AT发送指令/状态，确认ESP-AT处于什么发送状态，根据下放的接收指令进行解析
////

typedef enum
{
	CMNCT_ESP_E_UNITPROTOCOL_RESERVED = 0,

	CMNCT_ESP_E_UNITPROTOCAL_ATMODE

} CMNCT_ESP_enum_UNITPROTOCOL_SUBMODESELECT;


void CMNCT_Init(void);
int8_t CMNCT_Parameters_Modify(uint8_t *message, uint8_t msgLen);
void CMNCT_TypeConversion_MemoryCopyAndForwardPatched(void *dest, uint8_t **ptrToSrcBytePtr, size_t size);      // 取数据+地址递增 功能函数
uint8_t CMNCT_UnitProtocolResolve(uint8_t *message, uint8_t msgLen);     // 返回模式
void CMNCT_timerRunning(void);

// ESP-AT Related
void CMNCT_ESPATCommandProcess(uint8_t *message, uint8_t msgLen);   // ESP-AT中间解析
uint8_t CMNCT_ESP_ATSEND_toTransparent(uint8_t mode);
void CMNCT_ESP_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);


typedef struct 
{
    const char *ESPATCOMMAND_Mapping[CMNCT_E_ESPAT_RECE_TOTAL];     // ESP-AT指令对照表

    CMNCT_enum_ESPAT_RECECOMMAND ESPAT_receiveCmd;
    CMNCT_enum_ESPAT_SENDCOMMAND ESPAT_sendCmd;

    uint8_t ESPAT_toTransparentState;
    // uint8_t ESPAT_toAtState;
    

    uint8_t ESP_Mode;      // 0: AT模式，  1:透传模式

    uint16_t timerCounter;
    uint8_t timerEnabled;

} CMNCTVARS;

#endif
