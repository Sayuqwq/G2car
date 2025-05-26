#ifndef __TRACK_H__
#define __TRACK_H__

#include "alias.h"
#include "abstractInterfaceFunction.h"

/*      TRACK
    负责管理灰度传感器
    抽象灰度传感器状态为循迹状态呈递给其他模块
    管理循迹状态
*/

typedef enum
{
    TRPID_E_CORRECTION_TENDENCY = 0,    // 动作 "趋势型修正"
    TRPID_E_CORRECTION_TORQUE,          // 动作 "扭力型修正"
    TRPID_E_ROTATE,                     // 动作 "原地自转"
    TRPID_E_STRAIGHT,                   // 动作 "直线运动"

    TRPID_E_NULL,       // 无动作，标记，不应该被赋值给PID_action

    TRPID_E_TOTAL

} TRACK_enum_TRPID_ACTION;  // Track对PID的动作指导

typedef enum
{
    TRACK_E_COMMANDTYPE_RESERVED,

// 触发指令     1 ~ 16
    TRACK_E_COMMANDTYPE_Y_LEFT,
    TRACK_E_COMMANDTYPE_Y_RIGHT,
    TRACK_E_COMMANDTYPE_L_LEFT,
    TRACK_E_COMMANDTYPE_L_RIGHT,
    TRACK_E_COMMANDTYPE_STOP,

    TRACK_E_COMMANDTYPE_DIRECT_HEAD = 0x11,

// 非触发指令   18 ~ ...        决定用cmdType识别
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_DEVILEVEL,         // 修改Auto循迹的偏向等级乘数
    TRACK_E_COMMANDTYPE_DIRECT_TARGETLEVEL,                 // 修改循迹的基础目标速度
    TRACK_E_COMMANDTYPE_DIRECT_STOP,                        // 无条件停下，延时/永久，区别于指令循迹有条件的STOP指令
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_ONLY,                   // 仅Auto循迹一段时间，延时/永久，屏蔽指令循迹一段时间
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_ACTION,


    TRACK_E_COMMANDTYPE_TOTAL,

    TRACK_E_COMMANDTYPE_CUSTOMIZED = 255            // 非现有的封装指令
    
} TRACK_enum_TRACK_COMMAND_TYPE;


// 静态链表实现的堆内存
#define nullptr 0x00000000
typedef struct 
{
    uint8_t data;           // 传感器呈递的状态，高1位用来标注是否为<非状态>
    uint8_t mask;           // 掩码
    uint16_t maintain;      // 此状态触发所需的持续轮次
    uint16_t delay;         // 此状态触发后需要的延时轮次

    uint8_t next;

} TRACK_struct_StaticLinkedList_CmdUnit;

typedef struct 
{
    TRACK_struct_StaticLinkedList_CmdUnit *triggerStateList;          // 也作为特殊命令(auto系数，停止命令)的识别区，用高3位识别
    TRACK_struct_StaticLinkedList_CmdUnit *midStateList;
    TRACK_struct_StaticLinkedList_CmdUnit *endStateList;
        // 指令触发和结束的状态，用链表(可变长容器)表示，在此期间动作和偏向等级都一致，必须经由触发和结束状态后，此指令才结束
    TRACK_enum_TRPID_ACTION action;
    int16_t levelOrData;                            // 指令参数接口：偏向等级(负左正右) / 延时 / 数据
    uint8_t isLevelOrDataOpposite;
    #define TRACK_DEFINE_DELAY_FOREVER_VAL 255      // 永恒延时
    #define TRACK_DEFINE_CMD_SKIP 32767             // 仍然需要触发，但跳过执行<触发指令>的动作，可在其后跟随<非触发指令>自定义动作

    TRACK_enum_TRACK_COMMAND_TYPE cmdType;      // 补丁，使得COMMAND可以记录自己的<封装指令类型>，以便发回上位机

} TRACK_struct_TRPID_COMMAND;


// 五位二进制抽象宏，文字靠左是_低位_
#define TRACK_BINARY_5BITS(x1, x2, x3, x4, x5)  \
    ((x1) * 1               \
    + (x2) * 2              \
    + (x3) * 4              \
    + (x4) * 8              \
    + (x5) * 16)

#define   TRACK_BINARY_5BITS_MIRROR(x1, x2, x3, x4, x5)  \
    ((x1) * 16               \
    + (x2) * 8              \
    + (x3) * 4              \
    + (x4) * 2              \
    + (x5) * 1)

void TRACK_Init(void);
void TRACK_GetPIDDeviationLevel(void);     // 事件循环任务函数
void TRACK_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);
void TRACK_Reset(void);         // 循迹模块重置，包含rewind
void TRACK_ResetStates(void);

void TRACK_AppendCreatedCmd(TRACK_enum_TRPID_ACTION action, int16_t levelOrData
                    , TRACK_struct_StaticLinkedList_CmdUnit *triggerList
                    , TRACK_struct_StaticLinkedList_CmdUnit *midList
                    , TRACK_struct_StaticLinkedList_CmdUnit *endList);                // 尾部添加新指令
void TRACK_AppendExistingCmd(TRACK_enum_TRACK_COMMAND_TYPE cmdID, int16_t levelOrData);     // 尾部添加已有指令
void TRACK_ChopCmd(void);                   // 尾部移除指令
void TRACK_EraseCmd(void);                  // 抹除指令
void TRACK_RewindCmd(void);                 // 倒回指令表开始，准备重新执行指令

// 静态链表操作
void TRACK_StaticLinkedList_CmdUnit_Init(TRACK_struct_StaticLinkedList_CmdUnit *link, uint8_t size);
TRACK_struct_StaticLinkedList_CmdUnit *TRACK_StaticLinkedList_CmdUnit_Malloc(TRACK_struct_StaticLinkedList_CmdUnit *link);
uint8_t TRACK_StaticLinkedList_CmdUnit_Free(TRACK_struct_StaticLinkedList_CmdUnit *link, TRACK_struct_StaticLinkedList_CmdUnit *free);

// 链表操作
TRACK_struct_StaticLinkedList_CmdUnit *TRACK_LinkedList_CmdUnit_New(TRACK_struct_StaticLinkedList_CmdUnit *list, uint8_t data, uint16_t mask, uint16_t maintain, uint16_t delay);
void TRACK_LinkedList_CmdUnit_Destroy(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit **head);
uint8_t TRACK_LinkedList_CmdUnit_Append(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit *head, uint8_t data, uint16_t mask, uint16_t maintain, uint16_t delay);
uint8_t TRACK_LinkedList_CmdUnit_Chop(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit *head);


typedef enum
{
	TRACK_E_UNITPROTOCOL_RESERVED = 0,

	TRACK_E_UNITPROTOCAL_ENABLE,
	TRACK_E_UNITPROTOCAL_CMD,

} TRACK_enum_UNITPROTOCOL_SUBMODESELECT;

typedef enum 
{
    TRACK_E_COMMAND_RESERVED = 0,

    TRACK_E_COMMAND_ERASE,
    TRACK_E_COMMAND_APPEND,
    TRACK_E_COMMAND_REWIND,
    TRACK_E_COMMAND_FEEDBACK,
    
    TRACK_E_COMMAND_TOTAL
    
} TRACK_enum_COMMAND_OPERATION;


typedef struct 
{
// 基本状态
    uint8_t sensorBits;             // 低地址是OUT1(左端)
    uint8_t lastSensorBits;
    uint16_t maintainRounds;        // 状态持续轮次

    // 延时轮次
    uint16_t delayRounds;
    uint16_t cmdDelayRounds;

// 指令
    // 指令条带链表内存池
    #define TRACK_DEFINE_CMDTYPETOSTATEPOOL_SIZE 64             // 静态链表内存池大小   已扩容 48 -> 64
    TRACK_struct_StaticLinkedList_CmdUnit 
        state_sllPool[TRACK_DEFINE_CMDTYPETOSTATEPOOL_SIZE];       // 指令对应诸多状态的静态链表内存池

    #define TRACK_DEFINE_CMDSCOUNT 40       // 最大指令条目数
    #define TRACK_DEFINE_REPORT_PORT    ESP_USART
    #define TRACK_DEFINE_CMD_PERFORMING_END_CODE 0xFF
    TRACK_struct_TRPID_COMMAND cmds[TRACK_DEFINE_CMDSCOUNT];
    TRACK_struct_TRPID_COMMAND encapsulatedCmds[TRACK_E_COMMANDTYPE_TOTAL];

    // 指令执行状态
    uint8_t pCmds;
    uint8_t cmdsTotal;
    uint8_t stateListMonitering;

// 非指令，控制参数
    float autoCoefficient;
    TRACK_enum_TRPID_ACTION autoAction;
    #define TRACK_DEFINE_DEFAULT_AUTO_CO 1
    #define TRACK_DEFINE_FILTERCOUNT_MAX 3         // 滤波轮次

// 输出
    int8_t PID_deviationLevel;      // 偏向等级(程度), 正数是指导PID向右偏
    TRACK_enum_TRPID_ACTION PID_action;
    uint16_t basicTargetLevel;      // 循迹基础目标速度/等级，用于决定pid端循迹的基础目标速度
    #define TRACK_DEFINE_DEFAULT_BASIC_TARGET_LEVEL 20

// 功能使能
    uint8_t isEnabled;      // 这个是用于pid中使能循迹运动的
    uint8_t isFeedback;     // 是否反馈指令执行状态给上位机，仅在循迹使能时作用，与摄像头互斥

} TRACKVARS;


#endif


// typedef enum 
// {
//     // 一般偏差
//     TRS_DEVIATION_NO,
//     TRS_DEVIATION_LEFT_LITTLE,
//     TRS_DEVIATION_LEFT_BIG,
//     TRS_DEVIATION_RIGHT_LITTLE,
//     TRS_DEVIATION_RIGHT_BIG,

//     // 岔路路径选择
//         // 1 -> 2 Y形
//     TRS_JUNCTION_Y_TURN_LEFT,       // W B ? B W -> 左转
//     TRS_JUNCTION_Y_TURN_RIGHT,      // W B ? B W -> 右转
//         // 多分支路口
//     TRS_JUNCTION_MEET,              // 正遭遇岔路选择
//     TRS_JUNCTION_MULTIPLE_SELECT_0,         // 选择直走
//     TRS_JUNCTION_MULTIPLE_SELECT_L1,        // 选择左1道路
//     // TRS_JUNCTION_MULTIPLE_SELECT_L2,
//     // TRS_JUNCTION_MULTIPLE_SELECT_L3,
//     // TRS_JUNCTION_MULTIPLE_SELECT_L4,
//     TRS_JUNCTION_MULTIPLE_SELECT_R1,        // 选择右1道路
//     // TRS_JUNCTION_MULTIPLE_SELECT_R2,
//     // TRS_JUNCTION_MULTIPLE_SELECT_R3,
//     // TRS_JUNCTION_MULTIPLE_SELECT_R4,

//     // 特殊标识
//     TRS_SIGN_STOP,                  // B B B B B -> 停止
//         // 其他功能标识，不改变小车行为
//     TRS_SIGN_BUZZ,                  // B W B W W  or  W W B W B


//     // 动作 "趋势型修正": 其中一边轮子速度减弱
//     TRPID_E_CORRECTION_TENDENCY__GO_STRAIGHT,
//     TRPID_E_CORRECTION_TENDENCY__LITTLE_LEFT,
//     TRPID_E_CORRECTION_TENDENCY__BIG_LEFT,
//     TRPID_E_CORRECTION_TENDENCY__LITTLE_RIGHT,
//     TRPID_E_CORRECTION_TENDENCY__BIG_RIGHT,

//     // 动作 "扭力型修正": 两边轮子一加一减
//     TRPID_E_


//     TRS_TOTALCOUNT

// } TRACK_enum_TRPID_DIRECTION;



