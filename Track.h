#ifndef __TRACK_H__
#define __TRACK_H__

#include "alias.h"
#include "abstractInterfaceFunction.h"

/*      TRACK
    �������Ҷȴ�����
    ����Ҷȴ�����״̬Ϊѭ��״̬�ʵݸ�����ģ��
    ����ѭ��״̬
*/

typedef enum
{
    TRPID_E_CORRECTION_TENDENCY = 0,    // ���� "����������"
    TRPID_E_CORRECTION_TORQUE,          // ���� "Ť��������"
    TRPID_E_ROTATE,                     // ���� "ԭ����ת"
    TRPID_E_STRAIGHT,                   // ���� "ֱ���˶�"

    TRPID_E_NULL,       // �޶�������ǣ���Ӧ�ñ���ֵ��PID_action

    TRPID_E_TOTAL

} TRACK_enum_TRPID_ACTION;  // Track��PID�Ķ���ָ��

typedef enum
{
    TRACK_E_COMMANDTYPE_RESERVED,

// ����ָ��     1 ~ 16
    TRACK_E_COMMANDTYPE_Y_LEFT,
    TRACK_E_COMMANDTYPE_Y_RIGHT,
    TRACK_E_COMMANDTYPE_L_LEFT,
    TRACK_E_COMMANDTYPE_L_RIGHT,
    TRACK_E_COMMANDTYPE_STOP,

    TRACK_E_COMMANDTYPE_DIRECT_HEAD = 0x11,

// �Ǵ���ָ��   18 ~ ...        ������cmdTypeʶ��
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_DEVILEVEL,         // �޸�Autoѭ����ƫ��ȼ�����
    TRACK_E_COMMANDTYPE_DIRECT_TARGETLEVEL,                 // �޸�ѭ���Ļ���Ŀ���ٶ�
    TRACK_E_COMMANDTYPE_DIRECT_STOP,                        // ������ͣ�£���ʱ/���ã�������ָ��ѭ����������STOPָ��
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_ONLY,                   // ��Autoѭ��һ��ʱ�䣬��ʱ/���ã�����ָ��ѭ��һ��ʱ��
    TRACK_E_COMMANDTYPE_DIRECT_AUTO_ACTION,


    TRACK_E_COMMANDTYPE_TOTAL,

    TRACK_E_COMMANDTYPE_CUSTOMIZED = 255            // �����еķ�װָ��
    
} TRACK_enum_TRACK_COMMAND_TYPE;


// ��̬����ʵ�ֵĶ��ڴ�
#define nullptr 0x00000000
typedef struct 
{
    uint8_t data;           // �������ʵݵ�״̬����1λ������ע�Ƿ�Ϊ<��״̬>
    uint8_t mask;           // ����
    uint16_t maintain;      // ��״̬��������ĳ����ִ�
    uint16_t delay;         // ��״̬��������Ҫ����ʱ�ִ�

    uint8_t next;

} TRACK_struct_StaticLinkedList_CmdUnit;

typedef struct 
{
    TRACK_struct_StaticLinkedList_CmdUnit *triggerStateList;          // Ҳ��Ϊ��������(autoϵ����ֹͣ����)��ʶ�������ø�3λʶ��
    TRACK_struct_StaticLinkedList_CmdUnit *midStateList;
    TRACK_struct_StaticLinkedList_CmdUnit *endStateList;
        // ָ����ͽ�����״̬��������(�ɱ䳤����)��ʾ���ڴ��ڼ䶯����ƫ��ȼ���һ�£����뾭�ɴ����ͽ���״̬�󣬴�ָ��Ž���
    TRACK_enum_TRPID_ACTION action;
    int16_t levelOrData;                            // ָ������ӿڣ�ƫ��ȼ�(��������) / ��ʱ / ����
    uint8_t isLevelOrDataOpposite;
    #define TRACK_DEFINE_DELAY_FOREVER_VAL 255      // ������ʱ
    #define TRACK_DEFINE_CMD_SKIP 32767             // ��Ȼ��Ҫ������������ִ��<����ָ��>�Ķ���������������<�Ǵ���ָ��>�Զ��嶯��

    TRACK_enum_TRACK_COMMAND_TYPE cmdType;      // ������ʹ��COMMAND���Լ�¼�Լ���<��װָ������>���Ա㷢����λ��

} TRACK_struct_TRPID_COMMAND;


// ��λ�����Ƴ���꣬���ֿ�����_��λ_
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
void TRACK_GetPIDDeviationLevel(void);     // �¼�ѭ��������
void TRACK_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len);
void TRACK_Reset(void);         // ѭ��ģ�����ã�����rewind
void TRACK_ResetStates(void);

void TRACK_AppendCreatedCmd(TRACK_enum_TRPID_ACTION action, int16_t levelOrData
                    , TRACK_struct_StaticLinkedList_CmdUnit *triggerList
                    , TRACK_struct_StaticLinkedList_CmdUnit *midList
                    , TRACK_struct_StaticLinkedList_CmdUnit *endList);                // β�������ָ��
void TRACK_AppendExistingCmd(TRACK_enum_TRACK_COMMAND_TYPE cmdID, int16_t levelOrData);     // β���������ָ��
void TRACK_ChopCmd(void);                   // β���Ƴ�ָ��
void TRACK_EraseCmd(void);                  // Ĩ��ָ��
void TRACK_RewindCmd(void);                 // ����ָ���ʼ��׼������ִ��ָ��

// ��̬�������
void TRACK_StaticLinkedList_CmdUnit_Init(TRACK_struct_StaticLinkedList_CmdUnit *link, uint8_t size);
TRACK_struct_StaticLinkedList_CmdUnit *TRACK_StaticLinkedList_CmdUnit_Malloc(TRACK_struct_StaticLinkedList_CmdUnit *link);
uint8_t TRACK_StaticLinkedList_CmdUnit_Free(TRACK_struct_StaticLinkedList_CmdUnit *link, TRACK_struct_StaticLinkedList_CmdUnit *free);

// �������
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
// ����״̬
    uint8_t sensorBits;             // �͵�ַ��OUT1(���)
    uint8_t lastSensorBits;
    uint16_t maintainRounds;        // ״̬�����ִ�

    // ��ʱ�ִ�
    uint16_t delayRounds;
    uint16_t cmdDelayRounds;

// ָ��
    // ָ�����������ڴ��
    #define TRACK_DEFINE_CMDTYPETOSTATEPOOL_SIZE 64             // ��̬�����ڴ�ش�С   ������ 48 -> 64
    TRACK_struct_StaticLinkedList_CmdUnit 
        state_sllPool[TRACK_DEFINE_CMDTYPETOSTATEPOOL_SIZE];       // ָ���Ӧ���״̬�ľ�̬�����ڴ��

    #define TRACK_DEFINE_CMDSCOUNT 40       // ���ָ����Ŀ��
    #define TRACK_DEFINE_REPORT_PORT    ESP_USART
    #define TRACK_DEFINE_CMD_PERFORMING_END_CODE 0xFF
    TRACK_struct_TRPID_COMMAND cmds[TRACK_DEFINE_CMDSCOUNT];
    TRACK_struct_TRPID_COMMAND encapsulatedCmds[TRACK_E_COMMANDTYPE_TOTAL];

    // ָ��ִ��״̬
    uint8_t pCmds;
    uint8_t cmdsTotal;
    uint8_t stateListMonitering;

// ��ָ����Ʋ���
    float autoCoefficient;
    TRACK_enum_TRPID_ACTION autoAction;
    #define TRACK_DEFINE_DEFAULT_AUTO_CO 1
    #define TRACK_DEFINE_FILTERCOUNT_MAX 3         // �˲��ִ�

// ���
    int8_t PID_deviationLevel;      // ƫ��ȼ�(�̶�), ������ָ��PID����ƫ
    TRACK_enum_TRPID_ACTION PID_action;
    uint16_t basicTargetLevel;      // ѭ������Ŀ���ٶ�/�ȼ������ھ���pid��ѭ���Ļ���Ŀ���ٶ�
    #define TRACK_DEFINE_DEFAULT_BASIC_TARGET_LEVEL 20

// ����ʹ��
    uint8_t isEnabled;      // ���������pid��ʹ��ѭ���˶���
    uint8_t isFeedback;     // �Ƿ���ָ��ִ��״̬����λ��������ѭ��ʹ��ʱ���ã�������ͷ����

} TRACKVARS;


#endif


// typedef enum 
// {
//     // һ��ƫ��
//     TRS_DEVIATION_NO,
//     TRS_DEVIATION_LEFT_LITTLE,
//     TRS_DEVIATION_LEFT_BIG,
//     TRS_DEVIATION_RIGHT_LITTLE,
//     TRS_DEVIATION_RIGHT_BIG,

//     // ��··��ѡ��
//         // 1 -> 2 Y��
//     TRS_JUNCTION_Y_TURN_LEFT,       // W B ? B W -> ��ת
//     TRS_JUNCTION_Y_TURN_RIGHT,      // W B ? B W -> ��ת
//         // ���֧·��
//     TRS_JUNCTION_MEET,              // ��������·ѡ��
//     TRS_JUNCTION_MULTIPLE_SELECT_0,         // ѡ��ֱ��
//     TRS_JUNCTION_MULTIPLE_SELECT_L1,        // ѡ����1��·
//     // TRS_JUNCTION_MULTIPLE_SELECT_L2,
//     // TRS_JUNCTION_MULTIPLE_SELECT_L3,
//     // TRS_JUNCTION_MULTIPLE_SELECT_L4,
//     TRS_JUNCTION_MULTIPLE_SELECT_R1,        // ѡ����1��·
//     // TRS_JUNCTION_MULTIPLE_SELECT_R2,
//     // TRS_JUNCTION_MULTIPLE_SELECT_R3,
//     // TRS_JUNCTION_MULTIPLE_SELECT_R4,

//     // �����ʶ
//     TRS_SIGN_STOP,                  // B B B B B -> ֹͣ
//         // �������ܱ�ʶ�����ı�С����Ϊ
//     TRS_SIGN_BUZZ,                  // B W B W W  or  W W B W B


//     // ���� "����������": ����һ�������ٶȼ���
//     TRPID_E_CORRECTION_TENDENCY__GO_STRAIGHT,
//     TRPID_E_CORRECTION_TENDENCY__LITTLE_LEFT,
//     TRPID_E_CORRECTION_TENDENCY__BIG_LEFT,
//     TRPID_E_CORRECTION_TENDENCY__LITTLE_RIGHT,
//     TRPID_E_CORRECTION_TENDENCY__BIG_RIGHT,

//     // ���� "Ť��������": ��������һ��һ��
//     TRPID_E_


//     TRS_TOTALCOUNT

// } TRACK_enum_TRPID_DIRECTION;



