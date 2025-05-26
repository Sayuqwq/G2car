#include "stm32f10x.h"                  // Device header
#include "Track.h"
#include "communication.h"

#include "Serial.h"
 
#include "motor.h"

/*	friend visit
*/
// �������漰
#include "pid.h"
#include "remote.h"
extern PIDVARS pv;
extern REMOTEVARS rmv;
////

TRACKVARS trv;

void TRACKVARS_Constructor(void)
{
	trv.sensorBits = 0;
    trv.autoAction = TRPID_E_CORRECTION_TORQUE;

    TRACK_StaticLinkedList_CmdUnit_Init(trv.state_sllPool, TRACK_DEFINE_CMDTYPETOSTATEPOOL_SIZE);


/*      ��װָ������     */
    TRACK_struct_TRPID_COMMAND *tempCmd;

// ����ָ��
  // Y����ƫ
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_Y_LEFT];
    tempCmd->action = TRPID_E_CORRECTION_TENDENCY;
    tempCmd->levelOrData = 4;
    tempCmd->isLevelOrDataOpposite = 1;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����

    // ״̬������ȫλ����ģ�Ҳ�����ǲ���λ�����
    tempCmd->triggerStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,1,0,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(0,1,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->midStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,0,0,0,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->endStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,0,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,0,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,1,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

  // Y����ƫ
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_Y_RIGHT];
    tempCmd->action = TRPID_E_CORRECTION_TENDENCY;
    tempCmd->levelOrData = 4;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����

    // ״̬������ȫλ����ģ�Ҳ�����ǲ���λ�����
    tempCmd->triggerStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS_MIRROR(0,1,0,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS_MIRROR(0,1,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->midStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS_MIRROR(0,0,0,0,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->endStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS_MIRROR(0,0,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS_MIRROR(0,0,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS_MIRROR(0,1,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

  // ֱ����ת
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_L_LEFT];
    tempCmd->action = TRPID_E_ROTATE;
    tempCmd->levelOrData = 8;
    tempCmd->isLevelOrDataOpposite = 1;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    // ״̬������ȫλ����ģ�Ҳ�����ǲ���λ�����
    tempCmd->triggerStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS_MIRROR(0,0,0,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS_MIRROR(0,0,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS_MIRROR(0,1,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->midStateList = nullptr;

    tempCmd->endStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS_MIRROR(0,0,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS_MIRROR(0,0,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS_MIRROR(0,1,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

  // ֱ����ת
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_L_RIGHT];
    tempCmd->action = TRPID_E_ROTATE;
    tempCmd->levelOrData = 8;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    // ״̬������ȫλ����ģ�Ҳ�����ǲ���λ�����
    tempCmd->triggerStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,0,0,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(0,0,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(0,1,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->midStateList = nullptr;

    tempCmd->endStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,0,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,0,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,1,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

  // ֹͣ
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_STOP];
    tempCmd->action = TRPID_E_STRAIGHT;
    tempCmd->levelOrData = 0;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    // ״̬������ȫλ����ģ�Ҳ�����ǲ���λ�����
    tempCmd->triggerStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(1,1,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(1,1,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(0,1,1,1,1), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->triggerStateList, TRACK_BINARY_5BITS(0,1,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);

    tempCmd->midStateList = nullptr;

    tempCmd->endStateList = TRACK_LinkedList_CmdUnit_New(trv.state_sllPool, TRACK_BINARY_5BITS(0,0,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,0,1,1,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);
    TRACK_LinkedList_CmdUnit_Append(trv.state_sllPool, tempCmd->endStateList, TRACK_BINARY_5BITS(0,1,1,0,0), TRACK_BINARY_5BITS(1,1,1,1,1), 5, 0);



// �Ǵ���ָ��
  // �޸�Autoƫ�����
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_AUTO_DEVILEVEL];
    tempCmd->action = TRPID_E_NULL;
    tempCmd->levelOrData = TRACK_DEFINE_DEFAULT_AUTO_CO;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    tempCmd->triggerStateList = nullptr;
    tempCmd->midStateList = nullptr;
    tempCmd->endStateList = nullptr;

  // �޸�ѭ�������ٶ�/�ٶȳ̶�
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_TARGETLEVEL];
    tempCmd->action = TRPID_E_NULL;
    tempCmd->levelOrData = TRACK_DEFINE_DEFAULT_BASIC_TARGET_LEVEL;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    tempCmd->triggerStateList = nullptr;
    tempCmd->midStateList = nullptr;
    tempCmd->endStateList = nullptr;

  // ͣ��
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_STOP];
    tempCmd->action = TRPID_E_NULL;
    tempCmd->levelOrData = 500;     // 5��
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    tempCmd->triggerStateList = nullptr;
    tempCmd->midStateList = nullptr;
    tempCmd->endStateList = nullptr;

  // ��Autoѭ��
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_AUTO_ONLY];
    tempCmd->action = TRPID_E_NULL;
    tempCmd->levelOrData = 500;     // 5��
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    tempCmd->triggerStateList = nullptr;
    tempCmd->midStateList = nullptr;
    tempCmd->endStateList = nullptr;

  // autoAction
    tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_AUTO_ACTION];
    tempCmd->action = TRPID_E_NULL;
    tempCmd->levelOrData = (uint8_t)TRPID_E_CORRECTION_TORQUE;
    tempCmd->isLevelOrDataOpposite = 0;
    tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
    tempCmd->triggerStateList = nullptr;
    tempCmd->midStateList = nullptr;
    tempCmd->endStateList = nullptr;


//   // TENDENCY
//     tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_DO_TENDENCY];
//     tempCmd->action = TRPID_E_NULL;
//     tempCmd->levelOrData = 300;     // 3��
//     tempCmd->isLevelOrDataOpposite = 0;
//     tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
//     tempCmd->triggerStateList = nullptr;
//     tempCmd->midStateList = nullptr;
//     tempCmd->endStateList = nullptr;

//   // TORQUE
//     tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_DO_TORQUE];
//     tempCmd->action = TRPID_E_NULL;
//     tempCmd->levelOrData = 300;     // 3��
//     tempCmd->isLevelOrDataOpposite = 0;
//     tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
//     tempCmd->triggerStateList = nullptr;
//     tempCmd->midStateList = nullptr;
//     tempCmd->endStateList = nullptr;

//   // ROTATE
//     tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_DO_ROTATE];
//     tempCmd->action = TRPID_E_NULL;
//     tempCmd->levelOrData = 300;     // 3��
//     tempCmd->isLevelOrDataOpposite = 0;
//     tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
//     tempCmd->triggerStateList = nullptr;
//     tempCmd->midStateList = nullptr;
//     tempCmd->endStateList = nullptr;

//   // STRAIGHT
//     tempCmd = &trv.encapsulatedCmds[TRACK_E_COMMANDTYPE_DIRECT_DO_STRAIGHT];
//     tempCmd->action = TRPID_E_NULL;
//     tempCmd->levelOrData = 300;     // 3��
//     tempCmd->isLevelOrDataOpposite = 0;
//     tempCmd->cmdType = tempCmd - trv.encapsulatedCmds;          // ���䣺<��װָ������>      ��Ǩ�������캯����
 
//     tempCmd->triggerStateList = nullptr;
//     tempCmd->midStateList = nullptr;
//     tempCmd->endStateList = nullptr;

////

    trv.pCmds = 0;
    trv.cmdsTotal = 0;
    trv.stateListMonitering = 0;

    trv.maintainRounds = 1;
    trv.delayRounds = 0;
    trv.cmdDelayRounds = 0;

    trv.PID_deviationLevel = 0;
    trv.PID_action = TRPID_E_CORRECTION_TENDENCY;

    trv.autoCoefficient = 1;
    trv.basicTargetLevel = TRACK_DEFINE_DEFAULT_BASIC_TARGET_LEVEL;

	trv.isEnabled = 0;
    trv.isFeedback = 0;
}

void _TRACK_GetSensorPin(void);             // ��ȡ������״̬
uint8_t _TRACK_StateConditionJudge(const uint8_t state, const uint16_t mask, const uint16_t maintain);    // ״̬ƥ�� ��װ����
uint8_t _TRACK_DelayRound(void);            // �ִ���ʱ
uint8_t _TRACK_CmdUnitDelayRound(void);
void TRACK_ReportCmdPerforming(void);       // ͨ�ű���ָ��ִ�����
void TRACK_ReportCmdDone(void);             // �ر𱨸棬ָ��ִ�н���


void TRACK_Pin_Init(void)	// ѭ�����ų�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStruct;
    
	// Clock
	ABS_RCC_GPIO_ClockCmd(TR1X, ENABLE);
	ABS_RCC_GPIO_ClockCmd(TR3X, ENABLE);

    // GPIOD
	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = TR1 | TR2;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TR1X, &GPIO_InitStruct);
	
    // GPIOG
	// GPIO
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = TR3 | TR4 | TR5;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TR3X, &GPIO_InitStruct);
	
    // // GPIOG    reserved
	// // GPIO
	// GPIO_StructInit(&GPIO_InitStruct);
	// GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	// GPIO_InitStruct.GPIO_Pin = TR6 | TR7 | TR8;
	// GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(TR3X, &GPIO_InitStruct);
}

void TRACK_Init(void)
{
    TRACKVARS_Constructor();

    TRACK_Pin_Init();
}

/*  UNITPROTOCOL
*/
void TRACK_UNITPROTOCOL_CONFIG(uint8_t *data, uint8_t len)
{
	uint8_t uint8Num, first;
    int16_t int16Num;
	uint8_t *pointer = data;

	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&first, &pointer, sizeof(uint8Num));
	CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
	switch (first)
	{
	case (uint8_t)TRACK_E_UNITPROTOCAL_ENABLE:
        PID_StatusVars_Reset();     // pid����
        MOTOR_SetDuty_Zero();       // ���pwm����
        TRACK_ResetStates();        // ѭ��״̬����

		trv.isEnabled = uint8Num;		// ������, pidDebug, track, remote
		if (uint8Num == 1)
		{
			pv.isDebugEnabled = 0;
			rmv.isEnabled = 0;
		}
        break;

    case (uint8_t)TRACK_E_UNITPROTOCAL_CMD:
        switch (uint8Num)
        {
        case (uint8_t)TRACK_E_COMMAND_ERASE:
            TRACK_EraseCmd();
            break;

        case (uint8_t)TRACK_E_COMMAND_APPEND:
            CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
            CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&int16Num, &pointer, sizeof(int16Num));

            if (uint8Num <= TRACK_E_COMMANDTYPE_RESERVED || uint8Num >= TRACK_E_COMMANDTYPE_TOTAL)
                break;
            
            TRACK_AppendExistingCmd((TRACK_enum_TRACK_COMMAND_TYPE)uint8Num, int16Num);
            break;

        case (uint8_t)TRACK_E_COMMAND_REWIND:
            TRACK_Reset();      // ״̬���� + ����
            break;

        case (uint8_t)TRACK_E_COMMAND_FEEDBACK:
            CMNCT_TypeConversion_MemoryCopyAndForwardPatched(&uint8Num, &pointer, sizeof(uint8Num));
            trv.isFeedback = uint8Num;
            break;
       }
        break;
	}
}

/*  ѭ��ָ�����
*/
void TRACK_CommandProcessor(void)           // ����ѭ��ָ����ܺ���
{
    /*  �׵�������������·�Ĵ���������·ʱ��Ӧ������ʲô"����"��"ƫ��ȼ�"����Ӧ�����Զ����
        Ȼ����Autoģʽ��Ӧ���ܵ�ָ��Ŀ��ƣ�ָ����Կ���Autoģʽ�µ�"����"��"ƫ��ȼ�"��������Ĭ�ϲ����ܲ�������
        ����ѭ��ģʽ��ϵͳ����һ������״̬ʱ��������Ӧ��ָ����ƣ��������������"����"��"ƫ��ȼ�"
    */
    uint8_t i;
    static uint8_t pMid;

    if (trv.pCmds >= trv.cmdsTotal)
        return;

    if (_TRACK_CmdUnitDelayRound() == 0)
        return;
    
// ����ָ��(״̬����_ͨ�÷�ʽ)
    if (trv.cmds[trv.pCmds].cmdType > TRACK_E_COMMANDTYPE_RESERVED 
        && trv.cmds[trv.pCmds].cmdType < TRACK_E_COMMANDTYPE_DIRECT_HEAD)
        switch (trv.stateListMonitering)
        {
        case 0:
            i = trv.cmds[trv.pCmds].triggerStateList - trv.state_sllPool;
            while (i != 255)
            {
                if (_TRACK_StateConditionJudge(trv.state_sllPool[i].data, trv.state_sllPool[i].mask, trv.state_sllPool[i].maintain))   // ����"����״̬"
                {
                    TRACK_ReportCmdPerforming();        // �����򱨸�ָ��ִ�����

                    if (trv.cmds[trv.pCmds].levelOrData == TRACK_DEFINE_CMD_SKIP)
                    {
                        trv.pCmds++;    // �͵�ʲô��û������һ��
                        break;
                    }
                    
                    trv.PID_action= trv.cmds[trv.pCmds].action;

                    if (trv.cmds[trv.pCmds].isLevelOrDataOpposite)       // ֻ��Ҫ�˽��Ƿ���Ҫ���࣬������Ҫ�˽���ʲôָ��
                        trv.PID_deviationLevel = -trv.cmds[trv.pCmds].levelOrData;
                    else
                        trv.PID_deviationLevel = trv.cmds[trv.pCmds].levelOrData;

                    trv.stateListMonitering = 1;
                    trv.delayRounds = trv.state_sllPool[i].delay;

                    if (trv.cmds[trv.pCmds].midStateList == nullptr)
                    {
                        pMid = 0;
                        trv.stateListMonitering = 2;
                        break;
                    }
                    pMid = trv.cmds[trv.pCmds].midStateList - trv.state_sllPool;
                    break;
                }
                i = trv.state_sllPool[i].next;
            }
            break;

        case 1:
            if (pMid == 255)
            {
                pMid = 0;
                trv.stateListMonitering = 2;
                break;
            }
            if (_TRACK_StateConditionJudge(trv.state_sllPool[pMid].data, trv.state_sllPool[pMid].mask, trv.state_sllPool[pMid].maintain))
            {
                pMid = trv.state_sllPool[pMid].next;
                break;
            }
            break;

        case 2:
            i = trv.cmds[trv.pCmds].endStateList - trv.state_sllPool;
            while (i != 255)
            {
                if (_TRACK_StateConditionJudge(trv.state_sllPool[i].data, trv.state_sllPool[i].mask, trv.state_sllPool[i].maintain))
                {
                    trv.stateListMonitering = 0;
                    trv.pCmds++;
                    break;
                }
                i = trv.state_sllPool[i].next;
            }
            break;
        }

// �Ǵ���ָ��(����״̬����_�޷�ʽ)      ָ��������ʷ����ı䣨������levelOrData��, ������ͨ�÷�ʽ����ʹ��switch����Բ���
    // �������У�action, levelOrData, cmdType, triggerStateList;
    else if (trv.cmds[trv.pCmds].cmdType > TRACK_E_COMMANDTYPE_DIRECT_HEAD 
        && trv.cmds[trv.pCmds].cmdType < TRACK_E_COMMANDTYPE_TOTAL)
    {
        // �Ǵ���ָ����SKIP����Ϊû�б�Ҫ
        switch (trv.cmds[trv.pCmds].cmdType)
        {
            case TRACK_E_COMMANDTYPE_DIRECT_AUTO_DEVILEVEL:     // �޸�Autoѭ��Ť������
                trv.autoCoefficient = trv.cmds[trv.pCmds].levelOrData;
                break;
                
            case TRACK_E_COMMANDTYPE_DIRECT_TARGETLEVEL:        // �޸�ѭ������Ŀ���ٶ�
                trv.basicTargetLevel = trv.cmds[trv.pCmds].levelOrData;
                break;
                
            case TRACK_E_COMMANDTYPE_DIRECT_STOP:               // С��ͣ��һ��ʱ��, ���ö���ָ����ƫ��ȼ�
                trv.PID_action = TRPID_E_ROTATE;
                trv.PID_deviationLevel = 0;

                if (trv.cmds[trv.pCmds].levelOrData > TRACK_DEFINE_DELAY_FOREVER_VAL)
                    trv.delayRounds = trv.cmds[trv.pCmds].levelOrData + 1;
                else 
                    trv.delayRounds = trv.cmds[trv.pCmds].levelOrData;

                break;
                
            case TRACK_E_COMMANDTYPE_DIRECT_AUTO_ONLY:          // С��ֻAutoѭ��һ��ʱ��(�ڼ䲻������ִ���κ�ָ��)
                trv.cmdDelayRounds = trv.cmds[trv.pCmds].levelOrData;
                break;   

            case (uint8_t)TRACK_E_COMMANDTYPE_DIRECT_AUTO_ACTION:
                if (trv.cmds[trv.pCmds].levelOrData == 0)
                    trv.autoAction = TRPID_E_CORRECTION_TENDENCY;
                else
                    trv.autoAction = TRPID_E_CORRECTION_TORQUE;
                break;


            // case TRACK_E_COMMANDTYPE_DIRECT_DO_TENDENCY:
            //     trv.PID_action = TRPID_E_CORRECTION_TENDENCY;
            //     trv.PID_deviationLevel = trv.cmds[trv.pCmds].levelOrData;
            //     trv.delayRounds = 500;

            // case TRACK_E_COMMANDTYPE_DIRECT_DO_TORQUE:
            //     trv.PID_action = TRPID_E_CORRECTION_TORQUE;
            //     trv.PID_deviationLevel = trv.cmds[trv.pCmds].levelOrData;
            //     trv.delayRounds = 500;

            // case TRACK_E_COMMANDTYPE_DIRECT_DO_ROTATE:
            //     trv.PID_action = TRPID_E_ROTATE;
            //     trv.PID_deviationLevel = trv.cmds[trv.pCmds].levelOrData;
            //     trv.delayRounds = 500;

            // case TRACK_E_COMMANDTYPE_DIRECT_DO_STRAIGHT:
            //     trv.PID_action = TRPID_E_STRAIGHT;
            //     trv.PID_deviationLevel = trv.cmds[trv.pCmds].levelOrData;
            //     trv.delayRounds = 500;
        }

        TRACK_ReportCmdPerforming();

        trv.pCmds++;
    }


    if (trv.pCmds >= trv.cmdsTotal)     // ֻ�����һ�Σ�����ָ��ִ�����
        TRACK_ReportCmdDone();
}


/*  TrackPID���㣬����"����ָ��","ƫ��ȼ�"
*/
void TRACK_GetPIDDeviationLevel(void)
{
    uint8_t auto_isAutoTracking = 1;

	_TRACK_GetSensorPin();

    if (trv.isEnabled == 0)
        return ;

    if (_TRACK_DelayRound() == 0)   // ��Ĭ�ִκ���
        return;


    // Autoѭ��
    if (trv.stateListMonitering == 0)
    {
        switch (trv.sensorBits)
        {
        // һ��ƫ��ģʽ, Autoģʽ
        case TRACK_BINARY_5BITS(0,0,1,0,0):
            trv.PID_deviationLevel = 0;
            break;
        case TRACK_BINARY_5BITS(0,1,1,0,0):     // Ӧ����������
            trv.PID_deviationLevel = -4;
            break;
        case TRACK_BINARY_5BITS(0,1,0,0,0):
            trv.PID_deviationLevel = -8;
            break;
        case TRACK_BINARY_5BITS(0,0,1,1,0):
            trv.PID_deviationLevel = 4;
            break;
        case TRACK_BINARY_5BITS(0,0,0,1,0):
            trv.PID_deviationLevel = 8;
            break;

        default:
            auto_isAutoTracking = 0;
            break;
        }

        trv.PID_action = trv.autoAction;

        if (auto_isAutoTracking == 1)        // ֻ�д�����Autoѭ��״̬�Ž��г���������Ϊ��ߴ��븴���Զ�д�µ����
            trv.PID_deviationLevel *= trv.autoCoefficient;
    }

    // ָ��ѭ��, �������У�������Զ�޷�����ָ�����δ����ָ��ʱ�����������κ��޸ģ�   һ������ĳ��ָ���ض������Autoѭ��
    TRACK_CommandProcessor();
}

/*  ָ������
*/
void TRACK_AppendCreatedCmd(TRACK_enum_TRPID_ACTION action, int16_t levelOrData
                    , TRACK_struct_StaticLinkedList_CmdUnit *triggerList
                    , TRACK_struct_StaticLinkedList_CmdUnit *midList
                    , TRACK_struct_StaticLinkedList_CmdUnit *endList)     // �����ָ��
{
    trv.cmds[trv.cmdsTotal].action = action;
    trv.cmds[trv.cmdsTotal].levelOrData = levelOrData;
    trv.cmds[trv.cmdsTotal].triggerStateList = triggerList;
    trv.cmds[trv.cmdsTotal].midStateList = midList;
    trv.cmds[trv.cmdsTotal].endStateList = endList;
    trv.cmdsTotal++;
}

void TRACK_AppendExistingCmd(TRACK_enum_TRACK_COMMAND_TYPE cmdID, int16_t levelOrData)     // β���������ָ��
{
    trv.cmds[trv.cmdsTotal] = trv.encapsulatedCmds[cmdID];
    trv.cmds[trv.cmdsTotal].levelOrData = levelOrData;
    // trv.cmds[trv.cmdsTotal].cmdType = cmdID;        // ������<��װָ������>      ��Ǩ�������캯����
    trv.cmdsTotal++;
}

void TRACK_ChopCmd(void)
{
    if (trv.cmdsTotal == 0)
        return;

    trv.cmdsTotal--;
}

void TRACK_EraseCmd(void)
{
    TRACK_Reset();      // ͬʱ����ָ��
    trv.cmdsTotal = 0;
}

void TRACK_RewindCmd(void)       // �˺���������TRACK_Reset�غ�
{
    trv.pCmds = 0;          // ����ָ��ָ��
    trv.delayRounds = 0;
    trv.cmdDelayRounds = 0;
    trv.stateListMonitering = 0;
}

/*  ���ܺ���
*/
void TRACK_Reset(void)
{
    TRACK_ResetStates();
    TRACK_RewindCmd();
}

void TRACK_ResetStates(void)
{
    trv.maintainRounds = 1;
    trv.PID_action = TRPID_E_CORRECTION_TENDENCY;
    trv.PID_deviationLevel = 0;
    trv.basicTargetLevel = TRACK_DEFINE_DEFAULT_BASIC_TARGET_LEVEL;
    trv.autoCoefficient = TRACK_DEFINE_DEFAULT_AUTO_CO;
}

void TRACK_ReportCmdPerforming(void)
{
    if (trv.isFeedback == 0)
        return;

    Serial_SendByte(trv.pCmds, TRACK_DEFINE_REPORT_PORT);                              // ���к�
    Serial_SendByte(trv.cmds[trv.pCmds].cmdType, TRACK_DEFINE_REPORT_PORT);            // ��װָ������
    Serial_SendString_Patch((char *)&trv.cmds[trv.pCmds].levelOrData, 2, TRACK_DEFINE_REPORT_PORT);     // ָ��ƫ�����
    Serial_SendString_Patch("\r\n", 2, TRACK_DEFINE_REPORT_PORT);
}

void TRACK_ReportCmdDone(void)      // �ر𱨸棬ָ��ִ�н���
{
    if (trv.isFeedback == 0)
        return;

    Serial_SendByte(TRACK_DEFINE_CMD_PERFORMING_END_CODE, TRACK_DEFINE_REPORT_PORT);       // ���к�
    Serial_SendByte(TRACK_DEFINE_CMD_PERFORMING_END_CODE, TRACK_DEFINE_REPORT_PORT);       // ��װָ������
    Serial_SendByte(TRACK_DEFINE_CMD_PERFORMING_END_CODE, TRACK_DEFINE_REPORT_PORT);       // ָ��ƫ�����
    Serial_SendByte(TRACK_DEFINE_CMD_PERFORMING_END_CODE, TRACK_DEFINE_REPORT_PORT);       // ָ��ƫ�����
    Serial_SendString_Patch("\r\n", 2, TRACK_DEFINE_REPORT_PORT);
}

    // ������״̬�жϣ�������Ҫ�ж�state��maintain����״̬����Ҫ���븴�����Է�װ�ں�����
uint8_t _TRACK_StateConditionJudge(const uint8_t state, const uint16_t mask, const uint16_t maintain)    // ����ֵ:  1: ƥ��  0: ��ƥ��
{
    // �ֲ������ӿ�����ٶȣ����ṩ�����
    uint8_t sensorBits = trv.sensorBits & mask;
    uint16_t maintainRounds = trv.maintainRounds;

    // ��״̬, �����state��maintain��Ҫ�����Ƚ�    
    if ((state & 0x80) == 0x80)
    {
        if (sensorBits == (state & 0x7F))
            return 0;
    }
    else    // ����״̬�ȶ� 
    {
        if (sensorBits != state)
            return 0;
    }

    // maintain�ȶ�
    if (maintainRounds < maintain)
        return 0;

    return 1;       // ƥ��
}

uint8_t _TRACK_DelayRound(void)
{
    if (trv.delayRounds == TRACK_DEFINE_DELAY_FOREVER_VAL)        // ��þ�Ĭ!(����)
        return 0;

    if (trv.delayRounds == TRACK_DEFINE_DELAY_FOREVER_VAL + 1)
    {
        trv.delayRounds -= 2;       // �ܹ���þ�Ĭֵ������10ms�������ڸ�ֵʱ�򲹶�
    }    

    if (trv.delayRounds != 0)
    {
        trv.delayRounds--;
        return 0;
    }

    return 1;
}

uint8_t _TRACK_CmdUnitDelayRound(void)
{
    if (trv.cmdDelayRounds == TRACK_DEFINE_DELAY_FOREVER_VAL)        // ��þ�Ĭ!(����)
        return 0;

    if (trv.cmdDelayRounds == TRACK_DEFINE_DELAY_FOREVER_VAL + 1)
    {
        trv.cmdDelayRounds -= 2;       // �ܹ���þ�Ĭֵ������10ms�������ڸ�ֵʱ�򲹶�
    }    

    if (trv.cmdDelayRounds != 0)
    {
        trv.cmdDelayRounds--;
        return 0;
    }

    return 1;
}

uint8_t _TRACK_Filter(uint8_t pinID, uint8_t dataBit)      // _TRACK_GetSensorPin �˲�, 0: ����ֵ, 1:��ֵ
{
    static uint8_t filterCounts[TR_ACTIVATED_COUNT];

    pinID--;
    if ((trv.sensorBits & (0x01 << pinID)) != dataBit)      // ��һ�����������ﵽһ����ֵ���򷵻�"Ӧ��תλ״̬"
    {
        filterCounts[pinID]++;
        if (filterCounts[pinID] >= TRACK_DEFINE_FILTERCOUNT_MAX)   
        {
            filterCounts[pinID] = 0;
            return 1;
        }
    }
    else            // һ������������filterCounts������"λӦ����"
    {
        filterCounts[pinID] = 0;
        return 0;
    }

    return 0;
}

void _TRACK_GetSensorPin(void)
{  
    uint8_t temp;
    uint8_t i = 0;

	// ��˴洢��OUT1�ڸ�λ

#if (TR_ACTIVATED_COUNT >= 1)
	temp = GPIO_ReadInputDataBit(TR1X, TR1) << i;
    if (_TRACK_Filter(i + 1, temp) == 1)
        trv.sensorBits ^= (0x01 << i);
    i++;
#endif
#if (TR_ACTIVATED_COUNT >= 2)
	temp = GPIO_ReadInputDataBit(TR2X, TR2) << i;
    if (_TRACK_Filter(i + 1, temp) == 1)
        trv.sensorBits ^= (0x01 << i);
    i++;
#endif
#if (TR_ACTIVATED_COUNT >= 3)
	temp = GPIO_ReadInputDataBit(TR3X, TR3) << i;
    if (_TRACK_Filter(i + 1, temp) == 1)
        trv.sensorBits ^= (0x01 << i);
    i++;
#endif
#if (TR_ACTIVATED_COUNT >= 4)
	temp = GPIO_ReadInputDataBit(TR4X, TR4) << i;
    if (_TRACK_Filter(i + 1, temp) == 1)
        trv.sensorBits ^= (0x01 << i);
    i++;
#endif
#if (TR_ACTIVATED_COUNT >= 5)
	temp = GPIO_ReadInputDataBit(TR5X, TR5) << i;
    if (_TRACK_Filter(i + 1, temp) == 1)
        trv.sensorBits ^= (0x01 << i);
    i++;
#endif

    if (trv.lastSensorBits == trv.sensorBits)
    {
        if (trv.maintainRounds < 65535)
            trv.maintainRounds++;
    }
    else
        trv.maintainRounds = 1;

    trv.lastSensorBits = trv.sensorBits;



	// reserved
#if (TR_ACTIVATED_COUNT >= 6)
	trv.sensorBits |= GPIO_ReadInputDataBit(TR6X, TR6) << 5;
#endif
#if (TR_ACTIVATED_COUNT >= 7)
	trv.sensorBits |= GPIO_ReadInputDataBit(TR7X, TR7) << 6;
#endif
#if (TR_ACTIVATED_COUNT >= 8)
	trv.sensorBits |= GPIO_ReadInputDataBit(TR8X, TR8) << 7;
#endif
}



// // ����
// case TRACK_BINARY_5BITS(0,1,0,1,0):     // ������·������ָ��ִ��ģʽ��ֱ��Ŀ��״̬���֣�����һ��ƫ��ģʽ
// case TRACK_BINARY_5BITS(0,1,1,1,0):
//     // ����ָ��ϵͳ����...
//     // trv.PID_action = TRPID_E_CORRECTION_TENDENCY;
//     // trv.PID_deviationLevel = 0;
//     break;

// case TRACK_BINARY_5BITS(1,1,1,1,1):     // ԭ��ֹͣ
// case TRACK_BINARY_5BITS(0,0,0,0,0):
//     trv.PID_action = TRPID_E_ROTATE;
//     trv.PID_deviationLevel = 0;
//     break;
// case TRACK_BINARY_5BITS(1,0,0,0,0):
// case TRACK_BINARY_5BITS(0,0,0,0,1):
//     // ���ͷ���֪ͨ
//     break;



/*  ��̬����ʵ�ֵĶ��ڴ�, ����ͷ�ǿ�������ͷ(�̶�)��һ���ڿ��б�ͷ������
*/
void TRACK_StaticLinkedList_CmdUnit_Init(TRACK_struct_StaticLinkedList_CmdUnit *link, uint8_t size)
{
    link->data = size;
    link->next = 1;
	link++;
	
    for (uint8_t i = 1; i < size - 1; i++)
    {
        link->next = i + 1;
        link->data = 0;
        link->mask = 0;
        link->maintain = 0;
        link->delay = 0;

        link++;
    }

    link->next = 0;
}

TRACK_struct_StaticLinkedList_CmdUnit *TRACK_StaticLinkedList_CmdUnit_Malloc(TRACK_struct_StaticLinkedList_CmdUnit *link)
{
    TRACK_struct_StaticLinkedList_CmdUnit *retAddr;

    if (link->next != 0)
    {
        retAddr = link + link->next;
        link->next = link[link->next].next;
        return retAddr;
    }

    return nullptr;
}

uint8_t TRACK_StaticLinkedList_CmdUnit_Free(TRACK_struct_StaticLinkedList_CmdUnit *link, TRACK_struct_StaticLinkedList_CmdUnit *free)
{
    uint8_t cursor = free - link;

    if (cursor >= link->data)
        return 0;
	if (cursor == 0)
		return 0;
        
	int i = link->next;
	while (i != 0)
	{
		if (cursor == i)
			return 0;
		i = link[i].next;
	}

    free->next = link->next;
    link->next = cursor;

    return 1;
}

// �������
TRACK_struct_StaticLinkedList_CmdUnit * TRACK_LinkedList_CmdUnit_New(TRACK_struct_StaticLinkedList_CmdUnit *list, uint8_t data, uint16_t mask, uint16_t maintain, uint16_t delay)
{
	TRACK_struct_StaticLinkedList_CmdUnit *newNode = TRACK_StaticLinkedList_CmdUnit_Malloc(list);	
	if (newNode == nullptr)
		return nullptr;
	newNode->data = data;
    newNode->mask = mask;
    newNode->maintain = maintain;
    newNode->delay = delay;
	newNode->next = 255;
	
	return newNode;
}

void TRACK_LinkedList_CmdUnit_Destroy(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit **head)
{
	if ((*head)->next == 255)
	{
		TRACK_StaticLinkedList_CmdUnit_Free(list, *head);
		*head = nullptr;
	}
}


uint8_t TRACK_LinkedList_CmdUnit_Append(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit *head, uint8_t data, uint16_t mask, uint16_t maintain, uint16_t delay)
{
    uint8_t i = head - list;

    while (list[i].next != 255)
    {
        i = list[i].next;
    }

    TRACK_struct_StaticLinkedList_CmdUnit *newNode = TRACK_StaticLinkedList_CmdUnit_Malloc(list);
	if (newNode == nullptr)
		return 0;

    newNode->data = data;
    newNode->mask = mask;
    newNode->maintain = maintain;
    newNode->delay=  delay;
    list[i].next = newNode - list;
    newNode->next = 255;
    
    return 1;
}

uint8_t TRACK_LinkedList_CmdUnit_Chop(TRACK_struct_StaticLinkedList_CmdUnit *list, TRACK_struct_StaticLinkedList_CmdUnit *head)
{
    uint8_t i = head - list;
	uint8_t ret = 0;

    if (list[i].next == 255)
    {
        return 255;
    }

    while (list[list[i].next].next != 255)
    {
        i = list[i].next;
    }

    ret = list[list[i].next].data;
    TRACK_StaticLinkedList_CmdUnit_Free(list, list + list[i].next);
    list[i].next = 255;
    
	return ret;
}







