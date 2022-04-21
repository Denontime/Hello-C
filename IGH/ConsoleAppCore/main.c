#include "ecrt.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <native/mutex.h>
#include <native/sem.h>
#include <native/task.h>
#include <native/timer.h>
#include <pthread.h>
#include <rtdk.h>
#include <rtdm/rtdm.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#define Bool int
#define false 0
#define true 1
#define ETHERCAT_StatusWord_OP 0x08
#define StatusWord_SERVO_ENABLE_BIT (0x04)
#define CSPPositionKeepTime 1000000000 //到达目标位置保持时间
#define MilliSecond 1000000
#define servo_count 1 //电机数量
#define VFD_count 1   //变频器数量
#define CASE_GO_TO_WHERE_DATA_BUFFER 80
#define SECOND 1000000000
#define DIVERT_LANE_COUNT 3
#define CASE_FILTER_TIME_MIN 500000000  // 0.5s
#define CASE_FILTER_TIME_MAX 4000000000 // 4s
#define GAP_TIME_OUT 8000000000         // 8s
#define GAP_TIME_INNER 6000000000       // 6s
#define DIVERT_RESULT_TIME 2 * SECOND
#define LANE_FULL_TIME 3000000000
#define DIVERT_SUCCESS 0
#define DIVERT_FAILED 224
#define DIVERT_LANE_FULL 226
#define RECYCLE_SUCCESS 227
#define MyQUEUE_SIZE 100
#define SHOW_LOG 0
#define LIST_MAXSIZE 240
#define BIT_WIDTH 3

int home_point_all_find = false;
int csping_all_ready = false;
RT_TASK DivertLaneTaskList[servo_count];
int CSPMotorSwitch[servo_count] = {1};         //csp模式下电机转动开关
int HomePointFinded[servo_count] = {0};        //原点复归标志位
int MotorCurrentPosition[servo_count] = {0};   //电机当前位置
int CSPMotorStatus[servo_count] = {0};         //csp模式下电机运行状态
int CSPTargetPosition[servo_count] = {630000}; //每个电机摆动位置
int CSPHomePosition[servo_count] = {300000};   //每个电机前进方向位置
int CSPSpeed[servo_count] = {8000};            //csp转动速率
int ServoFaultReset = 0;                       //伺服Fault复位标志位
int DivertLaneSwitch[servo_count] = {0};       //所有Divert Lane对应斜轮是否分拣开关

int HomePointFindMode = 33;         //回零方式
int HomePointSwitchFindSpeed = 500; //寻找原点开关速度
int HomePointFindSpeed = 200;       //寻找原点速度
int HomePointAcceleration = 800;    //原点复归加速度
int HomePointOffset = 0;            //原点复归偏移量

int VFDRunStatus[VFD_count] = {0}; //变频器是否转动
int VFDFreq[VFD_count] = {5000};
int VFDTurnAround = 2; //变频器旋转方向

const char *filepath = "sharedfile";
char *share_file_data;
char tempchar[10];
int file_data;

int need_divert;
int need_divert_pop;
int last_case_go_to_where;
unsigned int case_go_to_where;
unsigned int predict_trigger_count[DIVERT_LANE_COUNT] = {0};
unsigned int real_trigger_count[DIVERT_LANE_COUNT] = {0};
unsigned int need_divert_count[DIVERT_LANE_COUNT] = {0};

int case_go_to_where_list[CASE_GO_TO_WHERE_DATA_BUFFER] = {0};
int case_go_to_where_index = 0;

unsigned int camera_pe_trigger = 0;
unsigned int send_pe_trigger = 0;
// divert lane pe
unsigned int divert_lane_pe_trigger[DIVERT_LANE_COUNT] = {0};
unsigned int divert_lane_has_trigger[DIVERT_LANE_COUNT] = {0};
unsigned int is_divert_flag[DIVERT_LANE_COUNT] = {0};
RTIME divert_pe_trigger_time[DIVERT_LANE_COUNT] = {0};

// divert success pe
unsigned int divert_success_pe_trigger[DIVERT_LANE_COUNT] = {0};
unsigned int divert_success_has_trigger[DIVERT_LANE_COUNT] = {0};

// divert lane full pe
unsigned int divert_lane_full_pe_trigger[DIVERT_LANE_COUNT] = {0};
unsigned int divert_lane_full_has_trigger[DIVERT_LANE_COUNT] = {0};
unsigned int divert_lane_full_flag[DIVERT_LANE_COUNT] = {0};
RTIME divert_lane_full_pe_trigger_time[DIVERT_LANE_COUNT] = {0};

typedef struct
{
    int arr[MyQUEUE_SIZE];
    int head; //记录最前面数字所在的下标
    int tail; //记录最后一个有效数字的下一个坐标
              //如果队列里一个数都没有的话head=tail
} MyQueue;
MyQueue need_divert_trigger_count_list[DIVERT_LANE_COUNT];
MyQueue need_divert_case_go_to_where_index_list[DIVERT_LANE_COUNT];
//drive StatusWord in csp
typedef enum _ServoStatusWordCSP
{
    wait,
    running,
    ok //到达位置
} ServoStatusWordCSP;
//master StatusWord
typedef enum _SysWorkingStatusWord
{
    SYS_WORKING_POWER_ON,
    SYS_WORKING_SAFE_MODE,
    SYS_WORKING_OP_MODE,
    SYS_WORKING_LINK_DOWN,
    SYS_WORKING_IDLE_StatusWord //系统空闲
} SysWorkingStatusWord;

//control mode
typedef enum _ServoContorlModel
{
    Reserved = 0,
    PP = 1, //位置控制
    PV = 3,
    PT = 4,
    HM = 6, //原点复归
    IP = 7,
    CSP = 8, //周期同步
    CSV = 9,
    CST = 10
} ServoContorlModel;

typedef struct _GSysRunningParm
{
    SysWorkingStatusWord m_gWorkStatusWord;
} GSysRunningParm;

GSysRunningParm gSysRunning;

int run = 1;
#define CLOCK_TO_USE CLOCK_REALTIME
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
static int64_t system_time_base = 0LL;
//获取当前系统时间
RTIME system_time_ns(void)
{
    struct timespec rt_time;
    clock_gettime(CLOCK_TO_USE, &rt_time);
    RTIME time = TIMESPEC2NS(rt_time);
    return time - system_time_base;
}
/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domainServoInput = NULL;
static ec_domain_state_t domainServoInput_state = {};
static ec_domain_t *domainServoOutput = NULL;
static ec_domain_state_t domainServoOutput_state = {};
static ec_slave_config_t *sc_servo[servo_count];
static ec_slave_config_state_t sc_servo_state[servo_count];
static ec_slave_config_t *sc_VFD[VFD_count];
static ec_slave_config_state_t sc_VFD_state[VFD_count];
static ec_slave_config_t *sc_dig_in_1 = NULL;
static ec_slave_config_t *sc_dig_in_3 = NULL;
static ec_slave_config_t *sc_dig_in_5 = NULL;
static ec_slave_config_t *sc_dig_out_7 = NULL;

static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

/****************************************************************************/
#define ServoMotor 0, 0
#define VFD 0, 1
#define BusCouplerPos 0, 2
#define DigInSlave_01_Pos 0, 3 //lane 0 divert pe
#define DigInSlave_03_Pos 0, 7 //lane 0 divert success pe
#define DigInSlave_05_Pos 0, 9 //lane 0 divert full pe
#define DigOutSlave_07_Pos 0, 5

/*Vendor ID  * Product code */
#define ASDAServoMotor 0x000001dd, 0x10305070
#define OMRONVFD 0x00000083, 0x00000053
#define Beckhoff_EL1809 0x00000002, 0x07113052 // Salve 1
#define Beckhoff_EL1889 0x00000002, 0x07613052 // Salve 3
#define Beckhoff_EL1088 0x00000002, 0x04403052 // Salve 5
#define Beckhoff_EL2809 0x00000002, 0x0af93052 // Salve 7 Dig Out

// offsets for PDO entries
static unsigned int off_dig_in_1;
static unsigned int off_dig_in_3;
static unsigned int off_dig_in_5;
static unsigned int off_dig_out_7;

static unsigned int ControlWord[servo_count];    //控制字
static unsigned int TargetPosition[servo_count]; //目标位置
static unsigned int StatusWord[servo_count];     //状态字
static unsigned int ActualPosition[servo_count]; //当前回授位置
static unsigned int OPMode[servo_count];         //6060
static unsigned int OPModeDisplay[servo_count];  //6061
static unsigned int ErrorCode[servo_count];      //错误代码
static unsigned int CurrentMode[servo_count];
static unsigned int CurrentStatusWord[servo_count];
static unsigned int cycle_counter = 0;

static unsigned int VFDCommand[VFD_count];
static unsigned int VFDFrequency[VFD_count];
static unsigned int VFDStatusWord[VFD_count];
static unsigned int VFDOutputFrequency[VFD_count];
// process data
ec_pdo_entry_reg_t domainServoOutput_regs[] = {
    {ServoMotor, ASDAServoMotor, 0x6040, 0x00, &ControlWord[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x607a, 0x00, &TargetPosition[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6060, 0x00, &OPMode[0], NULL},
    {VFD, OMRONVFD, 0x5000, 0x00, &VFDCommand[0], NULL},
    {VFD, OMRONVFD, 0x5010, 0x00, &VFDFrequency[0], NULL},
    {DigOutSlave_07_Pos, Beckhoff_EL2809, 0x7000, 1, &off_dig_out_7},
    {}};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {ServoMotor, ASDAServoMotor, 0x6064, 0x00, &ActualPosition[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6041, 0x00, &StatusWord[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6061, 0x00, &OPModeDisplay[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x603f, 0x00, &ErrorCode[0], NULL},
    {VFD, OMRONVFD, 0x5100, 0x00, &VFDStatusWord[0], NULL},
    {VFD, OMRONVFD, 0x5110, 0x00, &VFDOutputFrequency[0], NULL},
    {DigInSlave_01_Pos, Beckhoff_EL1809, 0x6000, 1, &off_dig_in_1},
    {DigInSlave_03_Pos, Beckhoff_EL1889, 0x6000, 1, &off_dig_in_3},
    {DigInSlave_05_Pos, Beckhoff_EL1088, 0x6000, 1, &off_dig_in_5},
    {}};
/****************************************************************************/
/* Vendor ID:       0x000001dd
 * Product code:    0x10305070
 * Revision number: 0x02040608
 */
static ec_pdo_entry_info_t servo_pdo_entries_output[] = {
    {0x6040, 0x00, 16}, //control word
    {0x607a, 0x00, 32}, //TargetPosition
    {0x6060, 0x00, 8},  //OPMode
};

static ec_pdo_entry_info_t servo_pdo_entries_input[] = {
    {0x6041, 0x00, 16}, //StatusWord word
    {0x6064, 0x00, 32}, //actualPosition
    {0x6061, 0x00, 8},  /*modes of operation display*/
    {0x603f, 0x00, 16}, /*error code*/
};

static ec_pdo_info_t servo_pdo[] = {
    //RxPDO
    {0x1600, 3, servo_pdo_entries_output},
    //TxPDO
    {0x1a00, 4, servo_pdo_entries_input},
};

static ec_sync_info_t servo_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, servo_pdo + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, servo_pdo + 1, EC_WD_DISABLE},
    {0xff}};

/* Vendor ID:       0x00000083
 * Product code:    0x00000053
 * Revision number: 0x00010001
 */
ec_pdo_entry_info_t vfd_pdo_entries[] = {
    {0x5000, 0x00, 16}, /* Command */
    {0x5010, 0x00, 16}, /* Frequency reference */
    {0x5100, 0x00, 16}, /* StatusWord */
    {0x5110, 0x00, 16}, /* Output frequency monitor */
};

ec_pdo_info_t vfd_pdo[] = {
    {0x1701, 2, vfd_pdo_entries + 0}, /* 258th receive PDO Mapping */
    {0x1b01, 2, vfd_pdo_entries + 2}, /* 258th transmit PDO Mapping */
};

ec_sync_info_t vfd_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, vfd_pdo + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, vfd_pdo + 1, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 1, "EL1809"
 * Vendor ID:       0x00000002
 * Product code:    0x07113052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1a00, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 3, "EL1889"
 * Vendor ID:       0x00000002
 * Product code:    0x07613052
 * Revision number: 0x00130000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1a00, 1, slave_3_pdo_entries + 0}, /* Channel 1 */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_INPUT, 1, slave_3_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 5, "EL1088"
 * Vendor ID:       0x00000002
 * Product code:    0x04403052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1a00, 1, slave_5_pdo_entries + 0}, /* Channel 1 */
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_INPUT, 1, slave_5_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 7, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_7_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_7_pdos[] = {
    {0x1600, 1, slave_7_pdo_entries + 0}, /* Channel 1 */
};

ec_sync_info_t slave_7_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_7_pdos + 0, EC_WD_ENABLE},
    {0xff}};

/**************** queue ********************/
//队列的初始化函数
void queue_init(MyQueue *p_queue)
{
    p_queue->head = 0;
    p_queue->tail = 0;
}

//队列清理函数
void queue_deinit(MyQueue *p_queue)
{
    p_queue->head = 0;
    p_queue->head = 0;
}

//计算数字个数
int queue_size(const MyQueue *p_queue)
{
    return (p_queue->tail - p_queue->head);
}

//判断队列是否为空
int queue_empty(const MyQueue *p_queue)
{
    return !(p_queue->tail - p_queue->head);
}

//判断队列是否满的
int queue_full(const MyQueue *p_queue)
{
    return p_queue->tail >= MyQUEUE_SIZE; //tail当吧最后一个SIZE-1使用后变为SIZE，为保险要大于
}

//向队列里加入数字
int queue_push(MyQueue *p_queue, int val)
{
    if (queue_full(p_queue))
    {
        return 0;
    }
    else
    {
        p_queue->arr[p_queue->tail] = val;
        p_queue->tail++;
        return 1; //表示将数字加进去了
    }
}

//从队列里获得数字的（会把数字从队列里删除）
int queue_pop(MyQueue *p_queue, int *p_num)
{
    if (queue_empty(p_queue))
    {
        return 0;
    }
    else
    {
        *p_num = p_queue->arr[p_queue->head]; //因为要删除，所以先给
        p_queue->head++;                      //将取过的数跳过去
        return 1;
    }
}

//从队列里获得数字（不会把数字从队列删除）
int queue_front(const MyQueue *p_queue, int *p_num)
{
    if (queue_empty(p_queue))
    {
        return 0;
    }
    else
    {
        *p_num = p_queue->arr[p_queue->head]; //多次调用是同一个数
        return 1;
    }
}
/**************** queue ********************/

/****************************************************************************/
int ConfigPDO()
{
    /********************/
    rt_printf("xenomai Configuring PDOs...\n");
    domainServoOutput = ecrt_master_create_domain(master);
    if (!domainServoOutput)
    {
        return -1;
    }
    domainServoInput = ecrt_master_create_domain(master);
    if (!domainServoInput)
    {
        return -1;
    }
    /********************/
    //servo motor
    rt_printf("xenomai Creating slave configurations...\n");
    for (int i = 0; i < servo_count; i++)
    {
        sc_servo[i] =
            ecrt_master_slave_config(master, ServoMotor, ASDAServoMotor);
        if (!sc_servo[i])
        {
            fprintf(stderr, "Failed to get servo motor %d configuration.\n", i);
            return -1;
        }
        if (ecrt_slave_config_pdos(sc_servo[i], EC_END, servo_syncs))
        {
            fprintf(stderr, "Failed to configure servo motor %d PDOs.\n", i);
            return -1;
        }

        ecrt_slave_config_sdo8(sc_servo[i], 0x60C2, 1, 1);                 //csp模式抑制抖动
        ecrt_slave_config_sdo8(sc_servo[i], 0x6098, 0, HomePointFindMode); //回零方式
        ecrt_slave_config_sdo32(sc_servo[i], 0x6099, 1, HomePointSwitchFindSpeed);
        ecrt_slave_config_sdo32(sc_servo[i], 0x6099, 2, HomePointFindSpeed);    //寻找原点速度
        ecrt_slave_config_sdo32(sc_servo[i], 0x609a, 0, HomePointAcceleration); //原点复归加速度
        ecrt_slave_config_sdo32(sc_servo[i], 0x607c, 0, HomePointOffset);       //原点偏移量
    }
    // VFD
    for (int i = 0; i < VFD_count; i++)
    {
        sc_VFD[i] = ecrt_master_slave_config(master, VFD, OMRONVFD);
        if (!sc_VFD[i])
        {
            fprintf(stderr, "Failed to get VFD %d configuration.\n", i);
            return -1;
        }
        if (ecrt_slave_config_pdos(sc_VFD[i], EC_END, vfd_syncs))
        {
            fprintf(stderr, "Failed to configure VFD %d PDOs.\n", i);
            return -1;
        }
    }
    // slave 1
    if (!(sc_dig_in_1 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_01_Pos, Beckhoff_EL1809)))
    {
        fprintf(stderr, "Failed to get slave 1 configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_in_1, EC_END, slave_1_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 3
    if (!(sc_dig_in_3 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_03_Pos, Beckhoff_EL1889)))
    {
        fprintf(stderr, "Failed to get slave 2 configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_in_3, EC_END, slave_3_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 5
    if (!(sc_dig_in_5 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_05_Pos, Beckhoff_EL1088)))
    {
        fprintf(stderr, "Failed to get slave 2 configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_in_5, EC_END, slave_5_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 7
    if (!(sc_dig_out_7 = ecrt_master_slave_config( //获取从站配置
              master, DigOutSlave_07_Pos, Beckhoff_EL2809)))
    {
        fprintf(stderr, "Failed to get slave 2 configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_out_7, EC_END, slave_7_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    /********************/
    if (ecrt_domain_reg_pdo_entry_list(domainServoOutput, domainServoOutput_regs))
    {
        fprintf(stderr, "PDOOutput entry registration failed!\n");
        return -1;
    }
    if (ecrt_domain_reg_pdo_entry_list(domainServoInput, domainServoInput_regs))
    {
        fprintf(stderr, "PDOInput entry registration failed!\n");
        return -1;
    }

    fprintf(stderr, "Creating SDO requests...\n");

    return 0;
}

void rt_check_domain_state(void)
{
    ec_domain_state_t dsInput = {};
    ec_domain_state_t dsOutput = {};
    //domainServoInput
    ecrt_domain_state(domainServoInput, &dsInput);
    if (dsInput.working_counter != domainServoInput_state.working_counter)
    {
        rt_printf("domainServoInput: WC %u.\n", dsInput.working_counter);
    }
    if (dsInput.wc_state != domainServoInput_state.wc_state)
    {
        rt_printf("domainServoInput: State %u.\n", dsInput.wc_state);
    }
    domainServoInput_state = dsInput;
    //domainServoOutput
    ecrt_domain_state(domainServoOutput, &dsOutput);
    if (dsOutput.working_counter != domainServoOutput_state.working_counter)
    {
        rt_printf("domainServoOutput: WC %u.\n", dsOutput.working_counter);
    }
    if (dsOutput.wc_state != domainServoOutput_state.wc_state)
    {
        rt_printf("domainServoOutput: State %u.\n", dsOutput.wc_state);
    }
    domainServoOutput_state = dsOutput;
}

void rt_check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
    {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

void check_slave_config_states(void)
{
    // servo motor
    for (int i = 0; i < servo_count; i++)
    {
        ec_slave_config_state_t ServoState;
        ecrt_slave_config_state(sc_servo[i], &ServoState);
        if (ServoState.al_state != sc_servo_state[i].al_state)
            rt_printf("servo %d: State 0x%02X.\n", i, ServoState.al_state);
        if (ServoState.online != sc_servo_state[i].online)
            rt_printf("servo %d: %s.\n", i, ServoState.online ? "online" : "offline");
        if (ServoState.operational != sc_servo_state[i].operational)
            rt_printf("servo %d: %soperational.\n", i, ServoState.operational ? "" : "Not ");
        sc_servo_state[i] = ServoState;
    }
    // VFD
    for (int i = 0; i < VFD_count; i++)
    {
        ec_slave_config_state_t VFDState;
        ecrt_slave_config_state(sc_VFD[i], &VFDState);
        if (VFDState.al_state != sc_VFD_state[i].al_state)
            rt_printf("servo %d: State 0x%02X.\n", i, VFDState.al_state);
        if (VFDState.online != sc_VFD_state[i].online)
            rt_printf("servo %d: %s.\n", i, VFDState.online ? "online" : "offline");
        if (VFDState.operational != sc_VFD_state[i].operational)
            rt_printf("servo %d: %soperational.\n", i, VFDState.operational ? "" : "Not ");
        sc_VFD_state[i] = VFDState;
    }
}

void ReleaseMaster()
{
    if (master)
    {
        rt_printf("xenomai End of Program, release master\n");
        ecrt_release_master(master);
        master = NULL;
    }
}

int ActivateMaster()
{
    int ret;
    rt_printf("xenomai Requesting master...\n");
    if (master)
    {
        return 0;
    }
    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    ConfigPDO();

    // configure SYNC signals for this slave
    for (int i = 0; i < servo_count; i++)
    {
        ecrt_slave_config_dc(sc_servo[i], 0x0300, 1000000, 0, 0, 0);
    }

    ecrt_master_application_time(master, system_time_ns());
    ret = ecrt_master_select_reference_clock(master, sc_servo[0]); //选择参考时钟
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }
    rt_printf("xenomai Activating master...\n");
    if (ecrt_master_activate(master))
    {
        rt_printf("xenomai Activating master...failed\n");
        return -1;
    }
    if (!(domainInput_pd = ecrt_domain_data(domainServoInput)))
    {
        fprintf(stderr, "xenomai Failed to get domain data pointer.\n");
        return -1;
    }
    if (!(domainOutput_pd = ecrt_domain_data(domainServoOutput)))
    {
        fprintf(stderr, "xenomai Failed to get domain data pointer.\n");
        return -1;
    }
    rt_printf("xenomai Activating master...success\n");
    return 0;
}

void RunCSP(int servo_index, int pos, int speed)
{
    int curpos = EC_READ_S32(domainInput_pd + ActualPosition[servo_index]);
    int abs_pos = 0;
    int abs_curpos = 0;

    abs_pos = abs(pos);
    abs_curpos = abs(curpos);
    if (abs(abs_pos - abs_curpos) > speed)
    {
        CSPMotorStatus[servo_index] = running;
    }
    if (curpos != pos)
    {
        if (curpos < pos)
        {
            curpos += speed;
            if (curpos > pos)
            {
                CSPMotorStatus[servo_index] = ok;
                CSPMotorSwitch[servo_index] = 0;
                curpos = pos;
            }
        }
        else
        {
            curpos -= speed;
            if (curpos < pos)
            {
                CSPMotorStatus[servo_index] = ok;
                CSPMotorSwitch[servo_index] = 0;
                curpos = pos;
            }
        }
        EC_WRITE_S32(domainOutput_pd + TargetPosition[servo_index], curpos);
    }
    CSPMotorSwitch[servo_index] = 1;
}

// int Servo_Keep(int Servo_Index)
// {
//     long int count = CSPPositionKeepTime / MilliSecond;
//     for (; count >= 0; count--)
//     {
//         RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
//         rt_task_sleep(MilliSecond);
//     }
//     return true;
// }

// void IPC_write()
// {
//     for (int list_index = 0; list_index < CASE_GO_TO_WHERE_DATA_BUFFER; ++list_index)
//     {
//         share_file_data[list_index * BIT_WIDTH + 0] = (case_go_to_where_list[list_index] / 100 + '0');
//         share_file_data[list_index * BIT_WIDTH + 1] = (case_go_to_where_list[list_index] / 10 % 10 + '0');
//         share_file_data[list_index * BIT_WIDTH + 2] = (case_go_to_where_list[list_index] % 100 % 10 + '0');
//     }
// }

void IPC_write(int case_to_index, char *content)
{
    for (int index = 0; index < strlen(content); index++)
    {
        share_file_data[case_to_index * BIT_WIDTH + index] = content[index];
    }
}

void IPC_read()
{
    for (int bit_index = 0; bit_index < LIST_MAXSIZE; bit_index += BIT_WIDTH)
    {
        case_go_to_where_list[bit_index / BIT_WIDTH] = (share_file_data[bit_index] - '0') * 100 + (share_file_data[bit_index + 1] - '0') * 10 + (share_file_data[bit_index + 2] - '0');
        //rt_printf("%d.....................%d\n", bit_index, share_file_data[bit_index]);
        rt_printf("%d\n", case_go_to_where_list[bit_index / BIT_WIDTH]);
    }
}

void IPC_init()
{
    if ((file_data = open(filepath, O_CREAT | O_RDWR, (mode_t)00700)) == -1)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }

    share_file_data = mmap(NULL, 12288, PROT_WRITE | PROT_READ, MAP_SHARED, file_data, 0);
    if (share_file_data == MAP_FAILED)
    {
        perror("mmap");
        exit(EXIT_FAILURE);
    }
}

void IPC_close()
{
    if (msync(share_file_data, 12288, MS_SYNC) == -1)
    {
        perror("Error sync to disk");
    }
    if (munmap(share_file_data, 12288) == -1)
    {
        close(file_data);
        perror("Error un-mmapping");
        exit(EXIT_FAILURE);
    }
    close(file_data);
}

int Servo_Move(int Servo_Index)
{
    if (!(Servo_Index >= servo_count) && csping_all_ready && CSPMotorSwitch[Servo_Index])
    {
        while (1)
        {
            RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
            if (CSPMotorStatus[Servo_Index] == ok)
            {
                break;
            }
            rt_task_sleep(MilliSecond);
        }
        return true;
    }
    return false;
}

int Servo_Back(int Servo_Index)
{
    if (!(Servo_Index >= servo_count) && csping_all_ready && CSPMotorSwitch[Servo_Index])
    {
        while (1)
        {
            RunCSP(Servo_Index, CSPHomePosition[Servo_Index], CSPSpeed[Servo_Index]);
            if (CSPMotorStatus[Servo_Index] == ok)
            {
                break;
            }
            rt_task_sleep(MilliSecond);
        }
        return true;
    }
    return false;
}

void DivertLaneTask(void *arg)
{
    int lane_index = *(int *)arg;
    while (run)
    {
        if (DivertLaneSwitch[lane_index] == 1)
        {
            rt_printf("Servo_Move\n");
            Servo_Move(lane_index);
            rt_printf("Servo_Keep\n");
            rt_task_sleep(CSPPositionKeepTime);
            rt_printf("Servo_Back\n");
            Servo_Back(lane_index);
            DivertLaneSwitch[lane_index] = 0;
        }
        if (csping_all_ready && CSPMotorSwitch[lane_index])
        {
            RunCSP(lane_index, CSPHomePosition[lane_index], CSPSpeed[lane_index]);
        }
        rt_task_sleep(MilliSecond);
    }
}

void VFDStateDisplay()
{
    for (int i = 0; i < VFD_count; i++)
    {
        if (!(cycle_counter % 1000))
        {
            rt_printf("                                                                                 ");
            rt_printf("VFD %d StatusWord = 0x%x\t", i, EC_READ_U16(domainInput_pd + VFDStatusWord[i]));
            rt_printf("VFD %d OutputFrequency = %d\n", i, EC_READ_U16(domainInput_pd + VFDOutputFrequency[i]));
        }
    }
}

void ServoStateDisplay()
{
    if (!(cycle_counter % 1000))
    {
        rt_printf("%d\t", cycle_counter);
        rt_printf("servo 0 CSPMotorStatus = %d\t", CSPMotorStatus[0]);
        rt_printf("mode = 0x%x\t", CurrentMode[0]);
        rt_printf("StatusWord = 0x%x\t", CurrentStatusWord[0]);
        rt_printf("ActualPosition = %d\n", MotorCurrentPosition[0]);
    }
}

int VFD_Stop(int VFD_Index)
{
    if (VFD_Index >= VFD_count)
    {
        return false;
    }
    else
    {
        VFDFreq[VFD_Index] = 0;
        if (VFDRunStatus[VFD_Index])
        {
            EC_WRITE_U16(domainOutput_pd + VFDFrequency[VFD_Index], VFDFreq[VFD_Index]);
            VFDRunStatus[VFD_Index] = 0;
        }
        return true;
    }
}

int VFD_Run(int VFD_Index, int Freq)
{
    if (VFD_Index >= VFD_count)
    {
        return false;
    }
    else
    {
        VFDFreq[VFD_Index] = Freq;
        if (!VFDRunStatus[VFD_Index] && !EC_READ_BIT(domainInput_pd + VFDStatusWord[VFD_Index], 3))
        {
            EC_WRITE_U16(domainOutput_pd + VFDCommand[VFD_Index], 0);
            EC_WRITE_U16(domainOutput_pd + VFDFrequency[VFD_Index], VFDFreq[VFD_Index]);
            EC_WRITE_U16(domainOutput_pd + VFDCommand[VFD_Index], VFDTurnAround);
            rt_printf("VFD %d RUN\n", VFD_Index);
            VFDRunStatus[VFD_Index] = 1;
        }
        else if (VFDRunStatus[VFD_Index])
        {
            EC_WRITE_U16(domainOutput_pd + VFDFrequency[VFD_Index], VFDFreq[VFD_Index]);
        }
        return true;
    }
}

void ServoErrorCapture()
{
    for (int i = 0; i < servo_count; i++)
    {
        CurrentMode[i] = EC_READ_U8(domainInput_pd + OPModeDisplay[i]);
        CurrentStatusWord[i] = EC_READ_U16(domainInput_pd + StatusWord[i]);
        MotorCurrentPosition[i] = EC_READ_S32(domainInput_pd + ActualPosition[i]);
        if (EC_READ_U16(domainInput_pd + ErrorCode[i]) != 0 || (CurrentStatusWord[i] & 0x0008))
        {
            for (int i = 0; i < servo_count; i++)
            {
                if (EC_READ_U16(domainInput_pd + ErrorCode[i]) != 0 || (CurrentStatusWord[i] & 0x0008))
                {
                    EC_WRITE_U16(domainOutput_pd + ControlWord[i], (EC_READ_U16(domainOutput_pd + ControlWord[i]) | 0x0080));
                }
                else
                {
                    EC_WRITE_U16(domainOutput_pd + ControlWord[i], (EC_READ_U16(domainOutput_pd + ControlWord[i]) & 0xff7f));
                }
            }

            if ((CurrentStatusWord[i] & 0x031) == 49)
            {
                if ((CurrentStatusWord[i] & 0x033) == 51)
                {
                    if ((CurrentStatusWord[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x1f);
                    }
                    else
                    {
                        EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x0f);
                    }
                }
                else
                {
                    EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x07);
                }
            }
        }
    }
}

void VFDErrorCapture()
{
    for (int i = 0; i < VFD_count; i++)
    {
        if (EC_READ_BIT(domainInput_pd + VFDStatusWord[i], 3))
        {
            EC_WRITE_BIT(domainOutput_pd + VFDCommand[i], 7, 1);
        }
    }
}

void DriverEtherCAT()
{
    static int csping[servo_count] = {0};
    static int home_param_setted = false;
    static Bool divert_lane_created = false;

    if (gSysRunning.m_gWorkStatusWord == SYS_WORKING_POWER_ON)
    {
        return;
    }

    cycle_counter++;

    ecrt_master_receive(master);
    ecrt_domain_process(domainServoOutput);
    ecrt_domain_process(domainServoInput);
    rt_check_domain_state();

    if (!(cycle_counter % 500))
    {
        rt_check_master_state();
        check_slave_config_states();
    }

    switch (gSysRunning.m_gWorkStatusWord)
    {
    case SYS_WORKING_SAFE_MODE:
    {
        rt_check_master_state();
        check_slave_config_states();
        if ((master_state.al_states & ETHERCAT_StatusWord_OP))
        {
            int tmp = true;
            for (int i = 0; i < servo_count; i++)
            {
                if (sc_servo_state[i].al_state != ETHERCAT_StatusWord_OP)
                {
                    tmp = false;
                    break;
                }
            }
            if (tmp)
            {
                gSysRunning.m_gWorkStatusWord = SYS_WORKING_OP_MODE;
                rt_printf("xenomai SYS_WORKING_OP_MODE\n");
            }
        }
    }
    break;

    case SYS_WORKING_OP_MODE:
    {
        if (!home_param_setted)
        {
            for (int i = 0; i < servo_count; i++)
            {
                EC_WRITE_S8(domainOutput_pd + OPMode[i], HM);
            }
            home_param_setted = true;
        }

        int tmp = false;
        for (int i = 0; i < servo_count; i++)
        {
            CurrentStatusWord[i] = EC_READ_U16(domainInput_pd + StatusWord[i]);
            EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x80); //清除故障
            if ((CurrentStatusWord[i] & 0x0250) == 592)
            {
                EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x06);
            }
            ServoErrorCapture();
            if ((CurrentStatusWord[i] & 0x031) == 49)
            {
                if ((CurrentStatusWord[i] & 0x033) == 51)
                {
                    if ((CurrentStatusWord[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x1f);
                        CurrentMode[i] = EC_READ_U8(domainInput_pd + OPModeDisplay[i]);
                        CurrentStatusWord[i] = EC_READ_U16(domainInput_pd + StatusWord[i]);
                        tmp = true;
                        if (CurrentStatusWord[i] == 5687)
                        {
                            tmp = false;
                            break;
                        }
                        if ((EC_READ_U16(domainInput_pd + StatusWord[i]) & (StatusWord_SERVO_ENABLE_BIT)) == 0)
                        {
                            tmp = false;
                            break;
                        }
                    }
                    else
                    {
                        EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x0f);
                    }
                }
                else
                {
                    EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x07);
                }
            }
        }
        if (tmp)
        {
            gSysRunning.m_gWorkStatusWord = SYS_WORKING_IDLE_StatusWord;
            rt_printf("xenomai SYS_WORKING_IDLE_StatusWord\n");
        }
    }
    break;

    default:
    {
        ServoErrorCapture();
        VFDErrorCapture();
        VFD_Run(0, 0x1388);
#if SHOW_LOG
        VFDStateDisplay();
        ServoStateDisplay();
#endif

        home_point_all_find = true;
        for (int i = 0; i < servo_count; i++)
        {
            if (!HomePointFinded[i] && ((CurrentStatusWord[i] >> 12) & 1) == 1)
            {
                rt_printf("Servo %d Homing Mode has been completed\n", i);
                HomePointFinded[i] = 1;
            }
            if (!HomePointFinded[i])
            {
                home_point_all_find = false;
            }
        }

        if (home_point_all_find && !csping_all_ready)
        {
            csping_all_ready = true;
            for (int i = 0; i < servo_count; i++)
            {
                if (!csping[i])
                {
                    csping_all_ready = false;

                    EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x07);
                    if (CurrentStatusWord[i] == 0x0233)
                    {
                        EC_WRITE_U8(domainOutput_pd + OPMode[i], CSP);
                        CurrentMode[i] = EC_READ_U8(domainInput_pd + OPModeDisplay[i]);
                        if (CurrentMode[i] == CSP)
                        {
                            csping[i] = 1;
                            EC_WRITE_U16(domainOutput_pd + ControlWord[i], 0x0f);
                            CSPMotorStatus[i] = wait; // 电机处于等待状态
                        }
                    }
                }
            }
        }
        if (csping_all_ready && !divert_lane_created)
        {
            for (int i = 0; i < servo_count; i++)
            {
                rt_task_create(&DivertLaneTaskList[i], "divert_lane_" + i, 0, 3, 0);
                rt_task_start(&DivertLaneTaskList[i], DivertLaneTask, &i);
                rt_printf("rt_task_create!\n");
            }
            divert_lane_created = true;
        }
    }
    break;
    }
    ecrt_master_application_time(master, system_time_ns());
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);
    ecrt_domain_queue(domainServoOutput);
    ecrt_domain_queue(domainServoInput);
    ecrt_master_send(master);
}

void LineshaftDiverterThread(void *arg)
{
    RTIME wait, previous;
    previous = rt_timer_read();
    wait = previous;

    while (run)
    {
        wait += MilliSecond; //1ms
        rt_task_sleep_until(wait);
        DriverEtherCAT();
    }
}

void signal_handler(int sig)
{
    run = 0;
}

void init_divert_data()
{
    int i;
    for (i = 0; i < DIVERT_LANE_COUNT; i++)
    {
        predict_trigger_count[i] = 0;
        real_trigger_count[i] = 0;
        need_divert_count[i] = 0;
        case_go_to_where_list[i] = 0;
        IPC_write(i, "0");
        queue_init(&need_divert_trigger_count_list[i]);
        queue_init(&need_divert_case_go_to_where_index_list[i]);
    }

    case_go_to_where_index = 0;
    rt_printf("init data success.\n");
}

void read_pe(void *arg)
{
    int cycle_counter = 0;
    rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns

    while (1)
    {
        rt_task_wait_period(NULL);

        cycle_counter++;

        // receive EtherCAT frames
        ecrt_master_receive(master);
        ecrt_domain_process(domainServoInput);
        ecrt_domain_process(domainServoOutput);

        rt_check_domain_state();

        if (!(cycle_counter % 1000))
        {
            rt_check_master_state();
        }

        // send_pe_trigger = EC_READ_BIT(domain1_pd + off_dig_in_1, 1);
        camera_pe_trigger = EC_READ_BIT(domainInput_pd + off_dig_in_1, 1);
        divert_lane_pe_trigger[0] = EC_READ_BIT(domainInput_pd + off_dig_in_1, 1);
        divert_success_pe_trigger[0] = EC_READ_BIT(domainInput_pd + off_dig_in_3, 0);
        divert_lane_full_pe_trigger[0] = EC_READ_BIT(domainInput_pd + off_dig_in_5, 0);
        // divert_success_pe_trigger[0] = EC_READ_BIT(domain1_pd + off_dig_in_3, 0);
        // divert_lane_pe_trigger[2] = EC_READ_BIT(domain1_pd + off_dig_in_5, 0);
        // for (i = 0; i < DIVERT_LANE_COUNT; i++)
        // {
        //     divert_lane_pe_trigger[i] = EC_READ_BIT(domain1_pd + off_dig_in_3, i);
        // }

        // divert_lane_full_pe_trigger[0] = EC_READ_BIT(domain1_pd + off_dig_in_5, 0);
        // divert_lane_pe_trigger[2] = EC_READ_BIT(domain1_pd + off_dig_in_3, 0);

        // divert_lane_full_flag[0] = 0;
        // divert_lane_full_flag[2] = 1;
        // rt_printf("%d\n", send_pe_trigger);

        // send process data
        ecrt_domain_queue(domainServoInput);
        ecrt_domain_queue(domainServoOutput);
        ecrt_master_send(master);
    }
}

void process_divert_data(int go_to_where)
{ // go_to_where: 1, 2, 3, 4, 5
    int lane_index = go_to_where - 1;
    // predict trigger count
    for (int i = 0; i < go_to_where; i++)
    {
        predict_trigger_count[i]++;
    }
    printf("predict_trigger_count: %u,%u,%u\n", predict_trigger_count[0], predict_trigger_count[1], predict_trigger_count[2]);

    // init need divert buffer
    need_divert_count[lane_index]++;
    printf("need_divert_count: %u,%u,%u\n", need_divert_count[0], need_divert_count[1], need_divert_count[2]);

    // init need divert & need divert list
    queue_push(&need_divert_trigger_count_list[lane_index], predict_trigger_count[lane_index]);
    queue_push(&need_divert_case_go_to_where_index_list[lane_index], case_go_to_where_index);

    // init case go to where
    case_go_to_where_list[case_go_to_where_index] = go_to_where;
    sprintf(tempchar, "%d", go_to_where);
    IPC_write(case_go_to_where_index, tempchar);
    case_go_to_where_index++;
    rt_printf("case_go_to_where_list: %d, case_go_to_where_index: %d\n", case_go_to_where_list[case_go_to_where_index - 1], case_go_to_where_index);
}

void vision_box_handler()
{
    int has_trigger = 0;
    int go_to_where_rand = 0;
    while (run)
    {
        rt_task_sleep(0.01 * SECOND);
        if (has_trigger == 0 && camera_pe_trigger == 1)
        {
            has_trigger = 1;
            rt_printf("camera_pe_trigger\n");
            go_to_where_rand = rand() % 4;
            go_to_where_rand = 1;
            process_divert_data(go_to_where_rand);
            rt_printf("case will go to %d\n", go_to_where_rand);
        }

        if (has_trigger == 1 && camera_pe_trigger == 0)
        {
            has_trigger = 0;
        }
    }
}

void send_pe_handler()
{
    RTIME send_pe_trigger_time = 0;
    RTIME send_pe_release_time = 0;
    RTIME temp_trigger_time = 0;

    unsigned int has_trigger = 0;
    unsigned int has_case_passed = 0;

    long long int trigger_time_inner = 0;
    long long int trigger_time_outer = 0;
    long long int case_filter_time = 0;

    while (run)
    {
        rt_task_sleep(0.01 * SECOND);
        if (has_trigger == 0 && send_pe_trigger == 1)
        {
            rt_printf("send pe trigger\n");
            temp_trigger_time = rt_timer_read();
            has_trigger = 1;
            if (has_case_passed)
            {
                rt_printf("case passed\n");

                trigger_time_outer = temp_trigger_time - send_pe_trigger_time;
                trigger_time_inner = temp_trigger_time - send_pe_release_time;
                rt_printf("trigger_time_outer: %16lld\n", trigger_time_outer);
                rt_printf("trigger_time_inner: %16lld\n", trigger_time_inner);

                if (trigger_time_outer < GAP_TIME_OUT)
                {
                    rt_printf("上上沿 < Time\n");
                }

                if (trigger_time_inner < GAP_TIME_INNER)
                {
                    rt_printf("下上沿 < Time\n");
                }

                // if (case_go_where(n) >= case_go_where(n - 1) && (temp_trigger_time - send_pe_trigger_time) < GAP_TIME_OUT) { // 上上沿 < Time && N >= N-1
                //     front_case_recycle(n);
                // }

                // if (case_go_where(n) < case_go_where(n - 1) && (temp_trigger_time - send_pe_release_time) < GAP_TIME_INNER) { // 下上沿 < Time && N < N-1
                //     case_recycle();
                //     if (temp_trigger_time - send_pe_trigger_time < GAP_TIME_OUT) {
                //         front_case_recycle();
                //     }
                // }
            }
        }

        if (has_trigger == 1 && send_pe_trigger == 0)
        {
            has_trigger = 0;
            rt_printf("send pe release\n");
            case_filter_time = rt_timer_read() - temp_trigger_time;
            rt_printf("case_filter_time: %16lld\n", case_filter_time);
            if (case_filter_time < CASE_FILTER_TIME_MIN)
            {
                rt_printf("not case passed\n");
                has_case_passed = 0;
                continue;
            }
            else if (case_filter_time > CASE_FILTER_TIME_MAX)
            {
                rt_printf("cases side by side\n");
                has_case_passed = 1;
                continue;
            }
            else
            {
                send_pe_release_time = rt_timer_read();
                send_pe_trigger_time = temp_trigger_time;
                rt_printf("send_pe_trigger_time: %16llu, send_pe_release_time: %16llu\n", send_pe_trigger_time, send_pe_release_time);
                has_case_passed = 1;
            }
        }
    }
}

void divert_pe_handler()
{
    RTIME case_filter_time = 0;
    int trigger_flag[DIVERT_LANE_COUNT] = {0};
    int i;
    int queue_pop_int = 0;
    while (run)
    {
        IPC_read();
        //rt_printf("%d\n", case_go_to_where_list[79]);
        if (case_go_to_where_list[79] == 999)
        {
            camera_pe_trigger = 1;
            divert_lane_pe_trigger[0] = 1;
        }

        rt_task_sleep(0.01 * SECOND);
        for (i = 0; i < DIVERT_LANE_COUNT; i++)
        {
            if (divert_lane_has_trigger[i] == 0 && divert_lane_pe_trigger[i] == 1)
            {
                divert_lane_has_trigger[i] = 1;
                divert_pe_trigger_time[i] = rt_timer_read();
                rt_printf("========================================\n");
                rt_printf("[divert pe %d is triggering] divert_lane_has_trigger[%d] = 1, divert_pe_trigger_time[%d]= %lld, trigger_flag[%d] = 1\n", i + 1, i, i, divert_pe_trigger_time[i], i);
            }

            if (trigger_flag[i] == 0 && divert_lane_has_trigger[i] == 1 && divert_lane_pe_trigger[i] == 1)
            {
                case_filter_time = rt_timer_read() - divert_pe_trigger_time[i];
                if (case_filter_time > CASE_FILTER_TIME_MIN)
                {
                    real_trigger_count[i]++;
                    trigger_flag[i] = 1;
                    rt_printf("is case pass, case_filter_time= %lld, real_trigger_count[%d] = %d, trigger_flag[%d] = 1\n", case_filter_time, i, real_trigger_count[i], i);
                }
            }

            if (divert_lane_has_trigger[i] == 1 && divert_lane_pe_trigger[i] == 0)
            {
                divert_lane_has_trigger[i] = 0;
                trigger_flag[i] = 0;
                rt_printf("[divert pe %d is release] trigger_flag[%d] = 0, trigger_flag[%d] = 0;\n", i + 1, i, i);

                if (i == (DIVERT_LANE_COUNT - 1))
                { // recycle success
                    need_divert_count[DIVERT_LANE_COUNT - 1]--;
                    queue_pop(&need_divert_trigger_count_list[DIVERT_LANE_COUNT - 1], &queue_pop_int);
                    queue_pop(&need_divert_case_go_to_where_index_list[DIVERT_LANE_COUNT - 1], &queue_pop_int);
                    rt_printf("Recycle success, lane %d\n", case_go_to_where_list[queue_pop_int]);
                    case_go_to_where_list[queue_pop_int] = RECYCLE_SUCCESS;
                    IPC_write(queue_pop_int, "277");
                }
            }
        }
    }
}

void divert(int i)
{
    DivertLaneSwitch[i] = 1;
    rt_printf("Lane %d diverting...\n", i);
}

void divert_case_handler()
{
    int current_case_go_to_where_index = 0;
    int last_case_go_to_where_index = 0;

    int queue_pop_case_go_to_where_index = 0;
    int queue_pop_trigger_count = 0;
    int trigger_count = 0;
    while (run)
    {
        rt_task_sleep(0.01 * SECOND);
        for (int i = 0; i < (DIVERT_LANE_COUNT - 1); i++)
        {
            if (need_divert_count[i] <= 0 || is_divert_flag[i])
            {
                continue;
            }

            rt_printf("is empty: %d\n", queue_empty(&need_divert_trigger_count_list[i]));
            queue_front(&need_divert_trigger_count_list[i], &trigger_count);
            rt_printf("???: %d, trigger_count: %d\n", i + 1, trigger_count);
            if (trigger_count != 0 && trigger_count == real_trigger_count[i])
            {
                if (!divert_lane_full_flag[i])
                {
                    // rt_printf("Can divert to lane %d\n", i + 1);
                    // rt_printf("need trigger_count: %d, real_trigger_count: %d\n", trigger_count, real_trigger_count[i]);
                    is_divert_flag[i] = 1;
                    divert(i);
                }
                else
                {
                    rt_printf("Divert lane %d full\n", i + 1);
                    queue_front(&need_divert_case_go_to_where_index_list[i], &current_case_go_to_where_index);

                    for (int j = i + 1; j < (DIVERT_LANE_COUNT); j++)
                    {
                        predict_trigger_count[j]++;

                        for (int m = 0; m < need_divert_count[j]; m++)
                        {
                            queue_pop(&need_divert_case_go_to_where_index_list[i], &last_case_go_to_where_index);
                            queue_pop(&need_divert_trigger_count_list[j], &queue_pop_trigger_count);

                            if (current_case_go_to_where_index < last_case_go_to_where_index || (case_go_to_where_index < current_case_go_to_where_index && ((1 < last_case_go_to_where_index && last_case_go_to_where_index < case_go_to_where_index) || (case_go_to_where_index < last_case_go_to_where_index && last_case_go_to_where_index < CASE_GO_TO_WHERE_DATA_BUFFER))))
                            {
                                queue_pop_trigger_count++;
                            }
                            queue_push(&need_divert_case_go_to_where_index_list[j], last_case_go_to_where_index);
                            queue_push(&need_divert_trigger_count_list[j], queue_pop_trigger_count);
                        }
                    }

                    // 清除去Lane i的数据
                    queue_pop(&need_divert_trigger_count_list[i], &queue_pop_trigger_count);
                    queue_pop(&need_divert_case_go_to_where_index_list[i], &queue_pop_case_go_to_where_index);
                    case_go_to_where_list[queue_pop_case_go_to_where_index] = DIVERT_LANE_FULL;
                    IPC_write(queue_pop_case_go_to_where_index, "226");
                    need_divert_count[i]--;
                }
            }
        }
    }
}

void divert_success_pe_handler()
{
    RTIME filter_time = 0;
    int queue_pop_int = 0;

    while (run)
    {
        rt_task_sleep(0.01 * SECOND);
        for (int i = 0; i < (DIVERT_LANE_COUNT - 1); i++)
        {
            if (!is_divert_flag[i])
            {
                continue;
            }

            filter_time = rt_timer_read() - divert_pe_trigger_time[i];
            if (filter_time >= DIVERT_RESULT_TIME)
            { // failed
                rt_printf("Divert failed, lane %d\n", case_go_to_where_list[queue_pop_int]);
                rt_printf("-------- time: %16llu ns ------\n", filter_time);
                init_divert_data();
                is_divert_flag[i] = 0;
                continue;
            }

            if (divert_success_has_trigger[i] == 0 && divert_success_pe_trigger[i] == 1)
            {
                need_divert_count[i]--;
                queue_pop(&need_divert_trigger_count_list[i], &queue_pop_int);
                queue_pop(&need_divert_case_go_to_where_index_list[i], &queue_pop_int);
                rt_printf("Divert success, lane %d\n", case_go_to_where_list[queue_pop_int]);
                case_go_to_where_list[queue_pop_int] = DIVERT_SUCCESS;
                IPC_write(queue_pop_int, "0");

                divert_success_has_trigger[i] = 1;
                is_divert_flag[i] = 0;
                rt_printf("-------- time: %16llu ns ------\n", filter_time);
            }

            if (divert_success_has_trigger[i] == 1 && divert_success_pe_trigger[i] == 0)
            {
                divert_success_has_trigger[i] = 0;
            }
        }
    }
}

void divert_lane_full_pe_handler()
{
    // RTIME pe_trigger_time[DIVERT_LANE_COUNT] = {0};
    RTIME lane_full_filter_time = 0;
    while (run)
    {
        rt_task_sleep(0.01 * SECOND);
        for (int i = 0; i < DIVERT_LANE_COUNT; i++)
        {
            if (divert_lane_full_has_trigger[i] == 0 && divert_lane_full_pe_trigger[i] == 1)
            {
                divert_lane_full_has_trigger[i] = 1;
                divert_lane_full_pe_trigger_time[i] = rt_timer_read();
                rt_printf("lane %d full pe trigger\n", i + 1);
            }

            if (divert_lane_full_flag[i] == 0 && divert_lane_full_has_trigger[i] == 1 && divert_lane_full_pe_trigger[i] == 1)
            {
                lane_full_filter_time = rt_timer_read() - divert_lane_full_pe_trigger_time[i];
                if (lane_full_filter_time > LANE_FULL_TIME)
                {
                    divert_lane_full_flag[i] = 1;
                    rt_printf("Lane %d full, %16lld ns\n", i + 1, lane_full_filter_time);
                }
            }

            if (divert_lane_full_has_trigger[i] == 1 && divert_lane_full_pe_trigger[i] == 0)
            {
                divert_lane_full_has_trigger[i] = 0;
                divert_lane_full_flag[i] = 0;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    RT_TASK LineshaftDiverterTask;
    RT_TASK read_pe_task;
    RT_TASK vision_box_handler_task;
    RT_TASK send_pe_handler_task;
    RT_TASK divert_pe_handler_task;
    RT_TASK divert_case_handler_task;
    RT_TASK divert_success_pe_handler_task;
    RT_TASK divert_lane_full_pe_handler_task;

    rt_print_auto_init(1);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    gSysRunning.m_gWorkStatusWord = SYS_WORKING_POWER_ON;
    if (gSysRunning.m_gWorkStatusWord == SYS_WORKING_POWER_ON)
    {
        ActivateMaster();
        gSysRunning.m_gWorkStatusWord = SYS_WORKING_SAFE_MODE;
        rt_printf("xenomai SYS_WORKING_SAFE_MODE\n");
    }

    rt_printf("Started...\n");
    IPC_init();
    init_divert_data();

    rt_printf("Create rt task...\n");
    rt_task_create(&LineshaftDiverterTask, "LineshaftDiverterTask", 0, 1, T_FPU);
    rt_task_create(&read_pe_task, "read_pe_task", 0, 1, 0);
    rt_task_create(&vision_box_handler_task, "vision_box_handler", 0, 3, 0);
    rt_task_create(&send_pe_handler_task, "send_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_pe_handler_task, "divert_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_case_handler_task, "divert_case_handler_task", 0, 2, 0);
    rt_task_create(&divert_success_pe_handler_task, "divert_success_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_lane_full_pe_handler_task, "divert_lane_full_pe_handler_task", 0, 3, 0);

    rt_task_start(&LineshaftDiverterTask, LineshaftDiverterThread, NULL);
    rt_task_start(&read_pe_task, read_pe, NULL);
    rt_task_start(&vision_box_handler_task, vision_box_handler, NULL);
    rt_task_start(&send_pe_handler_task, send_pe_handler, NULL);
    rt_task_start(&divert_pe_handler_task, divert_pe_handler, NULL);
    rt_task_start(&divert_case_handler_task, divert_case_handler, NULL);
    rt_task_start(&divert_success_pe_handler_task, divert_success_pe_handler, NULL);
    rt_task_start(&divert_lane_full_pe_handler_task, divert_lane_full_pe_handler, NULL);

    while (run)
    {
        rt_task_sleep(50000000);
    }

    IPC_close();
    rt_printf("Inter-Process Communication closed...\n");
    rt_printf("Deleting realtime task...\n");
    rt_task_delete(&LineshaftDiverterTask);
    for (int i = 0; i <= servo_count; i++)
    {
        rt_task_delete(&DivertLaneTaskList[i]);
    }
    rt_task_delete(&read_pe_task);
    rt_task_delete(&send_pe_handler_task);
    rt_task_delete(&divert_pe_handler_task);
    rt_task_delete(&divert_case_handler_task);
    rt_task_delete(&divert_success_pe_handler_task);
    rt_task_delete(&vision_box_handler_task);
    rt_task_delete(&divert_lane_full_pe_handler_task);

    ReleaseMaster();
    return 0;
}
