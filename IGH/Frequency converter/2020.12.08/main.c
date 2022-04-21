//gcc test.c -o test.out -I/opt/etherlab/include -L/opt/etherlab/lib/ -lethercat

// #include<stdio.h>
// #include<stdlib.h>
// #include <errno.h>
// #include <signal.h>
// #include <string.h>
// #include <sys/resource.h>
// #include <sys/time.h>
// #include <sys/types.h>
// #include <unistd.h>
// #include "ecrt.h" // 应用程序接口头文件ecrt.h可以在内核和用户上下文中使用。
// #include <sys/mman.h>
// // Xenomai Alchemy Api
// #include <alchemy/task.h>
// #include <alchemy/timer.h>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>
#include <math.h>

#include "ecrt.h"


/****************** LineshaftDiverter Start **************************/
#define Bool int
#define false 0
#define true 1
#define ETHERCAT_StatusWord_OP 0x08
#define StatusWord_SERVO_ENABLE_BIT (0x04)
#define CSPPositionKeepTime 50000000 //到达目标位置保持时间 50ms
#define MilliSecond 1000000

#define servo_count 1 //电机数量
#define VFD_count 1   //变频器数量
/****************** LineshaftDiverter End **************************/
/****************************************************************************/
// Application parameters
#define FREQUENCY 100 //控制数据发送频率
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS 1

/****************************************************************************/
static unsigned int counter = 0;

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_in_1 = NULL;
static ec_slave_config_t *sc_dig_in_3 = NULL;
static ec_slave_config_t *sc_dig_in_5 = NULL;
static ec_slave_config_t *sc_dig_out_7 = NULL;

/****************** LineshaftDiverter Start **************************/
static ec_slave_config_t *sc_servo[servo_count];
static ec_slave_config_state_t sc_servo_state[servo_count];

static ec_slave_config_t *sc_VFD[VFD_count];
static ec_slave_config_state_t sc_VFD_state[VFD_count];
/****************** LineshaftDiverter End **************************/

// Timer
static unsigned int sig_alarms = 0;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

// alias,position
#define ServoMotor 0, 0
#define VFD 0, 1
#define BusCouplerPos 0, 2
#define DigInSlave_01_Pos 0, 3 //lane 0 divert pe
#define DigInSlave_03_Pos 0, 7 //lane 0 divert success pe
#define DigInSlave_05_Pos 0, 9 //lane 0 divert full pe
#define DigOutSlave_07_Pos 0, 5

// vendor ID,product code
#define Beckhoff_EK1100 0x00000002, 0x044c2c52 // Salve 0
#define Beckhoff_EL1809 0x00000002, 0x07113052 // Salve 1
#define Beckhoff_EL1889 0x00000002, 0x07613052 // Salve 3
#define Beckhoff_EL1088 0x00000002, 0x04403052 // Salve 5
#define Beckhoff_EL2809 0x00000002, 0x0af93052 // Salve 7 Dig Out
#define ASDAServoMotor 0x000001dd, 0x10305070
#define OMRONVFD 0x00000083, 0x00000053

/****************** LineshaftDiverter Start **************************/
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
/****************** LineshaftDiverter End **************************/
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
// offsets for PDO entries
static unsigned int off_dig_in_1;
static unsigned int off_dig_in_3;
static unsigned int off_dig_in_5;
static unsigned int off_dig_out_7;
/****************** LineshaftDiverter Start **************************/
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

int home_point_all_find = false;
int csping_all_ready = false;
RT_TASK DivertLaneTaskList[servo_count];
int CSPMotorSwitch[servo_count] = {1};         //csp模式下电机转动开关
int HomePointFinded[servo_count] = {0};        //原点复归标志位
int MotorCurrentPosition[servo_count] = {0};   //电机当前位置
int CSPMotorStatus[servo_count] = {0};         //csp模式下电机运行状态
int CSPTargetPosition[servo_count] = {630000}; //每个电机摆动位置
int CSPHomePosition[servo_count] = {300000};   //每个电机前进方向位置
int CSPSpeed[servo_count] = {5000};            //csp转动速率
int ServoFaultReset = 0;                       //伺服Fault复位标志位
int DivertLaneSwitch[servo_count] = {1};       //所有Divert Lane对应斜轮是否分拣开关

int HomePointFindMode = 33;         //回零方式
int HomePointSwitchFindSpeed = 500; //寻找原点开关速度
int HomePointFindSpeed = 200;       //寻找原点速度
int HomePointAcceleration = 800;    //原点复归加速度
int HomePointOffset = 0;            //原点复归偏移量

int VFDRunStatus[VFD_count] = {0}; //变频器是否转动
int VFDFreq[VFD_count] = {5000};
int VFDTurnAround = 2; //变频器旋转方向
/****************** LineshaftDiverter End **************************/

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DigInSlave_01_Pos, Beckhoff_EL1809, 0x6000, 1, &off_dig_in_1},   // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {DigInSlave_03_Pos, Beckhoff_EL1889, 0x6000, 1, &off_dig_in_3},   // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {DigInSlave_05_Pos, Beckhoff_EL1088, 0x6000, 1, &off_dig_in_5},   // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {DigOutSlave_07_Pos, Beckhoff_EL2809, 0x7000, 1, &off_dig_out_7}, // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {ServoMotor, ASDAServoMotor, 0x6040, 0x00, &ControlWord[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x607a, 0x00, &TargetPosition[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6060, 0x00, &OPMode[0], NULL},
    {VFD, OMRONVFD, 0x5000, 0x00, &VFDCommand[0], NULL},
    {VFD, OMRONVFD, 0x5010, 0x00, &VFDFrequency[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6064, 0x00, &ActualPosition[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6041, 0x00, &StatusWord[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6061, 0x00, &OPModeDisplay[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x603f, 0x00, &ErrorCode[0], NULL},
    {VFD, OMRONVFD, 0x5100, 0x00, &VFDStatusWord[0], NULL},
    {VFD, OMRONVFD, 0x5110, 0x00, &VFDOutputFrequency[0], NULL},
    {}};

/*****************************************************************************/

#if CONFIGURE_PDOS
/****************** LineshaftDiverter Start **************************/
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
/****************** LineshaftDiverter End **************************/
/* Master 0, Slave 1, "EL1809"
 * Vendor ID:       0x00000002
 * Product code:    0x07113052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
    {0x6080, 0x01, 1}, /* Input */
    {0x6090, 0x01, 1}, /* Input */
    {0x60a0, 0x01, 1}, /* Input */
    {0x60b0, 0x01, 1}, /* Input */
    {0x60c0, 0x01, 1}, /* Input */
    {0x60d0, 0x01, 1}, /* Input */
    {0x60e0, 0x01, 1}, /* Input */
    {0x60f0, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1a00, 1, slave_1_pdo_entries + 0},  /* Channel 1 */
    {0x1a01, 1, slave_1_pdo_entries + 1},  /* Channel 2 */
    {0x1a02, 1, slave_1_pdo_entries + 2},  /* Channel 3 */
    {0x1a03, 1, slave_1_pdo_entries + 3},  /* Channel 4 */
    {0x1a04, 1, slave_1_pdo_entries + 4},  /* Channel 5 */
    {0x1a05, 1, slave_1_pdo_entries + 5},  /* Channel 6 */
    {0x1a06, 1, slave_1_pdo_entries + 6},  /* Channel 7 */
    {0x1a07, 1, slave_1_pdo_entries + 7},  /* Channel 8 */
    {0x1a08, 1, slave_1_pdo_entries + 8},  /* Channel 9 */
    {0x1a09, 1, slave_1_pdo_entries + 9},  /* Channel 10 */
    {0x1a0a, 1, slave_1_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_1_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_1_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_1_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_1_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_1_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 16, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 3, "EL1889"
 * Vendor ID:       0x00000002
 * Product code:    0x07613052
 * Revision number: 0x00130000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
    {0x6080, 0x01, 1}, /* Input */
    {0x6090, 0x01, 1}, /* Input */
    {0x60a0, 0x01, 1}, /* Input */
    {0x60b0, 0x01, 1}, /* Input */
    {0x60c0, 0x01, 1}, /* Input */
    {0x60d0, 0x01, 1}, /* Input */
    {0x60e0, 0x01, 1}, /* Input */
    {0x60f0, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1a00, 1, slave_3_pdo_entries + 0},  /* Channel 1 */
    {0x1a01, 1, slave_3_pdo_entries + 1},  /* Channel 2 */
    {0x1a02, 1, slave_3_pdo_entries + 2},  /* Channel 3 */
    {0x1a03, 1, slave_3_pdo_entries + 3},  /* Channel 4 */
    {0x1a04, 1, slave_3_pdo_entries + 4},  /* Channel 5 */
    {0x1a05, 1, slave_3_pdo_entries + 5},  /* Channel 6 */
    {0x1a06, 1, slave_3_pdo_entries + 6},  /* Channel 7 */
    {0x1a07, 1, slave_3_pdo_entries + 7},  /* Channel 8 */
    {0x1a08, 1, slave_3_pdo_entries + 8},  /* Channel 9 */
    {0x1a09, 1, slave_3_pdo_entries + 9},  /* Channel 10 */
    {0x1a0a, 1, slave_3_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_3_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_3_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_3_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_3_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_3_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_INPUT, 16, slave_3_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 5, "EL1088"
 * Vendor ID:       0x00000002
 * Product code:    0x04403052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1a00, 1, slave_5_pdo_entries + 0}, /* Channel 1 */
    {0x1a01, 1, slave_5_pdo_entries + 1}, /* Channel 2 */
    {0x1a02, 1, slave_5_pdo_entries + 2}, /* Channel 3 */
    {0x1a03, 1, slave_5_pdo_entries + 3}, /* Channel 4 */
    {0x1a04, 1, slave_5_pdo_entries + 4}, /* Channel 5 */
    {0x1a05, 1, slave_5_pdo_entries + 5}, /* Channel 6 */
    {0x1a06, 1, slave_5_pdo_entries + 6}, /* Channel 7 */
    {0x1a07, 1, slave_5_pdo_entries + 7}, /* Channel 8 */
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_INPUT, 8, slave_5_pdos + 0, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 7, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_7_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
    {0x7040, 0x01, 1}, /* Output */
    {0x7050, 0x01, 1}, /* Output */
    {0x7060, 0x01, 1}, /* Output */
    {0x7070, 0x01, 1}, /* Output */
    {0x7080, 0x01, 1}, /* Output */
    {0x7090, 0x01, 1}, /* Output */
    {0x70a0, 0x01, 1}, /* Output */
    {0x70b0, 0x01, 1}, /* Output */
    {0x70c0, 0x01, 1}, /* Output */
    {0x70d0, 0x01, 1}, /* Output */
    {0x70e0, 0x01, 1}, /* Output */
    {0x70f0, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_7_pdos[] = {
    {0x1600, 1, slave_7_pdo_entries + 0},  /* Channel 1 */
    {0x1601, 1, slave_7_pdo_entries + 1},  /* Channel 2 */
    {0x1602, 1, slave_7_pdo_entries + 2},  /* Channel 3 */
    {0x1603, 1, slave_7_pdo_entries + 3},  /* Channel 4 */
    {0x1604, 1, slave_7_pdo_entries + 4},  /* Channel 5 */
    {0x1605, 1, slave_7_pdo_entries + 5},  /* Channel 6 */
    {0x1606, 1, slave_7_pdo_entries + 6},  /* Channel 7 */
    {0x1607, 1, slave_7_pdo_entries + 7},  /* Channel 8 */
    {0x1608, 1, slave_7_pdo_entries + 8},  /* Channel 9 */
    {0x1609, 1, slave_7_pdo_entries + 9},  /* Channel 10 */
    {0x160a, 1, slave_7_pdo_entries + 10}, /* Channel 11 */
    {0x160b, 1, slave_7_pdo_entries + 11}, /* Channel 12 */
    {0x160c, 1, slave_7_pdo_entries + 12}, /* Channel 13 */
    {0x160d, 1, slave_7_pdo_entries + 13}, /* Channel 14 */
    {0x160e, 1, slave_7_pdo_entries + 14}, /* Channel 15 */
    {0x160f, 1, slave_7_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_7_syncs[] = {
    {0, EC_DIR_OUTPUT, 16, slave_7_pdos + 0, EC_WD_ENABLE},
    {0xff}};

#endif

#if SDO_ACCESS
static ec_sdo_request_t *sdo;
#endif

/*****************************************************************************/
int config_pdos()
{
    rt_printf("Xenomai: Configuring PDOs...\n");
    ec_slave_config_t *sc; // 存放从站配置的结构体

    rt_printf("Create Domain...\n");
    domain1 = ecrt_master_create_domain(master); // 创建新的进程数据域
    if (!domain1)
        return -1;

    rt_printf("Configuring PDOs...\n");
    /****************** LineshaftDiverter Start **************************/
    //servo motor
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
    /****************** LineshaftDiverter End **************************/
    // slave 1
    if (!(sc_dig_in_1 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_01_Pos, Beckhoff_EL1809)))
    {                                                              //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_in_1, EC_END, slave_1_syncs))
    { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 3
    if (!(sc_dig_in_3 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_03_Pos, Beckhoff_EL1889)))
    {                                                              //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 2 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_in_3, EC_END, slave_3_syncs))
    { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 5
    if (!(sc_dig_in_5 = ecrt_master_slave_config( //获取从站配置
              master, DigInSlave_05_Pos, Beckhoff_EL1088)))
    {                                                              //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 2 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_in_5, EC_END, slave_5_syncs))
    { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 7
    if (!(sc_dig_out_7 = ecrt_master_slave_config( //获取从站配置
              master, DigOutSlave_07_Pos, Beckhoff_EL2809)))
    {                                                              //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 2 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_out_7, EC_END, slave_7_syncs))
    { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc)
        return -1;

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    { //为进程数据域注册一组PDO项。参数一：创建的进程数据域，参数二：pdo注册数组
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    /****************** LineshaftDiverter Start **************************/
    // configure SYNC signals for this slave
    for (int i = 0; i < servo_count; i++)
    {
        ecrt_slave_config_dc(sc_servo[i], 0x0300, 1000000, 0, 0, 0);
    }
    // ecrt_slave_config_dc(sc_VFD, 0x0300, 1000000, 0, 0, 0);

    ecrt_master_application_time(master, system_time_ns());
    int ret = ecrt_master_select_reference_clock(master, sc_servo[0]); //选择参考时钟
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }
    /****************** LineshaftDiverter End **************************/

    rt_printf("Activating master...\n");
    if (ecrt_master_activate(master)) // 激活主站
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    { //返回域的进程数据
        return -1;
    }
    return 0;
}

int activate_master()
{
    rt_printf("Xenomai: Request Master...\n");
    if (master)
        return 0;
    master = ecrt_request_master(0); // 请求EtherCAT主机进行实时操作。取得master设备
    if (!master)
        return -1;
    config_pdos();
    return 0;
}

void release_master()
{
    if (master)
    {
        printf("End of Program, release master\n");
        ecrt_release_master(master);
        master = NULL;
    }
}

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
    {
        // rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state)
    {
        // rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        // rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states)
    {
        // rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up)
    {
        // rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

#if SDO_ACCESS
void read_sdo(void)
{
    switch (ecrt_sdo_request_state(sdo))
    {
    case EC_REQUEST_UNUSED:         // request was not used yet
        ecrt_sdo_request_read(sdo); // trigger first read
        break;
    case EC_REQUEST_BUSY:
        fprintf(stderr, "Still busy...\n");
        break;
    case EC_REQUEST_SUCCESS:
        fprintf(stderr, "SDO value: 0x%04X\n",
                EC_READ_U16(ecrt_sdo_request_data(sdo)));
        ecrt_sdo_request_read(sdo); // trigger next read
        break;
    case EC_REQUEST_ERROR:
        fprintf(stderr, "Failed to read SDO!\n");
        ecrt_sdo_request_read(sdo); // retry reading
        break;
    }
}
#endif

/*****************************************************************************/

/**************** queue ********************/
#define MyQUEUE_SIZE 100
/*
* 队列演示（队列的几个函数）
* */
typedef struct
{
    int arr[MyQUEUE_SIZE];
    int head; //记录最前面数字所在的下标
    int tail; //记录最后一个有效数字的下一个坐标
              //如果队列里一个数都没有的话head=tail
} MyQueue;

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

/****************** divert logic **************************/
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


int need_divert;
int need_divert_pop;
int last_case_go_to_where;
unsigned int case_go_to_where;
unsigned int predict_trigger_count[DIVERT_LANE_COUNT] = {0};
unsigned int real_trigger_count[DIVERT_LANE_COUNT] = {0};
unsigned int need_divert_count[DIVERT_LANE_COUNT] = {0};

MyQueue need_divert_trigger_count_list[DIVERT_LANE_COUNT];
MyQueue need_divert_case_go_to_where_index_list[DIVERT_LANE_COUNT];

int case_go_to_where_list[CASE_GO_TO_WHERE_DATA_BUFFER] = {0};
int case_go_to_where_index = 0;

// MyQueue need_divert_queue_lane_1;
// MyQueue need_divert_queue_lane_2;
// MyQueue need_divert_queue_lane_3;

// int need_divert_queue_i;
// for (need_divert_queue_i = 0; need_divert_queue_i < DIVERT_LANE_COUNT; need_divert_queue_i++) {
//     need_divert_queue_list[i] =
// }

// PE
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



void init_divert_data()
{
    int i;
    for (i = 0; i < DIVERT_LANE_COUNT; i++)
    {
        predict_trigger_count[i] = 0;
        real_trigger_count[i] = 0;
        need_divert_count[i] = 0;
        case_go_to_where_list[i] = 0;
        queue_init(&need_divert_trigger_count_list[i]);
        queue_init(&need_divert_case_go_to_where_index_list[i]);
    }

    case_go_to_where_index = 0;
    rt_printf("init data success.\n");
}

void process_divert_data(int go_to_where)
{ // go_to_where: 1, 2, 3, 4, 5

    int lane_index = go_to_where - 1;

    // predict trigger count
    int i;
    for (i = 0; i < go_to_where; i++)
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
    case_go_to_where_index++;
    rt_printf("case_go_to_where_list: %d, case_go_to_where_index: %d\n", case_go_to_where_list[case_go_to_where_index - 1], case_go_to_where_index);
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
        ecrt_domain_process(domain1);

        rt_check_domain_state();

        if (!(cycle_counter % 1000))
        {
            rt_check_master_state();
        }

        // send_pe_trigger = EC_READ_BIT(domain1_pd + off_dig_in_1, 1);
        camera_pe_trigger = EC_READ_BIT(domain1_pd + off_dig_in_1, 1);
        divert_lane_pe_trigger[0] = EC_READ_BIT(domain1_pd + off_dig_in_1, 1);
        divert_success_pe_trigger[0] = EC_READ_BIT(domain1_pd + off_dig_in_3, 0);
        divert_lane_full_pe_trigger[0] = EC_READ_BIT(domain1_pd + off_dig_in_5, 0);
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
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
}

// void front_case_recycle(unsigned int n) {
//     if (0 < case_go_where(n) < DIVERT_LANE_COUNT) {
//         int temp_need_divert;
//         int need_divert;
//         unsigned int i;
//         for (i = case_go_where(n); i <= DIVERT_LANE_COUNT; i++) {
//             predict_trigger_count[i]++;
//         }

//         if (case_go_where(n) > case_go_where(n - 1)) {
//             need_divert_count[case_go_where(n)]++;
//             queue_pop(&need_divert_queue_list[case_go_where(n - 1)], &need_divert);
//             // queue_pop(&need_divert_list_queue_list[case_go_where(n - 1)], &need_divert);
//         } else
//         {
//             queue_pop(&need_divert_queue_list[case_go_where(n - 1)], &temp_need_divert);
//             queue_pop(&need_divert_queue_list[case_go_where(n - 1)], &need_divert);
//             queue_pop(&need_divert_queue_list[case_go_where(n - 1)],&temp_need_divert);
//             // queue_pop(&need_divert_list_queue_list[case_go_where(n - 1)], &need_divert);
//         }

//         // buffer[case_go_where(n - 1)]--;
//         case_go_where(n - 1) = 228;
//     }
// }

// void case_recycle() {
//     if (0 < case_go_where(n) < DIVERT_LANE_COUNT) {
//         int need_divert;
//         unsigned int i;
//         for (i = case_go_where(n); i <= DIVERT_LANE_COUNT; i++) {
//             predict_trigger_count[i]++;
//         }

//         queue_pop(&need_divert_queue_list[case_go_where(n)], &need_divert);
//         // queue_pop(&need_divert_list_queue_list[case_go_where(n)], &need_divert);
//         // buffer[case_go_where(n)]--;
//         case_go_where(n) = 228;
//     }
// }

void vision_box_handler()
{

    int has_trigger = 0;
    int go_to_where_rand = 0;
    while (1)
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

    while (1)
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
    int queue_pop_int;
    while (1)
    {
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
                    need_divert_count[DIVERT_LANE_COUNT]--;
                    queue_pop(&need_divert_trigger_count_list[DIVERT_LANE_COUNT], &queue_pop_int);
                    queue_pop(&need_divert_case_go_to_where_index_list[DIVERT_LANE_COUNT], &queue_pop_int);
                    rt_printf("Recycle success, lane %d\n", case_go_to_where_list[queue_pop_int]);
                    case_go_to_where_list[queue_pop_int] = RECYCLE_SUCCESS;
                }
            }
        }
    }
}

// 0 ~ DIVERT_LANE_COUNT - 1
void divert_success_pe_handler()
{
    RTIME filter_time = 0;
    int queue_pop_int;

    int i = 0;
    while (1)
    {
        rt_task_sleep(0.01 * SECOND);
        for (i = 0; i < (DIVERT_LANE_COUNT - 1); i++)
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

// 0 ~ DIVERT_LANE_COUNT - 1
void divert_case_handler()
{
    int i = 0;
    int j = 0;
    int last_case_go_to_where = 0;
    int current_case_go_to_where_index = 0;
    int last_case_go_to_where_index = 0;

    int queue_pop_case_go_to_where_index = 0;
    int queue_pop_trigger_count = 0;
    int trigger_count = 0;
    while (1)
    {
        rt_task_sleep(0.01 * SECOND);
        for (i = 0; i < (DIVERT_LANE_COUNT - 1); i++)
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

                    j = i;
                    for (j = i + 1; j < (DIVERT_LANE_COUNT); j++)
                    {
                        predict_trigger_count[j]++;

                        int m;
                        for (m = 0; m < need_divert_count[j]; m++)
                        {
                            queue_pop(&need_divert_case_go_to_where_index_list[i], &last_case_go_to_where_index);
                            queue_pop(&need_divert_trigger_count_list[j], &queue_pop_trigger_count);

                            if (current_case_go_to_where_index < last_case_go_to_where_index || (case_go_to_where_index < current_case_go_to_where_index && (1 < last_case_go_to_where_index < case_go_to_where_index || case_go_to_where_index < last_case_go_to_where_index < CASE_GO_TO_WHERE_DATA_BUFFER)))
                            {
                                queue_pop_trigger_count++;
                            }
                            queue_push(&need_divert_case_go_to_where_index_list[j], &last_case_go_to_where_index);
                            queue_push(&need_divert_trigger_count_list[j], &queue_pop_trigger_count);
                        }
                    }

                    // 清除去Lane i的数据
                    queue_pop(&need_divert_trigger_count_list[i], &queue_pop_trigger_count);
                    queue_pop(&need_divert_case_go_to_where_index_list[i], &queue_pop_case_go_to_where_index);
                    case_go_to_where_list[queue_pop_case_go_to_where_index] = DIVERT_LANE_FULL;
                    need_divert_count[i]--;
                }
            }
        }
    }
}

void divert_lane_full_pe_handler()
{
    // RTIME pe_trigger_time[DIVERT_LANE_COUNT] = {0};
    RTIME lane_full_filter_time = 0;
    int i;
    while (1)
    {
        rt_task_sleep(0.01 * SECOND);
        for (i = 0; i < DIVERT_LANE_COUNT; i++)
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

void ServoErrorCapture()
{
    for (int i = 0; i < servo_count; i++)
    {
        CurrentMode[i] = EC_READ_U8(domain1_pd + OPModeDisplay[i]);
        CurrentStatusWord[i] = EC_READ_U16(domain1_pd + StatusWord[i]);
        MotorCurrentPosition[i] = EC_READ_S32(domain1_pd + ActualPosition[i]);
        if (EC_READ_U16(domain1_pd + ErrorCode[i]) != 0 || (CurrentStatusWord[i] & 0x0008))
        {
            // rt_printf("servo %d fault\n", i);
            for (int i = 0; i < servo_count; i++)
            {
                if (EC_READ_U16(domain1_pd + ErrorCode[i]) != 0 || (CurrentStatusWord[i] & 0x0008))
                {
                    EC_WRITE_U16(domain1_pd + ControlWord[i], (EC_READ_U16(domain1_pd + ControlWord[i]) | 0x0080));
                }
                else
                {
                    EC_WRITE_U16(domain1_pd + ControlWord[i], (EC_READ_U16(domain1_pd + ControlWord[i]) & 0xff7f));
                }
            }

            if ((CurrentStatusWord[i] & 0x031) == 49)
            {
                if ((CurrentStatusWord[i] & 0x033) == 51)
                {
                    if ((CurrentStatusWord[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domain1_pd + ControlWord[i], 0x1f);
                    }
                    else
                    {
                        EC_WRITE_U16(domain1_pd + ControlWord[i], 0x0f);
                    }
                }
                else
                {
                    EC_WRITE_U16(domain1_pd + ControlWord[i], 0x07);
                }
            }
            // rt_printf("servo %d fault reset.\n", i);
        }
    }
}

void VFDErrorCapture()
{
    for (int i = 0; i < VFD_count; i++)
    {
        // if (EC_READ_U16(domainInput_pd + VFDStatusWord[i]) & 0x8)
        // {
        //     EC_WRITE_U16(domainOutput_pd + VFDCommand[i], 0x80);
        //     // rt_printf("VFD %d fault reset\n", i);
        // }
        if (EC_READ_BIT(domain1_pd + VFDStatusWord[i], 3))
        {
            EC_WRITE_BIT(domain1_pd + VFDCommand[i], 7, 1);
            // rt_printf("VFD %d fault reset\n", i);
        }
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
        if (!VFDRunStatus[VFD_Index] && !EC_READ_BIT(domain1_pd + VFDStatusWord[VFD_Index], 3))
        {
            EC_WRITE_U16(domain1_pd + VFDCommand[VFD_Index], 0);
            EC_WRITE_U16(domain1_pd + VFDFrequency[VFD_Index], VFDFreq[VFD_Index]);
            EC_WRITE_U16(domain1_pd + VFDCommand[VFD_Index], VFDTurnAround);
            rt_printf("VFD %d RUN\n", VFD_Index);
            VFDRunStatus[VFD_Index] = 1;
        }
        else if (VFDRunStatus[VFD_Index])
        {
            EC_WRITE_U16(domain1_pd + VFDFrequency[VFD_Index], VFDFreq[VFD_Index]);
        }
        return true;
    }
}

void VFDStateDisplay()
{
    for (int i = 0; i < VFD_count; i++)
    {
        if (!(cycle_counter % 1000))
        {
            rt_printf("                                                                                 ");
            rt_printf("VFD %d StatusWord = 0x%x\t", i, EC_READ_U16(domain1_pd + VFDStatusWord[i]));
            rt_printf("VFD %d OutputFrequency = %d\n", i, EC_READ_U16(domain1_pd + VFDOutputFrequency[i]));
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

void RunCSP(int servo_index, int pos, int speed)
{
    int curpos = EC_READ_S32(domain1_pd + ActualPosition[servo_index]);
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
            rt_printf("\ncurpos == %d~~~~~~~~~~~~~~~~~~~~~~~~~~~~~pos == %d\n", curpos, pos);
            if (curpos > pos)
            {
                rt_printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@@@@@@@@@@@@@@@@@@@@@@@@@\n");
                CSPMotorStatus[servo_index] = ok;
                CSPMotorSwitch[servo_index] = 0;
                curpos = pos;
            }
        }
        else
        {
            curpos -= speed;
            rt_printf("\ncurpos == %d~~~~~~~~~~~~~~~~~~~~~~~~~~~~~pos == %d\n", curpos, pos);
            if (curpos < pos)
            {
                rt_printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@@@@@@@@@@@@@@@@@@@@@@@@@\n");
                CSPMotorStatus[servo_index] = ok;
                CSPMotorSwitch[servo_index] = 0;
                curpos = pos;
            }
        }
    }
    EC_WRITE_S32(domain1_pd + TargetPosition[servo_index], curpos);
    
}

int Servo_Keep(int Servo_Index)
{
    long int count = CSPPositionKeepTime / MilliSecond;
    for (; count >= 0; count--)
    {
        RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
        rt_task_sleep(MilliSecond);
    }
    return true;
}

int Servo_Move(int Servo_Index)
{
    // rt_printf("\ncsping_all_ready=%d--CSPMotorSwitch[Servo_Index]=%d--\n", csping_all_ready, CSPMotorSwitch[Servo_Index]);

    if (!(Servo_Index >= servo_count) && csping_all_ready && CSPMotorSwitch[Servo_Index])
    {
        while (1)
        {
            RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
            // rt_printf("\nCSPMotorStatus[Servo_Index] == %d\n", CSPMotorStatus[Servo_Index]);
            if (CSPMotorStatus[Servo_Index] == ok)
            {
                rt_printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@@@@@@@@@@@@@@@@@@@@@@@@@\n");
                // rt_printf("CSPMotorStatus[Servo_Index] == ok");
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
        // rt_printf("\n--------------CSPMotorStatus\n");
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
    RTIME wait, previous;
    previous = rt_timer_read();
    wait = previous;
    while (run)
    {
        wait += MilliSecond; //1ms
        rt_task_sleep_until(wait);
        rt_printf("DivertLaneSwitch = = %d\n",DivertLaneSwitch[lane_index]);
        if (DivertLaneSwitch[lane_index] == 1)
        {
            Servo_Move(lane_index);
            Servo_Keep(lane_index);
            Servo_Back(lane_index);
            DivertLaneSwitch[lane_index] = 1;
            rt_printf("DivertLaneSwitch___________________________\n");
            
        }
        if (csping_all_ready && CSPMotorSwitch[lane_index])
        {
            RunCSP(lane_index, CSPHomePosition[lane_index], CSPSpeed[lane_index]);
        }
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
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
    ecrt_domain_process(domain1);
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
                EC_WRITE_S8(domain1_pd + OPMode[i], HM);
            }
            home_param_setted = true;
        }

        int tmp = false;
        for (int i = 0; i < servo_count; i++)
        {
            CurrentStatusWord[i] = EC_READ_U16(domain1_pd + StatusWord[i]);
            // rt_printf("servo %d StatusWord = 0x%x\n", i, CurrentStatusWord[i]);
            // rt_printf("servo %d ErrorCode = 0x%x\n", i, EC_READ_U16(domainInput_pd + ErrorCode[i]));
            EC_WRITE_U16(domain1_pd + ControlWord[i], 0x80); //清除故障
            if ((CurrentStatusWord[i] & 0x0250) == 592)
            {
                EC_WRITE_U16(domain1_pd + ControlWord[i], 0x06);
            }
            ServoErrorCapture();
            if ((CurrentStatusWord[i] & 0x031) == 49)
            {
                if ((CurrentStatusWord[i] & 0x033) == 51)
                {
                    if ((CurrentStatusWord[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domain1_pd + ControlWord[i], 0x1f);
                        // rt_printf("servo %d enable success\n", i);
                        CurrentMode[i] = EC_READ_U8(domain1_pd + OPModeDisplay[i]);
                        // rt_printf("servo %d mode = 0x%x\n", i, CurrentMode[i]);
                        CurrentStatusWord[i] = EC_READ_U16(domain1_pd + StatusWord[i]);
                        // rt_printf("servo %d StatusWord = 0x%x\n", i, CurrentStatusWord[i]);

                        tmp = true;
                        if (CurrentStatusWord[i] == 5687)
                        {
                            tmp = false;
                            break;
                        }
                        if ((EC_READ_U16(domain1_pd + StatusWord[i]) & (StatusWord_SERVO_ENABLE_BIT)) == 0)
                        {
                            tmp = false;
                            break;
                        }
                    }
                    else
                    {
                        EC_WRITE_U16(domain1_pd + ControlWord[i], 0x0f);
                    }
                }
                else
                {
                    EC_WRITE_U16(domain1_pd + ControlWord[i], 0x07);
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
        // VFD_Run(0, 0x1388);
        VFDStateDisplay();
        ServoStateDisplay();

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

                    EC_WRITE_U16(domain1_pd + ControlWord[i], 0x07);
                    if (CurrentStatusWord[i] == 0x0233)
                    {
                        EC_WRITE_U8(domain1_pd + OPMode[i], CSP);
                        CurrentMode[i] = EC_READ_U8(domain1_pd + OPModeDisplay[i]);
                        if (CurrentMode[i] == CSP)
                        {
                            csping[i] = 1;
                            EC_WRITE_U16(domain1_pd + ControlWord[i], 0x0f);
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
                rt_printf("divert_lane_%d task create\n", i);
            }
            divert_lane_created = true;
        }
    }
    break;
    }
    ecrt_master_application_time(master, system_time_ns());
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

void divert(int i)
{
    DivertLaneSwitch[i] = 1;
    rt_printf("Lane %d diverting...\n", i);
}

void LineshaftDiverterThread()
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

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************/

int main(int argc, char **argv)
{
    rt_print_auto_init(1);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_printf("Starting activate master.....\n");
    gSysRunning.m_gWorkStatusWord = SYS_WORKING_POWER_ON;
    if (gSysRunning.m_gWorkStatusWord == SYS_WORKING_POWER_ON)
    {
        activate_master();
        gSysRunning.m_gWorkStatusWord = SYS_WORKING_SAFE_MODE;
        rt_printf("xenomai SYS_WORKING_SAFE_MODE\n");
    }

    rt_printf("Started...\n");
    init_divert_data();

    rt_printf("Create rt task...\n");
    RT_TASK LineshaftDiverterTask;
    RT_TASK read_pe_task;
    RT_TASK vision_box_handler_task;
    RT_TASK send_pe_handler_task;
    RT_TASK divert_pe_handler_task;
    RT_TASK divert_case_handler_task;
    RT_TASK divert_success_pe_handler_task;
    RT_TASK divert_lane_full_pe_handler_task;

    rt_task_create(&read_pe_task, "read_pe_task", 0, 1, 0);
    rt_task_create(&vision_box_handler_task, "vision_box_handler", 0, 3, 0);
    rt_task_create(&send_pe_handler_task, "send_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_pe_handler_task, "divert_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_case_handler_task, "divert_case_handler_task", 0, 2, 0);
    rt_task_create(&divert_success_pe_handler_task, "divert_success_pe_handler_task", 0, 3, 0);
    rt_task_create(&divert_lane_full_pe_handler_task, "divert_lane_full_pe_handler_task", 0, 3, 0);
    rt_task_create(&LineshaftDiverterTask, "LineshaftDiverterTask", 0, 1, T_FPU);

    rt_task_start(&read_pe_task, read_pe, NULL);
    rt_task_start(&vision_box_handler_task, vision_box_handler, NULL);
    rt_task_start(&send_pe_handler_task, send_pe_handler, NULL);
    rt_task_start(&divert_pe_handler_task, divert_pe_handler, NULL);
    rt_task_start(&divert_case_handler_task, divert_case_handler, NULL);
    rt_task_start(&divert_success_pe_handler_task, divert_success_pe_handler, NULL);
    rt_task_start(&divert_lane_full_pe_handler_task, divert_lane_full_pe_handler, NULL);
    rt_task_start(&LineshaftDiverterTask, &LineshaftDiverterThread, NULL);

    while (run)
    {
        sched_yield();
    }

    rt_printf("Deleting realtime task...\n");
    rt_task_delete(&read_pe_task);
    rt_task_delete(&send_pe_handler_task);
    rt_task_delete(&divert_pe_handler_task);
    rt_task_delete(&divert_case_handler_task);
    rt_task_delete(&divert_success_pe_handler_task);
    rt_task_delete(&vision_box_handler_task);
    rt_task_delete(&divert_lane_full_pe_handler_task);
    rt_task_delete(&LineshaftDiverterTask);
    for (int i = 0; i <= servo_count; i++)
    {
        rt_task_delete(&DivertLaneTaskList[i]);
    }

    release_master();
    return 0;
}

/****************************************************************************/

/**
先定义一些结构体，其中有存放从站配置的结构体，类型为ec_slave_config_t{}，用在接收函数ecrt_master_slave_config（）的返回值；用在描述信号到达时要采取的操作的结构体struct sigaction{}；计算时间，用于定时的结构体struct itimerval{}。

下面是关于主站和从站的一些配置
1. 请求EtherCAT主机进行操作，我感觉可以认为是创建一个主站实例对象。
2. 创建进程数据域。
3. 获取从站配置
4. 指定完整的pdo配置
5. 为进程数据域注册一组PDO项。
6. 激活主站
7. 返回域的进程数据
8. 可选选项：设置进程优先级
9. 可选选项：指定进程收到某种信号采取的操作
10. 可选选项：设置定时发送信号
11. 进入循环，开始循环任务

其中对于几个可选选项，设置优先级，是为了让实时性更好；另外两个定时发送的信号作为进程采取操作的条件，而操作内容又作为循环任务的周期时间间隔，不对，这两个并不再是可选项，应该必须要设置的，周期交换数据的时间间隔要大于等于从站处理 数据和传输等时间总和。

下面是在循环任务中做的事情：
1. 从硬件获取接收的帧并处理数据报。
2. 确定域数据报的状态。
3. 检查域的状态（可选项）
4. 周期性检查主站状态和从站配置状态（可选项）
5. 计算将要发送流水灯数据和数据改变周期大小
6. 将数据放入数据域
7. 将主数据报队列中的所有域数据报排队
8. 发送队列中的所有数据报。


其中对于配置PDO所涉及到的几个结构体
typedef struct {
    uint16_t index; /**< PDO entry index. 
    uint8_t subindex; /**< PDO entry subindex. 
    uint8_t bit_length; /**< Size of the PDO entry in bit. 
} ec_pdo_entry_info_t;


typedef struct {
    uint16_t index; /**< PDO index. 
    unsigned int n_entries; /**< Number of PDO entries in entries to map.
                              Zero means, that the default mapping shall be
                              used (this can only be done if the slave is
                              present at bus configuration time). 
    ec_pdo_entry_info_t *entries; /**< Array of PDO entries to map. Can 														either be NULL, or must contain at
                                    least n_entries values. 
} ec_pdo_info_t;


typedef struct {
    uint8_t index; /**< Sync manager index. Must be less
                     than #EC_MAX_SYNC_MANAGERS for a valid sync manager,
                     but can also be 0xff to mark the end of the list. 
    ec_direction_t dir; /**< Sync manager direction. 
    unsigned int n_pdos; /**< Number of PDOs in pdos.
    ec_pdo_info_t *pdos; /**< Array with PDOs to assign. This must contain
                            at least n_pdos PDOs. 
    ec_watchdog_mode_t watchdog_mode; /**< Watchdog mode. 
} ec_sync_info_t;


typedef struct {
    uint16_t alias; /**< Slave alias address. 
    uint16_t position; /**< Slave position. 
    uint32_t vendor_id; /**< Slave vendor ID. 
    uint32_t product_code; /**< Slave product code. 
    uint16_t index; /**< PDO entry index. 
    uint8_t subindex; /**< PDO entry subindex. 
    unsigned int *offset; /**< Pointer to a variable to store the PDO 					entry's(byte-)offset in the process data. 
    unsigned int *bit_position; /**< Pointer to a variable to store a bit
                                  position (0-7) within the offset. Can be
                                  NULL, in which case an error is raised if 					 							  the PDO entry does not byte-align. 
} ec_pdo_entry_reg_t;


#define EC_WRITE_U8(DATA, VAL) \
    do { \
        *((uint8_t *)(DATA)) = ((uint8_t) (VAL)); \
    } while (0)
*/
