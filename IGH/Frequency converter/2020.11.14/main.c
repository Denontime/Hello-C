#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
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

#include "ecrt.h"
#define Bool int
#define false 0
#define true 1
#define ETHERCAT_STATUS_OP 0x08
#define STATUS_SERVO_ENABLE_BIT (0x04)

#define drive_count 1//电机数量
int csp_drive_enable[drive_count] = {0};       //csp模式下电机转动开关
int home_point_find[drive_count] = {0};        //原点复归标志位
int drive_current_position[drive_count] = {0}; // 电机当前位置
int csp_running[drive_count] = {0};            //csp模式下电机运行状态
int csp_pos[drive_count] = {0};                //目标位置
int csp_speed[drive_count] = {3000};           //csp转动速率
int ServoFaultReset = 0;                       //伺服Fault复位标志位

int ECTFrequency = 0x1388;

//drive status in csp
typedef enum _DriveStatusCSP
{
    wait,
    running,
    ok //到达位置
} DriveStatusCSP;
//master status
typedef enum _SysWorkingStatus
{
    SYS_WORKING_POWER_ON,
    SYS_WORKING_SAFE_MODE,
    SYS_WORKING_OP_MODE,
    SYS_WORKING_LINK_DOWN,
    SYS_WORKING_IDLE_STATUS //系统空闲
} SysWorkingStatus;

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
    SysWorkingStatus m_gWorkStatus;
} GSysRunningParm;

GSysRunningParm gSysRunning;

RT_TASK InterpolationTask;

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

static ec_slave_config_t *sc_asda[drive_count];
static ec_slave_config_state_t sc_asda_state[drive_count];

static ec_slave_config_t *sc_dig_in_1 = NULL;
static ec_slave_config_state_t sc_dig_in_1_state = {};

static ec_slave_config_t *sc_dig_out_3 = NULL;
static ec_slave_config_state_t sc_dig_out_3_state = {};

static ec_slave_config_t *sc_MX2ECT = NULL;
static ec_slave_config_state_t sc_MX2ECT_state = {}; ///////////////******/////////////

static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

/****************************************************************************/
#define asda_Pos0 0, 0
#define BusCouplerPos  0, 1	
#define DigInSlave_01_Pos 0, 2
#define DigOutSlave_03_Pos 0, 4
#define MX2ECT_Pos 0, 5

/*Vendor ID  * Product code */
#define asda 0x000001dd, 0x10305070
//#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL1809 0x00000002, 0x07113052
#define Beckhoff_EL2809 0x00000002, 0x0af93052
#define MX2ECT 0x00000083, 0x00000053

// offsets for PDO entries
static unsigned int cntlwd[drive_count];                     //控制字
static unsigned int ipData[drive_count];                     //目标位置
static unsigned int status[drive_count];                     //状态字
static unsigned int actpos[drive_count];                     //当前回授位置
static unsigned int modes_of_operation[drive_count];         //6060
static unsigned int modes_of_operation_display[drive_count]; //6061
static unsigned int errcode[drive_count];                    //错误代码

static unsigned int off_dig_in_1;
static unsigned int off_dig_out_3;

static unsigned int cur_mode[drive_count];
static unsigned int cur_status[drive_count];

static unsigned int ECT_Command;
static unsigned int ECT_Frequency;

static unsigned int ECT_Status;
static unsigned int ECT_Output_frequency;
// process data
ec_pdo_entry_reg_t domainServoOutput_regs[] = {
    {asda_Pos0, asda, 0x6040, 0x00, &cntlwd[0], NULL},
    {asda_Pos0, asda, 0x607a, 0x00, &ipData[0], NULL},
    {asda_Pos0, asda, 0x6060, 0x00, &modes_of_operation[0], NULL},
    {DigOutSlave_03_Pos, Beckhoff_EL2809, 0x7050, 1, &off_dig_out_3, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5000, 0x00, &ECT_Command, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5010, 0x00, &ECT_Frequency, NULL},
    {}};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {asda_Pos0, asda, 0x6064, 0x00, &actpos[0], NULL},
    {asda_Pos0, asda, 0x6041, 0x00, &status[0], NULL},
    {asda_Pos0, asda, 0x6061, 0x00, &modes_of_operation_display[0], NULL},
    {asda_Pos0, asda, 0x603f, 0x00, &errcode[0], NULL},
    {DigInSlave_01_Pos, Beckhoff_EL1809, 0x6020, 1, &off_dig_in_1, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5100, 0x00, &ECT_Status, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5110, 0x00, &ECT_Output_frequency, NULL},
    {}};
/****************************************************************************/
/* Master 0, Slave 0
 * Vendor ID:       0x000001dd
 * Product code:    0x10305070
 * Revision number: 0x02040608
 */
//《 Delta_ASDA2-E_rev4-00_XML_TSE_20160620.xml》
static ec_pdo_entry_info_t asda_pdo_entries_output[] = {
    {0x6040, 0x00, 16}, //control word
    {0x607a, 0x00, 32}, //TargetPosition
    {0x6060, 0x00, 8},  //modes_of_operation
};

static ec_pdo_entry_info_t asda_pdo_entries_input[] = {
    {0x6041, 0x00, 16}, //status word
    {0x6064, 0x00, 32}, //actualPosition
    {0x6061, 0x00, 8},  /*modes of operation display*/
    {0x603f, 0x00, 16},  /*error code*/
};

static ec_pdo_info_t asda_pdo[] = {
    //RxPDO
    {0x1600, 3, asda_pdo_entries_output},
    //TxPDO
    {0x1a00, 4, asda_pdo_entries_input},
};

static ec_sync_info_t asda_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, asda_pdo + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, asda_pdo + 1, EC_WD_DISABLE},
    {0xff}};


/* Master 0, Slave 2, "EL1809"
 * Vendor ID:       0x00000002
 * Product code:    0x07113052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x6020, 0x01, 1}, /* Input  0*/
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1a02, 1, slave_2_pdo_entries}, /* Channel 3 */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_INPUT, 1, slave_2_pdos + 0, EC_WD_DISABLE},
    {0xff}
};
    
/* Master 0, Slave 3, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x7050, 0x01, 1}, /* Output 0000*/
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1605, 1, slave_3_pdo_entries}, /* Channel 6 */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_ENABLE},
    {0xff}
};

/* Master 0, Slave 5, "3G3AX-MX2-ECT"
 * Vendor ID:       0x00000083
 * Product code:    0x00000053
 * Revision number: 0x00010001
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x5000, 0x00, 16}, /* Command */
    {0x5010, 0x00, 16}, /* Frequency reference */
    {0x5100, 0x00, 16}, /* Status */
    {0x5110, 0x00, 16}, /* Output frequency monitor */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1701, 2, slave_5_pdo_entries + 0}, /* 258th receive PDO Mapping */
    {0x1b01, 2, slave_5_pdo_entries + 2}, /* 258th transmit PDO Mapping */
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_5_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_5_pdos + 1, EC_WD_DISABLE},
    {0xff}};

/****************************************************************************/
int ConfigPDO()
{
    /********************/
    printf("xenomai Configuring PDOs...\n");
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
    //slave 0
    printf("xenomai Creating slave configurations...\n");
    sc_asda[0] =
        ecrt_master_slave_config(master, asda_Pos0, asda);
    if (!sc_asda[0])
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_asda[0], EC_END, asda_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 2
    if (!(sc_dig_in_1 = ecrt_master_slave_config(//获取从站配置
                    master, DigInSlave_01_Pos, Beckhoff_EL1809))) {//第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_in_1, EC_END, slave_2_syncs)) {//指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 3
    if (!(sc_dig_out_3 = ecrt_master_slave_config(//获取从站配置
                    master, DigOutSlave_03_Pos, Beckhoff_EL2809))) {//第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
    fprintf(stderr, "Failed to get slave 2 configuration.\n"); // 第二个参数应该是从站的位置
    return -1;
    }
    if (ecrt_slave_config_pdos(sc_dig_out_3, EC_END, slave_3_syncs)) {//指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // slave 5
    if (!(sc_MX2ECT = ecrt_master_slave_config( //获取从站配置
                master, MX2ECT_Pos, MX2ECT)))
    {                                                              //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_MX2ECT, EC_END, slave_5_syncs))
    { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);


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
    /********************/
    for (int i = 0; i < drive_count; i++)
    {
        ecrt_slave_config_sdo8(sc_asda[i], 0x60C2, 1, 1); //csp模式抑制抖动

        ecrt_slave_config_sdo8(sc_asda[i], 0x6098, 0, 33); //回零方式
        ecrt_slave_config_sdo32(sc_asda[i], 0x6099, 1, 500);
        ecrt_slave_config_sdo32(sc_asda[i], 0x6099, 2, 200); //寻找原点速度
        ecrt_slave_config_sdo32(sc_asda[i], 0x609a, 0, 800); //原点复归加速度
        ecrt_slave_config_sdo32(sc_asda[i], 0x607c, 0, 0);   //原点偏移量
    }

    fprintf(stderr, "Creating SDO requests...\n");

    return 0;
}

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};
    ec_domain_state_t ds1 = {};
    //domainServoInput
    ecrt_domain_state(domainServoInput, &ds);
    if (ds.working_counter != domainServoInput_state.working_counter)
    {
        rt_printf("domainServoInput: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainServoInput_state.wc_state)
    {
        rt_printf("domainServoInput: State %u.\n", ds.wc_state);
    }
    domainServoInput_state = ds;
    //domainServoOutput
    ecrt_domain_state(domainServoOutput, &ds1);
    if (ds1.working_counter != domainServoOutput_state.working_counter)
    {
        rt_printf("domainServoOutput: WC %u.\n", ds1.working_counter);
    }
    if (ds1.wc_state != domainServoOutput_state.wc_state)
    {
        rt_printf("domainServoOutput: State %u.\n", ds1.wc_state);
    }
    domainServoOutput_state = ds1;
}

/****************************************************************************/

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

/****************************************************************************/
void check_slave_config_states(void)
{
    // slave 1
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_asda[0], &s);
    if (s.al_state != sc_asda_state[0].al_state)
        printf("sc_asda_state: State 0x%02X.\n", s.al_state);
    if (s.online != sc_asda_state[0].online)
        printf("sc_asda_state: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_asda_state[0].operational)
        printf("sc_asda_state: %soperational.\n", s.operational ? "" : "Not ");
    sc_asda_state[0] = s;

    // slave 5

}
/****************************************************************************/
void ReleaseMaster()
{
    if (master)
    {
        printf("xenomai End of Program, release master\n");
        ecrt_release_master(master);
        master = NULL;
    }
}
/****************************************************************************/
int ActivateMaster()
{
    int ret;
    printf("xenomai Requesting master...\n");
    if (master)
        return 0;
    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    ConfigPDO();

    // configure SYNC signals for this slave
    for (int i = 0; i < drive_count; i++)
    {
        ecrt_slave_config_dc(sc_asda[i], 0x0300, 1000000, 0, 0, 0);
    }

    // ecrt_slave_config_dc(sc_MX2ECT, 0x0300, 1000000, 0, 0, 0);

    ecrt_master_application_time(master, system_time_ns());
    ret = ecrt_master_select_reference_clock(master, sc_asda[0]); //选择参考时钟
    // ret1 = ecrt_master_select_reference_clock(master, sc_MX2ECT); //选择参考时钟
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }

    // if (ret1 < 0)
    // {
    //     fprintf(stderr, "xenomai **********MX2*********** Failed to select reference clock: %s\n",
    //             strerror(-ret));
    //     return ret;
    // }

    /********************/
    printf("xenomai Activating master...\n");
    if (ecrt_master_activate(master))
    {
        printf("xenomai Activating master...failed\n");
        return -1;
    }
    /********************/
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
    printf("xenomai Activating master...success\n");
    return 0;
}
/****************************************************************************/
void RunCSP(int servo_index, int pos, int speed)
{
    int curpos = EC_READ_S32(domainInput_pd + actpos[servo_index]);
    int abs_pos = 0;
    int abs_curpos = 0;

    abs_pos = abs(pos);
    abs_curpos = abs(curpos);
    if (abs(abs_pos - abs_curpos) > speed)
    {
        csp_running[servo_index] = running;
    }

    if (curpos != pos)
    {
        if (curpos < pos)
        {
            curpos += speed;
            if (curpos > pos)
            {
                csp_running[servo_index] = ok;
                csp_drive_enable[servo_index] = 0;
                curpos = pos;
            }
        }
        else
        {
            curpos -= speed;
            if (curpos < pos)
            {
                csp_running[servo_index] = ok;
                csp_drive_enable[servo_index] = 0;
                curpos = pos;
            }
        }
        EC_WRITE_S32(domainOutput_pd + ipData[servo_index], curpos);
    }
}
/****************************************************************************/

void ErrorCapture()
{
    for (int i = 0; i < drive_count; i++)
    {
        cur_mode[i] = EC_READ_U8(domainInput_pd + modes_of_operation_display[i]);
        cur_status[i] = EC_READ_U16(domainInput_pd + status[i]);
        drive_current_position[i] = EC_READ_S32(domainInput_pd + actpos[i]);

        if (EC_READ_U16(domainInput_pd + errcode[i]) != 0 || (cur_status[i] & 0x0008))
        {
            printf("Error Capture:servo %d error\n", i);

            for (int i = 0; i < drive_count; i++)
            {
                if (EC_READ_U16(domainInput_pd + errcode[i]) != 0 || (cur_status[i] & 0x0008))
                {
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], (EC_READ_U16(domainOutput_pd + cntlwd[i]) | 0x0080));
                }
                else
                {
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], (EC_READ_U16(domainOutput_pd + cntlwd[i]) & 0xff7f));
                }
            }

            if ((cur_status[i] & 0x031) == 49)
            {
                if ((cur_status[i] & 0x033) == 51)
                {
                    if ((cur_status[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x1f);

                    }
                    else
                    {
                        printf("33\n");
                        EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x0f);
                    }
                }
                else
                {
                    printf("31\n");
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x07);
                }
            }
            printf("Error Solved!!!!\n");
        }
    }
}
/****************************************************************************
void MX2ECT()
{
    ecrt_master_receive(master);
    ecrt_domain_process(domainServoOutput);
    ecrt_domain_process(domainServoInput);
    rt_check_domain_state();

    if (gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
    {
        return;
    }

    cycle_counter++;

}
****************************************************************************/
void DriverEtherCAT()
{
    static int csping[drive_count] = {0}; //位置模式运行标志位
    static int cycle_counter = 0;         //周期计数
    static int home_param_setted = 0;
    static int home_point_all_find = false; //所有电机原点复归完成标志位
    static int csping_all_ready = false;//所有电机切换csp成功标志位

    //处于刚开机(需要等待其他操作完成），返回等待下次周期
    if (gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
    {
        return;
    }

    cycle_counter++;

    // receive EtherCAT frames
    ecrt_master_receive(master);
    ecrt_domain_process(domainServoOutput);
    ecrt_domain_process(domainServoInput);
    rt_check_domain_state();

    if (!(cycle_counter % 500))
    {
        rt_check_master_state();
        check_slave_config_states();
    }

    //状态机操作
    switch (gSysRunning.m_gWorkStatus)
    {
    case SYS_WORKING_SAFE_MODE:
    {
        //检查主站是否处于 OP 模式, 若不是，则调整为 OP 模式
        rt_check_master_state();
        check_slave_config_states();
        if ((master_state.al_states & ETHERCAT_STATUS_OP))
        {
            int tmp = true;
            for (int i = 0; i < drive_count; i++)
            {
                if (sc_asda_state[i].al_state != ETHERCAT_STATUS_OP)
                {
                    tmp = false;
                    break;
                }
            }
            if (tmp)
            {
                gSysRunning.m_gWorkStatus = SYS_WORKING_OP_MODE;
                printf("xenomai SYS_WORKING_OP_MODE\n");
            }
        }
    }
    break;

    case SYS_WORKING_OP_MODE:
    {
        if (!home_param_setted)
        {
            for (int i = 0; i < drive_count; i++)
            {
                EC_WRITE_S8(domainOutput_pd + modes_of_operation[i], HM);
            }

            home_param_setted = 1;
        }

        int tmp = false;
        for (int i = 0; i < drive_count; i++)
        {
            cur_status[i] = EC_READ_U16(domainInput_pd + status[i]);
            printf("servo %d status = 0x%x\n", i, cur_status[i]);
            printf("Error code = 0x%x\n", EC_READ_U16(domainInput_pd + errcode[i]));

            EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x80); //清楚故障
            if ((cur_status[i] & 0x0250) == 592)
            {
                printf("*********************Servo Power on*********************\n");
                EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x06);
            }

            ErrorCapture();

            if ((cur_status[i] & 0x031) == 49)
            {
                if ((cur_status[i] & 0x033) == 51)
                {
                    if ((cur_status[i] & 0x0433) == 1075)
                    {
                        EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x1f);

                        printf("servo %d enable success!\n", i);
                        cur_mode[i] = EC_READ_U8(domainInput_pd + modes_of_operation_display[i]);
                        printf("servo %d mode = 0x%x\n", i, cur_mode[i]);
                        cur_status[i] = EC_READ_U16(domainInput_pd + status[i]);
                        printf("servo %d status = 0x%x\n", i, cur_status[i]);

                        tmp = true;
                        if (cur_status[i] == 5687)
                        {
                            tmp = false;
                            break;
                        }

                        if ((EC_READ_U16(domainInput_pd + status[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0)
                        {
                            tmp = false;
                            break;
                        }
                    }
                    else
                    {
                        printf("*********************Servo Waitting*********************\n");
                        EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x0f);
                    }

                }
                else
                {
                    printf("*********************Servo Enable*********************\n");
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x07);
                }    
            }
        }
        if (tmp)
        {
            gSysRunning.m_gWorkStatus = SYS_WORKING_IDLE_STATUS;
            printf("xenomai SYS_WORKING_IDLE_STATUS\n");
        }
    }
    break;

    default:
    {
        ErrorCapture();

        
        // servo 5
        if (EC_READ_U16(domainInput_pd + ECT_Status) & 0x8)
        {
            EC_WRITE_U16(domainOutput_pd + ECT_Command, 0x80);
            if (!(cycle_counter % 1000))
                printf("\nECT ERROR!!!!\n*************************ERROR SAVLED!*************************\n");
        }
        else
        {
            EC_WRITE_U16(domainOutput_pd + ECT_Command, 0);
            EC_WRITE_U16(domainOutput_pd + ECT_Frequency, ECTFrequency);
            EC_WRITE_U16(domainOutput_pd + ECT_Command, 0x1);
            if (!(cycle_counter % 1000))
            {
                printf("                                                                                 ");
                printf("ECT_Status:0x%x\t", EC_READ_U16(domainInput_pd + ECT_Status));
                printf("ECT_Output_frequency:0x%x\n", EC_READ_U16(domainInput_pd + ECT_Output_frequency));
            }
        }
        if (!(cycle_counter % 1000))
        {
            // servo 0
            printf("%d\t", EC_READ_U8(domainInput_pd + off_dig_in_1));
            printf("servo 0 csp_running = %d\t", csp_running[0]);
            printf("mode = 0x%x\t", cur_mode[0]);
            printf("status = 0x%x\t", cur_status[0]);
            printf("actpos = %d\n", drive_current_position[0]);
        }

        home_point_all_find = true;
        for (int i = 0; i < drive_count; i++)
        {
            if (!home_point_find[i] && ((cur_status[i] >> 12) & 1) == 1)
            {
                printf("Servo %d Homing Mode has been completed\n", i);
                home_point_find[i] = 1; //找到原点
            }
            if (!home_point_find[i])
            {
                home_point_all_find = false;
            }
        }

        if (home_point_all_find & !csping_all_ready)
        {
            csping_all_ready = true;
            for (int i = 0; i < drive_count; i++)
            {
                if (!csping[i])
                {
                    csping_all_ready = false;
                    
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x07);
                    if (cur_status[i] == 0x0233)
                    {
                        EC_WRITE_U8(domainOutput_pd + modes_of_operation[i], CSP);
                        cur_mode[i] = EC_READ_U8(domainInput_pd + modes_of_operation_display[i]);
                        if (cur_mode[i] == CSP)
                        {
                            csping[i] = 1;
                            EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x0f);
                            csp_running[i] = wait; // 电机处于等待状态
                        }
                    }
                }
            }
        }

        if (cycle_counter / 1000 == 25){
            printf("***************************STOP*************************\n");
            ECTFrequency = 0;
        }

        if (cycle_counter / 1000 == 35){
            printf("***************************RUN*************************\n");
            ECTFrequency = 0x2000;
        }

        if (cycle_counter / 1000 == 50){
            printf("***************************STOP*************************\n");
            ECTFrequency = 0;
        }

        if (csping_all_ready)
        {
            for (int i = 0; i < drive_count; i++)
            {
                if (cycle_counter / 1000 == 10)
                {
                    csp_drive_enable[i] = 1;
                    csp_pos[i] = 255000;
                }
                if (cycle_counter / 1000 == 20)
                {
                    csp_drive_enable[i] = 1;
                    csp_pos[i] = 500000;
                    // EC_WRITE_U16(domainOutput_pd + ECT_Command, 0x0);
                }
                if (csp_drive_enable[i])
                {
                    RunCSP(i, csp_pos[i], csp_speed[i]);
                }
            }
        }
    }
    break;
    }

    // write application time to master
    ecrt_master_application_time(master, system_time_ns());
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domainServoOutput);
    ecrt_domain_queue(domainServoInput);
    ecrt_master_send(master);
}

/****************************************************************************/
void InterpolationThread(void *arg)
{
    RTIME wait, previous;
    previous = rt_timer_read();
    wait = previous;

    while (run)
    {
        wait += 1000000; //1ms
        //Delay the calling task (absolute).Delay the execution of the calling task until a given date is reached.
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

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    int ret;
    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    gSysRunning.m_gWorkStatus = SYS_WORKING_POWER_ON;
    if (gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
    {
        ActivateMaster();
        gSysRunning.m_gWorkStatus = SYS_WORKING_SAFE_MODE;
        printf("xenomai SYS_WORKING_SAFE_MODE\n");
    }

    ret = rt_task_create(&InterpolationTask, "InterpolationTask", 0, 99, T_FPU);
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting InterpolationTask...\n");
    ret = rt_task_start(&InterpolationTask, &InterpolationThread, NULL);
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    while (run)
    {
        rt_task_sleep(50000000);
    }

    printf("xenomai Deleting realtime InterpolationTask task...\n");
    rt_task_delete(&InterpolationTask);

    ReleaseMaster();
    return 0;
}

/****************************************************************************/
