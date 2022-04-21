#include <errno.h>
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
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ecrt.h"

#define Bool int
#define false 0
#define true 1
#define ETHERCAT_StatusWord_OP 0x08
#define StatusWord_SERVO_ENABLE_BIT (0x04)
#define CSPPositionKeepTime 3000000000 //到达目标位置保持时间
#define millisecond 1000000

#define servo_count 1 //电机数量
#define VFD_count 1   //变频器数量

int home_point_all_find = false;
int csping_all_ready = false;
RT_TASK DivertLaneList[servo_count];
int CSPMotorSwitch[servo_count] = {1};         //csp模式下电机转动开关
int HomePointFinded[servo_count] = {0};        //原点复归标志位
int MotorCurrentPosition[servo_count] = {0};   //电机当前位置
int CSPMotorStatus[servo_count] = {0};         //csp模式下电机运行状态
int CSPTargetPosition[servo_count] = {630000}; //每个电机摆动位置
int CSPHomePosition[servo_count] = {10000};    //每个电机前进方向位置
int CSPSpeed[servo_count] = {5000};            //csp转动速率
// long long int CSPPositionKeepTime = 3000000000;          //到达目标位置保持时间
int ServoFaultReset = 0;                 //伺服Fault复位标志位
int DivertLaneSwitch[servo_count] = {1}; //所有Divert Lane对应斜轮是否分拣开关

int HomePointFindMode = 33;         //回零方式
int HomePointSwitchFindSpeed = 500; //寻找原点开关速度
int HomePointFindSpeed = 200;       //寻找原点速度
int HomePointAcceleration = 800;    //原点复归加速度
int HomePointOffset = 0;            //原点复归偏移量

int VFDRunStatus[VFD_count] = {0}; //变频器是否转动
int VFDFreq[VFD_count] = {5000};
int VFDTurnAround = 2; //变频器旋转方向

/**************************************  *************************************
int VFD_Stop();
int VFD_Run(int VFD_Index, int Freq);
int Servo_Back();
int Servo_Move(int Servo_Index, int position);
*************************************  **************************************/

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

static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

/****************************************************************************/
#define ServoMotor 0, 0
#define VFD 0, 1

/*Vendor ID  * Product code */
#define ASDAServoMotor 0x000001dd, 0x10305070
#define OMRONVFD 0x00000083, 0x00000053

// offsets for PDO entries
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
    {}};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {ServoMotor, ASDAServoMotor, 0x6064, 0x00, &ActualPosition[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6041, 0x00, &StatusWord[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x6061, 0x00, &OPModeDisplay[0], NULL},
    {ServoMotor, ASDAServoMotor, 0x603f, 0x00, &ErrorCode[0], NULL},
    {VFD, OMRONVFD, 0x5100, 0x00, &VFDStatusWord[0], NULL},
    {VFD, OMRONVFD, 0x5110, 0x00, &VFDOutputFrequency[0], NULL},
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
    // ecrt_slave_config_dc(sc_VFD, 0x0300, 1000000, 0, 0, 0);

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

void DivertLaneTask(void *arg)
{
    int lane_index = *(int *)arg;
    while (run)
    {
        if (DivertLaneSwitch[lane_index] == 1)
        {
            // DivertLaneSwitch[lane_index] == 0;
            Servo_Move(lane_index);
            Servo_Keep(lane_index);
            Servo_Back(lane_index);
        }
        rt_task_sleep(millisecond);
        // rt_printf("\n----------------------------------------------\n");
    }
}

int Servo_Keep(int Servo_Index)
{
    long int count = CSPPositionKeepTime / millisecond;
    for (; count >= 0; count--)
    {
        RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
        rt_task_sleep(millisecond);
    }
    return true;
}

int Servo_Move(int Servo_Index)
{
    rt_printf("\ncsping_all_ready=%d--CSPMotorSwitch[Servo_Index]=%d--\n", csping_all_ready, CSPMotorSwitch[Servo_Index]);

    if (csping_all_ready && CSPMotorSwitch[Servo_Index] && !Servo_Index >= servo_count)
    {
        while (1)
        {
            RunCSP(Servo_Index, CSPTargetPosition[Servo_Index], CSPSpeed[Servo_Index]);
            // if (!cycle_counter % 1000)
            // {
            //
            // }
            // rt_printf("\n--------------CSPMotorStatus = %d\n", CSPMotorStatus[0]);
            if (CSPMotorStatus[Servo_Index] == ok)
            {
                rt_printf("CSPMotorStatus[Servo_Index] == ok");
                break;
            }
            rt_task_sleep(millisecond);
        }
        return true;
    }
    return false;
}

int Servo_Back(int Servo_Index)
{
    // rt_printf("\ncsping_all_ready=%d--CSPMotorSwitch[Servo_Index]=%d--\n", csping_all_ready, CSPMotorSwitch[Servo_Index]);
    if (csping_all_ready && CSPMotorSwitch[Servo_Index] && !Servo_Index >= servo_count)
    {
        rt_printf("\n--------------CSPMotorStatus\n");
        while (1)
        {
            RunCSP(Servo_Index, CSPHomePosition[Servo_Index], CSPSpeed[Servo_Index]);
            if (CSPMotorStatus[Servo_Index] == ok)
            {
                // DivertLaneSwitch[Servo_Index] = 0;
                break;
            }
            rt_task_sleep(millisecond);
        }
        return true;
    }
    return false;
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
        if (!VFDRunStatus[VFD_Index] && !((EC_READ_U16(domainInput_pd + VFDStatusWord[VFD_Index]) >> 3) & 1))
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
            rt_printf("servo %d fault\n", i);
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
            rt_printf("servo %d fault reset.\n", i);
        }
    }
}

void VFDErrorCapture()
{
    for (int i = 0; i < VFD_count; i++)
    {
        if (EC_READ_U16(domainInput_pd + VFDStatusWord[i]) & 0x8)
        {
            EC_WRITE_U16(domainOutput_pd + VFDCommand[i], 0x80);
            rt_printf("VFD %d fault reset\n", i);
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
            rt_printf("servo %d StatusWord = 0x%x\n", i, CurrentStatusWord[i]);
            rt_printf("servo %d ErrorCode = 0x%x\n", i, EC_READ_U16(domainInput_pd + ErrorCode[i]));
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
                        rt_printf("servo %d enable success\n", i);
                        CurrentMode[i] = EC_READ_U8(domainInput_pd + OPModeDisplay[i]);
                        rt_printf("servo %d mode = 0x%x\n", i, CurrentMode[i]);
                        CurrentStatusWord[i] = EC_READ_U16(domainInput_pd + StatusWord[i]);
                        rt_printf("servo %d StatusWord = 0x%x\n", i, CurrentStatusWord[i]);

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
                rt_task_create(&DivertLaneList[i], "divert_lane_" + i, 0, 3, 0);
                rt_task_start(&DivertLaneList[i], DivertLaneTask, &i);

                rt_printf("rt_task_create!\n");
            }
            divert_lane_created = true;
        }
        // if (csping_all_ready)
        // {
        //     for (int i = 0; i < servo_count; i++)
        //     {
        //         if (CSPMotorSwitch[i])
        //         {
        //             // RunCSP(i, CSPTargetPosition[i], CSPSpeed[i]);
        //             Servo_Move(i);
        //         }
        //     }
        // }

        // Servo_Move(i);
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
        wait += millisecond; //1ms
        rt_task_sleep_until(wait);
        DriverEtherCAT();
    }
}

void signal_handler(int sig)
{
    run = 0;
}

int main(int argc, char *argv[])
{
    RT_TASK LineshaftDiverterTask;
    int ret;
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

    ret = rt_task_create(&LineshaftDiverterTask, "LineshaftDiverterTask", 0, 1, T_FPU);
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    rt_printf("Starting LineshaftDiverterTask...\n");
    ret = rt_task_start(&LineshaftDiverterTask, &LineshaftDiverterThread, NULL);
    if (ret < 0)
    {
        fprintf(stderr, "xenomai Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    while (run)
    {
        rt_task_sleep(50000000);
    }

    rt_printf("xenomai Deleting realtime LineshaftDiverterTask task...\n");
    rt_task_delete(&LineshaftDiverterTask);
    for (int i = 0; i <= servo_count; i++)
    {
        rt_task_delete(&DivertLaneList[i]);
    }

    ReleaseMaster();
    return 0;
}
