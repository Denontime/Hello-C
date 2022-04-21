#ifndef ETHERCAT_IGH
#define ETHERCAT_IGH

#include "ecrt.h"
/**************************************  *************************************/
typedef int Bool;
#define false 0
#define true 1
/**************************************  *************************************/
//Define
#define ETHERCAT_STATUS_OP 0x08
#define STATUS_SERVO_ENABLE_BIT (0x04)
/**************************************  *************************************/
//Clock
#define CLOCK_TO_USE CLOCK_REALTIME
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
/**************************************  *************************************/
//Master.Slaves
#define asda_Pos0           0, 0
#define BusCouplerPos       0, 1
#define DigInSlave_01_Pos   0, 2
#define DigOutSlave_03_Pos  0, 4
#define MX2ECT_Pos          0, 5
/**************************************  *************************************/
/*Vendor ID  * Product code */
#define asda            0x000001dd, 0x10305070
#define Beckhoff_EL1809 0x00000002, 0x07113052
#define Beckhoff_EL2809 0x00000002, 0x0af93052
#define MX2ECT          0x00000083, 0x00000053
/**************************************  *************************************/
//Count
#define drive_count     1   //电机数量
#define frequency_count 1   //变频数量
/**************************************  *************************************/
typedef enum _DriveStatusCSP
{
    wait,
    running,
    ok //到达位置
} DriveStatusCSP;
/**************************************  *************************************/
//master status
typedef enum _SysWorkingStatus
{
    SYS_WORKING_POWER_ON,
    SYS_WORKING_SAFE_MODE,
    SYS_WORKING_OP_MODE,
    SYS_WORKING_LINK_DOWN,
    SYS_WORKING_IDLE_STATUS //系统空闲
} SysWorkingStatus;
/**************************************  *************************************/
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
/**************************************  *************************************/
typedef struct _GSysRunningParm
{
    SysWorkingStatus m_gWorkStatus;
} GSysRunningParm;

/**************************************  *************************************/
// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainServoInput = NULL;
static ec_domain_state_t domainServoInput_state = {};

static ec_domain_t *domainServoOutput = NULL;
static ec_domain_state_t domainServoOutput_state = {};

/**************************************  *************************************/
static ec_slave_config_t *sc_asda[drive_count];
static ec_slave_config_state_t sc_asda_state[drive_count];

static ec_slave_config_t *sc_dig_in_1 = NULL;
static ec_slave_config_state_t sc_dig_in_1_state = {};

static ec_slave_config_t *sc_dig_out_3 = NULL;
static ec_slave_config_state_t sc_dig_out_3_state = {};

static ec_slave_config_t *sc_MX2ECT[fre_count];
static ec_slave_config_state_t sc_MX2ECT_state[fre_count];

/**************************************  *************************************/
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;
/**************************************  *************************************/
//RTime
RT_TASK InterpolationTask;
/**************************************  *************************************/
//System State
Bool run = true;
static int64_t system_time_base = 0LL;
/**************************************  *************************************/
//ServoValues
int csp_drive_enable[drive_count] = {0};                     //csp模式下电机转动开关
int home_point_find[drive_count] = {0};                      //原点复归标志位
int drive_current_position[drive_count] = {0};               // 电机当前位置
int csp_running[drive_count] = {0};                          //csp模式下电机运行状态
int csp_pos[drive_count] = {0};                              //目标位置
int csp_speed[drive_count] = {3000};                         //csp转动速率
int ServoFaultReset = 0;                                     //伺服Fault复位标志位

static unsigned int cur_mode[drive_count];
static unsigned int cur_status[drive_count];
static unsigned int cntlwd[drive_count];                     //控制字
static unsigned int ipData[drive_count];                     //目标位置
static unsigned int status[drive_count];                     //状态字
static unsigned int actpos[drive_count];                     //当前回授位置
static unsigned int modes_of_operation[drive_count];         //6060
static unsigned int modes_of_operation_display[drive_count]; //6061
static unsigned int errcode[drive_count];                    //错误代码
/**************************************  *************************************/
//IOValues
static unsigned int off_dig_in_1;
static unsigned int off_dig_out_3;
/**************************************  *************************************/
//FreQCValues
int ECTFrequency[fre_count] = {0x1388};
static unsigned int ECT_Command[fre_count];             //变频器控制字
static unsigned int ECT_Frequency[fre_count];           //变频器频率给定

static unsigned int ECT_Status[fre_count];              //变频器状态字
static unsigned int ECT_Output_frequency[fre_count];    //变频器频率反馈
/**************************************  *************************************/
// Process Data
ec_pdo_entry_reg_t domainServoOutput_regs[] = {
    {asda_Pos0, asda, 0x6040, 0x00, &cntlwd[0], NULL},
    {asda_Pos0, asda, 0x607a, 0x00, &ipData[0], NULL},
    {asda_Pos0, asda, 0x6060, 0x00, &modes_of_operation[0], NULL},              //伺服电机控制
    {DigOutSlave_03_Pos, Beckhoff_EL2809, 0x7050, 1, &off_dig_out_3, NULL},     //数字IO输入
    {MX2ECT_Pos, MX2ECT, 0x5000, 0x00, &ECT_Command, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5010, 0x00, &ECT_Frequency, NULL},                   //变频器控制
    {}};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {asda_Pos0, asda, 0x6064, 0x00, &actpos[0], NULL},
    {asda_Pos0, asda, 0x6041, 0x00, &status[0], NULL},
    {asda_Pos0, asda, 0x6061, 0x00, &modes_of_operation_display[0], NULL},
    {asda_Pos0, asda, 0x603f, 0x00, &errcode[0], NULL},                         //伺服电机状态反馈
    {DigInSlave_01_Pos, Beckhoff_EL1809, 0x6020, 1, &off_dig_in_1, NULL},       //数字IO输出
    {MX2ECT_Pos, MX2ECT, 0x5100, 0x00, &ECT_Status, NULL},
    {MX2ECT_Pos, MX2ECT, 0x5110, 0x00, &ECT_Output_frequency, NULL},            //变频器状态反馈
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
    {0x603f, 0x00, 16}, /*error code*/
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
/**************************************  *************************************/
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
    {0xff}};
/**************************************  *************************************/
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
    {0xff}};
/**************************************  *************************************/
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

/**************************************  *************************************/
RTIME system_time_ns(void);

int ConfigPDO();
int ActivateMaster();
/*****************
 * Realtime task *
 ****************/
void rt_check_master_state(void);
void rt_check_domain_state(void);
void check_slave_config_states(void);
void ReleaseMaster();
void RunCSP(int servo_index, int pos, int speed);
void Servo_Motor_ErrorCapture();
void DriverEtherCAT(int servo_num, int pos);
void InterpolationThread(void *arg);
void signal_handler(int sig);
/***********************************************************
#define MotorSlavePos0 0, 0
#define MotorSlavePos1 0, 1
#define MBDHT2510BA1 0x0000066f, 0x525100a1

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainInput_0 = NULL;
static ec_domain_state_t domainInput_state_0 = {};
static ec_domain_t *domainInput_1 = NULL;
static ec_domain_state_t domainInput_state_1 = {};

static ec_domain_t *domainOutput_0 = NULL;
static ec_domain_state_t domainOutput_state_0 = {};
static ec_domain_t *domainOutput_1 = NULL;
static ec_domain_state_t domainOutput_state_1 = {};

static ec_slave_config_t *sc_motor_0 = NULL;
static ec_slave_config_state_t sc_motor_state_0 = {};
static ec_slave_config_t *sc_motor_1 = NULL;
static ec_slave_config_state_t sc_motor_state_1 = {};

// process data
static uint8_t *domainOutput_pd_0 = NULL;
static uint8_t *domainInput_pd_0 = NULL;
static uint8_t *domainOutput_pd_1 = NULL;
static uint8_t *domainInput_pd_1 = NULL;

// offsets for PDO entries
static unsigned int mbdh_cntlwd_0;
static unsigned int mbdh_modeop_0;
static unsigned int mbdh_tarpos_0;
static unsigned int mbdh_tpbfnc_0;
static unsigned int mbdh_errcod_0;
static unsigned int mbdh_statwd_0;
static unsigned int mbdh_modedp_0;
static unsigned int mbdh_actpos_0;
static unsigned int mbdh_tpdsta_0;
static unsigned int mbdh_tpbpos_0;
static unsigned int mbdh_errval_0;
static unsigned int mbdh_digiin_0;

static unsigned int mbdh_cntlwd_1;
static unsigned int mbdh_modeop_1;
static unsigned int mbdh_tarpos_1;
static unsigned int mbdh_tpbfnc_1;
static unsigned int mbdh_errcod_1;
static unsigned int mbdh_statwd_1;
static unsigned int mbdh_modedp_1;
static unsigned int mbdh_actpos_1;
static unsigned int mbdh_tpdsta_1;
static unsigned int mbdh_tpbpos_1;
static unsigned int mbdh_errval_1;
static unsigned int mbdh_digiin_1;


const static ec_pdo_entry_reg_t domainOutput_regs_0[] = {
 { MotorSlavePos0, MBDHT2510BA1, 0x6040, 0, &mbdh_cntlwd_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x6060, 0, &mbdh_modeop_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x607A, 0, &mbdh_tarpos_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x60B8, 0, &mbdh_tpbfnc_0 },
 {}
};
const static ec_pdo_entry_reg_t domainOutput_regs_1[] = {
 { MotorSlavePos1, MBDHT2510BA1, 0x6040, 0, &mbdh_cntlwd_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x6060, 0, &mbdh_modeop_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x607A, 0, &mbdh_tarpos_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x60B8, 0, &mbdh_tpbfnc_1 },
 {}
};

const static ec_pdo_entry_reg_t domainInput_regs_0[] = {
 { MotorSlavePos0, MBDHT2510BA1, 0x603f, 0, &mbdh_errcod_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x6041, 0, &mbdh_statwd_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x6061, 0, &mbdh_modedp_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x6064, 0, &mbdh_actpos_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x60B9, 0, &mbdh_tpdsta_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x60BA, 0, &mbdh_tpbpos_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x60F4, 0, &mbdh_errval_0 },
 { MotorSlavePos0, MBDHT2510BA1, 0x60FD, 0, &mbdh_digiin_0 },
 {}
};
const static ec_pdo_entry_reg_t domainInput_regs_1[] = {
 { MotorSlavePos1, MBDHT2510BA1, 0x603f, 0, &mbdh_errcod_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x6041, 0, &mbdh_statwd_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x6061, 0, &mbdh_modedp_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x6064, 0, &mbdh_actpos_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x60B9, 0, &mbdh_tpdsta_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x60BA, 0, &mbdh_tpbpos_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x60F4, 0, &mbdh_errval_1 },
 { MotorSlavePos1, MBDHT2510BA1, 0x60FD, 0, &mbdh_digiin_1 },
 {}
};

static unsigned int counter = 0;
static unsigned int curr_pos_0 = 0;
static unsigned int curr_pos_1 = 0;
static unsigned int target_pos_0 = 0;
static unsigned int target_pos_1 = 0;

static ec_pdo_entry_info_t mbdh_pdo_entries_output_0[] = {
 { 0x6040, 0x00, 16 },
 { 0x6060, 0x00, 8 },
 { 0x607A, 0x00, 32 },
 { 0x60B8, 0x00, 16 },
};
static ec_pdo_entry_info_t mbdh_pdo_entries_output_1[] = {
 { 0x6040, 0x00, 16 },
 { 0x6060, 0x00, 8 },
 { 0x607A, 0x00, 32 },
 { 0x60B8, 0x00, 16 },
};

static ec_pdo_entry_info_t mbdh_pdo_entries_input_0[] = {
 { 0x603f, 0x00, 16 },
 { 0x6041, 0x00, 16 },
 { 0x6061, 0x00, 8 },
 { 0x6064, 0x00, 32 },
 { 0x60B9, 0x00, 16 },
 { 0x60BA, 0x00, 32 },
 { 0x60F4, 0x00, 32 },
 { 0x60FD, 0x00, 32 },
};
static ec_pdo_entry_info_t mbdh_pdo_entries_input_1[] = {
 { 0x603f, 0x00, 16 },
 { 0x6041, 0x00, 16 },
 { 0x6061, 0x00, 8 },
 { 0x6064, 0x00, 32 },
 { 0x60B9, 0x00, 16 },
 { 0x60BA, 0x00, 32 },
 { 0x60F4, 0x00, 32 },
 { 0x60FD, 0x00, 32 },
};

static ec_pdo_info_t mbdh_pdo_1600_0[] = {
 { 0x1600, 4, mbdh_pdo_entries_output_0 },
};
static ec_pdo_info_t mbdh_pdo_1603_1[] = {
 { 0x1603, 4, mbdh_pdo_entries_output_1 },
};

static ec_pdo_info_t mbdh_pdo_1a00_0[] = {
 { 0x1A00, 8, mbdh_pdo_entries_input_0 },
};
static ec_pdo_info_t mbdh_pdo_1a03_1[] = {
 { 0x1A03, 8, mbdh_pdo_entries_input_0 },
};

static ec_sync_info_t mbdh_syncs_0[] = {
 { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
 { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
 { 2, EC_DIR_OUTPUT, 1, mbdh_pdo_1600_0, EC_WD_DISABLE },
 { 3, EC_DIR_INPUT, 1, mbdh_pdo_1a00_0, EC_WD_DISABLE },
 { 0xff }
};

static ec_sync_info_t mbdh_syncs_1[] = {
 { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
 { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
 { 2, EC_DIR_OUTPUT, 1, mbdh_pdo_1603_1, EC_WD_DISABLE },
 { 3, EC_DIR_INPUT, 1, mbdh_pdo_1a03_1, EC_WD_DISABLE },
 { 0xff }
};

bool igh_configure();
bool igh_start();
int  igh_update(int);
void igh_stop();
void igh_cleanup();

int  ini_driver(int);
void check_domain_state();
void check_master_state();
void check_slave_config_states();

**************************************************************/

#endif
