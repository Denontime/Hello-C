
//gcc test.c -o test.out -I/opt/etherlab/include -L/opt/etherlab/lib/ -lethercat
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

/****************************************************************************/

#include "ecrt.h" // 应用程序接口头文件ecrt.h可以在内核和用户上下文中使用。

/****************************************************************************/

// Application parameters
#define FREQUENCY 100 //控制数据发送频率
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS  1

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_in_1 = NULL;
static ec_slave_config_state_t sc_dig_in_1_state = {};

static ec_slave_config_t *sc_dig_out_3 = NULL;
static ec_slave_config_state_t sc_dig_out_3_state = {};

// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

// alias,position
#define BusCouplerPos  0, 0	
#define DigInSlave_01_Pos 0, 2
#define DigOutSlave_03_Pos 0, 3

// vendor ID,product code
#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL1809 0x00000002, 0x07113052
#define Beckhoff_EL2809 0x00000002, 0x0af93052

// offsets for PDO entries
static unsigned int off_dig_in_1;
static unsigned int off_dig_out_3;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DigInSlave_01_Pos, Beckhoff_EL1809, 0x6000, 1, &off_dig_in_1}, // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {DigOutSlave_03_Pos, Beckhoff_EL2809, 0x7000, 1, &off_dig_out_3}, // [alias,position], [vendor ID,product code], [PDO entry index], [PDO entry subindex], [Pointer to a variable to store the PDO]
    {}
};

static unsigned int counter = 0;

/*****************************************************************************/

#if CONFIGURE_PDOS

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
    {0x1a00, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
    {0x1a01, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
    {0x1a02, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
    {0x1a03, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
    {0x1a04, 1, slave_1_pdo_entries + 4}, /* Channel 5 */
    {0x1a05, 1, slave_1_pdo_entries + 5}, /* Channel 6 */
    {0x1a06, 1, slave_1_pdo_entries + 6}, /* Channel 7 */
    {0x1a07, 1, slave_1_pdo_entries + 7}, /* Channel 8 */
    {0x1a08, 1, slave_1_pdo_entries + 8}, /* Channel 9 */
    {0x1a09, 1, slave_1_pdo_entries + 9}, /* Channel 10 */
    {0x1a0a, 1, slave_1_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_1_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_1_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_1_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_1_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_1_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 16, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 3, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
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

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 1, slave_3_pdo_entries + 0}, /* Channel 1 */
    {0x1601, 1, slave_3_pdo_entries + 1}, /* Channel 2 */
    {0x1602, 1, slave_3_pdo_entries + 2}, /* Channel 3 */
    {0x1603, 1, slave_3_pdo_entries + 3}, /* Channel 4 */
    {0x1604, 1, slave_3_pdo_entries + 4}, /* Channel 5 */
    {0x1605, 1, slave_3_pdo_entries + 5}, /* Channel 6 */
    {0x1606, 1, slave_3_pdo_entries + 6}, /* Channel 7 */
    {0x1607, 1, slave_3_pdo_entries + 7}, /* Channel 8 */
    {0x1608, 1, slave_3_pdo_entries + 8}, /* Channel 9 */
    {0x1609, 1, slave_3_pdo_entries + 9}, /* Channel 10 */
    {0x160a, 1, slave_3_pdo_entries + 10}, /* Channel 11 */
    {0x160b, 1, slave_3_pdo_entries + 11}, /* Channel 12 */
    {0x160c, 1, slave_3_pdo_entries + 12}, /* Channel 13 */
    {0x160d, 1, slave_3_pdo_entries + 13}, /* Channel 14 */
    {0x160e, 1, slave_3_pdo_entries + 14}, /* Channel 15 */
    {0x160f, 1, slave_3_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 8, slave_3_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_OUTPUT, 8, slave_3_pdos + 8, EC_WD_ENABLE},
    {0xff}
};

#endif

#if SDO_ACCESS
static ec_sdo_request_t *sdo;
#endif

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_in_1, &s);

    if (s.al_state != sc_dig_in_1_state.al_state)
        printf("DigIn: State 0x%02X.\n", s.al_state);
    if (s.online != sc_dig_in_1_state.online)
        printf("DigIn: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_dig_in_1_state.operational)
        printf("output: %soperational.\n",
                s.operational ? "" : "Not ");

    sc_dig_in_1_state = s;
}

/*****************************************************************************/

#if SDO_ACCESS
void read_sdo(void)
{
    switch (ecrt_sdo_request_state(sdo)) {
        case EC_REQUEST_UNUSED: // request was not used yet
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

void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state (optional)
    check_domain1_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // check for master state (optional)
        check_master_state();

        // check for islave configuration state(s) (optional)
        check_slave_config_states();
        
        #if SDO_ACCESS
        // read process data SDO
        read_sdo();
        #endif
    }

    // #if 0
    // read process data
    unsigned int slave_1_value = EC_READ_U8(domain1_pd + off_dig_in_1);
    printf("DigIn Slave 1 value %u\n", slave_1_value);
    unsigned int value = 0;
    if (slave_1_value) {
        value = 0xFF;
    } else {
        value = 0;
    }
    EC_WRITE_U16(domain1_pd + off_dig_out_3, value);
    // #endif

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void signal_handler(int signum) {//进程收到定时信号所做的操作
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
            break;
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;// 存放从站配置的结构体
    struct sigaction sa;// 描述信号到达时要采取的操作的结构体
    struct itimerval tv;//计算时间，用于定时的结构体

    printf("Request Master...\n");
    master = ecrt_request_master(0);// 请求EtherCAT主机进行实时操作。取得master设备
    if (!master)	
        return -1;

    printf("Create Domain...\n");
    domain1 = ecrt_master_create_domain(master);// 创建新的进程数据域
    if (!domain1)
        return -1;

    printf("Configuring PDOs...\n");
    // slave 1
    if (!(sc_dig_in_1 = ecrt_master_slave_config(//获取从站配置
                    master, DigInSlave_01_Pos, Beckhoff_EL1809))) {//第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
        fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_in_1, EC_END, slave_1_syncs)) {//指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
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

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc)
        return -1;

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {//为进程数据域注册一组PDO项。参数一：创建的进程数据域，参数二：pdo注册数组
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))// 激活主站
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {//返回域的进程数据
        return -1;
    }

    #if PRIORITY
    pid_t pid = getpid();//获得进程PID
    if (setpriority(PRIO_PROCESS, pid, -19))//设置进程优先级
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
    #endif

    sa.sa_handler = signal_handler;//指定信号关联函数
    sigemptyset(&sa.sa_mask);// 初始化给出的信号集为空
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0)) {//系统调用，用于更改进程在收到特定信号时采取的操作
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }

    printf("Starting timer...\n");
    tv.it_interval.tv_sec = 0;//next time
    tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    tv.it_value.tv_sec = 0;//current time
    tv.it_value.tv_usec = 1000;
    if (setitimer(ITIMER_REAL, &tv, NULL)) {//定时功能，以系统真实的时间来计算，每隔一段时间送出SIGALRM信号。
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    printf("Started.\n");
    while (1) {
        pause();//挂起，等待cpu唤醒

        while (sig_alarms != user_alarms) {//意思是每过一段时间才会执行一次，而不是一直死循环
            cyclic_task();
            user_alarms++;
        }
    }

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
