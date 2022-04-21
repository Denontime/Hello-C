#ifndef ETHERCAT_IGH
#define ETHERCAT_IGH


#include "ecrt.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/task.h>
#include <native/task.h>
#include <native/timer.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>






#define MotorSlavePos0 0, 0
#define JMK_2DM522 0x66668888, 0x20190303

// EtherCAT
extern ec_master_t *master;
extern ec_master_state_t master_state;

extern ec_slave_config_t *sc_motor[4];
extern ec_slave_config_state_t sc_motor_state[4];


extern ec_domain_t *domain_0;
extern ec_domain_state_t domain_state_0;




// process data
extern unsigned char *domain_pd_0;


extern unsigned int counter;
extern unsigned int curr_pos_0;
extern unsigned int target_pos_0;

// offsets for PDO entries
extern unsigned int cntlwd_set[4];
extern unsigned int mod_set[4];
extern unsigned int pos_set[4];
extern unsigned int tpbfnc_set[4];
extern unsigned int speed_set[4];
extern unsigned int startspeed_set[4];
extern unsigned int upspeed_set[4];
extern unsigned int downspeed_set[4];

extern unsigned int errcod_get[4];
extern unsigned int cntlwd_get[4];
extern unsigned int mod_get[4];
extern unsigned int pos_get[4];


#define DOMAI_REG_MAX 	30
extern const  ec_pdo_entry_reg_t domai_regs_0[] ;
extern ec_sync_info_t slave_0_syncs[];

#endif
