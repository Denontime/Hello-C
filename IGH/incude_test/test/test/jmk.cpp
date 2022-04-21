#include <ethercat_igh.h>

// EtherCAT
ec_master_t *master = NULL;
ec_master_state_t master_state = {};

ec_slave_config_t *sc_motor[4] = {NULL};
ec_slave_config_state_t sc_motor_state[4] = {};


ec_domain_t *domain_0 = NULL;
ec_domain_state_t domain_state_0 = {};




// process data
unsigned char *domain_pd_0 = NULL;


unsigned int counter = 0;
unsigned int curr_pos_0 = 0;
unsigned int target_pos_0 = 0;

// offsets for PDO entries
unsigned int cntlwd_set[4];
unsigned int mod_set[4];
unsigned int pos_set[4];
unsigned int tpbfnc_set[4];
unsigned int speed_set[4];
unsigned int startspeed_set[4];
unsigned int upspeed_set[4];
unsigned int downspeed_set[4];

unsigned int errcod_get[4];
unsigned int cntlwd_get[4];
unsigned int mod_get[4];
unsigned int pos_get[4];


const  ec_pdo_entry_reg_t domai_regs_0[DOMAI_REG_MAX] = {
 { MotorSlavePos0, JMK_2DM522, 0x6040, 0, &cntlwd_set[0], 	NULL},
 { MotorSlavePos0, JMK_2DM522, 0x6060, 0, &mod_set[0], 	NULL},
 { MotorSlavePos0, JMK_2DM522, 0x607A, 0, &pos_set[0], 	NULL },
 { MotorSlavePos0, JMK_2DM522, 0x60B8, 0, &tpbfnc_set[0], 	NULL },
 { MotorSlavePos0, JMK_2DM522, 0x6081, 0x00, &speed_set[0], NULL},  		
 { MotorSlavePos0, JMK_2DM522, 0x6082, 0x00, &startspeed_set[0], NULL},  		
 { MotorSlavePos0, JMK_2DM522, 0x6083, 0x00, &upspeed_set[0], NULL},  		
 { MotorSlavePos0, JMK_2DM522, 0x6084, 0x00, &downspeed_set[0], NULL}, 
 
 { MotorSlavePos0, JMK_2DM522, 0x603F, 0x00, &errcod_get[0], NULL},   
 { MotorSlavePos0, JMK_2DM522, 0x6041, 0x00, &cntlwd_get[0], NULL},    
 { MotorSlavePos0, JMK_2DM522, 0x6061, 0x00, &mod_get[0], NULL},    
 { MotorSlavePos0, JMK_2DM522, 0x6064, 0x00, &pos_get[0], NULL},    
 {}
};


static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x607a, 0x00, 32},
    {0x60b8, 0x00, 16},
    {0x60fe, 0x01, 32},
    {0x60fe, 0x02, 32},
    {0x6081, 0x00, 32},
    {0x6082, 0x00, 32},
    {0x6083, 0x00, 32},
    {0x6084, 0x00, 32},
    {0x603f, 0x00, 16},
    
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x6064, 0x00, 32},
    {0x60b9, 0x00, 16},
    {0x60ba, 0x00, 32},
    {0x60bb, 0x00, 32},
    {0x60bc, 0x00, 32},
    {0x60bd, 0x00, 32},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 10, slave_0_pdo_entries + 0}, /* RxPdoMapping */
    {0x1a00, 10, slave_0_pdo_entries + 10}, /* TxPdoMapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_ENABLE},
    {0xff}
};



