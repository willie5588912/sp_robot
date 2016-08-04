#ifndef SP_ETHERCAT
#define SP_ETHERCAT

#include <stdbool.h>
#include "ecrt.h"

#define NSLAVE 3
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
int *igh_update(int *);
int *igh_get_home_pos();
void igh_stop();
void igh_cleanup();

int  ini_driver(int);
void check_domain_state();
void check_master_state();
void check_slave_config_states();

int eth_curr_pos_[NSLAVE];
int eth_tar_pos_[NSLAVE];
int eth_home_pos_[NSLAVE];
#endif
