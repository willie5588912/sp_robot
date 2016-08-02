#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sp_ethercat/sp_ethercat.h>

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

bool igh_configure()
{
  // Requests an EtherCAT master for realtime operation.
  master = ecrt_request_master(0); // Index of the master to request.
  if (!master)
    return false;

  // Creates a new process data domain
  domainOutput_0 = ecrt_master_create_domain(master);
  if (!domainOutput_0)
    return false;
  domainInput_0 = ecrt_master_create_domain(master);
  if (!domainInput_0)
    return false;
  domainOutput_1 = ecrt_master_create_domain(master);
  if (!domainOutput_1)
    return false;
  domainInput_1 = ecrt_master_create_domain(master);
  if (!domainInput_1)
    return false;

  // Obtains a slave configuration
  if (!(sc_motor_0 = ecrt_master_slave_config(master, MotorSlavePos0, MBDHT2510BA1))) 
  {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return false;
  }
  if (!(sc_motor_1 = ecrt_master_slave_config(master, MotorSlavePos1, MBDHT2510BA1))) 
  {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return false;
  }

  // Configuring PDOs
  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(sc_motor_0, EC_END, mbdh_syncs_0))
  {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return false;
  }
  if (ecrt_slave_config_pdos(sc_motor_1, EC_END, mbdh_syncs_1))
  {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return false;
  }

  if (ecrt_domain_reg_pdo_entry_list(domainOutput_0, domainOutput_regs_0)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
   }
  if (ecrt_domain_reg_pdo_entry_list(domainOutput_1, domainOutput_regs_1)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
   }

  if (ecrt_domain_reg_pdo_entry_list(domainInput_0, domainInput_regs_0)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
  }
  if (ecrt_domain_reg_pdo_entry_list(domainInput_1, domainInput_regs_1)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
  }

  printf("Activating master...\n");
  if (ecrt_master_activate(master))
    return false;

  if (!(domainOutput_pd_0 = ecrt_domain_data(domainOutput_0))) 
  {
    return false;
  }
  if (!(domainOutput_pd_1 = ecrt_domain_data(domainOutput_1))) 
  {
    return false;
  }

  if (!(domainInput_pd_0 = ecrt_domain_data(domainInput_0))) 
  {
    return false;
  }
  if (!(domainInput_pd_1 = ecrt_domain_data(domainInput_1))) 
  {
    return false;
  }

  return true;
}


bool igh_start()
{
  int state = -500;
  while (state <= 5) 
  {
    ini_driver(state);
    state++;
    usleep(1000);
  }

  uint16_t statwd_0 = EC_READ_U16(domainInput_pd_0 + mbdh_statwd_0);
  printf("6041h_0 = %4.4x\n",statwd_0); 
  if( CHECK_BIT(statwd_0, 0) && !CHECK_BIT(statwd_0, 1) &&
     !CHECK_BIT(statwd_0, 2) && !CHECK_BIT(statwd_0, 3) &&
      CHECK_BIT(statwd_0, 5) && !CHECK_BIT(statwd_0, 6))
  {
    printf("Now slave 0 is in CSP mode, ready to receive commands.\n");
    return true;
  }
  else
  {
    printf("Slave 0 Servo on fail.\n");
    return false;
  }

  uint16_t statwd_1 = EC_READ_U16(domainInput_pd_1 + mbdh_statwd_1);
  printf("6041h_1 = %4.4x\n",statwd_1); 
  if( CHECK_BIT(statwd_1, 0) && !CHECK_BIT(statwd_1, 1) &&
     !CHECK_BIT(statwd_1, 2) && !CHECK_BIT(statwd_1, 3) &&
      CHECK_BIT(statwd_1, 5) && !CHECK_BIT(statwd_1, 6))
  {
    printf("Now slave 0 is in CSP mode, ready to receive commands.\n");
    return true;
  }
  else
  {
    printf("Slave 1 Servo on fail.\n");
    return false;
  }

}

int igh_update(int enc_count)
{
  counter++;

  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput_0);
  ecrt_domain_process(domainInput_0);
  ecrt_domain_process(domainOutput_1);
  ecrt_domain_process(domainInput_1);

  // periodically check the states and show the current pose
  //if(counter % 100 == 0)
  curr_pos_0 = EC_READ_S32(domainInput_pd_0 + mbdh_actpos_0);
  curr_pos_1 = EC_READ_S32(domainInput_pd_1 + mbdh_actpos_1);
  //printf("curr_pos = %d\n", curr_pos);

  // write target position
  target_pos_0 += enc_count; 
  target_pos_1 += enc_count; 
  //printf("target_pos = %d\n", target_pos);
  EC_WRITE_S32(domainOutput_pd_0 + mbdh_tarpos_0, target_pos_0);
  EC_WRITE_S32(domainOutput_pd_1 + mbdh_tarpos_1, target_pos_1);

  // send process data
  ecrt_domain_queue(domainOutput_0);
  ecrt_domain_queue(domainInput_0);
  ecrt_domain_queue(domainOutput_1);
  ecrt_domain_queue(domainInput_1);
  ecrt_master_send(master);

  return curr_pos_0;
}

void igh_stop()
{
  //ecrt_master_deactivate(master);

  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput_0);
  ecrt_domain_process(domainInput_0);
  ecrt_domain_process(domainOutput_1);
  ecrt_domain_process(domainInput_1);

  EC_WRITE_U16(domainOutput_pd_0 + mbdh_cntlwd_0, 0x00);
  EC_WRITE_U16(domainOutput_pd_1 + mbdh_cntlwd_1, 0x00);

  // send process data
  ecrt_domain_queue(domainOutput_0);
  ecrt_domain_queue(domainInput_0);
  ecrt_domain_queue(domainOutput_1);
  ecrt_domain_queue(domainInput_1);
  ecrt_master_send(master);
}

void igh_cleanup() 
{
  ecrt_release_master(master);
}

int ini_driver(int state)
{
  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput_0);
  ecrt_domain_process(domainInput_0);
  ecrt_domain_process(domainOutput_1);
  ecrt_domain_process(domainInput_1);

  curr_pos_0 = EC_READ_S32(domainInput_pd_0 + mbdh_actpos_0);
  curr_pos_1 = EC_READ_S32(domainInput_pd_1 + mbdh_actpos_1);
  //printf("curr_pos = %d\n", curr_pos);

  target_pos_0 = EC_READ_S32(domainInput_pd_0 + mbdh_actpos_0);
  target_pos_1 = EC_READ_S32(domainInput_pd_1 + mbdh_actpos_1);
  //printf("target_pos = %d\n", target_pos);

  switch(state)
  {
    case -100:
      printf("fault reset\n");
      EC_WRITE_U16(domainOutput_pd_0 + mbdh_cntlwd_0, 0x80);
      EC_WRITE_U16(domainOutput_pd_1 + mbdh_cntlwd_1, 0x80);
    break;

    case 0:
      printf("change mode to csp\n");
      EC_WRITE_S8(domainOutput_pd_0 + mbdh_modeop_0, 8);
      EC_WRITE_S8(domainOutput_pd_1 + mbdh_modeop_1, 8);
    break;

    case 3:
      printf("shutdown\n");
      EC_WRITE_U16(domainOutput_pd_0 + mbdh_cntlwd_0, 0x06);
      EC_WRITE_U16(domainOutput_pd_1 + mbdh_cntlwd_1, 0x06);
    break;

    case 4:
      printf("switch on\n");
      EC_WRITE_U16(domainOutput_pd_0 + mbdh_cntlwd_0, 0x07);
      EC_WRITE_U16(domainOutput_pd_1 + mbdh_cntlwd_1, 0x07);
    break;

    case 5:
      printf("enable operation (should servo on now)\n");
      EC_WRITE_U16(domainOutput_pd_0 + mbdh_cntlwd_0, 0xF);
      EC_WRITE_U16(domainOutput_pd_1 + mbdh_cntlwd_1, 0xF);
    break;
  }

  // send process data
  ecrt_domain_queue(domainOutput_0);
  ecrt_domain_queue(domainInput_0);
  ecrt_domain_queue(domainOutput_1);
  ecrt_domain_queue(domainInput_1);
  ecrt_master_send(master);

  return state;
}

void check_domain_state()
{
  ec_domain_state_t ds_0;
  ec_domain_state_t ds_1;

  ecrt_domain_state(domainOutput_0, &ds_0);
  ecrt_domain_state(domainOutput_1, &ds_1);

  if (ds_0.working_counter != domainOutput_state_0.working_counter)
    printf("domainOutput: WC %u.\n", ds_0.working_counter);
  if (ds_0.wc_state != domainOutput_state_0.wc_state)
    printf("domainOutput: State %u.\n", ds_0.wc_state);
  if (ds_1.working_counter != domainOutput_state_1.working_counter)
    printf("domainOutput: WC %u.\n", ds_1.working_counter);
  if (ds_1.wc_state != domainOutput_state_1.wc_state)
    printf("domainOutput: State %u.\n", ds_1.wc_state);

  domainOutput_state_0 = ds_0;
  domainOutput_state_1 = ds_1;

  ecrt_domain_state(domainInput_0, &ds_0);
  ecrt_domain_state(domainInput_1, &ds_1);

  if (ds_0.working_counter != domainInput_state_0.working_counter)
    printf("domainInput: WC %u.\n", ds_0.working_counter);
  if (ds_0.wc_state != domainInput_state_0.wc_state)
    printf("domainInput: State %u.\n", ds_0.wc_state);
  if (ds_1.working_counter != domainInput_state_1.working_counter)
    printf("domainInput: WC %u.\n", ds_1.working_counter);
  if (ds_1.wc_state != domainInput_state_1.wc_state)
    printf("domainInput: State %u.\n", ds_1.wc_state);

  domainInput_state_0 = ds_0;
  domainInput_state_1 = ds_1;
}

void check_master_state()
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

void check_slave_config_states()
{
  ec_slave_config_state_t s_0;
  ecrt_slave_config_state(sc_motor_0, &s_0);
  ec_slave_config_state_t s_1;
  ecrt_slave_config_state(sc_motor_1, &s_1);

  if (s_0.al_state != sc_motor_state_0.al_state)
    printf("Motor: State 0x%02X.\n", s_0.al_state);
  if (s_0.online != sc_motor_state_0.online)
    printf("Motor: %s.\n", s_0.online ? "online" : "offline");
  if (s_0.operational != sc_motor_state_0.operational)
    printf("Motor: %soperational.\n",s_0.operational ? "" : "Not ");
  if (s_1.al_state != sc_motor_state_1.al_state)
    printf("Motor: State 0x%02X.\n", s_1.al_state);
  if (s_1.online != sc_motor_state_1.online)
    printf("Motor: %s.\n", s_1.online ? "online" : "offline");
  if (s_1.operational != sc_motor_state_1.operational)
    printf("Motor: %soperational.\n",s_1.operational ? "" : "Not ");

  sc_motor_state_0 = s_0;
  sc_motor_state_1 = s_1;
}
