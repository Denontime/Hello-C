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


GSysRunningParm gSysRunning;

RT_TASK InterpolationTask;

/**************************************  *************************************/
RTIME system_time_ns(void)
{
  struct timespec rt_time;
  clock_gettime(CLOCK_TO_USE, &rt_time);
  RTIME time = TIMESPEC2NS(rt_time);
  return time - system_time_base;
}
/**************************************  *************************************/
int ConfigPDO()
{
  /**************************************/
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
  /**************************************/
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
  /**************************************/
  // slave 2
  if (!(sc_dig_in_1 = ecrt_master_slave_config( //获取从站配置
            master, DigInSlave_01_Pos, Beckhoff_EL1809)))
  {                                                            //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
    fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
    return -1;
  }
  if (ecrt_slave_config_pdos(sc_dig_in_1, EC_END, slave_2_syncs))
  { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }
  /**************************************/
  // slave 3
  if (!(sc_dig_out_3 = ecrt_master_slave_config( //获取从站配置
            master, DigOutSlave_03_Pos, Beckhoff_EL2809)))
  {                                                            //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
    fprintf(stderr, "Failed to get slave 2 configuration.\n"); // 第二个参数应该是从站的位置
    return -1;
  }
  if (ecrt_slave_config_pdos(sc_dig_out_3, EC_END, slave_3_syncs))
  { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }
  /**************************************/
  // slave 5
  sc_MX2ECT[0] = ecrt_master_slave_config(master, MX2ECT_Pos, MX2ECT); //获取从站配置
  if (!sc_MX2ECT[0])
  {                                                            //第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
    fprintf(stderr, "Failed to get slave 1 configuration.\n"); // 第二个参数应该是从站的位置
    return -1;
  }
  if (ecrt_slave_config_pdos(sc_MX2ECT[0], EC_END, slave_5_syncs))
  { //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }
  /**************************************/
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
  
  /**************************************/
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
/**************************************  *************************************/
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
/**************************************  *************************************/
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
/**************************************  *************************************/
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
}
/**************************************  *************************************/
void ReleaseMaster()
{
  if (master)
  {
    printf("xenomai End of Program, release master\n");
    ecrt_release_master(master);
    master = NULL;
  }
}
/**************************************  *************************************/
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
  ecrt_master_application_time(master, system_time_ns());
  ret = ecrt_master_select_reference_clock(master, sc_asda[0]); //选择参考时钟
  if (ret < 0)
  {
    fprintf(stderr, "xenomai Failed to select reference clock: %s\n",
            strerror(-ret));
    return ret;
  }
  printf("xenomai Activating master...\n");
  if (ecrt_master_activate(master))
  {
    printf("xenomai Activating master...failed\n");
    return -1;
  }
  /**************************************/
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
/**************************************  *************************************/
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
/**************************************  *************************************/
void Servo_Motor_ErrorCapture()
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
/**************************************  *************************************/
void DriverEtherCAT()
{
  static int csping[drive_count] = {0}; //位置模式运行标志位
  static int cycle_counter = 0;         //周期计数
  static int home_param_setted = 0;
  static int home_point_all_find = false; //所有电机原点复归完成标志位
  static int csping_all_ready = false;    //所有电机切换csp成功标志位

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

      Servo_Motor_ErrorCapture();

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
    Servo_Motor_ErrorCapture();
    for (int i = 0; i < fre_count; i++)
    {
      // servo 5
      if (EC_READ_U16(domainInput_pd + ECT_Status[i]) & 0x8)
      {
        EC_WRITE_U16(domainOutput_pd + ECT_Command[i], 0x80);
        if (!(cycle_counter % 1000))
        {
          printf("\nECT ERROR!!!!\n*************************ERROR SAVLED!*************************\n");
        }
      }
      else
      {
        EC_WRITE_U16(domainOutput_pd + ECT_Command[i], 0);
        EC_WRITE_U16(domainOutput_pd + ECT_Frequency[i], ECTFrequency[i]);
        EC_WRITE_U16(domainOutput_pd + ECT_Command[i], 0x1);
        if (!(cycle_counter % 1000))
        {
          printf("                                                                                 ");
          printf("ECT_Status:0x%x\t", EC_READ_U16(domainInput_pd + ECT_Status[i]));
          printf("ECT_Output_frequency:0x%x\n", EC_READ_U16(domainInput_pd + ECT_Output_frequency[i]));
        }
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
/**********************************FreQC Move*********************************/
    for (int i = 0; i < fre_count; i++)
    {
      if (cycle_counter / 1000 == 20)
      {
        if (!(cycle_counter % 1000))
          printf("***************************STOP*************************\n");
        ECTFrequency[i] = 0;
      }
      if (cycle_counter / 1000 == 30)
      {
        if (!(cycle_counter % 1000))
          printf("***************************RUN*************************\n");
        ECTFrequency[i] = 0x2000;
      }
      if (cycle_counter / 1000 == 45)
      {
        if (!(cycle_counter % 1000))
          printf("***************************STOP*************************\n");
        ECTFrequency[i] = 0;
      }
    }
/**************************************  *************************************/
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
        }
        if (csp_drive_enable[i])
        {
          RunCSP(i, csp_pos[i], csp_speed[i]);
        }
      }
    }
  }
  break;
  
  // write application time to master
  ecrt_master_application_time(master, system_time_ns());
  ecrt_master_sync_reference_clock(master);
  ecrt_master_sync_slave_clocks(master);
  // send process data
  ecrt_domain_queue(domainServoOutput);
  ecrt_domain_queue(domainServoInput);
  ecrt_master_send(master);
}

/**************************************  *************************************
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <ethercat_igh/ethercat_igh.h>

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
**************************************  *************************************/
