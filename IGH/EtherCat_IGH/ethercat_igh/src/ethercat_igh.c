/*
 * @Descripttion: Null
 * @version: 1.0
 * @Author: Mar Ping
 * @Date: 2020-11-14 13:50:47
 * @LastEditors: Mar Ping
 * @LastEditTime: 2020-11-14 16:55:56
 */
#include <errno.h>
#include <ethercat_igh/ethercat_igh.h>
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

/****************************************************************************/
//  Time type
static int64_t system_time_base = 0LL;
//  获取当前系统时间
RTIME system_time_ns(void)
{
    struct timespec rt_time;
    clock_gettime(CLOCK_TO_USE, &rt_time);
    RTIME time = TIMESPEC2NS(rt_time);
    return time - system_time_base;
}
/****************************************************************************/

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

    // slave N
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

void ErrorCapture()
{
    int drive_count = 1;
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

/****************************************************************************/
