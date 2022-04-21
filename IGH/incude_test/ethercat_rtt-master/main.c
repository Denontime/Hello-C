#include "ethercat_igh.h"

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