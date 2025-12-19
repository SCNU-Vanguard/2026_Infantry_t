/**
******************************************************************************
* @file    chassis_task.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "chassis_task.h"

#define CHASSIS_TASK_PERIOD 1 // ms
uint32_t chassis_task_diff;
osThreadId_t robot_cmd_task_handel;

//static publisher_t *chassis_publisher;
//static subscriber_t *chassis_subscriber;

static void Chassis_Task(void *argument);

void Chassis_Task_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Chassis_Task",
        .stack_size = 128 * 8,
        .priority = (osPriority_t)osPriorityRealtime4,
    };
    robot_cmd_task_handel = osThreadNew(Chassis_Task, NULL, &attr);

//    chassis_publisher = Publisher_Register("chassis_transmit_feed", sizeof(chassis_behaviour_t));
//    chassis_subscriber = Subscriber_Register("chassis_receive_cmd", sizeof(chassis_cmd_t));
}


static void Chassis_Task(void *argument)
{
    uint32_t time = osKernelGetTickCount();


    for (;;)
    {
        //底盘控制任务
        Chassis_Ctrl_Remote();

        chassis_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + CHASSIS_TASK_PERIOD);
    }
}