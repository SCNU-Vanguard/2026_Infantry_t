/**
******************************************************************************
* @file    robot_frame_init.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include "robot_frame_init.h"

#include "robot_frame_config.h"

// TODO(GUATAI):daemon_task是否需要看个人理解，可以把daemon_task去掉，
// 替换成其他功能任务，任务最好不要超过8个
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "daemon_task.h"
#include "INS_task.h"
#include "procotol_task.h"
#include "control_task.h"


#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "daemon.h"
#include "INS.h"
#include "procotol.h"


#include "bmi088.h"
#include "ws2812.h"
#include "buzzer.h"
#include "vofa.h"

#include "bsp_dwt.h"
#include "bsp_usart.h"

#include "BMI088driver.h"
#include "remote_control.h"

float init_time;
RC_ctrl_t *rc_ctl;

static void Frame_MCU_Init(void)
{
    DWT_Init(480);
}

static void Frame_Device_Init(void)
{
    Buzzer_Register();
    ws2812_instance = WS2812_Register(&ws2812_config);

    bmi088_h7 = BMI088_Register(&bmi088_init_h7);

    rc_ctl = Remote_Control_Init(&huart5);//遥控器数据

    VOFA_Register(&huart7);

    Chassis_Init();

    Gimbal_Init();
    //Shoot_Init();
}

/*
    @brief 任务初始化函数
*/
static void Frame_Task_Init(void)
{
  
    Buzzer_Task_Init();

    INS_Task_Init();
    
    Control_Task_Init();
    
    Chassis_Task_Init();

    Gimbal_Task_Init();

    //Shoot_Task_Init();

    //Procotol_Task_Init();

    //VPC_Task_Init();
}

void Robot_Frame_Init(void)
{
    Frame_MCU_Init();//DWT初始化,延时初始化

    Frame_Device_Init();//框架使用外设初始化

    Frame_Task_Init();//任务初始化
}
