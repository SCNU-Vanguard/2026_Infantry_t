/**
******************************************************************************
* @file    gimbal_task.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "gimbal_task.h"
#include "gimbal.h"

#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "rs485.h"
#include "remote_control.h"
#include "VPC.h"
#include "Serial.h"

#define GIMBAL_TASK_PERIOD 1 // ms

uint32_t cnt_test;

osThreadId_t gimbal_task_handel;

//static publisher_t *gimbal_publisher;
//static subscriber_t *gimbal_subscriber;

static void Gimbal_Task(void *argument);

void Gimbal_Task_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Gimbal_Task",
        .stack_size = 512 * 8,
        .priority = (osPriority_t)osPriorityRealtime4,
    };
    gimbal_task_handel = osThreadNew(Gimbal_Task, NULL, &attr);

//    gimbal_publisher = Publisher_Register("gimbal_transmit_feed", sizeof(gimbal_behaviour_t));
//    gimbal_subscriber = Subscriber_Register("gimbal_receive_cmd", sizeof(Gimbal_CmdTypedef));
}

uint32_t gimbal_task_diff;

static void Gimbal_Task(void *argument)
{
    uint32_t time = osKernelGetTickCount();
	Yaw_6006_Initial_Value = DM_6006_yaw -> receive_data.position;//获取初始上电位置编码器值用于处理IMU零点
	Yaw_IMU_Reference = Yaw_6006_Initial_Value - ANGLE_REFERENCE;
	
//	temp_v_yaw = Yaw_IMU_Reference;

    for (;;)
    {
		//云台遥控器控制
        Gimbal_Control_Remote();
		
		cnt_test++;
		Choose_VPC_Type();
        VPC_UpdatePackets(); //接收准备发送的数据
        //NV_Pack_And_Send_Data_ROS2(&nv_aim_packet_to_nuc); //导航数据包发送
        VS_Pack_And_Send_Data_ROS2(&vs_aim_packet_to_nuc); //视觉数据包发送
        // Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);

        gimbal_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + GIMBAL_TASK_PERIOD);
    }
}