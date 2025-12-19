/**
* @file chassis_task.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"


#include "chassis.h"
#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "remote_control.h"

#include <stdint.h>
#include "robot_frame_init.h"
#include "pid.h"

extern void Chassis_Task_Init(void);

#endif /* __CHASSIS_TASK_H__ */