#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_frame_config.h"
#include "robot_frame_init.h"





static void Control_Task(void *argument);
void Control_Task_Init(void);


#endif
