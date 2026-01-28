/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot_motor.h"

#define SHOOT_V 6000  //摩擦轮转速 rpm

DJI_motor_instance_t *chassis_shoot_motor;
shoot_motor_instance_t *friction_motor[3];

uint16_t target_shoot_frequence = 0;
uint8_t shoot_mode = 0;
uint8_t shoot_permission = 0;
uint8_t test = 0;

PID_t chassis_2006_speed_pid = {
    .kp = 30.0f,
    .ki = 2.0f,
    .kd = 0.0f,
    .output_limit = 9000.0f,
    .integral_limit = 9000.0f,
    .dead_band = 0.0f,
};

PID_t friction_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 480.0f,
    .output_limit = 10.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t friction_speed_pid = {
    .kp = 400.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_2006_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_2006_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M2006,

    .can_init_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x06,
        .rx_id = 0x06,
    },

    .motor_control_type = 0,
};

motor_init_config_t friction_motor_init = {
    .controller_param_init_config = {
        .angle_PID = &friction_angle_pid,
        .speed_PID = &friction_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = SNAIL,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x011,
    },
};

void Shoot_Init(void)
{
    chassis_shoot_motor = DJI_Motor_Init(&chassis_2006_init);
	
	for(int i = 0;i < 3;i++)
	{
		friction_motor_init.can_init_config.tx_id = 0x01 + i;
        friction_motor_init.can_init_config.rx_id = 0x011 + i;
		friction_motor[i] = Shoot_Motor_Init(&friction_motor_init);
	}
}

void Shoot_Enable(void)
{
    DJI_Motor_Enable(chassis_shoot_motor);
	
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Enable(friction_motor[i]);
    }
}

void Shoot_Stop(void)
{
    DJI_Motor_Stop(chassis_shoot_motor);
	
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Stop(friction_motor[i]);
    }
}

void Shoot_Set_All_Friction(int16_t speed)
{
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_SetTar(friction_motor[i],speed);
    }
}

void Shoot_State_Machine(void)
{
    static uint16_t init_count = 0;

}

void Shoot_Control_Remote(void)
{
	switch(shoot_mode)
	{
		case SHOOT_MODE_STOP:
		{
			test = 0;
			Shoot_Stop();
			break;
		}
		case SHOOT_MODE_FIRE://需要写保护，龟头必须抬起来才能转摩擦轮，摩擦轮转起来才能转拨弹盘
		{
			if(shoot_permission)
			{
				test = 1;
				
				Shoot_Enable();
				
				Shoot_Set_All_Friction(SHOOT_V);
				
				if(friction_motor[0] -> receive_flag == 0xA5 && friction_motor[1] -> receive_flag == 0xA5 && friction_motor[2] -> receive_flag == 0xA5)
				{
					DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
				}
			}
			else
			{
				test = 0;
				
				Shoot_Stop();
			}
			break;
		}
		default:
		{
			Shoot_Stop();
			break;
		}
	}
	//DJI_Motor_Control();
	Shoot_Motor_Send();
}
