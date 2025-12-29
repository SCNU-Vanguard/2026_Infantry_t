/**
******************************************************************************
* @file    gimbal.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include "gimbal.h"

Gimbal_CmdTypedef gimbal_cmd;
//2*4310 + 6006 + 1*2006
DM_motor_t *DM_4310_pitch_head;//云台头，imu串级控制
DM_motor_t *DM_4310_pitch_neck;//云台脖子，单位置环
DM_motor_t *DM_6006_yaw;//云台yaw电机，imu串级控制



/*云台目标值*/
float temp_v_yaw;
float temp_v_pitch_neck;
float temp_v_pitch_head;

/*测试值*/
float yaw_speed_target = 0;
float yaw_speed_measure = 0;

PID_t angle_pid_yaw = {
	.kp = -1.5f,        //注意pid输出方向
	.ki = 0.0f,
	.kd = -100,
	.integral_limit = 0.0f,
	.output_limit = 2.0f,//3rad/s
	.dead_band = 0.0f,
};

PID_t speed_pid_yaw = {
	.kp = 5.0f,
	.ki = 0.01f,
	.kd = 15.0f,
	.integral_limit = 100.0f,
	.output_limit = 50.0f,
	.dead_band = 0.0f,
};


PID_t angle_pid_pitch_head = {
	.kp = 10.0f,	//注意输出方向
	.ki = 0.0f,
	.kd = 200.0f,
	.integral_limit = 0.0f,
	.output_limit = 500.0f,
	.dead_band = 0.0f,
};

PID_t speed_pid_pitch_head = {
	.kp = 0.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.integral_limit = 0.0f,
	.output_limit = 500.0f,
	.dead_band = 0.0f,
};

/* 使用speed_pos_mode ，pid不计算 */
PID_t angle_pid_pitch_neck = {
	.kp = 0.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.integral_limit = 0.0f,
	.output_limit = 500.0f,
	.dead_band = 0.0f,
};

PID_t speed_pid_pitch_neck = {
	.kp = 0.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.integral_limit = 0.0f,
	.output_limit = 500.0f,
	.dead_band = 0.0f,
};

/*  
* @brief 达妙电机配置初始化结构体
*/
//DM_MOTOR_ABSOLUTE
motor_init_config_t dm_6006_yaw = {
    .controller_param_init_config = {
        .angle_PID = &angle_pid_yaw,
        .speed_PID = &speed_pid_yaw,
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
        .outer_loop_type = ANGLE_LOOP,//双环
        //.outer_loop_type = SPEED_LOOP,//单速度环测试
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = OTHER_FEED, //使用imu作为角度反馈
        //.angle_feedback_source = MOTOR_FEED,//使用电机can反馈信息
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = SPEED_FEEDFORWARD,//添加小陀螺转速补偿
    },

    .motor_type = DM6220,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
    .motor_control_type = MIT_MODE_E,//达妙电机力矩模式
};

motor_init_config_t p_n_4310 = {

	.motor_type = DM4310,

	.controller_param_init_config = {
			.other_angle_feedback_ptr = NULL, 
			.other_speed_feedback_ptr = NULL,//其他反馈来源的数据指针

			.angle_feedforward_ptr = NULL, 
			.speed_feedforward_ptr = NULL, 
			.current_feedforward_ptr = NULL, 
			.torque_feedforward_ptr = NULL, //ptr,前馈数据指针

			.angle_PID = &angle_pid_pitch_neck,//电机控制器的pid指针
			.speed_PID = &speed_pid_pitch_neck,
			.current_PID = NULL,
			.torque_PID = NULL,
			
			.pid_ref = 0.0f,//pid目标值
	},

	.controller_setting_init_config = {
		.outer_loop_type = OPEN_LOOP,//串级PID外环模式
		.close_loop_type = OPEN_LOOP,//串级PID内环模式 ， dm4310_neck使用电机的位置控制模式，不适用双环

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,//默认为正转模式
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,//反馈量正向

		.angle_feedback_source = MOTOR_FEED,//角度反馈来源，可以为电机can反馈信息或其他来源
		.speed_feedback_source = MOTOR_FEED,//速度反馈来源，一般选择电机can反馈信息 ， 其他来源则使用other_speed_feedback_ptr指针

		.feedforward_flag = FEEDFORWARD_NONE,//前馈反馈模式，可选择扭矩/速度，或两者皆有
	},

	.can_init_config = {    // can配置
		.can_handle = &hfdcan2,
		.tx_id = 0x02,
		.rx_id = 0x12,
		.can_module_callback = NULL,//在电机里具体定义
		.id = NULL,
	},
	
	.motor_control_type = POS_MODE_E,//电机控制模式，用于初始化，适配达妙电机三种模式
};


motor_init_config_t p_h_4310 = {
    .controller_param_init_config = {
        .angle_PID = &angle_pid_pitch_head,
        .speed_PID = &speed_pid_pitch_head,
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
        .outer_loop_type = ANGLE_LOOP,//
        .close_loop_type = ANGLE_LOOP,//单角度环，TODO:后续可改为双环/使用位置速度模式

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = OTHER_FEED,//注意 使用imu作为角度反馈
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = DM4310,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x13,
    },
    .motor_control_type = MIT_MODE_E,//达妙电机力矩模式,TODO:后续可改为speed_pos_mode ,更简单且不用调参，但需和视觉做适配
};


/*head -->  （0.01） 0 ~ -0.35      rad */
/*neck -->  （0.002） -0.01 ~ -1  rad */


void Gimbal_Init(void)
{
    /*DM电机注册*/
	#if ROLL_TO_PITCH	//排放位置导致roll为头的pitch
			p_h_4310.controller_param_init_config.other_angle_feedback_ptr = &INS.Roll;//使用imu的pitch角度作为head电机的角度反馈
    #else
        p_h_4310.controller_param_init_config.other_angle_feedback_ptr = &INS.Pitch;
    #endif
	  dm_6006_yaw.controller_param_init_config.other_angle_feedback_ptr = &INS.Yaw;//使用imu的yaw角度作为yaw电机的角度反馈
    dm_6006_yaw.controller_param_init_config.speed_feedforward_ptr = &chassis_cmd.omega_follow;//添加小陀螺转速补偿

    DM_4310_pitch_head = DM_Motor_Init(&p_h_4310);

    DM_4310_pitch_neck = DM_Motor_Init(&p_n_4310);

    DM_6006_yaw = DM_Motor_Init(&dm_6006_yaw);

}

void Gimbal_Enable(void)
{
    DM_Motor_ENABLE(DM_4310_pitch_head);
    DM_Motor_ENABLE(DM_4310_pitch_neck);
    DM_Motor_ENABLE(DM_6006_yaw);
}

void Gimbal_Disable(void)
{
    DM_Motor_DISABLE(DM_4310_pitch_head);
    DM_Motor_DISABLE(DM_4310_pitch_neck);
    DM_Motor_DISABLE(DM_6006_yaw);
}

void Gimbal_Stop(void)
{
    DM_Motor_Stop(DM_4310_pitch_head);
    DM_Motor_Stop(DM_4310_pitch_neck);
    DM_Motor_Stop(DM_6006_yaw);
}

void Gimbal_Control_Remote(void)
{
	//获取初始pitch角度
    //temp_v_yaw = g_c->v_yaw + DM_6006_yaw.motor_controller.pid_ref;
    while(INS_GET_PITCH() != 0 && gimbal_cmd.pitch_init == 0)//首次初始化 , 先确保imu初始化完成避免初始值错误
    {
        gimbal_cmd.pitch_init = INS_GET_PITCH();
				temp_v_pitch_head = gimbal_cmd.pitch_init + 0.005f; // 滤波最终收敛左右
    }
    
 
	//根据neck角度来更改pitch上下限角度
    if (gimbal_cmd.ctrl_mode == CTRL_NECK)
    {
        Gimbal_Enable();
        DM_Motor_Start(DM_4310_pitch_head);
        DM_Motor_Start(DM_4310_pitch_neck);
        DM_Motor_Start(DM_6006_yaw);
        
        // temp_v_yaw = gimbal_cmd.v_yaw; //单速度环测试 ，注意gimbal_cmd.v_yaw的量级
        // if ( temp_v_yaw > YAW_MAX_SPEED )	
        //         temp_v_yaw = YAW_MAX_SPEED;
        // else if( temp_v_yaw < -YAW_MAX_SPEED )
        //         temp_v_yaw = -YAW_MAX_SPEED;//保持在2 rad/s 范围内
			
        temp_v_yaw += gimbal_cmd.v_yaw;
        if ( temp_v_yaw > PI )
                temp_v_yaw -= 2 * PI;
        else if( temp_v_yaw < -PI )
                temp_v_yaw += 2 * PI;//保持在-pi ~ PI范围内
																							
        DM_Motor_SetTar(DM_6006_yaw, temp_v_yaw);//设置目标值  ， pid_out 顺负逆正  ， v 顺正
			
		/*p_head*/	
        temp_v_pitch_head = gimbal_cmd.pitch_init;
        USER_LIMIT_MIN_MAX(temp_v_pitch_head, PITCH_HEAD_MAX_ANGLE, PITCH_HEAD_MIN_ANGLE);//head 最小限制
        DM_Motor_SetTar(DM_4310_pitch_head, temp_v_pitch_head);//设置目标值

        /*p_neck*/ /*POS_mode 做特殊处理*/
		temp_v_pitch_neck += gimbal_cmd.v_pitch_neck;
        USER_LIMIT_MIN_MAX(temp_v_pitch_neck, PITCH_NECK_MAX_ANGLE, PITCH_NECK_MIN_ANGLE);//限制脖子电机转动范围
        DM_4310_pitch_neck -> transmit_data.velocity_des = PITCH_NECK_MAX_SPEED;   //设置转动时的最大速度
        DM_Motor_SetTar(DM_4310_pitch_neck, temp_v_pitch_neck);//设置目标值
    }

    else if(gimbal_cmd.ctrl_mode == CTRL_HEAD)
    {
		temp_v_yaw += gimbal_cmd.v_yaw;
        if ( temp_v_yaw > PI )
                temp_v_yaw -= 2 * PI;
        else if( temp_v_yaw < -PI )
                temp_v_yaw += 2 * PI;//保持在-pi ~ PI范围内
																							
        DM_Motor_SetTar(DM_6006_yaw, temp_v_yaw);//设置目标值  ， pid_out 顺负逆正  ， v 顺正
		
			//
		temp_v_pitch_head += gimbal_cmd.v_pitch_head;
        USER_LIMIT_MIN_MAX(temp_v_pitch_head, PITCH_HEAD_MAX_ANGLE, PITCH_HEAD_MIN_ANGLE);
        DM_Motor_SetTar(DM_4310_pitch_head, temp_v_pitch_head);//设置目标值
            
        //维持neck位置
        DM_Motor_SetTar(DM_4310_pitch_neck, temp_v_pitch_neck);
    }
		
     else if (gimbal_cmd.ctrl_mode == STOP_GIMBAL)
     {
			 //temp_v_yaw = 0;
			 temp_v_pitch_head = gimbal_cmd.pitch_init + 0.005f;
			 temp_v_pitch_neck = PITCH_NECK_MIN_ANGLE;
			 //维持当前角度但不作PID计算，pid_ref清零，不计算
			 
			 Gimbal_Stop();
			 Gimbal_Disable();
			 //TODU:优化DM_motor
			 DM_6006_yaw->motor_controller.speed_PID->output = 0;
       DM_6006_yaw->motor_controller.angle_PID->output = 0;
     }

    //全部电机计算
    DM_Motor_Control();
		 
		yaw_speed_measure = DM_6006_yaw->motor_controller.speed_PID->measure;
		yaw_speed_target = DM_6006_yaw->motor_controller.speed_PID->target;
}