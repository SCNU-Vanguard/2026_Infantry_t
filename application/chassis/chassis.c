/**
******************************************************************************
* @file    chassis.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "chassis.h"
#include "vofa.h"

DJI_motor_instance_t *chassis_m3508[4];
Chassis_CmdTypedef chassis_cmd;
float target_speed[4] = {0};//底盘解算出的电机目标值
float VOFA_speed[4];//vofa调参用的

/*TEST*/
float test_omega;

//注意堆栈大小，使用同一个结构体，堆栈太小到会导致配置错误
PID_t chassis_3508_speed_pid = {
    .kp = 20.005f,
    .ki = 0.105f,
    .kd = 0.0f,
    .output_limit = 5000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_3508_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },

    .motor_control_type = 0,//和dji无关
};


/*
 * @brief  	麦轮运动学逆解算(3508电机)
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑麦轮俯视打叉，仰视画圆且轴长不等的情况,以辊子与地面接触为准
 */
void Mecanum_Solve(Chassis_CmdTypedef *cmd, float *ret)
{
 	/*         vx
				^
				|
		   0//      \\1
		   // \    / \\
			   top      -->vy
		   \\ /    \ //
		   3\\      //2
					
	*/
  float omega_z = cmd->omega_z + cmd->omega_follow;

  ret[0] = (((omega_z * (( LENGTH + WIDTH) / 2) / M3508_REDUCTION_RATIO) - (-cmd->vx - cmd->vy)) / WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS);
  ret[1] = (((omega_z * (( LENGTH + WIDTH) / 2) / M3508_REDUCTION_RATIO) - (cmd->vx - cmd->vy)) / WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS);
  ret[2] = (((omega_z * (( LENGTH + WIDTH) / 2) / M3508_REDUCTION_RATIO) - (cmd->vx + cmd->vy)) / WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS);
  ret[3] = (((omega_z * (( LENGTH + WIDTH) / 2) / M3508_REDUCTION_RATIO) - (-cmd->vx + cmd->vy)) / WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS);
}

/*
 * @brief  	麦轮逆解算(3508电机)
 * @param	底盘四电机结构体指针ID，顺序2-5
 * @retval 	底盘自旋速度，rad/s
 * @todo    按需增加对里程的解算
 */
float Chassis_Get_Omega (DJI_motor_instance_t *M3508[])
{
	uint8_t i = 0;
	float v_total = 0;//四个3508转子的角速度之和

	for(i = 0; i < 4; i++)
    {
        v_total += M3508[i]->measure.speed; //单位rpm
    }
		 
	v_total = v_total/4;                                            //平均一个3508 ， 单位rpm
	v_total = v_total * RPM_2_RAD_PER_SEC /60;                      // 转子 --> 转轴   rpm-->rps
	v_total = v_total * WHEEL_RADIUS / ( ( LENGTH + WIDTH) / 2); // rps （轮子的线速度 = 底盘的线速度） 单位变换时要调内环PID
	return v_total;    
}

/*
 * @brief  	底盘电机初始化
 * @param	无
 * @retval 	无
 */
void Chassis_Init(void)
{
    for(int i = 0; i <4; i++)
    {
        chassis_3508_init.can_init_config.tx_id = 0x01 + i;
        chassis_3508_init.can_init_config.rx_id = 0x01 + i;
        chassis_m3508[i] = DJI_Motor_Init(&chassis_3508_init);
    }
}

void Chassis_Enable(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Enable(chassis_m3508[i]);
    }
}

void Chassis_Stop(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Stop(chassis_m3508[i]);
    }
}

void Chassis_Ctrl_Remote(void)
{
        //底盘解算
        Mecanum_Solve(&chassis_cmd, target_speed);
        //获取底盘自旋速度
        chassis_cmd.omega_follow = - Chassis_Get_Omega(chassis_m3508);
        //设目标值
        for(int i = 0; i < 4; i++)
        {
			
			//////////////////////////////////////调试用
//			if(target_speed[i] < -10)
//			{
//				target_speed[i] = -200;
//			}
//			else if(target_speed[i] > 10)
//			{
//				target_speed[i] = 200;
//			}
			//////////////////////////////////////////////////
			
            DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
        }

        //模式处理
        if(chassis_cmd.mode == SPIN)
        {
            Chassis_Enable();
            chassis_cmd.omega_z = 0.3f;
        }

        else if(chassis_cmd.mode == FOLLOW)
        {
            Chassis_Enable();
        }
        else if(chassis_cmd.mode == STOP_C)
        {
            chassis_cmd.omega_z = 0;
            chassis_cmd.vx = 0;
            chassis_cmd.vy = 0; 
            Chassis_Stop();
           
        }

        DJI_Motor_Control();//电机pid计算及发送控制报文 , 与波弹盘拆解 
		
		/////////////////////////////////////////vofa调参用的代码
		VOFA_JustFloat(VOFA_speed, 4);
}
