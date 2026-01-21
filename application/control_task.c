#include "control_task.h"


#define CONTROL_TASK_PERIOD 1 // ms
#define ANGLE_REFERENCE -3.0359385  //底盘坐标系转云台坐标系角度参考值
#define BIAS_DEADBAND 0.1 //底盘坐标系转云台坐标系角度死区
#define YAW_DEADBAND 3 //yaw轴死区值
osThreadId_t control_task_handle;
uint32_t control_task_diff;//任务周期

// static publisher_t *chassis_publisher;
// static subscriber_t *chassis_subscriber;

static void Control_Task(void *argument);

void Control_Task_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Control_Task",
        .stack_size = 128 * 8,
        .priority = (osPriority_t)osPriorityRealtime2,//优先级最低
    };
    control_task_handle = osThreadNew(Control_Task, NULL, &attr);

    // chassis_publisher = Publisher_Register("chassis_transmit_feed", sizeof(chassis_behaviour_t));
    // chassis_subscriber = Subscriber_Register("chassis_receive_cmd", sizeof(chassis_cmd_t));
}


/*
 * @brief  	底盘坐标系转云台坐标系
 * @param	底盘命令结构体指针
 * @param   底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度（编码值）
 * @param   yaw电机当前角度（-PI ~ PI）
 * @retval 	无
 */
void Chassis_Cmd_Trans (Chassis_CmdTypedef *cmd, float chs_zeropoint, float gim_ang)
{
	float bias_tmp = (gim_ang - chs_zeropoint + 2 * PI) ; 

	if ( bias_tmp > 2 * PI )
		bias_tmp -= 2 * PI;
	else if ( bias_tmp < 0 )
		bias_tmp += 2 * PI;
	
	if(bias_tmp < BIAS_DEADBAND)//死区
	{
		bias_tmp = 0;
	}
	
	yaw_to_mid = cosf (bias_tmp);
	
//	float vx_tmp = cmd -> vx * cosf (bias_tmp) - cmd -> vy * sinf (bias_tmp);
//	float vy_tmp = cmd -> vx * sinf (bias_tmp) + cmd -> vy * cosf (bias_tmp);
//	cmd -> vx = vx_tmp;
//	cmd -> vy = vy_tmp;
}


void Remote_Ctrl (Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{
//注意正反
	chs -> vx = (float) rc_ctl -> rc . rocker_l1 * REMOTE_X_SEN ;
	chs -> vy = (float) rc_ctl -> rc . rocker_l_ * REMOTE_Y_SEN ;

	if(rc_ctl -> rc . rocker_r_ < YAW_DEADBAND && rc_ctl -> rc . rocker_r_ > -YAW_DEADBAND)//yaw轴死区
	{
		rc_ctl -> rc . rocker_r_ = 0;
	}
	gim -> v_yaw = (float) - rc_ctl -> rc . rocker_r_ * REMOTE_YAW_SEN;

	
	//模式处理
	if( rc_ctl -> rc . switch_left == 1 )//暂时用于测试
	{
//		chs -> mode = SPIN;
		//chs -> omega_z = 0.5 ;
		chs -> mode = STOP_C;
		shoot_mode = SHOOT_MODE_FIRE;
		target_shoot_frequence = rc_ctl->rc . dial / 660.0 * 300.0;
	}
	else if( rc_ctl -> rc . switch_left == 3 )
	{
		chs -> mode = FOLLOW;
		chs -> omega_z = -(rc_ctl->rc . dial * REMOTE_OMEGA_Z_SEN) ;
		gim -> status = GIMBAL_ENABLE;
		shoot_mode = SHOOT_MODE_STOP;
	}
	else if( rc_ctl -> rc . switch_left == 2 ||  rc_ctl -> rc . switch_left == 0)
	{
		chs -> mode = STOP_C;
		gim -> status = GIMBAL_DISABLE;
		shoot_mode = SHOOT_MODE_STOP;
	}
	

	//云台控制模式处理
	if( rc_ctl -> rc . switch_right == 1 )
	{
//		gim -> ctrl_mode = CTRL_HEAD; //动pitch_head,锁neck
//		gim -> v_pitch_head = (float)- rc_ctl -> rc . rocker_r1 * REMOTE_PITCH_SEN ;//
	}
	else if( rc_ctl -> rc . switch_right == 3 )
	{
		gim -> ctrl_mode = STAND_NECK; //动pitch_neck,head水平
		gim -> v_pitch_head = (float)- rc_ctl -> rc . rocker_r1 * REMOTE_PITCH_SEN ;//
	}
	else if( rc_ctl -> rc . switch_right == 2 || rc_ctl -> rc . switch_right == 0)
	{
		gim -> ctrl_mode = SIT_NECK;
//		gim -> v_pitch_neck = (float)- rc_ctl -> rc . rocker_r1 * REMOTE_PITCH_SEN;//对应加减
	}

}

static void Control_Task(void *argument)
{
    uint32_t time = osKernelGetTickCount();

    //Send_Packet_Init(&aim_packet_to_nuc);//与上位机通讯
    for (;;)
    {
        
        Remote_Ctrl (&gimbal_cmd, &chassis_cmd);
		Chassis_Cmd_Trans(&chassis_cmd, ANGLE_REFERENCE, DM_6006_yaw -> receive_data.position);//底盘坐标系转云台坐标系

        control_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + CONTROL_TASK_PERIOD);
    }
}

