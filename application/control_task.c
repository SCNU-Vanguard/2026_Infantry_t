#include "control_task.h"
#include <math.h>

#define CONTROL_TASK_PERIOD 1 // ms
#define BIAS_DEADBAND 0.1 //底盘坐标系转云台坐标系角度死区
#define YAW_DEADBAND 3 //yaw轴死区值
#define MAX_ABS_INCREMENT 0.003 //限制电机转速增量的值
#define CONTROL_MODE 1	//1为遥控器，0为键鼠

osThreadId_t control_task_handle;
uint32_t control_task_diff;//任务周期
float vx_current;
float vx_target;
float vy_current;
float vy_target;
float temp_x;

float A = 3.0f;
float w = 0.5f;
uint32_t x = 0;
uint32_t t = 0;

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
	
//	if(bias_tmp < BIAS_DEADBAND)//死区
//	{
//		bias_tmp = 0;
//	}
	
	yaw_to_mid = cosf (bias_tmp);
	
	///////////////////////////////////////////////////////////////////////////////////////////不要底盘坐标系转云台坐标系就注释这一段
	float vx_tmp = cmd -> vx * cosf (bias_tmp) - cmd -> vy * sinf (bias_tmp);
	float vy_tmp = cmd -> vx * sinf (bias_tmp) + cmd -> vy * cosf (bias_tmp);
	cmd -> vx = vx_tmp;
	cmd -> vy = vy_tmp;
	///////////////////////////////////////////////////////////////////////////////////////////
}

/*
 * @brief  	限制电机转速增量
 * @param	当前值
 * @param   目标值
* @retval 	加上限制后的目标值
 */
float Get_target_with_limitation(float current, float target)
{
	float limited_output = current;//默认值为当前值，避免异常
	float diff = target - current;
	
	if(fabsf(target) > fabsf(current))//速度增大才有限制
	{
		if(diff > 0)
		{
			limited_output = current + ((diff > MAX_ABS_INCREMENT) ? MAX_ABS_INCREMENT : diff);
		}
		else if (diff < 0) 
        {
            limited_output = current - ((fabsf(diff) > MAX_ABS_INCREMENT) ? MAX_ABS_INCREMENT : fabsf(diff));
        }
	}
	else 
    {
        limited_output = target;
    }
	
	return limited_output;
}

void Remote_Ctrl (Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)
{
//注意正反
//	vx_current = chs -> vx;
//	vx_target = (float) rc_ctl -> rc . rocker_l1 * REMOTE_X_SEN ;
//	vy_current = chs -> vy;
//	vy_target = (float) rc_ctl -> rc . rocker_l_ * REMOTE_Y_SEN ;
//	
//	chs -> vx = Get_target_with_limitation(vx_current, vx_target);
//	chs -> vy = Get_target_with_limitation(vy_current, vy_target);
	
	
	chs -> vx = (float) rc_ctl -> rc . rocker_l1 * REMOTE_X_SEN ;
	chs -> vy = (float) rc_ctl -> rc . rocker_l_ * REMOTE_Y_SEN ;

	if(rc_ctl -> rc . rocker_r_ < YAW_DEADBAND && rc_ctl -> rc . rocker_r_ > -YAW_DEADBAND)//yaw轴死区
	{
		rc_ctl -> rc . rocker_r_ = 0;
	}
	gim -> v_yaw = (float) - rc_ctl -> rc . rocker_r_ * REMOTE_YAW_SEN;

	
	//模式处理
	if( rc_ctl -> rc . switch_left == 1 )										//启动发射结构
	{
//		chs -> mode = SPIN;
		//chs -> omega_z = 0.5 ;
		chs -> mode = FOLLOW;
		chs -> omega_z = 0;
		
		shoot_mode = SHOOT_MODE_FIRE;
		target_shoot_frequence = abs(rc_ctl->rc . dial) / 660.0 * 300.0;
	}
	else if( rc_ctl -> rc . switch_left == 3 )									//全车使能
	{
		chs -> mode = FOLLOW;
		chs -> omega_z = -(rc_ctl->rc . dial * REMOTE_OMEGA_Z_SEN) ;			//小陀螺转速
		
		gim -> status = GIMBAL_ENABLE;
		
		shoot_mode = SHOOT_MODE_STOP;
	}
	else if( rc_ctl -> rc . switch_left == 2 ||  rc_ctl -> rc . switch_left == 0)//全车失能
	{
		chs -> mode = STOP_C;
		
		gim -> status = GIMBAL_DISABLE;
		
		shoot_mode = SHOOT_MODE_STOP;
	}
	

	//云台控制模式处理
	if( rc_ctl -> rc . switch_right == 1 )
	{
//		x += 1;						//中期文档底盘使用
////		if(w*x > 2*PI)
////		{
////			x = 0;
////		}
//		if(A > 3.3)
//		{
//			A = 3.3;
//		}
//		chs -> vx = (float)(A*sinf(0.01*w*x));
		
//		if(++t > 1500)				//中期文档云台使用
//		{
//			gim -> v_yaw = PI / 6;
//			t = 0;
//		}
		
		gim -> ctrl_mode = AUTOMATIC_AIMING;	//自瞄
	}
	else if( rc_ctl -> rc . switch_right == 3 )
	{
		gim -> ctrl_mode = STAND_NECK;			//伸头
		gim -> v_pitch_head = (float)- rc_ctl -> rc . rocker_r1 * REMOTE_PITCH_SEN ;//
	}
	else if( rc_ctl -> rc . switch_right == 2 || rc_ctl -> rc . switch_right == 0)
	{
		gim -> ctrl_mode = SIT_NECK;			//缩头
	}

}

void KeyboardCtrl(Gimbal_CmdTypedef *gim, Chassis_CmdTypedef *chs)//键鼠
{
	if(rc_ctl -> rc . switch_left == 3)//先加个遥控器的档位做保护
	{
		chs -> vx = Ramp_data1((uint8_t)rc_ctl -> key[KEY_PRESS].w) - Ramp_data2((uint8_t)rc_ctl -> key[KEY_PRESS].s);
		chs -> vy = Ramp_data3((uint8_t)rc_ctl -> key[KEY_PRESS].d) - Ramp_data4((uint8_t)rc_ctl -> key[KEY_PRESS].a);
		chs -> omega_z = Ramp_data5((uint8_t)rc_ctl -> key[KEY_PRESS].e) - Ramp_data6((uint8_t)rc_ctl -> key[KEY_PRESS].q);	//小陀螺转速
		
		if(fabsf(temp_x - rc_ctl -> mouse.x) <= KEYBOARD_YAW_MAX_ADD)
		{
			temp_x = rc_ctl -> mouse.x;
		}
		else if(temp_x < rc_ctl -> mouse.x)
		{
			temp_x += KEYBOARD_YAW_MAX_ADD;
		}
		else if(temp_x > rc_ctl -> mouse.x)
		{
			temp_x -= KEYBOARD_YAW_MAX_ADD;
		}
		gim -> v_yaw = (float)Gimbal_limit(temp_x) * KEYBOARD_YAW_SEN;
//		gim -> v_yaw = (float)Gimbal_limit((float)rc_ctl -> mouse.x) * KEYBOARD_YAW_SEN;
		gim -> v_pitch_head = (float)Gimbal_limit((float)rc_ctl -> mouse.y) * KEYBOARD_PITCH_SEN;
		
		if(rc_ctl -> key[KEY_PRESS_WITH_CTRL].b)//全车使能
		{
			chs -> mode = FOLLOW;
			gim -> status = GIMBAL_ENABLE;
			gim -> ctrl_mode = SIT_NECK;//缩头
			shoot_mode = SHOOT_MODE_STOP;
		}
		else if(rc_ctl -> key[KEY_PRESS].b)//全车失能
		{
			chs -> mode = STOP_C;
			gim -> status = GIMBAL_DISABLE;
			shoot_mode = SHOOT_MODE_STOP;
		}
		
		if(rc_ctl -> key[KEY_PRESS].c)//伸头
		{
			gim -> ctrl_mode = STAND_NECK;
		}
		else if(rc_ctl -> key[KEY_PRESS].v)//缩头
		{
			gim -> ctrl_mode = SIT_NECK;
		}
		
		if(gim -> ctrl_mode == STAND_NECK)//伸头时判断是否开自瞄
		{
			if(rc_ctl -> mouse.press_r)
			{
				gim -> ctrl_mode = AUTOMATIC_AIMING;//自瞄
			}
		}
		else if(gim -> ctrl_mode == AUTOMATIC_AIMING)//自瞄时判断是否退出自瞄
		{
			if(!(rc_ctl -> mouse.press_r))
			{
				gim -> ctrl_mode = STAND_NECK;//自瞄
			}
		}
		
		if(rc_ctl -> key[KEY_PRESS].f)//开摩擦轮
		{
			shoot_mode = SHOOT_MODE_FIRE;
		}
		else if(rc_ctl -> key[KEY_PRESS].r)//关摩擦轮
		{
			shoot_mode = SHOOT_MODE_STOP;
		}
		
		if(gim -> ctrl_mode == STAND_NECK || gim -> ctrl_mode == AUTOMATIC_AIMING)//抬头和自瞄时检测是否开枪
		{
			if(rc_ctl -> mouse.press_l)
			{
				target_shoot_frequence = 80;
			}
			else
			{
				target_shoot_frequence = 0;
			}
		}
	}
	else
	{
		chs -> mode = STOP_C;
		gim -> status = GIMBAL_DISABLE;
		shoot_mode = SHOOT_MODE_STOP;
	}
//	chs -> vx = ( Ramp_data1(RC_Ctl . keyboard . W) * (-SPEED_LMT) ) +  ( Ramp_data2(RC_Ctl . keyboard . S) * ( SPEED_LMT) ) ;
//	chs -> vy = ( Ramp_data3(RC_Ctl . keyboard . A) * ( SPEED_LMT) ) +  ( Ramp_data4(RC_Ctl . keyboard . D) *  (-SPEED_LMT) ) ;
//	gim -> v_yaw   = -(float) RC_Ctl . keyboard . x * KEYBOARD_YAW_SEN; //x -- 鼠标左右
//	gim -> v_pitch =  (float) RC_Ctl . keyboard . y * KEYBOARD_PITCH_SEN;//y -- 上下
	
}

static void Control_Task(void *argument)
{
    uint32_t time = osKernelGetTickCount();

    //Send_Packet_Init(&aim_packet_to_nuc);//与上位机通讯
    for (;;)
    {
#if CONTROL_MODE
        Remote_Ctrl (&gimbal_cmd, &chassis_cmd);//遥控器操纵
#else
		KeyboardCtrl(&gimbal_cmd, &chassis_cmd);//键鼠操纵
#endif
		Chassis_Cmd_Trans(&chassis_cmd, ANGLE_REFERENCE, DM_6006_yaw -> receive_data.position);//底盘坐标系转云台坐标系

        control_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + CONTROL_TASK_PERIOD);
    }
}

