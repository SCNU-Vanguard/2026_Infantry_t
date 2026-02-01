/**
* @file robot_frame_config.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_CONFIG_H__
#define __ROBOT_FRAME_CONFIG_H__

/*常用函数定义*/
#define USER_LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))


/*标志位*/
#define ROLL_TO_PITCH 1 //根据H7板摆放的位置决定



/*归一化参数*/
#define RAD_2_DEGREE 57.2957795f    //rad/s to °/s 180/pi
#define DEGREE_2_RAD 0.01745329252f //°/s to rad/s pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       //rpm to °/s ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f //rpm to rad/s ×2pi/60sec    

#define BMI088_Frame 1

/*底盘参数*/
#define M3508_REDUCTION_RATIO (3591.0f/187.0f) //M3508减速比

#define LENGTH 0.405f		//轴距（wheelbase） 即前后轮轴间距离
#define WIDTH 0.424f		//轮距（track） 即左右驱动轮中心距离        精确度
    
#define WHEEL_RADIUS 0.077f	//驱动轮半径（diameter）

    
/*遥控器参数*/
#define REMOTE_X_SEN 0.005   //660 ~ -660 
#define REMOTE_Y_SEN 0.005
#define REMOTE_OMEGA_Z_SEN 0.005f  //6.6

#define REMOTE_YAW_SEN 0.000015f       
#define REMOTE_PITCH_SEN 0.000002f

/*云台相关参数*/    //注意大小问题
#define PITCH_NECK_MIN_ANGLE  -0.03f //rad , 对应缩头时发送的角度
#define PITCH_NECK_MAX_ANGLE  -1.0f  //rad , 对应伸出头时发送的角度
#define PITCH_NECK_ACTUAL_MIN_ANGLE  -0.0076f //rad , 对应缩头时返回的角度
#define PITCH_NECK_ACTUAL_MAX_ANGLE  -0.25f  //rad , 对应伸出头时返回的角度
#define PITCH_NECK_MAX_SPEED  1.5f // rad/s
#define PITCH_NECK_TRANSFORM_SPEED 0.0018 //伸缩变换时使用的过渡增量
#define PITCH_NECK_TRANSFORM_JUDGEMENT 0.03 //伸缩变换时使用的判断值

#define PITCH_HEAD_MIN_ANGLE  0.24f //rad , 对应伸头时的最低角度
#define PITCH_HEAD_MID_ANGLE  -0.02f //rad , 对应中间的角度
#define PITCH_HEAD_MAX_ANGLE  -0.38f  //rad , 对应伸出头时的最高角度
#define PITCH_HEAD_MAX_SPEED  0.001f // rad/s
#define PITCH_HEAD_TRANSFORM_SPEED 0.0016 //伸缩变换时使用的过渡增量
#define PITCH_HEAD_TRANSFORM_JUDGEMENT 0.05 //伸缩变换时使用的判断值
#define PITCH_HEAD_COEFFICIENT 1.5 //遥控器增量乘的系数

#define YAW_MAX_SPEED 2.0f //rad/s
#define YAW_COEFFICIENT 0.6 //遥控器增量乘的系数

#define YAW_FEEDFORWAED_COEFFICIENT 0.22	//小陀螺yaw轴前馈用的系数

#endif /* __ROBOT_FRAME_CONFIG_H__ */