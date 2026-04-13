/*
 * @brief	  粗略版斜坡函数(使用整形运算来优化运行，防止大量浮点数运算溢出)
 * @history
 * 	版本			作者		  编写日期			内容
 * 	v1.0		  FJJ		  2025/3/10		 适配键盘控制模式
 */
#include <stdint.h>
#include <stdio.h>
#include "ramp.h"
#include "control_task.h"

/*
 * @brief  	专门应用与按键的斜坡函数
 * @param	x -- 输入值(0到1)
 * @param   y -- 输出值(0到100)
 * @retval 	y -- 输出值(0到100)
 */
float y_1;
float y_2;
float y_3;
float y_4;
float y_5;
float y_6;

float MAX_SPEED = 3.5;
 
float Ramp_data1(uint8_t x)
{
    if(x)
      y_1 = (y_1 + RAMP_INCREASE_STEP)> MAX_SPEED ? MAX_SPEED :(y_1 + RAMP_INCREASE_STEP);
    else
      y_1 = (y_1 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_1 - RAMP_DECREASE_STEP);
    return y_1;
}

float Ramp_data2(uint8_t x)
{
    if(x)
      y_2 = (y_2 + RAMP_INCREASE_STEP)> MAX_SPEED ? MAX_SPEED :(y_2 + RAMP_INCREASE_STEP);
    else
      y_2 = (y_2 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_2 - RAMP_DECREASE_STEP);
    return y_2;
}

float Ramp_data3(uint8_t x)
{
    if(x)
      y_3 = (y_3 + RAMP_INCREASE_STEP)> MAX_SPEED ? MAX_SPEED :(y_3 + RAMP_INCREASE_STEP);
    else
      y_3 = (y_3 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_3 - RAMP_DECREASE_STEP);
    return y_3;
}

float Ramp_data4(uint8_t x)
{
    if(x)
      y_4 = (y_4 + RAMP_INCREASE_STEP)> MAX_SPEED ? MAX_SPEED :(y_4 + RAMP_INCREASE_STEP);
    else
      y_4 = (y_4 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_4 - RAMP_DECREASE_STEP);
    return y_4;
}

float Ramp_data5(uint8_t x)
{
    if(x)
      y_5 = (y_5 + RAMP_INCREASE_STEP)> MAX_OMEGA ? MAX_OMEGA :(y_5 + RAMP_INCREASE_STEP);
    else
      y_5 = (y_5 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_5 - RAMP_DECREASE_STEP);
    return y_5;
}

float Ramp_data6(uint8_t x)
{
    if(x)
      y_6 = (y_6 + RAMP_INCREASE_STEP)> MAX_OMEGA ? MAX_OMEGA :(y_6 + RAMP_INCREASE_STEP);
    else
      y_6 = (y_6 - RAMP_DECREASE_STEP)< 0 ? 0 :(y_6 - RAMP_DECREASE_STEP);
    return y_6;
}

float Gimbal_limit(float x)
{
	if(x > MAX_YAW)
		x = MAX_YAW;
	else if(x < -MAX_YAW)
		x = -MAX_YAW;
	
	return -x;
}