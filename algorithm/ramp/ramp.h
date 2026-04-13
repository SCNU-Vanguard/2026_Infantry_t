#ifndef RAMP_H
#define RAMP_H

#include "main.h"

#define RAMP_INCREASE_STEP 0.005f //斜坡步长
#define RAMP_DECREASE_STEP 0.01f //斜坡步长
//#define MAX_SPEED 2.5f		//最大速度
#define MAX_OMEGA 3.3f		//最大速度
#define MAX_YAW 120.0f

float Ramp_data1(uint8_t x);
float Ramp_data2(uint8_t x);
float Ramp_data3(uint8_t x);
float Ramp_data4(uint8_t x);
float Ramp_data5(uint8_t x);
float Ramp_data6(uint8_t x);
float Gimbal_limit(float x);

extern float MAX_SPEED;

#endif
