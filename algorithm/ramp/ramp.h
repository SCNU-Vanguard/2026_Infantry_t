#ifndef RAMP_H
#define RAMP_H

#include "main.h"

#define RAMP_INCREASE_STEP 0.005 //斜坡步长
#define RAMP_DECREASE_STEP 0.01 //斜坡步长
#define MAX_SPEED 2		//最大速度
#define MAX_OMEGA 3.3		//最大速度
#define MAX_YAW 120

float Ramp_data1(uint8_t x);
float Ramp_data2(uint8_t x);
float Ramp_data3(uint8_t x);
float Ramp_data4(uint8_t x);
float Ramp_data5(uint8_t x);
float Ramp_data6(uint8_t x);
int16_t Gimbal_limit(float x);

#endif
