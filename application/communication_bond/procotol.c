/**
******************************************************************************
 * @file    procotol.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "procotol.h"

#include "vofa.h"
#include "bmi088.h"
#include "chassis.h"
#include "gimbal.h"
#include "PowerCtrl.h"

extern bmi088_data_t imu_data;

void VOFA_Display_IMU(void)
{
    vofa_data_view[0] = imu_data.acc[0];
    vofa_data_view[1] = imu_data.acc[1];
    vofa_data_view[2] = imu_data.acc[2];

    vofa_data_view[3] = imu_data.gyro[0];
    vofa_data_view[4] = imu_data.gyro[1];
    vofa_data_view[5] = imu_data.gyro[2];

    vofa_data_view[6] = imu_data.temperature;

    //VOFA_Send_Data(vofa_data_view, 7);
    VOFA_JustFloat(vofa_data_view, 7);
}

void VOFA_Display_Pitch_Head(void)
{
	vofa_data_view[0] = INS.Roll;
	vofa_data_view[1] = pitch_head_target;
	vofa_data_view[2] = pitch_head_measure;
	vofa_data_view[3] = temp_v_pitch_head;
	
	VOFA_JustFloat(vofa_data_view, 4);
}

void VOFA_Display_Yaw(void)
{
	vofa_data_view[0] = INS.Yaw;
	vofa_data_view[1] = yaw_speed_target;
	vofa_data_view[2] = yaw_speed_measure;
	vofa_data_view[3] = temp_v_yaw;
	
	VOFA_JustFloat(vofa_data_view, 4);
}

void VOFA_Display_Speed(void)
{
//	vofa_data_view[0] = chassis_m3508[0]->measure.speed;
//	vofa_data_view[1] = chassis_m3508[1]->measure.speed;
//	vofa_data_view[2] = chassis_m3508[2]->measure.speed;
//	vofa_data_view[3] = chassis_m3508[3]->measure.speed;
//	vofa_data_view[0] *= -1;
//	vofa_data_view[3] *= -1;
//	
//	vofa_data_view[4] = omega_z;
//	
//	
//	vofa_data_view[5] = -target_speed[0];
//	vofa_data_view[6] = target_speed[1];
//	vofa_data_view[7] = target_speed[2];
//	vofa_data_view[8] = -target_speed[3];
	
	vofa_data_view[0] = fabsf(chassis_m3508[0]->measure.speed);
	vofa_data_view[1] = fabsf(chassis_m3508[1]->measure.speed);
	vofa_data_view[2] = fabsf(chassis_m3508[2]->measure.speed);
	vofa_data_view[3] = fabsf(chassis_m3508[3]->measure.speed);
	
	vofa_data_view[4] = omega_z;
	
	
	vofa_data_view[5] = fabsf(target_speed[0]);
	vofa_data_view[6] = fabsf(target_speed[1]);
	vofa_data_view[7] = fabsf(target_speed[2]);
	vofa_data_view[8] = fabsf(target_speed[3]);
	
	
	VOFA_JustFloat(vofa_data_view, 9);
}	

void VOFA_Display_Power(void)
{
	vofa_data_view[0] = chassis_max_power;
	vofa_data_view[1] = P_total;
	vofa_data_view[2] = P_test;
	
	VOFA_JustFloat(vofa_data_view, 3);
}