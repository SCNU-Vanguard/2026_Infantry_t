/**
* @file shoot.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include <stdint.h>
#include "DJI_motor.h"
#include "shoot_motor.h"
#include "rs485.h"


#define SHOOT_MODE_STOP 0
#define SHOOT_MODE_FIRE 1

typedef struct 
{
    /* data */
}__attribute__((packed))shoot_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))shoot_cmd_t;   

extern uint8_t shoot_mode;
extern uint8_t shoot_mode_last;
extern uint8_t shoot_mode;
extern uint16_t target_shoot_frequence;
extern uint8_t shoot_permission;
extern shoot_motor_instance_t *friction_motor[3];

extern float temp_v_yaw;
extern float temp_v_pitch_neck;
extern float temp_v_pitch_head;

void Shoot_Init(void);
void Shoot_Enable(void);
void Shoot_Stop(void);
void Shoot_Set_All_Friction(int16_t speed);
void Get_Shoot_Mode(void);
void Shoot_State_Machine(void);
void Shoot_Control_Remote(void);

#endif /* __SHOOT_H__ */