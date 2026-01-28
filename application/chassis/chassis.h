#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "DJI_motor.h"
#include "DM_motor.h"
#include "robot_frame_config.h"

typedef enum
{
    SPIN    = 1, //小陀螺
    FOLLOW  = 3,//底盘跟随
    STOP_C    = 2//不动
}chassis_mode_e;


typedef struct
{
    float vx;
    float vy;

    float omega_z;       //底盘小陀螺时的角速度(rad/s)
    float omega_follow;   //底盘跟随时的角速度  (rad/s)
    chassis_mode_e mode;
}__attribute__((__packed__))Chassis_CmdTypedef;

//typedef struct
//{
//    float target;
//    float value;
//    float error;
//    float output;

//    float omega_z_ref;
//    PID_TypeDef pid[2];    //outer(or angle)(0)、inner(or speed)(1) circle
//}__attribute__((__packed__))Chassis_TypeDef;



void Chassis_Init(void);
float Chassis_Get_Omega (DJI_motor_instance_t *M3508[]);
void Mecanum_Solve(Chassis_CmdTypedef *cmd, float *ret);
void Chassis_Ctrl_Remote(void);

/*外部参数*/
extern DJI_motor_instance_t *chassis_m3508[4];
extern Chassis_CmdTypedef chassis_cmd;
extern float omega_z;
extern float target_speed[4];

#endif /* __CHASSIS_H__ */
