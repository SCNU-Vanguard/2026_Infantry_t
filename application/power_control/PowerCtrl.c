
#include "PowerCtrl.h"
//#include "motor.h"// quote chassis_motor information 
#include "chassis.h"
#include "referee.h"
//#include "SuperCap.h"

float P_cal[4] = {0}; // 电机功率模型计算出的功率
float P_total = 0;   // 测量总功率
float P_total_temp = 0; // 临时总功率

float P_cmd[4] = {0}; // 以最大功率上线分配所得的功率
float chassis_max_power = 100; // 底盘最大功率 , 5~8W 的误差 

float I_temp[4] = {0};

float I_test[4];
float P_test;

void chassis_power_control(void)
{
    float speed_rpm = 0.0f; 
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float delta = 0.0f;
    float  E_k = 0.0f;


////  超电断联错误判断,例子，还待实际上超级电容后验证
//    if(supercap_rx_data.errorCode != 0 || refree_info.Game_Robot_state.chassis_power_limit == 0)
//    {
//        chassis_max_power = 30; // 出错时，给一个较低的功率限制
//    }
////    对chassis_max_power处理（充电模式）
//    else if( supercap_rx_data.capEnergy < 50 || refree_info.Power_Heat_Data.buffer_energy < 10)
//    {
//        //底盘功率限制
//        chassis_max_power = refree_info.Game_Robot_state.chassis_power_limit - 10;//使底盘功率略低于裁判系统功率上限，给超级电容充电
//    }
////    超电能量 + 缓冲能量 
//    else if(supercap_rx_data.capEnergy > 50 && refree_info.Power_Heat_Data.buffer_energy > 10)
//    {
//        E_k = (supercap_rx_data.capEnergy - 70)/ 255;
//        chassis_max_power =  refree_info.Game_Robot_state.chassis_power_limit - 7 + E_k * 300;  //-7是因为存在相对误差，300为超电最大输出功率
//    }



//功率模型
    for(uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
    {
        speed_rpm = chassis_m3508[i]->measure.speed; //motor_list[i]->data.speed ---> rpm

        P_cal[i] =  K0 * chassis_m3508[i] -> target.current *speed_rpm +
                        K1 * speed_rpm * speed_rpm +  
                        K2 * chassis_m3508[i] -> target.current * chassis_m3508[i]->target.current + 
                        constant;//静态功耗

        if(P_cal[i] < 0) // 负功率不计入（过渡性）
            continue;

        P_total_temp += P_cal[i];//输出总功率
    }

        P_total = P_total_temp;//用于观察计算出来的底盘总功率
        P_total_temp = 0;//防止累加


    if(P_total > chassis_max_power)//总功率超过
    {
        float power_scale = chassis_max_power / P_total; //功率分配系数

        for(uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
        {
            speed_rpm = chassis_m3508[i]->measure.speed;          //motor_list[i]->data.speed ---> rpm  , 注意获取新的电机转速

            P_cmd[i] = P_cal[i] * power_scale; // 得到分配后的功率
            if(P_cmd[i] < 0)// 不考虑负功率
                continue;

            a = K2;
            b = K0 * speed_rpm;
            c = K1 * speed_rpm * speed_rpm + constant - P_cmd[i];
            delta = b*b - 4*a*c;
            
            if(delta < 0)
                continue;

            else if(chassis_m3508[i]->target.current > 0) // 根据电机原本转向分配
            {
                I_temp[i] = (-b + sqrt(delta)) / (2*a);

                if(I_temp[i] > 16000)
                    chassis_m3508[i]->target.current = 16000;
                else
                    chassis_m3508[i]->target.current = I_temp[i];
            }
            else
            {
                I_temp[i] = (-b - sqrt(delta)) / (2*a);
				
                if(I_temp[i] < -16000)
                    chassis_m3508[i]->target.current = -16000;
                else
                    chassis_m3508[i]->target.current = I_temp[i];
            }
		}
    }
	
////////////////////////////////////////////////////////////////////////////////////////////	
	//I_test = (float)(abs(chassis_m3508[0]->target.current) + abs(chassis_m3508[1]->target.current) + abs(chassis_m3508[2]->target.current) + abs(chassis_m3508[3]->target.current));
	I_test[0] = (float)abs(chassis_m3508[0]->target.current);
	I_test[1] = (float)abs(chassis_m3508[1]->target.current);
	I_test[2] = (float)abs(chassis_m3508[2]->target.current);
	I_test[3] = (float)abs(chassis_m3508[3]->target.current);
	
	
	for(uint8_t i = 0;i < 4;i++)
	{
		speed_rpm = chassis_m3508[i]->measure.speed; //motor_list[i]->data.speed ---> rpm


        P_cal[i] =  K0 * chassis_m3508[i] -> target.current *speed_rpm +
                        K1 * speed_rpm * speed_rpm +  
                        K2 * chassis_m3508[i] -> target.current * chassis_m3508[i]->target.current + 
                        constant;//静态功耗

        if(P_cal[i] < 0) // 负功率不计入（过渡性）
            continue;

        P_total_temp += P_cal[i];//输出总功率
	}
	
	P_test = P_total_temp;//用于观察功率控制后的底盘总功率
	P_total_temp = 0;//防止累加
}
