#ifndef __SUPERPOWER_H__
#define __SUPERPOWER_H__
#include "bsp_can.h"



// can_init_config_t Chassis_power_can_init_config = {
//     .can_handle = &hfdcan2,
//     .tx_id = 0x061,
//     .rx_id = 0x051,
// };

typedef struct
{
    // float voltage;
    // float current;
    // float power;
    uint8_t errorcode;
    float ChassisPower;
    float ChassisPowerLimit;
    uint8_t CapEnergy;
    CAN_instance_t *Chassis_power_can_instance;
} Chassis_Power_instance_t;

void Super_Power_Init(void);
static void Chassis_Power_Decode(CAN_instance_t *can_instance);
Chassis_Power_instance_t *Chassis_Power_Init(void);
void Chassis_Power_send_cmd(uint16_t power_limit, uint16_t RefereeEnergyBuffer);

extern Chassis_Power_instance_t *chassis_power;

#endif /* __SPUERPOWER_H__ */