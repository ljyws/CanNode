#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"


#if defined ( __CC_ARM   )
#pragma anon_unions
#endif



//大疆电机闭环控制板(dji motor controller)反馈的数据
typedef struct
{
  float motor_spd[4]; //电机当前速度
  float motor_pos[4]; //电机当前位置
  float motor_cur[4]; //电机当前电流

} dmc_feedback_t;
void CAN_Filter_Init(void);


/*******************************函数声明区*********************************/
void CAN_Transmit_Messages_Byte(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t data0, uint8_t data1,
                                uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);


#endif
