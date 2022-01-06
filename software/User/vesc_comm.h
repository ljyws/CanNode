#ifndef __VESCCOM__H
#define __VESCCOM__H
#include <main.h>
#define VESC_BOARD_BASEID			0x0002


//VESC数据结构体
typedef enum {
	  CAN_PACKET_SET_DUTY = 0,
	  CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5,
    CAN_PACKET_POLL_TS5700N8501_STATUS
} VESC_CAN_COMMAND_ID;

typedef struct
{
    int32_t RPM;
    float current;
    float duty_cycle;
    int16_t pid_position;
    int16_t last_pid_position;
    int16_t motor_temp;
    int16_t fet_temp;
    int32_t last_received_time;
}vesc_motor_status_t;

typedef enum {
    CURRENT=0,
    POSITION,
    RPM,
    DUTY
} vesc_command_e;


typedef struct 
{

        int32_t current;//单位为毫安
        float positoion;//单位为°
        int32_t rpm;
        float duty;

}vesc_motor_command_t;



//vesc can接收函数
void VESC_CAN_hook(CAN_HandleTypeDef *hcan);

//can发送函数
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len,CAN_HandleTypeDef* CANx);

void SetMotor(uint8_t id, vesc_command_e type, int32_t date);

//以电流环方式控制底盘电机
static void CAN_CMD_VESC_CURRENT(uint8_t id,int32_t current);
//以速度环方式控制底盘电机
static void CAN_CMD_VESC_RPM(uint8_t id, int32_t RPM);
//以位置环方式控制底盘电机
static void CAN_CMD_VESC_POSITION(uint8_t id, float position);
//以占空比方式控制底盘电机
static void CAN_CMD_VESC_DUTY(uint8_t id, float duty);
//返回VESC变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204
const vesc_motor_status_t *get_VESC_Motor_Status_Point(uint8_t i);

#endif



