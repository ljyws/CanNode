#include "vesc_comm.h"

/**
  ****************************(C) COPYRIGHT 2021 HCRT****************************
  * @file       vesc_can.c/h
  * @brief      VESC can通信函数--
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0    2020-12-30        LJY               1. done

  @verbatim
  ==============================================================================
	2020-12-30：
	* 实现VESC CAN协议，用法：调用SetMotor（）函数，第一个参数是VESC_CAN_ID,第二个
	* 参数是控制模式，有电流、速度、位置、占空比四种，第三个参数是控制量。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 HCRT****************************
  */

#include "vesc_comm.h"
#include "can.h"
#include "bsp_can.h"
#include "string.h"
vesc_command_e type;
//static vesc_motor_status_t motor_vesc[4];



/*******************VESC******************/


//缓冲区处理函数
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) 
{
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}
int vesc2=0;

/**
* @brief  电机控制函数
* @param  id: VESC_CAN_ID
* @param  type: 控制模式1.电流2.速度3.位置4.占空比
* @param  date 控制量
*/
void SetMotor(uint8_t id, vesc_command_e type, int32_t date)
{
    switch (type)
    {
        case CURRENT:
            CAN_CMD_VESC_CURRENT(id, date);
            break;
        case RPM:
					  vesc2++;
            CAN_CMD_VESC_RPM(id, date);
            break;
        case POSITION:
            CAN_CMD_VESC_POSITION(id, date);
            break;
        case DUTY:
            CAN_CMD_VESC_DUTY(id, date);
            break;
        default:
            break;
    }
}

//VESC数据回收
//void VESC_CAN_hook(CAN_HandleTypeDef *hcan)
//{
//    int32_t ind;
//	  CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//    switch (	 rx_header.ExtId>>8){
//        case CAN_PACKET_STATUS:{
//            ind = 0;
//            motor_vesc[((	 rx_header.ExtId)&0x03)].last_received_time=xTaskGetTickCountFromISR();
//            motor_vesc[((	 rx_header.ExtId)&0x03)].RPM=buffer_get_int32(rx_data,&ind);
//			motor_vesc[((	 rx_header.ExtId)&0x03)].current=(float)buffer_get_int16(rx_data,&ind)/10;
//			motor_vesc[((	 rx_header.ExtId)&0x03)].duty_cycle=(float)buffer_get_int16(rx_data,&ind)/1000;
//            break;
//        }
//        case CAN_PACKET_STATUS_2:{
//            ind = 0;
//            motor_vesc[((rx_header.ExtId)&0x03)].last_received_time=xTaskGetTickCountFromISR();
//            
//        }
//    }
//}
int vesc3=0;

/**
* @brief  CAN发送函数
* @param  id: VESC_CAN_ID
* @param  data: 数据包
* @param  len： 数据长度
* @param  hcan: can通道
*/
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len,CAN_HandleTypeDef* hcan) 
	{
	if (len > 8) {
		len = 8;
	}
  
	uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxMessage[8];
	
	TxHeader.IDE = CAN_ID_EXT;//extrenal frame
	TxHeader.ExtId = id;
	TxHeader.RTR = CAN_RTR_DATA;//normal data
	TxHeader.DLC = len;
	memcpy(TxMessage, data, len);//copy the payload to TxMessage
  vesc3++;  
	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 )
	{};
  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
	
  }

/**
* @brief  电流模式
* @param  id: VESC_CAN_ID
* @param  current: 电流
*/
static void CAN_CMD_VESC_CURRENT(uint8_t id,int32_t current)
	{
  int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, current, &send_index);
	can_transmit_eid(id |((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index,&hcan2);
	}

/**
* @brief  速度模式
* @param  id: VESC_CAN_ID
* @param  RPM: 电机电气转速，电气转速/极对数为机械转速
*/
	uint32_t jjjj=0;
static void CAN_CMD_VESC_RPM(uint8_t id, int32_t RPM)
{

	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, RPM, &send_index);
	can_transmit_eid(id |((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index,&hcan1);
	jjjj = id |((uint32_t)CAN_PACKET_SET_RPM << 8);
}

/**
* @brief  位置模式
* @param  id: VESC_CAN_ID
* @param  position: 位置
*/
static void CAN_CMD_VESC_POSITION(uint8_t id, float position)
{
    
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(position * 1000000.0f), &send_index);
	can_transmit_eid(id |((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index,&hcan2);

}

/**
* @brief  占空比模式
* @param  id: VESC_CAN_ID
* @param  duty: 占空比
*/
static void CAN_CMD_VESC_DUTY(uint8_t id, float duty)
{
   int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)duty, &send_index);
	can_transmit_eid(id |((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index,&hcan2);
}


