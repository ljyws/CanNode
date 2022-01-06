/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-06     zxy       the first version
 */

/*****
NOTE:
0x00010001 0xXXXXXXXX电机校准
0x00010002 0xXXXXXXXX编码器偏移校准
0x00010003 0xXXXXXXXX编码器索引
0x00010004 0xXXXXXXXX电机进入闭环
0x00010005 ：控制器模式、输入类型选择
						0x00XXXXXX电压控制
						0x01XXXXXX力矩控制
						0x02XXXXXX速度控制
						0x03XXXXXX位置控制
						0xXX00XXXX  INPUT_MODE_INACTIVE
						0xXX01XXXX  INPUT_MODE_PASSTHROUGH
						0xXX02XXXX	INPUT_MODE_VEL_RAMP
						0xXX03XXXX	INPUT_MODE_POS_FILTER
0x00010006 0xXXXXXXXX速度模式下速度控制
0x00010007 0xXXXXXXXX位置模式下位置控制			


odrive CanID配置解释：
若要把ID设置为2 则设置为0x010 :
0x010 : 0000 0001 0000
              (01 0) = 2
同理
0x018 ：0000 0001 1000
		      (01 1) = 3   则CanID为 3
*****/

#include "odrive_com.h"
#include "can.h"
/**
 * @brief 将浮点数转为16进制4字节型
 * @author zxy
 */
void FloatToByte(float floatNum,uint16_t* byteArry)
{
    char* pchar=(char*)&floatNum;
    for(int i=0;i<sizeof(float);i++)
    {
        *byteArry=*pchar;
        pchar++;
        byteArry++;
    }
}

/**
 * @brief 数组右移
 * @author zxy
 * @param Array[]：传入数组
 * @param n：数组元素个数，即数组长度
 * @param k：右移位数
 */
void Array_right_move(uint16_t A[],int n, int k)
{
    for (int i = 0; i < k % n; i++)
    {
        int x = A[0];
        for(int j = n - 1; j > 0; j--)
        {
            A[j] = A[j - 1];
        }
        A[n - 1] = x;
    }
}

/**
 * @brief 通过can通讯向Odrive发送数据
 * @param address：由NodeID和CMDID组成，如NodeID = 2, CMDID = 0x007, 则address = 0x207
 * @param data[8]：存储8位数据位的数组
 * @note 在调用此函数前需要提前配置好Odrive：
 *       1.odrv0.axis0.config.can_node_id = 0x010
 *       2.odrv0.can.set_baud_rate(1000000)
 *       3.odrv0.save_configuration()
 */


void Can_send_odrive(CAN_HandleTypeDef *hcan,uint16_t address,uint16_t data[8]) //can发送数据
{
  uint32_t TxMailbox;
	uint8_t TxMessage[8];
  CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = address;
  TxHeader.IDE = CAN_ID_STD; // type of identifier for the message is Standard
  TxHeader.RTR = CAN_RTR_DATA;    // the type of frame for the message that will be transmitted
  TxHeader.DLC = 8;

  TxMessage[0] = data[0];
  TxMessage[1] = data[1];
  TxMessage[2] = data[2];
  TxMessage[3] = data[3];
  TxMessage[4] = data[4];
  TxMessage[5] = data[5];
  TxMessage[6] = data[6];
  TxMessage[7] = data[7];
	
	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
}

/**
 * @brief 进行电机参数校准
 */
void Odrive_motor_calibration()
{
    uint16_t odrive_data[8] = {0x04,0,0,0,0,0,0,0};
    Can_send_odrive(&hcan1,0x407, odrive_data);
}

/**
 * @brief 进行编码器偏移校准
 */
void Odrive_encoder_offset_calibration()
{
    uint16_t odrive_data[8] = {0x07,0,0,0,0,0,0,0};
    Can_send_odrive(&hcan1,0x407, odrive_data);
}

/**
 * @brief 进行编码器索引校准
 * @note 对于增量编码器，每次重启电机都应进行编码器索引校准
 */
void Odrive_encoder_index_search()
{
    uint16_t odrive_data[8] = {0x06,0,0,0,0,0,0,0};
    Can_send_odrive(&hcan1,0x407, odrive_data);
}

/**
 * @brief 设置电机为闭环运行模式
 */
void Odrive_set_closed_loop()
{
    uint16_t odrive_data[8] = {0x08,0,0,0,0,0,0,0};
    Can_send_odrive(&hcan1,0x407, odrive_data);
}

/**
 * @brief 设置电机为待机模式
 * @note 此时电机可以任意转动
 */
void Odrive_set_idle()
{
    uint16_t odrive_data[8] = {0x01,0,0,0,0,0,0,0};
    Can_send_odrive(&hcan1,0x407, odrive_data);
}

/**
 * @brief 配置电机控制器模式
 * @param control_mode：控制模式
 * @param input_mode：输入模式
 */
void Odrive_set_control_mode(uint8_t control_mode,uint8_t input_mode)
{
    uint16_t odrive_data[8] = {0,0,0,0,0,0,0,0};
    switch(control_mode)
    {
    case CONTROL_MODE_VOLTAGE_CONTROL:
        odrive_data[0] = 0;
        break;
    case CONTROL_MODE_TORQUE_CONTROL:
        odrive_data[0] = 0x01;
        break;
    case CONTROL_MODE_VELOCITY_CONTROL:
        odrive_data[0] = 0x02;
        break;
    case CONTROL_MODE_POSITION_CONTROL:
        odrive_data[0] = 0x03;
        break;
    default:
        break;
    }
    switch(input_mode)
    {
    case INPUT_MODE_INACTIVE:
        odrive_data[4] = 0;
        break;
    case INPUT_MODE_PASSTHROUGH:
        odrive_data[4] = 0x01;
        break;
    case INPUT_MODE_VEL_RAMP:
        odrive_data[4] = 0x02;
        break;
    case INPUT_MODE_POS_FILTER:
        odrive_data[4] = 0x03;
        break;
    case INPUT_MODE_MIX_CHANNELS:
        odrive_data[4] = 0x04;
        break;
    case INPUT_MODE_TRAP_TRAJ:
        odrive_data[4] = 0x05;
        break;
    case INPUT_MODE_TORQUE_RAMP:
        odrive_data[4] = 0x06;
        break;
    case INPUT_MODE_MIRROR:
        odrive_data[4] = 0x07;
        break;
    default:
        break;

    }
    Can_send_odrive(&hcan1,0x40B, odrive_data);
}

/**
 * @brief 控制电机在速度模式下转动
 * @param vel：电机转速，单位为圈/秒
 */
void Odrive_set_motor_vel(float_t vel)
{
    uint16_t odrive_data[8];
    FloatToByte(vel,odrive_data);
    Can_send_odrive(&hcan1,0x40D, odrive_data);
}

/**
 * @brief 控制电机在位置模式下转动
 * @param pos：对应位置
 * @note pos < 0, 未测试
 */
void Odrive_set_motor_pos(float_t pos)
{
    uint16_t odrive_data[8];
    FloatToByte(pos,odrive_data);
    Can_send_odrive(&hcan1,0x40C, odrive_data);
}

/**
 * @brief 重启odrive
 */
void Odrive_reboot()
{
    uint16_t odrive_data[8];
    Can_send_odrive(&hcan1,0x416, odrive_data);
    printf("Rebooting... \n");
}


