/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-06     zxy       the first version
 */
#ifndef APPLICATIONS_MOTOR_ODRIVE_COM_H_
#define APPLICATIONS_MOTOR_ODRIVE_COM_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "main.h"

#define ODRIVE_BOARD_BASEID 0x0001
enum Comtrol_mode{
    CONTROL_MODE_VOLTAGE_CONTROL = 0,
    CONTROL_MODE_TORQUE_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3,
};
enum Input_mode{
    INPUT_MODE_INACTIVE = 0,
    INPUT_MODE_PASSTHROUGH = 1,
    INPUT_MODE_VEL_RAMP = 2,
    INPUT_MODE_POS_FILTER = 3,
    INPUT_MODE_MIX_CHANNELS = 4,
    INPUT_MODE_TRAP_TRAJ = 5,
    INPUT_MODE_TORQUE_RAMP = 6,
    INPUT_MODE_MIRROR = 7,
};

/**
 * @brief 将浮点数转为16进制4字节型
 * @author zxy
 */
void FloatToByte(float floatNum,uint16_t* byteArry);

/**
 * @brief 数组右移
 * @author zxy
 * @param Array[]：传入数组
 * @param n：数组元素个数，即数组长度
 * @param k：右移位数
 */
void Array_right_move(uint16_t A[],int n, int k);

/**
 * @brief 通过can通讯向Odrive发送数据
 * @param address：由NodeID和CMDID组成，如NodeID = 2, CMDID = 0x007, 则address = 0x207
 * @param data[8]：存储8位数据位的数组
 */
void Can_send_odrive(CAN_HandleTypeDef *hcan,uint16_t address,uint16_t data[8]);

/**
 * @brief 进行电机参数校准
 *
 */
void Odrive_motor_calibration(void);

/**
 * @brief 进行编码器偏移校准
 */
void Odrive_encoder_offset_calibration(void);

/**
 * @brief 进行编码器索引校准
 */
void Odrive_encoder_index_search(void);

/**
 * @brief 设置电机为闭环运行模式
 */
void Odrive_set_closed_loop(void);

/**
 * @brief 设置电机为待机模式
 * @note 此时电机可以任意转动
 */
void Odrive_set_idle(void);

/**
 * @brief 配置电机控制器模式
 * @param control_mode：控制模式
 * @param input_mode：输入模式
 */
void Odrive_set_control_mode(uint8_t control_mode,uint8_t input_mode);

/**
 * @brief 控制电机在速度模式下转动
 * @param vel：电机转速，单位为圈/秒
 */
void Odrive_set_motor_vel(float_t vel);

/**
 * @brief 控制电机在位置模式下转动
 * @param pos：对应位置
 */
void Odrive_set_motor_pos(float_t pos);

/**
 * @brief 重启odrive
 */
void Odrive_reboot(void);

#endif /* APPLICATIONS_MOTOR_ODRIVE_COM_H_ */
