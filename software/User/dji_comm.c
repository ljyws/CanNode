/**
  ******************************************************************************
  * @file    comunication
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017.12.31
  * @brief   用于与主控通信，与DJI电调通信
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------*/
#include "dji_comm.h"
#include "can.h"
#include "ctrl.h"
#include "rm_motor.h"

extern MotorType Motor[8];
extern DriverType Driver[8];

/**
  * @brief  设定电机的运行电流
  * @param
  * @param
  * @retval
  */
	int ljy6=0;
void SetCur(CAN_HandleTypeDef *hcan,float *cur)
{
  uint32_t TxMailbox;
	uint8_t TxMessage[8];
  CAN_TxHeaderTypeDef TxHeader;
  int16_t data[4] = {0};
	ljy6++;
  for (int i = 0; i < 4; i++)
    data[i] = (int16_t)(cur[i]);

  TxHeader.StdId = 0x200;         // standard identifier=0
  TxHeader.ExtId = 0x200;         // extended identifier=StdId
  TxHeader.IDE = CAN_ID_STD; // type of identifier for the message is Standard
  TxHeader.RTR = CAN_RTR_DATA;    // the type of frame for the message that will be transmitted
  TxHeader.DLC = 8;

  TxMessage[0] = (data[0] >> 8) & 0xff;
  TxMessage[1] = data[0] & 0xff;
  TxMessage[2] = (data[1] >> 8) & 0xff;
  TxMessage[3] = data[1] & 0xff;
  TxMessage[4] = (data[2] >> 8) & 0xff;
  TxMessage[5] = data[2] & 0xff;
  TxMessage[6] = (data[3] >> 8) & 0xff;
  TxMessage[7] = data[3] & 0xff;

	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
    ; //等待238us
}

/**
  * @brief  通过CAN2返回周期消息
  * @attention CAN发送时间较长，此函数一定要放在低优先级任务中
  * @param
  * @param
  * @retval
  */
void CanSendPeriodMsgs(void)
{
  UnionDataType txData;

  //发送速度给主控
  for (int i = 0; i < 4; i++)
  {
    txData.data32[0] = 0x00005856;
    txData.data32[1] = (int32_t)(Driver[i].velCtrl.speed * 1000);
    CanSendData(&hcan2,Driver[i].command.canId, txData);
  }
}

/**
  * @brief  回应主控请求
  * @attention CAN发送时间较长，此函数一定要放在低优先级任务中
  * @param
  * @param
  * @retval
  */
void CANRespond(void)
{
  UnionDataType txData;

  for (int i = 0; i < 4; i++)
  {
    if (Motor[i].type == NONE)
      break;

    switch (Driver[i].command.can_status)
    {
    case 0:
      break;
    case 0x00005856: //VX   读取速度
      txData.data32[0] = 0x00005856;
      txData.data32[1] = (int32_t)(Driver[i].velCtrl.speed * 1000);
      CanSendData(&hcan2,Driver[i].command.canId, txData);
      Driver[i].command.can_status = 0;
      break;

    case 0x00005149: //IQ	 读取电流
      txData.data32[0] = 0x00005149;
      txData.dataf[1] = Motor[i].cur;
      CanSendData(&hcan2,Driver[i].command.canId, txData);
      Driver[i].command.can_status = 0;
      break;

    case 0x00005850: //PX   读取位置
      txData.data32[0] = 0x00005850;
      txData.data32[1] = (int32_t)(Driver[i].posCtrl.actualPos);
      CanSendData(&hcan2,Driver[i].command.canId, txData);
      Driver[i].command.can_status = 0;
      break;

    default:
      break;
    }
  }
}

/**
  * @brief  CanSendData
  * @param
  * @param
  * @retval
  */
void CanSendData(CAN_HandleTypeDef *hcan,int id, UnionDataType txData)
{
  uint32_t TxMailbox;
	uint8_t TxMessage[8];
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.StdId = (0x280 + id);  // standard identifier=0
  TxHeader.ExtId = (0x280 + id);  // extended identifier=StdId
  TxHeader.IDE = CAN_ID_EXT; // type of identifier for the message is Standard
  TxHeader.RTR = CAN_RTR_DATA;    // the type of frame for the message that will be transmitted
  TxHeader.DLC = 8;

  for (int i = 0; i < 8; i++)
  {
    TxMessage[i] = txData.data8[i]; // 帧信息
  }

	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
    ; //等待238us
}
/************************ (C) COPYRIGHT 2019 ACTION ********************/
