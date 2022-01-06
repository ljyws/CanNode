/**
  ******************************************************************************
  * @file    comunication
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017.12.31
  * @brief   ����������ͨ�ţ���DJI���ͨ��
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
  * @brief  �趨��������е���
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
    ; //�ȴ�238us
}

/**
  * @brief  ͨ��CAN2����������Ϣ
  * @attention CAN����ʱ��ϳ����˺���һ��Ҫ���ڵ����ȼ�������
  * @param
  * @param
  * @retval
  */
void CanSendPeriodMsgs(void)
{
  UnionDataType txData;

  //�����ٶȸ�����
  for (int i = 0; i < 4; i++)
  {
    txData.data32[0] = 0x00005856;
    txData.data32[1] = (int32_t)(Driver[i].velCtrl.speed * 1000);
    CanSendData(&hcan2,Driver[i].command.canId, txData);
  }
}

/**
  * @brief  ��Ӧ��������
  * @attention CAN����ʱ��ϳ����˺���һ��Ҫ���ڵ����ȼ�������
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
    case 0x00005856: //VX   ��ȡ�ٶ�
      txData.data32[0] = 0x00005856;
      txData.data32[1] = (int32_t)(Driver[i].velCtrl.speed * 1000);
      CanSendData(&hcan2,Driver[i].command.canId, txData);
      Driver[i].command.can_status = 0;
      break;

    case 0x00005149: //IQ	 ��ȡ����
      txData.data32[0] = 0x00005149;
      txData.dataf[1] = Motor[i].cur;
      CanSendData(&hcan2,Driver[i].command.canId, txData);
      Driver[i].command.can_status = 0;
      break;

    case 0x00005850: //PX   ��ȡλ��
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
    TxMessage[i] = txData.data8[i]; // ֡��Ϣ
  }

	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
    ; //�ȴ�238us
}
/************************ (C) COPYRIGHT 2019 ACTION ********************/
