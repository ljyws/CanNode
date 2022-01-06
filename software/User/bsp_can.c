/**
  ****************************(C) COPYRIGHT 2021 HCRT****************************
  * @file       bsp_can.c/h
  * @brief      CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  V1.2.0			2020.2.4        YLT             支持hdc驱动
  @verbatim
  ==============================================================================
	2020.2.4：
	* 修改原本大疆程序，读取elmo驱动发回来的速度，位置，电流
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 HCRT****************************
  */

#include "bsp_can.h"
#include "main.h"
#include <string.h>


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;



/**
  * @brief          hal库CAN回调函数,接收数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
int can5=0;
int can6=0;
int can7=0;
int can9=0;
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	CAN_RxHeaderTypeDef rx_header;

//  uint8_t rx_data[8];
//  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//	if(rx_header.StdId == 0x300+0)
//	{
//	}
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

/**
  * @brief          CAN滤波器初始化
  * @param[in]      void
  * @retval         none
  */
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


int cansb=0;
void CAN_Transmit_Messages_Byte(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t data0, uint8_t data1,
                                uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{

	cansb ++;
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxMessage[8];

  TxHeader.StdId = ID;
  TxHeader.ExtId = ID;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxMessage[0] = data0;
  TxMessage[1] = data1;
  TxMessage[2] = data2;
  TxMessage[3] = data3;
  TxMessage[4] = data4;
  TxMessage[5] = data5;
  TxMessage[6] = data6;
  TxMessage[7] = data7;

  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxMessage, &TxMailbox);
}

