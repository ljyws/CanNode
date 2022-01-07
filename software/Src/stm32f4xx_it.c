/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ctrl.h"
#include "dji_comm.h"
#include "odrive_com.h"
#include "bsp_usart.h"
#include "dgm_com.h"
#include "vesc_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern MotorType Motor[8];
extern DriverType Driver[8];
extern DGMCanFrame dgmCanFrame;
extern float dgmDesPosition;
extern float dgmDesVelocity;
extern float dgmDesCurrent;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
	
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}
int ljy9= 0;
/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
		ljy9++;
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uart_receive_handler(&huart3);
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupts.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int ljy3=0;
int ljy4 = 0;
float veltest;
  UnionDataType Msg,Msg1;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8],n;


	if(hcan == &hcan1)
	{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  for (uint8_t i = 0; i < 8; i++)
    Msg.data8[i] = rx_data[i];

  if ((rx_header.StdId == 0x201) || (rx_header.StdId == 0x202) || (rx_header.StdId == 0x203) || (rx_header.StdId == 0x204) ||
      (rx_header.StdId == 0x205) || (rx_header.StdId == 0x206) || (rx_header.StdId == 0x207) || (rx_header.StdId == 0x208))
  {
    n = rx_header.StdId - 0x201;
    Motor[n].pos = (Msg.data8[0] << 8) + Msg.data8[1];
		if(Motor[n].cnt < 50)
		{
			Motor[n].posLast = Motor[n].pos;
			Motor[n].cnt ++;
		}	
    Motor[n].vel = (int16_t)(Msg.data8[2] << 8) + Msg.data8[3];
    Motor[n].cur = (Msg.data8[4] << 8) + Msg.data8[5];
    Motor[n].temp = Msg.data8[6];
    CalculSpeed_Pos(&Driver[n], &Motor[n]);
  }
	veltest = Motor[0].pos;
	}
	if(hcan == &hcan2)
	{

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  for (uint8_t i = 0; i < 8; i++)
    Msg1.data8[i] = rx_data[i];
	if (Msg1.data16[1] == 0x0000)
    {
  for (int j = 0; j < 8; j++)
  {

    if (Motor[j].type == NULL)
      break;
		if( rx_header.StdId == (0x300 + Driver[j].command.canId))
		{
      switch (Msg1.data16[0])
    {
      case 0x0002: //JV
        Driver[j].velCtrl.desiredVel[CMD] = (float)(Msg1.data32[1]) / 1000;
        Driver[j].velCtrl.desiredVel[CMD] = MaxMinLimit(Driver[j].velCtrl.desiredVel[CMD], Driver[j].velCtrl.desiredVel[MAX_V]);
        break;
      case 0x0003: //PA绝对位置
        Driver[j].posCtrl.desiredPos = (float)(Msg1.data32[1]);
				veltest = (float)(Msg1.data32[1]);
        break;
      case 0x0004: //PR相对位置
        Driver[j].posCtrl.desiredPos = Driver[j].posCtrl.actualPos + (float)(Msg1.data32[1]);
        break;
      case 0x5149: //IQ	 读取电流
        Driver[j].command.can_status = 0x40005149;
        break;
      case 0x5856: //VX   读取速度
        Driver[j].command.can_status = 0x40005856;
        break;
      case 0x5850: //PX   读取位置
        Driver[j].command.can_status = 0x40005850;
        break;
      default:
        break;
      }
		}
    }
	}
    else if (Msg1.data16[1] == ODRIVE_BOARD_BASEID) //Odrive 
    {
			if(rx_header.StdId == 0x401)
			{
				switch (Msg1.data16[0])
				{
					case 0x0001:  //电机参数校准
						Odrive_motor_calibration();break;

					case 0x0002:   //进行编码器偏移校准
						Odrive_encoder_offset_calibration();break;
					case 0x0003:
						Odrive_encoder_index_search();break;
					case 0x0004:  //设置电机为闭环运行模式
						Odrive_set_closed_loop();break;
					case 0x0005: //配置电机控制器模式
						Odrive_set_control_mode(Msg1.data8[5],Msg1.data8[4]);break;
					case 0x0006: //控制电机在速度模式下转动
						Odrive_set_motor_vel((float)Msg1.data32[1]);break;
					case 0x0007:  //控制电机在位置模式下转动
						Odrive_set_motor_pos(Msg1.data32[1]);break;
			
					default:
						break;
				}
			}	
		}
		else if(Msg1.data16[1] == DGM_BOARD_BASEID)
		{

				if(rx_header.StdId == 0x501)
				{

					switch(Msg1.data16[0])
					{
						case 0x0001:  //速度控制
							ljy3++;
							dgmCanFrame.dgmCmd = DGM_CMD_SET_TARGET_VELOCITY;
							dgmDesVelocity = (float)Msg1.data32[1];
							break;
						case 0x0002:
							break;
						case 0x0003:
							break;
						default:break;
					}
				}
			}
		else if(Msg1.data16[1] == VESC_BOARD_BASEID)
		{
			for(int i = 1;i<4;i++)
			{
				if(rx_header.StdId == 0x500 + i)
					switch(Msg1.data16[0])
					{
						case 0x0001:
							break;
						case 0x0002:
												ljy4++;
							SetMotor(i,RPM,(float)Msg1.data32[1]);
							break;	                 
						case 0x0003:
							break;
						default :break;
					}
			}
		}


	
		
	}
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
int sbcx=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	sbcx++;
	DGMMotorCtrl();
    DJIMotorCtrl();
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
