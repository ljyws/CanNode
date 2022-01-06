#include "dgm_com.h"
#include "can.h"
#include "odrive_com.h"

DGMCanFrame dgmCanFrame = {0};
float dgmDesPosition;
float dgmDesVelocity;
float dgmDesCurrent;

static inline void int_to_data(int val, uint8_t *data)
{
	data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
	int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

void Can_send_dgm(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t data[8]) //can发送数据
{
  uint32_t TxMailbox;
  uint8_t TxMessage[8];
  CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = ID;
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

void Dgm_Motor_ON(DGMCanFrame *dgmCanFrame)
{
	uint32_t nodeID;
	nodeID = dgmCanFrame->can_id	&	0xF;
  int_to_data(0x00, dgmCanFrame->data);   
	Can_send_dgm(&hcan1,0x011,dgmCanFrame->data);
}
int ljy7 = 0;
void DgmSetMotorVel(float vel)
{
	ljy7++;
	uint8_t dgm_data[8];
	float_to_data(vel,dgm_data);
	Can_send_dgm(&hcan1,0x0D1,dgm_data);
}
void DGMMotorCtrl()
{
		switch(dgmCanFrame.dgmCmd)
		{
			case DGM_CMD_SET_TARGET_VELOCITY:
				DgmSetMotorVel(dgmDesVelocity);
			default:break;
		}
}





