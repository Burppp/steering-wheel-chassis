#include "bsp_can.h"
#include "main.h"
#include <string.h>

#include "object_dictionary.h"
#include "packet_param.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_t motor_list[8];

void can_filter_init(void)
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

void motor_init(Motor_t* motor, ID_ENUM id)
{
    motor->od = OD_Create();
    if (motor->od == NULL) {
        return;
    }
    OD_User_Init(motor->od);

    motor->motor_id = id;
    motor->mode = TORQUE_CONTROL;
    motor->is_enable = false;
    motor->position_feedback = 0;
    motor->speed_feedback = 0;
    motor->torque_feedback = 0;

    motor->position_ref = 0;
    motor->speed_ref = 0;
    motor->torque_ref = 0;

    motor->position_feedback_last = 0;
    motor->speed_feedback_last = 0;

    motor->offset = 0;

    if(id % 2)
    {
        pid_init(&motor->speed_loop,
            DRV_SPEED_LOOP_KP, 
            DRV_SPEED_LOOP_KI, 
            DRV_SPEED_LOOP_KD, 
            DRV_SPEED_LOOP_OUTPUT_MAX, 
            DRV_SPEED_LOOP_OUTPUT_MIN, 
            DRV_SPEED_LOOP_INTEGRAL_MAX, 
            DRV_SPEED_LOOP_INTEGRAL_MIN);
    }
    else
    {
        pid_init(&motor->position_loop, 
            STR_POSITION_LOOP_KP, 
            STR_POSITION_LOOP_KI, 
            STR_POSITION_LOOP_KD, 
            STR_POSITION_LOOP_OUTPUT_MAX, 
            STR_POSITION_LOOP_OUTPUT_MIN, 
            STR_POSITION_LOOP_INTEGRAL_MAX, 
            STR_POSITION_LOOP_INTEGRAL_MIN);
        pid_init(&motor->speed_loop, 
            STR_SPEED_LOOP_KP, 
            STR_SPEED_LOOP_KI, 
            STR_SPEED_LOOP_KD, 
            STR_SPEED_LOOP_OUTPUT_MAX, 
            STR_SPEED_LOOP_OUTPUT_MIN, 
            STR_SPEED_LOOP_INTEGRAL_MAX, 
            STR_SPEED_LOOP_INTEGRAL_MIN);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef can_msg_header;
    uint8_t can_msg_data[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_msg_header,can_msg_data);
    if(hcan == &hcan1)
    {
        uint8_t ctrl_byte = can_msg_data[0];
        uint8_t index_low_byte = can_msg_data[1];
        uint8_t index_high_byte = can_msg_data[2];
        uint16_t index = (index_high_byte << 8) | index_low_byte;
        uint8_t subindex = can_msg_data[3];

        uint8_t data_length = 0;
        uint8_t data[8] = {0};
        switch(ctrl_byte)
        {
            case CAN_RECV_1_BYTE:
                data_length = 1;
                memcpy(data, &can_msg_data[4], data_length);
                break;
            case CAN_RECV_2_BYTE:
                data_length = 2;
                memcpy(data, &can_msg_data[4], data_length);
                break;
            case CAN_RECV_4_BYTE:
                data_length = 4;
                memcpy(data, &can_msg_data[4], data_length);
                break;
            case CAN_SEND_SUCCESS:
                break;
            case CAN_FAILED:
                break;
            default:
                break;
        }
        
        switch(can_msg_header.StdId - SDO_RECV_OFFSET)
        {
            case LF_STEER_ID:
                OD_SetValue(motor_list[0].od, index, subindex, data);
                break;
            case LF_DRIVE_ID:
                OD_SetValue(motor_list[1].od, index, subindex, data);
                break;
            case RF_STEER_ID:
                OD_SetValue(motor_list[2].od, index, subindex, data);
                break;
            case RF_DRIVE_ID:
                OD_SetValue(motor_list[3].od, index, subindex, data);
                break;
            case LB_STEER_ID:
                OD_SetValue(motor_list[4].od, index, subindex, data);
                break;
            case LB_DRIVE_ID:
                OD_SetValue(motor_list[5].od, index, subindex, data);
                break;
            case RB_STEER_ID:
                OD_SetValue(motor_list[6].od, index, subindex, data);
                break;
            case RB_DRIVE_ID:
                OD_SetValue(motor_list[7].od, index, subindex, data);
                break;
            default:
                break;
        }
    }
}

/********************************************第一层接口，调用SDO/PDO*****************************************************/
/**
  * @brief     SDO 发送
  * @param     motor_id：电机ID
  * @param     ctrl：控制字
  * @param     index：索引
  * @param     subindex：子索引
  * @param     value：值
  * @param     dataType：数据类型
  * @return    是否发送成功
  */
bool SDO_Send(uint8_t motor_id, uint8_t ctrl, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType) 
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef chassis_tx_message;
    uint8_t chassis_can_send_data[8];
    
    chassis_tx_message.StdId = SDO_SEND_OFFSET + motor_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    
    if(ctrl == CAN_SDO_READ_CTRL_BYTE)
    {
        //SDO read
        chassis_can_send_data[0] = CAN_SDO_READ_CTRL_BYTE;
        chassis_can_send_data[1] = index & 0xFF;
        chassis_can_send_data[2] = (index >> 8) & 0xFF;
        chassis_can_send_data[3] = subindex;  
        for(int i = 4; i < 8; i++)
        {
            chassis_can_send_data[i] = 0x00;
        }
        return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box) == HAL_OK;
    }
    else
    {
        //SDO write
        uint8_t data_length_ctrl = 0;
        uint8_t data_length = 0;

        switch (dataType) {
            case OD_TYPE_INT8:
            case OD_TYPE_UINT8:
                data_length_ctrl = CAN_SEND_1_BYTE;
                data_length = 1;
                break;
            case OD_TYPE_INT16:
            case OD_TYPE_UINT16:
                data_length_ctrl = CAN_SEND_2_BYTE;
                data_length = 2;
                break;
            case OD_TYPE_INT32:
            case OD_TYPE_UINT32:
                data_length_ctrl = CAN_SEND_4_BYTE;
                data_length = 4;
                break;
            default:
                return false;
        }
        
        chassis_can_send_data[0] = data_length_ctrl;
        chassis_can_send_data[1] = index & 0xFF;
        chassis_can_send_data[2] = (index >> 8) & 0xFF;
        chassis_can_send_data[3] = subindex;
        if(value != NULL)
            memcpy(&chassis_can_send_data[4], value, data_length);
        else 
            return HAL_ERROR;
        
        for(int i = 4 + data_length; i < 8; i++) {
            chassis_can_send_data[i] = 0x00;
        }
        
        return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box) == HAL_OK;
    }
}

/**
  * @brief     PDO 发送
  * @param     ctrl：控制字
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool PDO_Send(uint8_t ctrl, uint8_t motor_id)
{
	uint32_t send_mail_box;
    CAN_TxHeaderTypeDef chassis_tx_message;
    uint8_t chassis_can_send_data[8];
    
    chassis_tx_message.StdId = 0x000;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
	
	chassis_can_send_data[0] = ctrl;
	chassis_can_send_data[1] = motor_id;
	chassis_can_send_data[2] = 0;
	chassis_can_send_data[3] = 0;
	chassis_can_send_data[4] = 0;
	chassis_can_send_data[5] = 0;
	chassis_can_send_data[6] = 0;
	chassis_can_send_data[7] = 0;
	
    return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box) == HAL_OK;
}

/********************************************第二层接口，通过索引读写操作*****************************************************/
bool SDO_Read(uint8_t motor_id, uint16_t index, uint8_t subindex, OD_DataType dataType)
{
    return SDO_Send(motor_id, CAN_SDO_READ_CTRL_BYTE, index, subindex, NULL, dataType);
}

bool SDO_Write(uint8_t motor_id, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType)
{
    return SDO_Send(motor_id, CAN_SDO_WRITE_CTRL_BYTE, index, subindex, value, dataType);
}


/********************************************第三层接口，功能接口************************************************************/
/**
  * @brief     使能电机
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_enable(uint8_t motor_id) 
{
	uint16_t value = 0;
	if(motor_id & 0x01)
		value = CONTROL_WORD_ABSOLUTE_POSITION_ENABLE;
	else
		value = CONTROL_WORD_ENABLE;
    return SDO_Write(motor_id, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_UINT16);
}

/**
  * @brief     设置电机模式
  * @param     mode：模式
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_mode(int8_t mode, uint8_t motor_id) 
{
    return SDO_Write(motor_id, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT, &mode, OD_TYPE_INT8);
}

/**
  * @brief     设置梯形速度
  * @param     profile_speed：梯形速度
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id) 
{
	float dec_f = (float)(profile_speed);
	dec_f = dec_f * 512 * 65535 / 1875;
	uint32_t dec_u = (uint32_t)dec_f;
    return SDO_Write(motor_id, OD_INDEX_PROFILE_SPEED, OD_SUBINDEX_DEFAULT, &dec_u, OD_TYPE_UINT32);
}

/**
  * @brief     设置位置
  * @param     position_ref：位置
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_setPos(int32_t position_ref, uint8_t motor_id) 
{
    return SDO_Write(motor_id, OD_INDEX_TARGET_POS, OD_SUBINDEX_DEFAULT, &position_ref, OD_TYPE_INT32);
}   

/**
  * @brief     设置转矩
  * @param     torque_ref：转矩
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_setTorque(int16_t torque_ref, uint8_t motor_id) 
{
    return SDO_Write(motor_id, OD_INDEX_TARGET_TORQUE, OD_SUBINDEX_DEFAULT, &torque_ref, OD_TYPE_INT16);
}

/**
  * @brief     设置速度
  * @param     speed_ref：速度
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_setSpeed(int32_t speed_ref, uint8_t motor_id) 
{
    return SDO_Write(motor_id, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT, &speed_ref, OD_TYPE_INT32);
}

/**
  * @brief     复位编码器多圈
  * @param     value：值
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_encoderDataReset(uint8_t value, uint8_t motor_id) 
{
    return SDO_Write(motor_id, OD_INDEX_ENCODER_DATA_RESET, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_UINT8);
}

/**
  * @brief     配置TPDO
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_TPDO_config(uint8_t motor_id)
{
    uint8_t ret = 0;
	uint8_t value = 3;
	uint32_t pos_fdb_address = OD_INDEX_ACTUAL_POSITION;
	uint32_t vel_fdb_address = OD_INDEX_ACTUAL_VELOCITY;
	uint32_t tor_fdb_address = OD_INDEX_ACTUAL_CURRENT;
	ret += SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_INT8);
	ret += SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_1, &pos_fdb_address, OD_TYPE_UINT32);
	ret += SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_2, &vel_fdb_address, OD_TYPE_UINT32);
	ret += SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_3, &tor_fdb_address, OD_TYPE_UINT32);
	return ret;
}

/**
  * @brief     使能TPDO
  * @param     ctrl：控制字
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_TPDO_enable(uint8_t ctrl, uint8_t motor_id)
{
	return PDO_Send(ctrl, motor_id);
}

/**
  * @brief     读取实际位置
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_readRealPosition(uint8_t motor_id)
{
    return SDO_Read(motor_id, OD_INDEX_ACTUAL_POSITION, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32);
}

/**
  * @brief     读取实际速度
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_readRealVelocity(uint8_t motor_id)
{
    return SDO_Read(motor_id, OD_INDEX_ACTUAL_VELOCITY, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32);
}

/**
  * @brief     读取实际转矩
  * @param     motor_id：电机ID
  * @return    是否发送成功
 */
bool motor_readRealTorque(uint8_t motor_id)
{
    return SDO_Read(motor_id, OD_INDEX_ACTUAL_CURRENT, OD_SUBINDEX_DEFAULT, OD_TYPE_INT16);
}
