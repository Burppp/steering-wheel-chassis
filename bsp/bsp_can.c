#include "bsp_can.h"
#include "main.h"
#include <string.h>

#include "object_dictionary.h"
#include "packet_param.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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

/********************************************第一层接口，调用SDO/PDO*****************************************************/
/**
 * 发送SDO报文
 * 参数：
 * motor_id：电机ID
 * ctrl：控制字
 * index：索引
 * subindex：子索引
 * value：值
 * dataType：数据类型
 * 返回值：是否发送成功
 */
bool SDO_Send(uint8_t motor_id, uint8_t ctrl, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType) 
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef chassis_tx_message;
    uint8_t chassis_can_send_data[8];
    
    chassis_tx_message.StdId = 0x600 + motor_id;
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
    return HAL_ERROR;
}

/**
 * 发送PDO报文
 * 参数：
 * ctrl：控制字
 * motor_id：电机ID
 * 返回值：是否发送成功
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
 * 使能电机
 * 参数：
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_enable(uint8_t motor_id) 
{
	uint16_t value = 0;
	if(motor_id & 0x01)
		value = CONTROL_WORD_ABSOLUTE_POSITION_ENABLE;
	else
		value = CONTROL_WORD_ENABLE;
    SDO_Write(motor_id, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_UINT16);
}

/**
 * 设置电机模式
 * 参数：
 * mode：模式
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_mode(int8_t mode, uint8_t motor_id) 
{
    SDO_Write(motor_id, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT, &mode, OD_TYPE_INT8);
}

/**
 * 设置梯形速度
 * 参数：
 * profile_speed：梯形速度
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id) 
{
	float dec_f = (float)(profile_speed);
	dec_f = dec_f * 512 * 65535 / 1875;
	uint32_t dec_u = (uint32_t)dec_f;
    SDO_Write(motor_id, OD_INDEX_PROFILE_SPEED, OD_SUBINDEX_DEFAULT, &dec_u, OD_TYPE_UINT32);
}

/**
 * 设置位置
 * 参数：
 * position_ref：位置
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_setPos(int32_t position_ref, uint8_t motor_id) 
{
    SDO_Write(motor_id, OD_INDEX_TARGET_POS, OD_SUBINDEX_DEFAULT, &position_ref, OD_TYPE_INT32);
}   

/**
 * 设置转矩
 * 参数：
 * torque_ref：转矩
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_setTorque(int16_t torque_ref, uint8_t motor_id) 
{
    SDO_Write(motor_id, OD_INDEX_TARGET_TORQUE, OD_SUBINDEX_DEFAULT, &torque_ref, OD_TYPE_INT16);
}

/**
 * 设置速度
 * 参数：
 * speed_ref：速度
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_setSpeed(int32_t speed_ref, uint8_t motor_id) 
{
    SDO_Write(motor_id, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT, &speed_ref, OD_TYPE_INT32);
}

/**
 * 复位编码器多圈
 * 参数：
 * value：值
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_encoderDataReset(uint8_t value, uint8_t motor_id) 
{
    SDO_Write(motor_id, OD_INDEX_ENCODER_DATA_RESET, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_UINT8);
}

/**
 * 配置TPDO
 * 参数：
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_TPDO_config(uint8_t motor_id)
{
	uint8_t value = 3;
	uint32_t pos_fdb_address = OD_INDEX_ACTUAL_POSITION;
	uint32_t vel_fdb_address = OD_INDEX_ACTUAL_VELOCITY;
	uint32_t tor_fdb_address = OD_INDEX_ACTUAL_CURRENT;
	SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_INT8);
	SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_1, &pos_fdb_address, OD_TYPE_UINT32);
	SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_2, &vel_fdb_address, OD_TYPE_UINT32);
	SDO_Write(motor_id, OD_INDEX_TPDO_VALUE, OD_SUBINDEX_3, &tor_fdb_address, OD_TYPE_UINT32);
}

/**
 * 使能TPDO
 * 参数：
 * ctrl：控制字
 * motor_id：电机ID
 * 返回值：是否发送成功
 */
void motor_TPDO_enable(uint8_t ctrl, uint8_t motor_id)
{
	PDO_Send(ctrl, motor_id);
}

