//
// Created by nobody_knows on 23-3-13.
//

#include "can_hardwares.h"
#include "main.h"
#include "FreeRTOS.h" //FreeRTOS.h must code before queue.h!!!!!!!!
#include "queue.h"
#include "bsp_can.h"
#include "string.h"
#include "chassis.h"
#include "launcher.h"
#include "steering_wheel.h"
#include "travelling_wheel.h"
#include "can_common.h"

extern STR_Info_t str;
extern TRA_Info_t tra;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//extern chassis_t chassis;
extern gimbal_t gimbal;
extern launcher_t launcher;

extern void STR_Motor_Can_Decode(STR_MOTOR_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void TRA_Motor_Can_Decode(TRA_Motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
CANCommInstance *register_can_comm[MAX_CAN_COMM_COUNT]={NULL};

void register_can_comm_instance(CANCommInstance *ins)
{
    for (int i = 0; i < MAX_CAN_COMM_COUNT; i++)
    {
        if (register_can_comm[i] == NULL)
        {
            register_can_comm[i] = ins;
            idx++;
            break;
        }
    }
}

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

    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_MSG can_msg;
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_msg.rx_header,can_msg.rx_date);
    if(hcan == &hcan1)
    {
//        chassis_motor_decode(chassis.motor_chassis,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
//        chassis_steering_motor_decode(chassis.motor_steering,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);

        DM_MotorDecode(&yaw_motor, CAN_1, can_msg.rx_header.StdId,can_msg.rx_date);
        STR_Motor_Can_Decode(str.STR_Axis,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
        TRA_Motor_Can_Decode(tra.TRA_Axis,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
//        Cap_Decode(CAN_1, can_msg.rx_header.StdId, can_msg.rx_date);

//        gimbal_motor_decode(gimbal.motor_gimbal,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
//        launcher_motor_decode(launcher.motor_launcher,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
    }
//    else if(hcan == &hcan2)
//    {
//        STR_Motor_Can_Decode(str.STR_Axis,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
//        TRA_Motor_Can_Decode(tra.TRA_Axis,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
//
//        yaw_encoder_decode(&gimbal.encoder_yaw,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
//        launcher_motor_decode(launcher.motor_launcher,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
//    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        CAN_MSG can_msg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &can_msg.rx_header, can_msg.rx_date);
        if (hcan == &hcan2)
        {
            STR_Motor_Can_Decode(str.STR_Axis,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
            TRA_Motor_Can_Decode(tra.TRA_Axis,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
            Cap_Decode(CAN_2, can_msg.rx_header.StdId, can_msg.rx_date);
//            yaw_encoder_decode(&gimbal.encoder_yaw,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
//            launcher_motor_decode(launcher.motor_launcher,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);
            for(size_t i=0;i<idx;i++)
            {
                if(register_can_comm[i]->rx_id == can_msg.rx_header.StdId)
                {
                    register_can_comm[i]->rx_len = can_msg.rx_header.DLC; // 保存接收到的数据长度
                    memcpy(register_can_comm[i]->rx_buff,can_msg.rx_date,can_msg.rx_header.DLC); // 消息拷贝到对应实例的缓存中
                    CANCommRxCallback(register_can_comm[i]);  // 触发回调进行数据解析和处理
                }
            }
        }
    }
}
