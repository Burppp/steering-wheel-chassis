#include "bsp_can.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "Detection.h"
#include "can_hardwares.h"
#include "chassis.h"
#include "steering_wheel.h"
#include "travelling_wheel.h"

extern QueueHandle_t CAN1_receive_queue;
extern QueueHandle_t CAN2_receive_queue;
extern QueueHandle_t CHASSIS_motor_queue;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static motor_measure_t motor_1[8];
static motor_measure_t motor_2[8];

extern first_order_filter_type_t DM_velocity_filter;
extern moving_Average_Filter speed_average_filter;
extern second_lowPass_filter DM_velocity_f;
extern fp32 DM_velocity;
extern float aver_speed;
Power_feedback powerFeedback;
/**
  * @brief          通过can1/can2，按id以0x201～0x204（CMD_ID=0x200）或0x205～0x208（CMD_ID=0x1FF）的 顺 序 发送电机控制数据
  * @param[in]      选择发送总线为CAN1/CAN2
  * @param[in]      选择所发送的标识符
  * @param[in]      给id为 0x201/0x205 的电机发送控制数据
  * @param[in]      给id为 0x202/0x206 的电机发送控制数据
  * @param[in]      给id为 0x203/0x207 的电机发送控制数据
  * @param[in]      给id为 0x204/0x208 的电机发送控制数据
  * @retval         返回空
  */
void can_send_dji_motor(CAN_TYPE can_type, DJI_MOTOR_ID CMD_ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if (can_type == CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    } else if (can_type == CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }
}

/*!
 *
 * @brief 大疆C610/C620/GM6020信息解码
 * @param motor 电机数据结构体
 * @param data  can消息报文
 * @retval none
 */



void dji_motor_decode(motor_measure_t *motor,const uint8_t *data)
{
    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)(data[0]<<8 | data[1]);
    motor->speed_rpm = (int16_t)(data[2]<<8 | data[3]);
    motor->given_current = (int16_t)(data[4]<<8 | data[5]);
    motor->temperate = data[6];
}

/*!
 *
 * @brief 维特智能单圈绝对值编码器信息解码
 * @param encoder 电机数据结构体
 * @param data  can消息报文
 * @retval none
 */
void encoder_decode(encoder_t *encoder,const uint8_t *data)
{
    if(data[0] == 0x55&&data[1]==0x55)
    {

        encoder->real_angle= (float )(data[3]<<8 | data[2]) * 360 / 32768;//角度 = 角度寄存器数值*360/32768
        encoder->angle = encoder->real_angle - encoder->offset_angle;
        if(encoder->angle >= 180)
            encoder->angle -=360;
        else if(encoder->angle<=-180)
            encoder->angle +=360;
        if(encoder->angle>=180 || encoder->angle<=-180)
            encoder->angle = encoder->last_angle;
        encoder->last_angle = encoder->angle;
        encoder->speed_rpm = (int16_t)(data[5]<<8 | data[4]);
        encoder->angular_v = (float )(data[7]<<8 | data[6]) * 360 /32768;//角速度 = 角速度寄存器数值*360/32768
    }
    else if(data[0]==0x55&&data[1]==0x56)
    {
        encoder->temperate = (float )(data[3]<<8 |data[2])/100;
    }
}

void dji_motor_round_count(motor_measure_t *motor)
{

    if(motor->ecd - motor->last_ecd>4192){
        motor->round_count--;
    }
    else if(motor->ecd-motor->last_ecd< -4192)
    {
        motor->round_count++;
    }
    motor->total_ecd = motor->round_count*8192 + (motor->ecd - motor->offset_ecd);
}

fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
    int32_t tmp = 0;
    if (offset_ecd >= 4096) {
        if (ecd > offset_ecd - 4096) {
            tmp = ecd - offset_ecd;
        } else {
            tmp = ecd + 8192 - offset_ecd;
        }
    } else {
        if (ecd > offset_ecd + 4096) {
            tmp = ecd - 8192 - offset_ecd;
        } else {
            tmp = ecd - offset_ecd;
        }
    }
    return (fp32) tmp / 8192.f * 360;
}


/*!
 *
 * @brief 底盘信息解码
 * @param motor 电机数据结构体
 * @param can_id can_id
 * @param can_msg  can消息报文
 * @retval none
 */
void chassis_motor_decode(chassis_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_CHASSIS_3508_MOTOR_LF: {
                dji_motor_decode(&motor[1].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LB: {
                dji_motor_decode(&motor[2].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LB);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_CHASSIS_3508_MOTOR_RF: {
                dji_motor_decode(&motor[0].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RF);
            }
                break;
            case CAN_CHASSIS_3508_MOTOR_RB: {
                dji_motor_decode(&motor[3].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RB);
            }
                break;
            default: {
                break;
            }
        }
    }
}

void chassis_steering_motor_decode(steering_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_STEER_MOTOR_LF:{
                dji_motor_decode(&motor[0].motor_info,can_msg);
                detect_handle(DETECT_CHASSIS_6020_LF);
            }
                break;
            case CAN_STEER_MOTOR_LB:{
                dji_motor_decode(&motor[3].motor_info,can_msg);
                detect_handle(DETECT_CHASSIS_6020_LB);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_STEER_MOTOR_RF:{
                dji_motor_decode(&motor[1].motor_info,can_msg);
                detect_handle(DETECT_CHASSIS_6020_RF);
            }
                break;
            case CAN_STEER_MOTOR_RB:{
                dji_motor_decode(&motor[2].motor_info,can_msg);
                detect_handle(DETECT_CHASSIS_6020_RB);
            }
                break;
            default: {
                break;
            }
        }
    }
}

void STR_Motor_Can_Decode(STR_MOTOR_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_STEER_MOTOR_LF:{
                dji_motor_decode(&(motor[0].motor_data.Can_GetData),can_msg);
                detect_handle(DETECT_CHASSIS_6020_LF);
            }
                break;
            case CAN_STEER_MOTOR_LB:{
                dji_motor_decode(&(motor[3].motor_data.Can_GetData),can_msg);
                detect_handle(DETECT_CHASSIS_6020_LB);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_STEER_MOTOR_RF:{
                dji_motor_decode(&(motor[1].motor_data.Can_GetData),can_msg);
                detect_handle(DETECT_CHASSIS_6020_RF);
            }
                break;
            case CAN_STEER_MOTOR_RB:{
                dji_motor_decode(&(motor[2].motor_data.Can_GetData),can_msg);
                detect_handle(DETECT_CHASSIS_6020_RB);
            }
                break;
            default: {
                break;
            }
        }
    }
}

void TRA_Motor_Can_Decode(TRA_Motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_CHASSIS_3508_MOTOR_LF: {
                dji_motor_decode(&(motor[1].motor_data.Can_GetData), can_msg);
                detect_handle(DETECT_CHASSIS_3508_LF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LB: {
                dji_motor_decode(&(motor[2].motor_data.Can_GetData), can_msg);
                detect_handle(DETECT_CHASSIS_3508_LB);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_CHASSIS_3508_MOTOR_RF: {
                dji_motor_decode(&(motor[0].motor_data.Can_GetData), can_msg);
                detect_handle(DETECT_CHASSIS_3508_RF);
            }
                break;
            case CAN_CHASSIS_3508_MOTOR_RB: {
                dji_motor_decode(&(motor[3].motor_data.Can_GetData), can_msg);
                detect_handle(DETECT_CHASSIS_3508_RB);
            }
                break;
            default: {
                break;
            }
        }
    }
}

CANx_t can_1,can_2;
DM_Motor yaw_motor;
uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//电机保存零点命令
uint8_t Data_Clear_Error[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};   //电机清错命令

#define P_MIN 8192		//位置最小值
#define P_MAX 0		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

//达妙电机使能
void DM_enable(void)
{
    CANx_SendStdData(&hcan1, 0x01, Data_Enable, 8);
}

void Speed_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID,fp32 _vel)
{
    static CAN_TxHeaderTypeDef   Tx_Header;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    Tx_Header.StdId = ID;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = 0x04;

    can_1.Tx_Data[0] = *vbuf;
    can_1.Tx_Data[1] = *(vbuf+1);
    can_1.Tx_Data[2] = *(vbuf+2);
    can_1.Tx_Data[3] = *(vbuf+3);

    //寻空邮箱发送数据
    if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
                   float _KP, float _KD, float _torq)
{
    static CAN_TxHeaderTypeDef   Tx_Header;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    Tx_Header.StdId=id;
    Tx_Header.IDE=CAN_ID_STD;
    Tx_Header.RTR=CAN_RTR_DATA;
    Tx_Header.DLC=0x08;

    can_1.Tx_Data[0] = (pos_tmp >> 8);
    can_1.Tx_Data[1] = pos_tmp;
    can_1.Tx_Data[2] = (vel_tmp >> 4);
    can_1.Tx_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    can_1.Tx_Data[4] = kp_tmp;
    can_1.Tx_Data[5] = (kd_tmp >> 4);
    can_1.Tx_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    can_1.Tx_Data[7] = tor_tmp;

    //寻空邮箱发送数据
    if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, can_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}



/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
int status;
void DM_MotorDecode(DM_Motor *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        if (0x11 == can_id) {
            status = can_msg[0] & 0xF0;
            if(status == 0)
            {
                DM_enable();
            }
            yaw_motor.p_int=(can_msg[1]<<8)|can_msg[2];
            yaw_motor.v_int=(can_msg[3]<<4)|(can_msg[4]>>4);
            yaw_motor.t_int=((can_msg[4]&0xF)<<8)|can_msg[5];
//            fp32 position = uint_to_float(yaw_motor.p_int, P_MIN, P_MAX , 16); // (-12.5,12.5)
//            position *= 4;
//            while(position > 8192)
//                position -= 8192;
            yaw_motor.position = uint_to_float(yaw_motor.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
//            yaw_motor.position = position;
            yaw_motor.velocity = uint_to_float(yaw_motor.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
            yaw_motor.torque = uint_to_float(yaw_motor.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)

            first_order_filter_cali(&DM_velocity_filter, yaw_motor.velocity);
            DM_velocity = DM_velocity_filter.out;
//            average_add(&speed_average_filter,DM_velocity);
            detect_handle(DETECT_GIMBAL_DM_YAW);
        }
    }
}

void CAN_cmd_cap(uint16_t power_limit)
{
    static CAN_TxHeaderTypeDef tx_message;
    power_limit*=100;
    uint8_t sendBuf[8];
    sendBuf[0]=power_limit>>8;
    sendBuf[1]=power_limit;
    tx_message.StdId = 0x210;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, sendBuf, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, sendBuf, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(&hcan2, &tx_message, sendBuf, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

void Cap_Decode(uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_2)
    {
        if(can_id == 0x211)
        {
            uint16_t *PowerData = (uint16_t*) can_msg;
            powerFeedback.voltage_input = (fp32)PowerData[0] / 100.0f;//输入电压
            powerFeedback.voltage_cap = (fp32)PowerData[1] / 100.0f;//电容电压
            powerFeedback.current_input = (fp32)PowerData[2] / 100.0f;//输入电流
            powerFeedback.power_target = (fp32)PowerData[3] / 100.0f;//目标功率

            detect_handle(DETECT_CAP);
        }
    }
}
