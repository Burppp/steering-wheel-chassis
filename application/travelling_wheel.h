//
// Created by Lumos on 2023/12/18.
//

#ifndef REMOTE_CUBE_SENTRY_TRAVELLING_WHEEL_H
#define REMOTE_CUBE_SENTRY_TRAVELLING_WHEEL_H

#include "struct_typedef.h"
#include "protocol_shaob.h"
#include "steering_wheel.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "remote.h"
#include "PID.h"
#include "user_lib.h"
#include "AHRS_MiddleWare.h"

#define TRA_MOTOR_CNT 4

#define STEERING_WHEEL_CHASSIS_KP 10.0
#define STEERING_WHEEL_CHASSIS_KI 0.2
#define STEERING_WHEEL_CHASSIS_KD 3.0
#define STEERING_WHEEL_CHASSIS_OUT 11000.0
#define STEERING_WHEEL_CHASSIS_IOUT 4000.0

#define SPEED_MINIMUM 16

#define CHASSIS_TASK_INIT_TIME 157

#define TRAVELLING_WHEEL_RADIUS 0.11

extern STR_Info_t str;

extern RC_ctrl_t rc_ctrl;
//上位机下发数据
extern robot_ctrl_info_t robot_ctrl;

//由云台YAW轴朝向确定底盘朝前或朝后
typedef enum
{
    HEAD = 1,
    TAIL = -1
}Dir_t;

//底盘行进轮电机ID，左CAN1，右CAN2
typedef enum
{
    TRA_RF = 0,
    TRA_LF = 1,
    TRA_LB = 2,
    TRA_RB = 3
}TRA_MOTOR_ID;

//pid和motor_measure
typedef struct
{
    Motor_Data_t motor_data;
    pid_t Speed_Loop;
    pid_t Angle_Loop;

    bool reverse_180;
    fp32 reverse_num;
}TRA_Motor_t;

//底盘矢量，由steering_wheel中来
typedef struct
{
    fp32 X_Speed;
    fp32 Y_Speed;
    fp32 Z_Speed;

    fp32 X_Speed_k;
    fp32 Y_Speed_k;
    fp32 Z_Speed_k;

    fp32 Motor_Target_Speed[TRA_MOTOR_CNT];
    fp32 increase;
}TRA_Vector_Move_t;

typedef struct
{
    Dir_t head_or_tail;
    fp32 num;
}TRA_Dir_t;

//行进轮总结构体
typedef struct
{
    Mode_t ctrl_mode;

    TRA_Motor_t TRA_Axis[TRA_MOTOR_CNT];
    fp32 Speed_Max;
    TRA_Vector_Move_t Vector;
    TRA_Dir_t Dir;
}TRA_Info_t;

extern TRA_Info_t tra;

void TRA_Set_Mode();
void TRA_Motor_PID_Init(TRA_Motor_t *motor);
void TRA_PID_Init();
fp32 TRA_LF_Steer_Angle_Ramp(fp32 Speed_Target);
fp32 TRA_LB_Steer_Angle_Ramp(fp32 Speed_Target);
fp32 TRA_RB_Steer_Angle_Ramp(fp32 Speed_Target);
fp32 TRA_RF_Steer_Angle_Ramp(fp32 Speed_Target);
fp32 TRA_Get_Fusion_Speed_LF();
fp32 TRA_Get_Fusion_Speed_LB();
fp32 TRA_Get_Fusion_Speed_RF();
fp32 TRA_Get_Fusion_Speed_RB();
void TRA_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase);
void TRA_PID_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase);
void TRA_180_Reverse();
void TRA_Remote_Ctrl();
fp32 TRA_RPM_To_Speed(int16_t speed_rpm);
void TRA_Motor_Info_Update(TRA_Motor_t *motor);
fp32 TRA_Get_Output(TRA_Motor_t *motor);
void TRA_Can_Send(int16_t *give_current);
void TRA_Output();
void TRA_Relax_Handle();

void TRA_Motor_Reverse_Handler(TRA_Motor_t *motor);
void TRA_Reverse_Handler();

_Noreturn void TRA_ctrl(void const * pvParameters);

#endif //REMOTE_CUBE_SENTRY_TRAVELLING_WHEEL_H
