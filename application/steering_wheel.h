//
// Created by Lumos on 2023/12/18.
//

#ifndef REMOTE_CUBE_SENTRY_STEERING_WHEEL_H
#define REMOTE_CUBE_SENTRY_STEERING_WHEEL_H

#include <stdbool.h>
//#include <arm_math.h>
#include "PID.h"
#include "bsp_can.h"

#define CHASSIS_BUFFER_SIZE 200
static uint8_t usart1_buff[CHASSIS_BUFFER_SIZE] = {0};

#define STR_MOTOR_CNT 4

#define ROUND_ECD 8192
#define HALF_ROUND_ECD 4096
#define QUARTER_ROUND_ECD 2048

//初始舵向
#define STR_LF_OFFSET 7681
#define STR_RF_OFFSET 2005
#define STR_RB_OFFSET 4131
#define STR_LB_OFFSET 7568

#define STEERING_WHEEL_STEER_ANGLE_KP 0.3
#define STEERING_WHEEL_STEER_ANGLE_KI 0.0
#define STEERING_WHEEL_STEER_ANGLE_KD 1.0
#define STEERING_WHEEL_STEER_ANGLE_OUT 150.0
#define STEERING_WHEEL_STEER_ANGLE_IOUT 30.0

#define STEERING_WHEEL_STEER_SPEED_KP 10.0
#define STEERING_WHEEL_STEER_SPEED_KI 5.0
#define STEERING_WHEEL_STEER_SPEED_KD 100.0
#define STEERING_WHEEL_STEER_SPEED_OUT 28000.0
#define STEERING_WHEEL_STEER_SPEED_IOUT 8000.0

#define STR_SPIN_MINIMUM_CH2 22
#define STR_Z_LR_XY_SPEED_MINIMUM_LIMIT 3
#define STR_XY_DIR_XY_SPEED_MINIMUM_LIMIT 2

#define STR_GYRO_PARAM 100  //300

#define RADIUS_6020 0.03     // m

#define CHASSIS_TASK_INIT_TIME 157

//底盘转向轮电机ID，左CAN1，右CAN2
typedef enum
{
    STR_LF = 0,
    STR_RF = 1,
    STR_RB = 2,
    STR_LB = 3,
}STR_MOTOR_ID;

//陀螺旋转方向，LEFT逆时针，RIGHT顺时针
typedef enum
{
    LEFT = -1,
    RIGHT = 1,
}Z_LR_dir_t;

typedef enum
{
    CHASSIS_INIT,
    CHASSIS_RELAX,
    CHASSIS_SPEED,
    CHASSIS_GYRO,
    CHASSIS_AUTO
}Mode_t;

//底盘矢量
typedef struct
{
    fp32 F_dir;
    fp32 XY_dir;
    fp32 Z_dir;
    fp32 XY_Z_Angle;
    fp32 prev_Z_Angle;
    fp32 XYZ_Speed;
    Z_LR_dir_t Z_LR;
    fp32 Dir;
    fp32 Dir_last;
    bool JUST_SPIN;         //底盘旋转
}STR_Move_t;

//motor_measure和速度环、角度环数据
typedef struct
{
    motor_measure_t Can_GetData;

    fp32 PID_Speed;
    fp32 PID_Speed_target;
    fp32 PID_Angle;
    fp32 PID_Angle_target;
}Motor_Data_t;

typedef struct
{
    Motor_Data_t motor_data;
    pid_t Speed_Loop;
    pid_t Angle_Loop;

    fp32 Move_Mid_Angle;
    fp32 Spin_Mid_Angle;

    STR_Move_t Move;
    bool tra_reverse;
    // 用于计算反舵
    fp32 STR_Reverse_Ecd;
}STR_MOTOR_t;

//底盘矢量
typedef struct
{
    fp32 X_speed;
    fp32 Y_speed;
    fp32 Z_speed;

    Z_LR_dir_t Z_LR;

    fp32 XY_speed;
    fp32 XYZ_speed;
}STR_Vector_Move_t;

//底盘转向轮总结构体
typedef struct
{
    Mode_t ctrl_mode;

    STR_MOTOR_t STR_Axis[STR_MOTOR_CNT];
    STR_Vector_Move_t Vector;

}STR_Info_t;

void STR_Set_Mode();
void STR_Offset_Init();
void STR_Motor_PID_Init(STR_MOTOR_t *motor);
void STR_PID_Init();
void STR_Motor_First_Angle(STR_MOTOR_t *motor);
void STR_First_Angle();
bool STR_JUST_SPIN();
fp32 STR_Z_Speed(fp32 Z_Speed);
void STR_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase, fp32 limit);
fp32 STR_Encoder_Limit(int16_t Encoder);
void STR_Motor_F_dir(STR_MOTOR_t *motor);
void STR_F_dir_Update();
void STR_Z_LR(STR_MOTOR_t *motor, fp32 Z_speed);
void STR_Motor_Z_dir(STR_MOTOR_t *motor);
void STR_Z_dir_Update();
fp32 STR_Get_XY_Fusion_Speed(fp32 X_Speed, fp32 Y_Speed);
fp32 STR_Get_XY_Dir();
fp32 STR_Get_XY_Z_Dir_Err(fp32 xy_dir, fp32 z_dir);
void STR_Get_XYZ_Speed(fp32 XY_Speed, fp32 Z_Speed, STR_MOTOR_t *motor);
fp32 STR_Get_Z_Dir(STR_MOTOR_t *motor);
void STR_Motor_Dir(STR_MOTOR_t *motor);
void STR_Dir_Update();
void STR_Remote_Ctrl();
void STR_Motor_Info_Update(STR_MOTOR_t *motor);
fp32 STR_Get_Set_Angle(fp32 set_angle, fp32 get_angle);
fp32 STR_Get_Output(STR_MOTOR_t *motor);
void STR_Output();
void STR_GYRO_Ctrl();
void STR_Gyro_F_Angle();
void STR_Gyro_Z_Angle();
void STR_Gyro_Fusion();
void STR_UART_Ctrl();
void chassis_device_offline_handle();

// 反舵的函数
void STR_Motor_Reverse_Handler(STR_MOTOR_t *motor);
void STR_Reverse_Handler();

_Noreturn void STR_ctrl(void const * pvParameters);

#endif //REMOTE_CUBE_SENTRY_STEERING_WHEEL_H
