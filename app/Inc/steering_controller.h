#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include "packet_param.h"
#include "bsp_can.h"
#include "cmsis_os.h"

#define WHEEL_CNT 2

typedef enum
{
    CHASSIS_INIT,
    CHASSIS_RELAX,
    CHASSIS_SPEED,
    CHASSIS_GYRO,
    CHASSIS_AUTO
}ChassisMode_t;

typedef enum
{
    LEFT = -1,
    RIGHT = 1
}Z_LR_dir_t;

typedef struct
{
    float X_speed;
    float Y_speed;
    float Z_speed;

    float X_speed_k;
    float Y_speed_k;
    float Z_speed_k;

    float target_speed[WHEEL_CNT];
    float increase;

    Z_LR_dir_t Z_LR;

    float XY_speed;
    float XYZ_speed;
}Vector_t;

typedef struct
{
    ChassisMode_t mode;
    Motor_t STR_Axis[WHEEL_CNT];
    Vector_t vector;
}ChassisSteering_t;

void chassis_init(void);
void steeringCtrl_task(void const * pvParameters);

void STR_Set_Mode();
void STR_Motor_PID_Init(Motor_t *motor);
void STR_Motor_First_Angle(Motor_t *motor);
void STR_First_Angle();
bool STR_JUST_SPIN();
float STR_Z_Speed(float Z_Speed);
void STR_Speed_Ramp(float *receive, float source, float increase, float limit);
float STR_Encoder_Limit(int16_t Encoder);
void STR_Motor_F_dir(Motor_t *motor);
void STR_F_dir_Update();
void STR_Z_LR(Motor_t *motor, float Z_speed);
void STR_Motor_Z_dir(Motor_t *motor);
void STR_Z_dir_Update();
float STR_Get_XY_Fusion_Speed(float X_Speed, float Y_Speed);
float STR_Get_XY_Dir();
float STR_Get_XY_Z_Dir_Err(float xy_dir, float z_dir);
void STR_Get_XYZ_Speed(float XY_Speed, float Z_Speed, Motor_t *motor);
float STR_Get_Z_Dir(Motor_t *motor);
void STR_Motor_Dir(Motor_t *motor);
void STR_Dir_Update();
void STR_Remote_Ctrl();
void STR_Motor_Info_Update(Motor_t *motor);
float STR_Get_Set_Angle(float set_angle, float get_angle);
float STR_Get_Output(Motor_t *motor);
void STR_Output();
void STR_GYRO_Ctrl();
void STR_Gyro_F_Angle();
void STR_Gyro_Z_Angle();
void STR_Gyro_Fusion();
void STR_UART_Ctrl();
void chassis_device_offline_handle();

void STR_Motor_Reverse_Handler(Motor_t *motor);
void STR_Reverse_Handler();

#endif
