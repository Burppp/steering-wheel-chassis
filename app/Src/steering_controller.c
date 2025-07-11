#include "steering_controller.h"

extern Motor_t motor_list[8];

ChassisSteering_t str = {
    //@TODO: init
};

void steeringCtrl_task(void const * pvParameters)
{
    chassis_init();
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    while(1)
    {
        vTaskSuspendAll();

        xTaskResumeAll();
        vTaskDelay(1);
    }
}

void chassis_init(void)
{
    motor_init(&motor_list[0], LF_STEER_ID);
    motor_init(&motor_list[1], RF_STEER_ID);
    motor_init(&motor_list[2], LB_STEER_ID);
    motor_init(&motor_list[3], RB_STEER_ID);
    motor_init(&motor_list[4], LF_DRIVE_ID);
    motor_init(&motor_list[5], RF_DRIVE_ID);
    motor_init(&motor_list[6], LB_DRIVE_ID);
    motor_init(&motor_list[7], RB_DRIVE_ID);

    motor_mode(TORQUE_CONTROL, LF_STEER_ID);
    motor_mode(TORQUE_CONTROL, RF_STEER_ID);
    motor_mode(TORQUE_CONTROL, LB_STEER_ID);
    motor_mode(TORQUE_CONTROL, RB_STEER_ID);
    motor_mode(TORQUE_CONTROL, LF_DRIVE_ID);
    motor_mode(TORQUE_CONTROL, RF_DRIVE_ID);
    motor_mode(TORQUE_CONTROL, LB_DRIVE_ID);
    motor_mode(TORQUE_CONTROL, RB_DRIVE_ID);

    //@TODO: offset init
}

void STR_Set_Mode()
{

}

void STR_Motor_First_Angle(Motor_t *motor)
{

}

bool STR_Just_Spin()
{

}

void STR_Speed_Ramp(float *receive, float source, float increase, float limit)
{

}

float STR_Encoder_Limit(int16_t Encoder)
{

}

void STR_Motor_F_dir(Motor_t* motor)
{

}

void STR_F_dir_Update()
{

}

void STR_Z_LR(Motor_t* motor, float Z_speed)
{

}

void STR_Motor_Z_dir(Motor_t* motor)
{

}

void STR_Z_dir_Update()
{

}

float STR_Get_XY_Fusion_Speed(float X_Speed, float Y_Speed)
{

}

float STR_Get_XY_Dir()
{

}

float STR_Get_XY_Z_Dir_Err(float xy_dir, float z_dir)
{

}

void STR_Get_XYZ_Speed(float XY_Speed, float Z_Speed, Motor_t *motor)
{

}

float STR_Get_Z_Dir(Motor_t* motor)
{

}

void STR_Motor_Dir(Motor_t* motor)
{

}

void STR_Dir_Update()
{

}

void STR_Remote_Ctrl()
{

}

void STR_UART_Ctrl()
{

}

float STR_Gyro_Ecd_Proc(int16_t angle)
{

}

void STR_Gyro_F_Angle()
{

}

void STR_Gyro_Z_Angle()
{

}

void STR_Gyro_Fusion()
{

}

void STR_GYRO_Ctrl()
{

}

void STR_Motor_Info_Update(Motor_t* motor)
{

}

float STR_Get_Set_Angle(float set_angle, float get_angle)
{

}

float STR_Get_Output(Motor_t* motor)
{

}

void STR_Can_Send(int16_t* give_current)
{

}

void STR_Output()
{

}

void STR_Relax_Handle()
{

}

void chassis_device_offline_handle()
{
    
}

void Chassis_Cmd_Update(uint8_t* uart_buffer)
{

}

void STR_Motor_Reverse_Handler(Motor_t* motor)
{

}

void STR_Reverse_Handler()
{

}
