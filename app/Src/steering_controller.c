#include "steering_controller.h"

extern Motor_t motor_list[8];
extern bool is_init;
extern RC_ctrl_t rc_ctrl;
Motor_t target_motor;
int32_t target_position = 0;

ChassisSteering_t str = {
    .mode = CHASSIS_INIT,
    .vector = {
        .X_speed = 0,
        .Y_speed = 0,
        .Z_speed = 0,
        .X_speed_k = 0,
        .Y_speed_k = 0,
        .Z_speed_k = 0,
        .target_speed = {0, 0},
        .increase = 0,
        .Z_LR = LEFT,
    }
};

void steeringCtrl_task(void const * pvParameters)
{
    chassis_init();

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    while(1)
    {
        vTaskSuspendAll();

        // if(is_init)
        // {
        //     motor_list[1].position_loop.ref += rc_ctrl.rc.ch[0] / 10;

        //     motor_list[1].speed_loop.feedback = motor_list[1].speed_feedback;
        //     motor_list[1].position_loop.feedback = motor_list[1].position_feedback;
    
        //     pid_calc(&motor_list[1].position_loop);
    
        //     motor_list[1].speed_loop.ref = motor_list[1].position_loop.output;
        //     pid_calc(&motor_list[1].speed_loop);
    
        //     motor_setTorque(motor_list[1].speed_loop.output, LF_DRIVE_ID);
        // }

        target_position += rc_ctrl.rc.ch[0] / 10;

        motor_setPos(target_position, LF_DRIVE_ID);
		
        xTaskResumeAll();
        vTaskDelay(1);
    }
}

void chassis_init(void)
{

    motor_encoderDataReset(0x10, LF_DRIVE_ID);
    osDelay(100);

    motor_init(&motor_list[1], LF_DRIVE_ID);
    osDelay(100);

    motor_mode(POSITION_CONTROL, LF_DRIVE_ID);
    osDelay(100);

    motor_enable(LF_DRIVE_ID);
    osDelay(100);

    motor_setProfileSpeed(150, LF_DRIVE_ID);

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
