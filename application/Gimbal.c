/*  Include */
#include "Gimbal.h"
#include "remote.h"
#include "bsp_can.h"
#include "can.h"

#include "steering_wheel.h"
/*      define      */

/*      变量      */
extern RC_ctrl_t rc_ctrl;
gimbal_t gimbal = {
        .mode = GIMBAL_INIT,
        .last_mode = GIMBAL_INIT,
        .yaw_offset = 0.0f,
        .kd=3.0f
};

gimbal_mode_e prev_ctrl_mode = GIMBAL_INIT;
moving_Average_Filter speed_average_filter;
moving_Average_Filter yaw_ctrl_average_filter;
first_order_filter_type_t yaw_ctrl_first_order_filter;
first_order_filter_type_t yaw_info_first_order_filter;
float aver_yaw;
float aver_speed;
extern chassis_cmd chassis_cmd_data;
extern STR_Info_t str;
fp32 INS_Angle_Yaw;
fp32 yaw_cmd;
first_order_filter_type_t DM_velocity_filter;
first_order_filter_type_t yaw_current_filter;
fp32 DM_velocity;
_Noreturn void gimbal_task(void const*pvParameters)
{
    //任务初始化时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    Gimbal_Init();
    //主任务体
    while(1)
    {
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
//        first_order_filter_cali(&DM_velocity_filter, yaw_motor.velocity);
//        DM_velocity = DM_velocity_filter.out;
//        average_add(&speed_average_filter,DM_velocity);
        aver_speed= average_get(&speed_average_filter,19);
//        aver_yaw = average_get(&yaw_ctrl_average_filter, 200);
        aver_yaw = yaw_ctrl_first_order_filter.out;
        INS_Angle_Yaw = -chassis_cmd_data.yaw_feedback;
        if(gimbal.yaw_offset == 0.0f)
            gimbal.yaw_offset = INS_Angle_Yaw;

        Gimbal_Set_Mode();
//        if(str.ctrl_mode == CHASSIS_AUTO && gimbal.yaw_offset != 0.0f)
//        {
//            yaw_cmd = gimbal.yaw_offset - chassis_cmd_data.yaw_angle;
//        }
        if(chassis_cmd_data.mode == CHASSIS_AUTO)
        {
            yaw_cmd = aver_yaw;
        }
        if(prev_ctrl_mode != gimbal.mode)
        {
            prev_ctrl_mode = gimbal.mode;
        }
        gimbal_device_offline_handle();
        switch (gimbal.mode)
        {
            case GIMBAL_RELAX:
                Gimbal_Relax_Handle();
                break;
            case GIMBAL_AUTO:
                if(chassis_cmd_data.mode != CHASSIS_RELAX)
                {
                    gimbal.DM_angle_ref += (fp32)chassis_cmd_data.vw * 0.0005;
                    if(chassis_cmd_data.mode == CHASSIS_AUTO)
                    {
                        gimbal.DM_angle_ref = yaw_cmd;
                    }
                    gimbal.DM_angle_get = INS_Angle_Yaw;
                }
                Gimbal_Output();
                break;
            case GIMBAL_GYRO:
                if(chassis_cmd_data.mode != CHASSIS_RELAX)
                {
//                    gimbal.DM_angle_ref += (fp32)rc_ctrl.rc.ch[2] * 0.0003;
                    gimbal.DM_angle_ref += (fp32)chassis_cmd_data.vw * 0.003;

                    gimbal.DM_angle_get = INS_Angle_Yaw;
                }
                Gimbal_Output();
                break;
        }
//        xTaskResumeAll();
        vTaskDelay(1);
    }
}

void gimbal_device_offline_handle()
{
    if(gimbal.mode != GIMBAL_RELAX)
    {
//        if(detect_list[DETECT_CHASSIS_3508_RF].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_3508_LF].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_3508_LB].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_3508_RB].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_6020_RF].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_6020_RB].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_6020_LF].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
//        if(detect_list[DETECT_CHASSIS_6020_LB].status == OFFLINE)
//            gimbal.mode = GIMBAL_RELAX;
        if(detect_list[DETECT_GIMBAL_DM_YAW].status == OFFLINE)
            gimbal.mode = GIMBAL_RELAX;
    }
}
fp32 yaw_speed;
fp32 yaw_current;
fp32 test = 0.5;
static void Gimbal_Output()
{
    if(gimbal.mode == GIMBAL_AUTO)
    {
//        yaw_speed = pid_loop_calc(&(gimbal.DM_angle_loop), gimbal.DM_angle_get, gimbal.DM_angle_ref, 180, -180);
//        average_add(&speed_average_filter, yaw_speed);
//        fp32 speed_ref = average_get(&speed_average_filter, 200);
//        yaw_current = pid_calc(&(gimbal.DM_speed_loop), aver_speed, -speed_ref);
////        yaw_current = pid_calc(&gimbal.DM_speed_loop, aver_speed, -yaw_speed);
//        MIT_CtrlMotor(&hcan1, 0x01, 0, yaw_current, 0, 1, 0);
        yaw_speed = pid_loop_calc(&(gimbal.DM_angle_loop), gimbal.DM_angle_get, gimbal.DM_angle_ref, 180, -180);
        MIT_CtrlMotor(&hcan1, 0x01, 0, -yaw_speed, 0, gimbal.kd, 0);
    }
    else if(gimbal.mode == GIMBAL_GYRO)
    {
        yaw_speed = pid_loop_calc(&(gimbal.DM_angle_loop), gimbal.DM_angle_get, gimbal.DM_angle_ref, 180, -180);
        yaw_current = pid_calc(&(gimbal.DM_speed_loop), yaw_motor.velocity, -yaw_speed);
        MIT_CtrlMotor(&hcan1, 0x01, 0, -yaw_speed, 0, gimbal.kd, 0);
//        MIT_CtrlMotor(&hcan1, 0x01, gimbal.DM_angle_ref,  (fp32)chassis_cmd_data.vw * 0.003f, gimbal.kp, gimbal.kd, gimbal.tff);
    }
}

static void Gimbal_Relax_Handle()
{
    MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, 0);
}

static void Gimbal_Init()
{
    pid_init(&(gimbal.DM_angle_loop),
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&(gimbal.DM_speed_loop),
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    first_order_filter_init(&DM_velocity_filter, 0.005, 0.8);
    first_order_filter_init(&yaw_current_filter,0.05,0.5);
    first_order_filter_init(&yaw_ctrl_first_order_filter, 0.05, 0.5);
    first_order_filter_init(&yaw_info_first_order_filter, 0.05, 0.5);
    average_init(&speed_average_filter,201);
    average_init(&yaw_ctrl_average_filter, 201);
    INS_Angle_Yaw = yaw_info_first_order_filter.out;
    gimbal.yaw_offset = 0.0f;
    gimbal.DM_angle_ref = 52.0f;
    INS_Angle_Yaw = yaw_info_first_order_filter.out;
    gimbal.yaw_offset = INS_Angle_Yaw;
}

static void Gimbal_Set_Mode()
{
//    //右拨杆下
//    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = gimbal.mode;
//        gimbal.mode = GIMBAL_RELAX;
//    }
//    //右拨杆中
//    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = gimbal.mode;
//        gimbal.mode = GIMBAL_AUTO;
//        if(prev_ctrl_mode != gimbal.mode)
//            gimbal.DM_angle_ref = INS_Angle_Yaw;
//    }
//    //右拨杆上
//    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = gimbal.mode;
//        gimbal.mode = GIMBAL_GYRO;
//        if(prev_ctrl_mode != gimbal.mode)
//            gimbal.DM_angle_ref = INS_Angle_Yaw;
//    }
//    switch (str.ctrl_mode)
//    {
//        case CHASSIS_AUTO:
//        {
//            prev_ctrl_mode = gimbal.mode;
//            gimbal.mode = GIMBAL_AUTO;
//            break;
//        }
//        case CHASSIS_GYRO:
//        case CHASSIS_SPEED:
//        {
//            prev_ctrl_mode = gimbal.mode;
//            gimbal.mode = GIMBAL_GYRO;
//            break;
//        }
//        case CHASSIS_RELAX:
//        {
//            if(chassis_cmd_data.chassis_online == 1)
//            {
//                prev_ctrl_mode = gimbal.mode;
//                gimbal.mode = GIMBAL_RELAX;
//            }
//            else
//            {
//                switch (chassis_cmd_data.mode)
//                {
//                    case CHASSIS_SPEED:
//                    case CHASSIS_GYRO:
//                    {
//                        prev_ctrl_mode = gimbal.mode;
//                        gimbal.mode = GIMBAL_GYRO;
//                        break;
//                    }
//                    case CHASSIS_AUTO:
//                    {
//                        prev_ctrl_mode = gimbal.mode;
//                        gimbal.mode = GIMBAL_AUTO;
//                        break;
//                    }
//                }
//            }
//            break;
//        }
//    }
    switch (chassis_cmd_data.mode)
    {
        case CHASSIS_INIT:
        case CHASSIS_RELAX:
            prev_ctrl_mode = gimbal.mode;
            gimbal.mode = GIMBAL_RELAX;
            break;
        case CHASSIS_SPEED:
        case CHASSIS_GYRO:
            prev_ctrl_mode = gimbal.mode;
            gimbal.mode = GIMBAL_GYRO;
            break;
        case CHASSIS_AUTO:
            prev_ctrl_mode = gimbal.mode;
            gimbal.mode =GIMBAL_AUTO;
            break;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        prev_ctrl_mode = gimbal.mode;
        gimbal.mode = GIMBAL_RELAX;
    }
    if(prev_ctrl_mode != gimbal.mode)
        gimbal.DM_angle_ref = INS_Angle_Yaw;
}

fp32 YAW_Relative_Angle()
{
    return yaw_motor.position;
}