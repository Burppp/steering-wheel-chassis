//
// Created by Lumos on 2023/12/18.
//

#include "travelling_wheel.h"
TRA_Info_t tra = {
        .ctrl_mode = CHASSIS_RELAX,
        .Dir.head_or_tail = HEAD,
        .Vector.increase = 100,

        .TRA_Axis[TRA_LF].reverse_180 = false,
        .TRA_Axis[TRA_LB].reverse_180 = false,
        .TRA_Axis[TRA_RF].reverse_180 = false,
        .TRA_Axis[TRA_RB].reverse_180 = false,
        .TRA_Axis[TRA_LF].reverse_num = 1,
        .TRA_Axis[TRA_LB].reverse_num = 1,
        .TRA_Axis[TRA_RF].reverse_num = 1,
        .TRA_Axis[TRA_RB].reverse_num = 1,
};

static Mode_t prev_ctrl_mode = CHASSIS_INIT;

void TRA_Set_Mode()
{
//    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = tra.ctrl_mode;
//        tra.ctrl_mode = CHASSIS_RELAX;
//    }
//    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = tra.ctrl_mode;
//        tra.ctrl_mode = CHASSIS_YAW;
//    }
//    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = tra.ctrl_mode;
//        tra.ctrl_mode = CHASSIS_GYRO;
//    }
    prev_ctrl_mode = tra.ctrl_mode;
    tra.ctrl_mode = str.ctrl_mode;
}

void TRA_Motor_PID_Init(TRA_Motor_t *motor)
{
    motor->Speed_Loop.max_output = STEERING_WHEEL_CHASSIS_OUT;
    motor->Speed_Loop.integral_limit = STEERING_WHEEL_CHASSIS_IOUT;
    motor->Speed_Loop.p = STEERING_WHEEL_CHASSIS_KP;
    motor->Speed_Loop.i = STEERING_WHEEL_CHASSIS_KI;
    motor->Speed_Loop.d = STEERING_WHEEL_CHASSIS_KD;
}

void TRA_PID_Init()
{
    TRA_Motor_PID_Init(&tra.TRA_Axis[TRA_LF]);
    TRA_Motor_PID_Init(&tra.TRA_Axis[TRA_LB]);
    TRA_Motor_PID_Init(&tra.TRA_Axis[TRA_RF]);
    TRA_Motor_PID_Init(&tra.TRA_Axis[TRA_RB]);
}

fp32 TRA_LF_Steer_Angle_Ramp(fp32 Speed_Target)
{
    fp32 steer_delta_angle, slow_k = 1.0f;
//    steer_delta_angle = str.STR_Axis[STR_LF].motor_data.PID_Angle_target - str.STR_Axis[STR_LF].motor_data.PID_Angle;
//    if(abs(steer_delta_angle) > ROUND_ECD * 3 / 360)
//        slow_k = 1 / (abs(steer_delta_angle * 360 / ROUND_ECD));
//    if(slow_k > 1.0f)
//        slow_k = 1.0f;
    return Speed_Target * slow_k;
}

fp32 TRA_LB_Steer_Angle_Ramp(fp32 Speed_Target)
{
    fp32 steer_delta_angle, slow_k = 1.0f;
//    steer_delta_angle = str.STR_Axis[STR_LB].motor_data.PID_Angle_target - str.STR_Axis[STR_LB].motor_data.PID_Angle;
//    if(abs(steer_delta_angle) > ROUND_ECD * 3 / 360)
//        slow_k = 1 / (abs(steer_delta_angle));
//    if(slow_k > 1.0f)
//        slow_k = 1.0f;
    return Speed_Target * slow_k;
}

fp32 TRA_RB_Steer_Angle_Ramp(fp32 Speed_Target)
{
    fp32 steer_delta_angle, slow_k = 1.0f;
//    steer_delta_angle = str.STR_Axis[STR_RB].motor_data.PID_Angle_target - str.STR_Axis[STR_RB].motor_data.PID_Angle;
//    if(abs(steer_delta_angle) > ROUND_ECD * 3 / 360)
//        slow_k = 1 / (abs(steer_delta_angle));
//    if(slow_k > 1.0f)
//        slow_k = 1.0f;
    return Speed_Target * slow_k;
}

fp32 TRA_RF_Steer_Angle_Ramp(fp32 Speed_Target)
{
    fp32 steer_delta_angle, slow_k = 1.0f;
//    steer_delta_angle = str.STR_Axis[STR_RF].motor_data.PID_Angle_target - str.STR_Axis[STR_RF].motor_data.PID_Angle;
//    if(abs(steer_delta_angle) > ROUND_ECD * 3 / 360)
//        slow_k = 1 / (abs(steer_delta_angle));
//    if(slow_k > 1.0f)
//        slow_k = 1.0f;
    return Speed_Target * slow_k;
}

fp32 TRA_Get_Fusion_Speed_LF()
{
    fp32 result = str.STR_Axis[STR_LF].Move.XYZ_Speed;
    if(tra.TRA_Axis[TRA_LF].reverse_180)
        result *= -1;   // 当判定为反舵时，反转速度输入，注意不能反转PID相关
    if(result > 0) {
        if (result <= SPEED_MINIMUM && !str.STR_Axis[STR_LF].Move.JUST_SPIN) {
            result = 0;
        }
    } else if(result < 0) {
        if (result >= -SPEED_MINIMUM && !str.STR_Axis[STR_LF].Move.JUST_SPIN) {
            result = 0;
        }
    }
    return result;
}

fp32 TRA_Get_Fusion_Speed_LB()
{
    fp32 result = str.STR_Axis[STR_LB].Move.XYZ_Speed;
    if(tra.TRA_Axis[TRA_LB].reverse_180)
        result *= -1;
    if(result > 0) {
        if (result <= SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    } else if(result < 0) {
        if (result >= -SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    }
    return result;
}

fp32 TRA_Get_Fusion_Speed_RF()
{
    fp32 result = str.STR_Axis[STR_RF].Move.XYZ_Speed;
    if(tra.TRA_Axis[TRA_RF].reverse_180)
        result *= -1;
    if(result > 0) {
        if (result <= SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    } else if(result < 0) {
        if (result >= -SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    }
    return result;
}

fp32 TRA_Get_Fusion_Speed_RB()
{
    fp32 result = str.STR_Axis[STR_RB].Move.XYZ_Speed;
    if(tra.TRA_Axis[TRA_RB].reverse_180)
        result *= -1;
    if(result > 0) {
        if (result <= SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    } else if(result < 0) {
        if (result >= -SPEED_MINIMUM && !str.STR_Axis[STR_LB].Move.JUST_SPIN) {
            result = 0;
        }
    }
    return result;
}

fp32 TRA_Get_XY_Fusion_Speed_RF()
{
    int32_t errAngle = (str.STR_Axis[STR_RF].motor_data.PID_Angle_target - str.STR_Axis[STR_RF].Move_Mid_Angle) / HALF_ROUND_ECD * 2;
    fp32 vx = str.Vector.Y_speed, vy = str.Vector.X_speed;
    fp32 dir = 0;
    if((errAngle == 0 && str.STR_Axis[STR_RF].motor_data.PID_Angle_target - str.STR_Axis[STR_RF].Move_Mid_Angle > 0) || errAngle == -3)
        dir = 1;
    else if(errAngle == 1 || errAngle == -2)
        dir = 4;
    else if(errAngle == 2 || errAngle == -1)
        dir = 3;
    else if(errAngle == 3 || (errAngle == 0 && str.STR_Axis[STR_RF].motor_data.PID_Angle_target - str.STR_Axis[STR_RF].Move_Mid_Angle < 0))
        dir = 2;

    if((vx > 0 && vy > 0 && dir != 1) ||
       (vx > 0 && vy < 0 && dir != 4) ||
       (vx < 0 && vy > 0 && dir != 2) ||
       (vx < 0 && vy < 0 && dir != 3))
        tra.TRA_Axis[TRA_RF].reverse_num *= -1;
}

void TRA_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase)
{
    if(abs(*receive) - abs(source) < 0)
    {
        if(abs(*receive) > 10)
        {
            increase *= 5;
        }
    }
    if(abs(*receive) - abs(source) > 0)
    {
        increase *= 15;
    }
    if(abs(*receive - source) < increase)
    {
        *receive = source;
    }
    else
    {
        if((*receive) > source)
        {
            (*receive) -= increase;
        }
        if((*receive) < source)
        {
            (*receive) += increase;
        }
    }
}

void TRA_PID_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase)
{
    if(abs(*receive) - abs(source) > 0)
    {
        increase *= 6;
    }
    if(abs(*receive - source) < increase)
    {
        (*receive) = source;
    }
    else
    {
        if((*receive) > source)
        {
            (*receive) -= increase;
        }
        if((*receive) < source)
        {
            (*receive) += increase;
        }
    }
}

void TRA_Remote_Ctrl()
{
    // 反舵判断
    TRA_Reverse_Handler();

    tra.Vector.X_Speed_k = 13.2;
    tra.Vector.Y_Speed_k = 13.2;
    tra.Vector.Z_Speed_k = 10;

    fp32 Speed_Target[TRA_MOTOR_CNT] = {0};
    Speed_Target[TRA_LF] = TRA_LF_Steer_Angle_Ramp(tra.Vector.X_Speed_k * TRA_Get_Fusion_Speed_LF() * tra.Dir.head_or_tail * tra.TRA_Axis[TRA_LF].reverse_num);
    Speed_Target[TRA_LB] = TRA_LB_Steer_Angle_Ramp(tra.Vector.X_Speed_k * TRA_Get_Fusion_Speed_LB() * tra.Dir.head_or_tail * tra.TRA_Axis[TRA_LB].reverse_num);
    Speed_Target[TRA_RF] = TRA_RF_Steer_Angle_Ramp(-tra.Vector.X_Speed_k * TRA_Get_Fusion_Speed_RF() * tra.Dir.head_or_tail * tra.TRA_Axis[TRA_RF].reverse_num);
    Speed_Target[TRA_RB] = TRA_RB_Steer_Angle_Ramp(-tra.Vector.X_Speed_k * TRA_Get_Fusion_Speed_RB() * tra.Dir.head_or_tail * tra.TRA_Axis[TRA_RB].reverse_num);

    TRA_Speed_Ramp(&tra.Vector.Motor_Target_Speed[TRA_LF], Speed_Target[TRA_LF], 3);
    TRA_Speed_Ramp(&tra.Vector.Motor_Target_Speed[TRA_LB], Speed_Target[TRA_LB], 3);
    TRA_Speed_Ramp(&tra.Vector.Motor_Target_Speed[TRA_RF], Speed_Target[TRA_RF], 3);
    TRA_Speed_Ramp(&tra.Vector.Motor_Target_Speed[TRA_RB], Speed_Target[TRA_RB], 3);

    TRA_PID_Speed_Ramp(&tra.TRA_Axis[TRA_LF].motor_data.PID_Speed_target, tra.Vector.Motor_Target_Speed[TRA_LF], tra.Vector.increase);
    TRA_PID_Speed_Ramp(&tra.TRA_Axis[TRA_LB].motor_data.PID_Speed_target, tra.Vector.Motor_Target_Speed[TRA_LB], tra.Vector.increase);
    TRA_PID_Speed_Ramp(&tra.TRA_Axis[TRA_RF].motor_data.PID_Speed_target, tra.Vector.Motor_Target_Speed[TRA_RF], tra.Vector.increase);
    TRA_PID_Speed_Ramp(&tra.TRA_Axis[TRA_RB].motor_data.PID_Speed_target, tra.Vector.Motor_Target_Speed[TRA_RB], tra.Vector.increase);

    // 解除反舵标志
    tra.TRA_Axis[TRA_LF].reverse_180 = false;
    tra.TRA_Axis[TRA_LB].reverse_180 = false;
    tra.TRA_Axis[TRA_RF].reverse_180 = false;
    tra.TRA_Axis[TRA_RB].reverse_180 = false;
}

fp32 TRA_RPM_To_Speed(int16_t speed_rpm)
{
    return (fp32) (speed_rpm * 2 * PI * TRAVELLING_WHEEL_RADIUS / 60);
}

void TRA_Motor_Info_Update(TRA_Motor_t *motor)
{
    //motor->motor_data.PID_Speed = TRA_RPM_To_Speed(motor->motor_data.Can_GetData.speed_rpm);
    motor->motor_data.PID_Speed = motor->motor_data.Can_GetData.speed_rpm;
    motor->motor_data.PID_Angle = motor->motor_data.Can_GetData.ecd;
}

fp32 TRA_Get_Output(TRA_Motor_t *motor)
{
    fp32 result = 0;

    TRA_Motor_Info_Update(motor);

    result = pid_calc(&motor->Speed_Loop, motor->motor_data.PID_Speed, motor->motor_data.PID_Speed_target);
//    else
//    {
//        fp32 speed_rpm_target = (int16_t) (motor->motor_data.PID_Speed_target / TRAVELLING_WHEEL_RADIUS * 60 / (2 * PI) / 14);
//        result = pid_calc(&motor->Speed_Loop, motor->motor_data.Can_GetData.speed_rpm, speed_rpm_target);
//    }
    return result;
}

void TRA_Can_Send(int16_t *give_current)
{
    can_send_dji_motor(
            CAN_1,
            CAN_DJI_MOTOR_0x200_ID,
            0,
            give_current[1],
            give_current[2],
            0);
    can_send_dji_motor(
            CAN_2,
            CAN_DJI_MOTOR_0x200_ID,
            give_current[0],
            0,
            0,
            give_current[3]);
}

void TRA_Output()
{
    static int16_t give_current[TRA_MOTOR_CNT] = {0};

    give_current[TRA_LF] = (int16_t)TRA_Get_Output(&tra.TRA_Axis[TRA_LF]);
    give_current[TRA_LB] = (int16_t)TRA_Get_Output(&tra.TRA_Axis[TRA_LB]);
    give_current[TRA_RF] = (int16_t)TRA_Get_Output(&tra.TRA_Axis[TRA_RF]);
    give_current[TRA_RB] = (int16_t)TRA_Get_Output(&tra.TRA_Axis[TRA_RB]);

    // CAN发送
    TRA_Can_Send(give_current);
}

void TRA_Relax_Handle()
{


    TRA_Motor_Info_Update(&tra.TRA_Axis[TRA_LF]);
    TRA_Motor_Info_Update(&tra.TRA_Axis[TRA_LB]);
    TRA_Motor_Info_Update(&tra.TRA_Axis[TRA_RF]);
    TRA_Motor_Info_Update(&tra.TRA_Axis[TRA_RB]);
    can_send_dji_motor(
            CAN_1,
            CAN_DJI_MOTOR_0x200_ID,
            0,
            0,
            0,
            0);
    can_send_dji_motor(
            CAN_2,
            CAN_DJI_MOTOR_0x200_ID,
            0,
            0,
            0,
            0);
}

void TRA_Motor_Reverse_Handler(TRA_Motor_t *motor) {
    motor->reverse_180 = 1;
}

void TRA_Reverse_Handler() {
    if(str.STR_Axis[STR_LF].tra_reverse)
        TRA_Motor_Reverse_Handler(&tra.TRA_Axis[TRA_LF]);
    if(str.STR_Axis[STR_LB].tra_reverse)
        TRA_Motor_Reverse_Handler(&tra.TRA_Axis[TRA_LB]);
    if(str.STR_Axis[STR_RF].tra_reverse)
        TRA_Motor_Reverse_Handler(&tra.TRA_Axis[TRA_RF]);
    if(str.STR_Axis[STR_RB].tra_reverse)
        TRA_Motor_Reverse_Handler(&tra.TRA_Axis[TRA_RB]);
}

_Noreturn void TRA_ctrl(void const * pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    while(1)
    {
        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
        TRA_Set_Mode();
        if(prev_ctrl_mode != tra.ctrl_mode)
        {
            prev_ctrl_mode = tra.ctrl_mode;
            TRA_PID_Init();
        }
        chassis_device_offline_handle();
        switch (tra.ctrl_mode)
        {
            case CHASSIS_SPEED:
                TRA_Remote_Ctrl();
                TRA_Output();
                break;
            case CHASSIS_GYRO:
                TRA_Remote_Ctrl();
                TRA_Output();
                break;
            case CHASSIS_AUTO:
                TRA_Remote_Ctrl();
                TRA_Output();
                break;
            case CHASSIS_RELAX:
                TRA_Relax_Handle();
                break;
            default:
                break;
        }
        xTaskResumeAll();

        vTaskDelay(1);
    }
}