//
// Created by Lumos on 2023/12/18.
//

#include <math.h>
#include "steering_wheel.h"
#include "remote.h"
#include "Gimbal.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "protocol_shaob.h"
#include "usart.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
//#include "stdlib.h"

STR_Info_t str = {
        .ctrl_mode = CHASSIS_RELAX,

        .STR_Axis[STR_LF].Move_Mid_Angle = STR_LF_OFFSET,
        .STR_Axis[STR_RB].Move_Mid_Angle = STR_RB_OFFSET,
        .STR_Axis[STR_LB].Move_Mid_Angle = STR_LB_OFFSET,
        .STR_Axis[STR_RF].Move_Mid_Angle = STR_RF_OFFSET,

        .STR_Axis[STR_LF].tra_reverse = false,
        .STR_Axis[STR_RB].tra_reverse = false,
        .STR_Axis[STR_LB].tra_reverse = false,
        .STR_Axis[STR_RF].tra_reverse = false,
};

static Mode_t prev_ctrl_mode = CHASSIS_INIT;
extern gimbal_t gimbal;
extern RC_ctrl_t rc_ctrl;
//上位机下发数据
extern robot_ctrl_info_t robot_ctrl;
chassis_cmd chassis_cmd_data;
extern moving_Average_Filter yaw_ctrl_average_filter;
extern first_order_filter_type_t yaw_ctrl_first_order_filter;
extern first_order_filter_type_t yaw_info_first_order_filter;

void STR_Set_Mode()
{
//    //右拨杆下
//    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = str.ctrl_mode;
//        str.ctrl_mode = CHASSIS_RELAX;
//    }
//    //右拨杆中
//    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
//    {
//        prev_ctrl_mode = str.ctrl_mode;
//        str.ctrl_mode = CHASSIS_AUTO;
//    }
//    //右拨杆上  左拨杆下
//    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]) && switch_is_down(rc_ctrl.rc.s[RC_s_L]))
//    {
//        prev_ctrl_mode = str.ctrl_mode;
//        str.ctrl_mode = CHASSIS_SPEED;
//    }
//        //右拨杆上  左拨杆中
//    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]) && switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
//    {
//        prev_ctrl_mode = str.ctrl_mode;
//        str.ctrl_mode = CHASSIS_GYRO;
//    }
    // TODO：以下是原来未注释的代码
    str.ctrl_mode = chassis_cmd_data.mode;
    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        prev_ctrl_mode = str.ctrl_mode;
        str.ctrl_mode = CHASSIS_RELAX;
    }
    if(prev_ctrl_mode == CHASSIS_AUTO && str.ctrl_mode != CHASSIS_AUTO)
    {
        str.Vector.X_speed = 0;
        str.Vector.Y_speed = 0;
        str.Vector.XY_speed = 0;
        str.Vector.XYZ_speed = 0;
        str.Vector.Z_speed = 0;
    }
    if(prev_ctrl_mode != CHASSIS_AUTO && str.ctrl_mode == CHASSIS_AUTO)
    {
        chassis_cmd_data.vx = 0;
        chassis_cmd_data.vy = 0;
        chassis_cmd_data.vw = 0;
        chassis_cmd_data.gyro_vw = 0;
    }
}

void STR_Offset_Init()
{
    //head or tail?
    str.STR_Axis[STR_LF].Move_Mid_Angle = STR_LF_OFFSET;
    str.STR_Axis[STR_RB].Move_Mid_Angle = STR_RB_OFFSET;
    str.STR_Axis[STR_LB].Move_Mid_Angle = STR_LB_OFFSET;
    str.STR_Axis[STR_RF].Move_Mid_Angle = STR_RF_OFFSET;

    //转向轮陀螺朝向
    str.STR_Axis[STR_LF].Spin_Mid_Angle = STR_Encoder_Limit(STR_LF_OFFSET + ROUND_ECD / 8);
    str.STR_Axis[STR_RF].Spin_Mid_Angle = STR_Encoder_Limit(STR_RF_OFFSET + 3 * ROUND_ECD / 8);
    str.STR_Axis[STR_RB].Spin_Mid_Angle = STR_Encoder_Limit(STR_RB_OFFSET - 3 * ROUND_ECD / 8);
    str.STR_Axis[STR_LB].Spin_Mid_Angle = STR_Encoder_Limit(STR_LB_OFFSET - ROUND_ECD / 8);
}

void STR_Motor_PID_Init(STR_MOTOR_t *motor)
{
    motor->Speed_Loop.max_output = STEERING_WHEEL_STEER_SPEED_OUT;
    motor->Speed_Loop.integral_limit = STEERING_WHEEL_STEER_SPEED_IOUT;
    motor->Speed_Loop.p = STEERING_WHEEL_STEER_SPEED_KP;
    motor->Speed_Loop.i = STEERING_WHEEL_STEER_SPEED_KI;
    motor->Speed_Loop.d = STEERING_WHEEL_STEER_SPEED_KD;

    motor->Angle_Loop.max_output = STEERING_WHEEL_STEER_ANGLE_OUT;
    motor->Angle_Loop.integral_limit = STEERING_WHEEL_STEER_ANGLE_IOUT;
    motor->Angle_Loop.p = STEERING_WHEEL_STEER_ANGLE_KP;
    motor->Angle_Loop.i = STEERING_WHEEL_STEER_ANGLE_KI;
    motor->Angle_Loop.d = STEERING_WHEEL_STEER_ANGLE_KD;
}

void STR_PID_Init()
{
    STR_Motor_PID_Init(&str.STR_Axis[STR_LF]);
    STR_Motor_PID_Init(&str.STR_Axis[STR_LB]);
    STR_Motor_PID_Init(&str.STR_Axis[STR_RF]);
    STR_Motor_PID_Init(&str.STR_Axis[STR_RB]);
}

void STR_Motor_First_Angle(STR_MOTOR_t *motor)
{
    motor->Move.Dir = motor->Move_Mid_Angle;
    motor->Move.Dir_last = motor->Move.Dir;
}

//上电归中
void STR_First_Angle()
{
    STR_Motor_First_Angle(&str.STR_Axis[STR_LF]);
    STR_Motor_First_Angle(&str.STR_Axis[STR_LB]);
    STR_Motor_First_Angle(&str.STR_Axis[STR_RF]);
    STR_Motor_First_Angle(&str.STR_Axis[STR_RB]);
}

//旋转死区
bool STR_JUST_SPIN()
{
    bool res = 0;
    if(abs(str.STR_Axis[STR_LF].Move.Dir - str.STR_Axis[STR_LF].Move.Z_dir) < 5 &&
       abs(str.STR_Axis[STR_LB].Move.Dir - str.STR_Axis[STR_LB].Move.Z_dir) < 5 &&
       abs(str.STR_Axis[STR_RF].Move.Dir - str.STR_Axis[STR_RF].Move.Z_dir) < 5 &&
       abs(str.STR_Axis[STR_RB].Move.Dir - str.STR_Axis[STR_RB].Move.Z_dir) < 5)
    {
        res = 1;
    }

    str.STR_Axis[STR_LF].Move.JUST_SPIN = res;
    str.STR_Axis[STR_LB].Move.JUST_SPIN = res;
    str.STR_Axis[STR_RF].Move.JUST_SPIN = res;
    str.STR_Axis[STR_RB].Move.JUST_SPIN = res;

    return res;
}

//Z_Speed死区
fp32 STR_Z_Speed(fp32 Z_Speed)
{
    fp32 DeadArea = STR_SPIN_MINIMUM_CH2;
    if(STR_JUST_SPIN() == 0)
    {
        //若XY速度够大，Z速度不设死区
        if(str.Vector.XY_speed >= STR_Z_LR_XY_SPEED_MINIMUM_LIMIT)
        {
            DeadArea = 0;
        }
        if(abs(Z_Speed) < DeadArea)
        {
            Z_Speed = 0;
        }
    }
    return Z_Speed;
}

//单轴速度爬坡
void STR_Speed_Ramp(fp32 *receive, fp32 source, fp32 increase, fp32 limit)
{
    fp32 increase_add = 0;
    increase_add = abs(*receive - source) / limit;
    increase_add *= increase * 6;
    increase += increase_add;

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

//编码器限幅
fp32 STR_Encoder_Limit(int16_t Encoder)
{
    while(Encoder < 0 || Encoder > ROUND_ECD)
    {
        if(Encoder < 0)
        {
            Encoder += ROUND_ECD;
        }
        else if(Encoder > ROUND_ECD)
        {
            Encoder -= ROUND_ECD;
        }
    }
    return (fp32) Encoder;
}

void STR_Motor_F_dir(STR_MOTOR_t *motor)
{
    if(str.ctrl_mode == CHASSIS_AUTO)
        motor->Move.F_dir = STR_Encoder_Limit(motor->Move_Mid_Angle + yaw_motor.position + 45 + 1000 + 4900 - 170);
    else
        motor->Move.F_dir = STR_Encoder_Limit(motor->Move_Mid_Angle + YAW_Relative_Angle() - HALF_ROUND_ECD / 2 + 2420 + 1000 + 4900 - 170);
}

void STR_F_dir_Update()
{
    STR_Motor_F_dir(&str.STR_Axis[STR_LF]);
    STR_Motor_F_dir(&str.STR_Axis[STR_LB]);
    STR_Motor_F_dir(&str.STR_Axis[STR_RF]);
    STR_Motor_F_dir(&str.STR_Axis[STR_RB]);
}

//底盘陀螺顺时针或逆时针
void STR_Z_LR(STR_MOTOR_t *motor, fp32 Z_speed)
{
    fp32 DeadArea = STR_SPIN_MINIMUM_CH2;
    if(str.Vector.XY_speed >= STR_Z_LR_XY_SPEED_MINIMUM_LIMIT)
    {
        DeadArea = 0;
    }
    if(Z_speed < -DeadArea)
    {
        str.Vector.Z_LR = LEFT;
        motor->Move.Z_LR = LEFT;
    }
    else if(Z_speed > DeadArea)
    {
        str.Vector.Z_LR = RIGHT;
        motor->Move.Z_LR = RIGHT;
    }
}

void STR_Motor_Z_dir(STR_MOTOR_t *motor)
{
    STR_Z_LR(motor, str.Vector.Z_speed);
    if(motor->Move.Z_LR == RIGHT)
    {
        motor->Move.Z_dir = STR_Encoder_Limit(motor->Spin_Mid_Angle);
    }
    else if(motor->Move.Z_LR == LEFT)
    {
        motor->Move.Z_dir = STR_Encoder_Limit(motor->Spin_Mid_Angle + HALF_ROUND_ECD);
    }
}

void STR_Z_dir_Update()
{
    STR_Motor_Z_dir(&str.STR_Axis[STR_LF]);
    STR_Motor_Z_dir(&str.STR_Axis[STR_LB]);
    STR_Motor_Z_dir(&str.STR_Axis[STR_RF]);
    STR_Motor_Z_dir(&str.STR_Axis[STR_RB]);
}

//融合X_Speed和Y_Speed
fp32 STR_Get_XY_Fusion_Speed(fp32 X_Speed, fp32 Y_Speed)
{
    fp32 result = 0;
    result = my_sqrt(X_Speed * X_Speed + Y_Speed * Y_Speed);
    str.Vector.XY_speed = result;
    return str.Vector.XY_speed;
}

//得到XY平面上的绝对方向
fp32 STR_Get_XY_Dir()
{
    fp32 vx = str.Vector.X_speed;
    fp32 vy = str.Vector.Y_speed;
    fp32 result = 0;
    static fp32 prev_result = 0;

    fp32 xy_speed = STR_Get_XY_Fusion_Speed(vx, vy);
    if(xy_speed >= STR_XY_DIR_XY_SPEED_MINIMUM_LIMIT)
    {
        if(vy > 0)
        {
            if(vx > 0)
            {
                result = atan2(vy, vx) * MOTOR_RAD_TO_ANGLE;
            }
            if(vx < 0)
            {
                result = atan2(-vx, vy) * MOTOR_RAD_TO_ANGLE + 90;
            }
            if(vx == 0)
            {
                result = 90;
            }
        }
        if(vy == 0)
        {
            if(vx > 0)
            {
                result = 0;
            }
            if(vx < 0)
            {
                result = -180;
            }
        }
        if(vy < 0)
        {
            if(vx > 0)
            {
                result = atan2(vy, vx) * MOTOR_RAD_TO_ANGLE;
            }
            if(vx < 0)
            {
                result = -atan2(-vx, -vy) * MOTOR_RAD_TO_ANGLE -90;
            }
            if(vx == 0)
            {
                result = -90;
            }
        }
        prev_result = result;
    }
    else
    {
        result = prev_result;
    }
    return result * HALF_ROUND_ECD / 180;
}

//XY平面方向和Z轴方向差值
fp32 STR_Get_XY_Z_Dir_Err(fp32 xy_dir, fp32 z_dir)
{
    fp32 e = z_dir - xy_dir;
    if(e <= -HALF_ROUND_ECD)
    {
        e += ROUND_ECD;
    }
    if(e > HALF_ROUND_ECD)
    {
        e -= ROUND_ECD;
    }
    return e;
}

//融合XY平面速度和Z轴速度
void STR_Get_XYZ_Speed(fp32 XY_Speed, fp32 Z_Speed, STR_MOTOR_t *motor)
{
    fp32 result = 0;
    bool just_spin;
    just_spin = STR_JUST_SPIN();

    if(!just_spin)
    {
        result = sqrtf(XY_Speed * XY_Speed + Z_Speed * Z_Speed);
    }
    else
    {
        result = str.Vector.Z_speed * (fp32) motor->Move.Z_LR;
    }
    motor->Move.XYZ_Speed = result;
}

fp32 STR_Get_Z_Dir(STR_MOTOR_t *motor)
{
    fp32 result = 0;
    fp32 XY_Z_Err = STR_Get_XY_Z_Dir_Err(motor->Move.XY_dir, motor->Move.Z_dir);
    fp32 spin_front = fabsf(str.Vector.Z_speed) * cosf(XY_Z_Err * MOTOR_ECD_TO_RAD);
    fp32 spin_veri_front = fabsf(str.Vector.Z_speed) * sinf(XY_Z_Err * MOTOR_ECD_TO_RAD);
    fp32 opposite = spin_veri_front;
    fp32 side = spin_front + str.Vector.XY_speed;

    result = atan2(opposite, side) * MOTOR_RAD_TO_ECD;

    if(str.Vector.XY_speed != 0 || str.Vector.Z_speed != 0)
    {
        motor->Move.prev_Z_Angle = result;
    }
    if(str.Vector.XY_speed == 0 && str.Vector.Z_speed == 0)
    {
        result = motor->Move.prev_Z_Angle;
    }

    motor->Move.XY_dir = XY_Z_Err;

    STR_Get_XYZ_Speed(side, opposite, motor);

    return result;
}

void STR_Motor_Dir(STR_MOTOR_t *motor)
{
    // 记录在旋转舵向之前的位置
    motor->STR_Reverse_Ecd = motor->Move.Dir;
    motor->Move.XY_dir = STR_Encoder_Limit(motor->Move.F_dir + STR_Get_XY_Dir());
    motor->Move.Dir = STR_Encoder_Limit(motor->Move.XY_dir + STR_Get_Z_Dir(motor));
    motor->motor_data.PID_Angle_target = motor->Move.Dir;
    motor->Move.Dir_last = motor->Move.Dir;
}

void STR_Dir_Update()
{
    STR_Motor_Dir(&str.STR_Axis[STR_LF]);
    STR_Motor_Dir(&str.STR_Axis[STR_LB]);
    STR_Motor_Dir(&str.STR_Axis[STR_RF]);
    STR_Motor_Dir(&str.STR_Axis[STR_RB]);
}

void STR_Remote_Ctrl()
{
    str.STR_Axis[STR_LF].tra_reverse = 0;
    str.STR_Axis[STR_LB].tra_reverse = 0;
    str.STR_Axis[STR_RF].tra_reverse = 0;
    str.STR_Axis[STR_RB].tra_reverse = 0;

    //REMOTE
//    STR_Speed_Ramp(&str.Vector.X_speed, -rc_ctrl.rc.ch[0], 0.4, 660);//符合操作逻辑，但vx是反的
//    STR_Speed_Ramp(&str.Vector.Y_speed, rc_ctrl.rc.ch[1], 0.4, 660);
//    STR_Speed_Ramp(&str.Vector.Z_speed, STR_Z_Speed(rc_ctrl.rc.ch[3]), 0.7, 660);


    STR_Speed_Ramp(&str.Vector.X_speed, -chassis_cmd_data.vx * 5, 0.4, 660);//符合操作逻辑，但vx是反的
    STR_Speed_Ramp(&str.Vector.Y_speed, -chassis_cmd_data.vy * 5, 0.4, 660);
    str.Vector.Z_speed = chassis_cmd_data.gyro_vw;

    //F_dir
    STR_F_dir_Update();

    //Z_dir
    STR_Z_dir_Update();

    //Dir
    STR_Dir_Update();

    // 反舵
    STR_Reverse_Handler();
}

void STR_UART_Ctrl()
{
    //UART
    STR_Speed_Ramp(&str.Vector.X_speed, -chassis_cmd_data.vy * 300, 0.2, 660);//左-   右+
    STR_Speed_Ramp(&str.Vector.Y_speed, -chassis_cmd_data.vx * 300, 0.2, 660);//前+   后-
    str.Vector.Z_speed = chassis_cmd_data.gyro_vw;

    STR_Gyro_F_Angle();

    STR_Gyro_Z_Angle();

    STR_Gyro_Fusion();
}

fp32 STR_Gyro_Ecd_Proc(int16_t angle)
{
    while(angle > 8192 || angle < 0)
    {
        if(angle < 0)
            angle += ROUND_ECD;
        if(angle > 8192)
            angle -= ROUND_ECD;
    }
    return angle;
}

void STR_Gyro_F_Angle()
{
    str.STR_Axis[STR_LF].Move.F_dir = STR_Gyro_Ecd_Proc(str.STR_Axis[STR_LF].Move_Mid_Angle + YAW_Relative_Angle() - HALF_ROUND_ECD / 2 + 3420 + 4900 - 170);
    str.STR_Axis[STR_RF].Move.F_dir = STR_Gyro_Ecd_Proc(str.STR_Axis[STR_RF].Move_Mid_Angle + YAW_Relative_Angle() - HALF_ROUND_ECD / 2 + 3420 + 4900 - 170);
    str.STR_Axis[STR_LB].Move.F_dir = STR_Gyro_Ecd_Proc(str.STR_Axis[STR_LB].Move_Mid_Angle + YAW_Relative_Angle() - HALF_ROUND_ECD / 2 + 3420 + 4900 - 170);
    str.STR_Axis[STR_RB].Move.F_dir = STR_Gyro_Ecd_Proc(str.STR_Axis[STR_RB].Move_Mid_Angle + YAW_Relative_Angle() - HALF_ROUND_ECD / 2 + 3420 + 4900 - 170);
}

void STR_Gyro_Z_Angle()
{
    STR_Z_LR(&str.STR_Axis[STR_LF], str.Vector.Z_speed);
    STR_Z_LR(&str.STR_Axis[STR_LB], str.Vector.Z_speed);
    STR_Z_LR(&str.STR_Axis[STR_RF], str.Vector.Z_speed);
    STR_Z_LR(&str.STR_Axis[STR_RB], str.Vector.Z_speed);

    if(str.STR_Axis[STR_LF].Move.Z_LR == RIGHT)
        str.STR_Axis[STR_LF].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_LF].Spin_Mid_Angle);
    else if(str.STR_Axis[STR_LF].Move.Z_LR = LEFT)
        str.STR_Axis[STR_LF].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_LF].Spin_Mid_Angle + HALF_ROUND_ECD);

    if(str.STR_Axis[STR_LB].Move.Z_LR == RIGHT)
        str.STR_Axis[STR_LB].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_LB].Spin_Mid_Angle);
    else if(str.STR_Axis[STR_LB].Move.Z_LR = LEFT)
        str.STR_Axis[STR_LB].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_LB].Spin_Mid_Angle + HALF_ROUND_ECD);

    if(str.STR_Axis[STR_RF].Move.Z_LR == RIGHT)
        str.STR_Axis[STR_RF].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_RF].Spin_Mid_Angle);
    else if(str.STR_Axis[STR_RF].Move.Z_LR = LEFT)
        str.STR_Axis[STR_RF].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_RF].Spin_Mid_Angle + HALF_ROUND_ECD);

    if(str.STR_Axis[STR_RB].Move.Z_LR == RIGHT)
        str.STR_Axis[STR_RB].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_RB].Spin_Mid_Angle);
    else if(str.STR_Axis[STR_RB].Move.Z_LR = LEFT)
        str.STR_Axis[STR_RB].Move.Z_dir = STR_Encoder_Limit(str.STR_Axis[STR_RB].Spin_Mid_Angle + HALF_ROUND_ECD);

}

void STR_Gyro_Fusion()
{
    str.STR_Axis[STR_LF].Move.XY_dir = STR_Encoder_Limit(str.STR_Axis[STR_LF].Move.F_dir + STR_Get_XY_Dir());
    str.STR_Axis[STR_LF].Move.Dir = STR_Encoder_Limit(str.STR_Axis[STR_LF].Move.XY_dir + STR_Get_Z_Dir(&str.STR_Axis[STR_LF]));
    str.STR_Axis[STR_LF].motor_data.PID_Angle_target = str.STR_Axis[STR_LF].Move.Dir;

    str.STR_Axis[STR_RF].Move.XY_dir = STR_Encoder_Limit(str.STR_Axis[STR_RF].Move.F_dir + STR_Get_XY_Dir());
    str.STR_Axis[STR_RF].Move.Dir = STR_Encoder_Limit(str.STR_Axis[STR_RF].Move.XY_dir + STR_Get_Z_Dir(&str.STR_Axis[STR_RF]));
    str.STR_Axis[STR_RF].motor_data.PID_Angle_target = str.STR_Axis[STR_RF].Move.Dir;

    str.STR_Axis[STR_LB].Move.XY_dir = STR_Encoder_Limit(str.STR_Axis[STR_LB].Move.F_dir + STR_Get_XY_Dir());
    str.STR_Axis[STR_LB].Move.Dir = STR_Encoder_Limit(str.STR_Axis[STR_LB].Move.XY_dir + STR_Get_Z_Dir(&str.STR_Axis[STR_LB]));
    str.STR_Axis[STR_LB].motor_data.PID_Angle_target = str.STR_Axis[STR_LB].Move.Dir;

    str.STR_Axis[STR_RB].Move.XY_dir = STR_Encoder_Limit(str.STR_Axis[STR_RB].Move.F_dir + STR_Get_XY_Dir());
    str.STR_Axis[STR_RB].Move.Dir = STR_Encoder_Limit(str.STR_Axis[STR_RB].Move.XY_dir + STR_Get_Z_Dir(&str.STR_Axis[STR_RB]));
    str.STR_Axis[STR_RB].motor_data.PID_Angle_target = str.STR_Axis[STR_RB].Move.Dir;

}

fp32 increase = 0.2;

void STR_GYRO_Ctrl()
{
//    STR_Speed_Ramp(&str.Vector.X_speed, rc_ctrl.rc.ch[0], increase, 660);
//    STR_Speed_Ramp(&str.Vector.Y_speed, -rc_ctrl.rc.ch[1], increase, 660);
//    str.Vector.Z_speed = STR_GYRO_PARAM;

    for(int i = 0; i < 4; ++i) {
        str.STR_Axis[i].tra_reverse = false;
    }

    STR_Speed_Ramp(&str.Vector.X_speed, -chassis_cmd_data.vx * 5, 0.4, 660);//符合操作逻辑，但vx是反的
    STR_Speed_Ramp(&str.Vector.Y_speed, -chassis_cmd_data.vy * 5, 0.4, 660);
    str.Vector.Z_speed = chassis_cmd_data.gyro_vw;

    STR_Gyro_F_Angle();

    STR_Gyro_Z_Angle();

    STR_Gyro_Fusion();
}

void STR_Motor_Info_Update(STR_MOTOR_t *motor)
{
    motor->motor_data.PID_Speed = motor->motor_data.Can_GetData.speed_rpm;
    motor->motor_data.PID_Angle = motor->motor_data.Can_GetData.ecd;
}


fp32 STR_Get_Set_Angle(fp32 set_angle, fp32 get_angle)
{
    fp32 result = set_angle;
    if(set_angle - get_angle <= -HALF_ROUND_ECD - 40)
    {
        result += ROUND_ECD;
    }
    if(set_angle - get_angle >= HALF_ROUND_ECD - 40)
    {
        result -= ROUND_ECD;
    }
    return result;
}

fp32 STR_Get_Output(STR_MOTOR_t *motor)
{
    fp32 result = 0;

    STR_Motor_Info_Update(motor);
    fp32 get_angle = 0, set_angle = 0;
    get_angle = motor->motor_data.PID_Angle;
    set_angle = STR_Get_Set_Angle(motor->motor_data.PID_Angle_target, motor->motor_data.PID_Angle);
    motor->motor_data.PID_Speed_target = pid_calc(&(motor->Angle_Loop), get_angle, set_angle);
    result = pid_calc(&(motor->Speed_Loop), motor->motor_data.PID_Speed, motor->motor_data.PID_Speed_target);


    return result;
}

void STR_Can_Send(int16_t *give_current)
{
    can_send_dji_motor(
            CAN_1,
            CAN_DJI_MOTOR_0x1FF_ID,
            give_current[0],
            0,
            0,
            give_current[3]
    );
    can_send_dji_motor(
            CAN_2,
            CAN_DJI_MOTOR_0x1FF_ID,
            0,
            give_current[1],
            give_current[2],
            0);
}

void STR_Output()
{
    int16_t give_current[STR_MOTOR_CNT] = {0};

    give_current[STR_LF] = (int16_t)STR_Get_Output(&str.STR_Axis[STR_LF]);
    give_current[STR_LB] = (int16_t)STR_Get_Output(&str.STR_Axis[STR_LB]);
    give_current[STR_RF] = (int16_t)STR_Get_Output(&str.STR_Axis[STR_RF]);
    give_current[STR_RB] = (int16_t)STR_Get_Output(&str.STR_Axis[STR_RB]);

    STR_Can_Send(give_current);
}

void STR_Relax_Handle()
{
    STR_Motor_Info_Update(&str.STR_Axis[STR_LF]);
    STR_Motor_Info_Update(&str.STR_Axis[STR_LB]);
    STR_Motor_Info_Update(&str.STR_Axis[STR_RF]);
    STR_Motor_Info_Update(&str.STR_Axis[STR_RB]);

    str.Vector.X_speed = 0;
    str.Vector.Y_speed = 0;
    str.Vector.XY_speed = 0;
    str.Vector.XYZ_speed = 0;
    str.Vector.Z_speed = 0;

    can_send_dji_motor(
            CAN_1,
            CAN_DJI_MOTOR_0x1FF_ID,
            0,
            0,
            0,
            0
    );
    can_send_dji_motor(
            CAN_2,
            CAN_DJI_MOTOR_0x1FF_ID,
            0,
            0,
            0,
            0);
}
void chassis_device_offline_handle()
{
    if(str.ctrl_mode != CHASSIS_RELAX)
    {
        if(detect_list[DETECT_CHASSIS_3508_RF].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_3508_LF].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_3508_LB].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_3508_RB].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_6020_RF].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_6020_RB].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_6020_LF].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_CHASSIS_6020_LB].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(detect_list[DETECT_GYRO_UART].status == OFFLINE)
            str.ctrl_mode = CHASSIS_RELAX;
        if(chassis_cmd_data.chassis_online == 0)
            str.ctrl_mode = CHASSIS_RELAX;
//        if(detect_list[DETECT_REMOTE].status==OFFLINE)
//            str.ctrl_mode = CHASSIS_RELAX;//防止出现底盘疯转
    }
}

uint8_t length;
chassis_cmd_cp temp;

void Chassis_Cmd_Update(uint8_t *uart_buffer)
{
    uint8_t frame_head = 'h';
    uint8_t frame_tail = 'j';
    length = uart_buffer[2];
    if(uart_buffer[0] == frame_head && uart_buffer[length + 4] == frame_tail)
    {
        if(uart_buffer[length + 3] == get_CRC8_check_sum(uart_buffer + 3, length, 0xff))
        {
            if(uart_buffer[1] == CHASSIS_CTRL)
            {
                memcpy(&temp, uart_buffer + 3, sizeof(temp));
                chassis_cmd_data.mode = temp.mode;
                chassis_cmd_data.vx = (fp32)temp.vx / 100;
                chassis_cmd_data.vy = (fp32)temp.vy / 100;
                chassis_cmd_data.vw = (fp32)temp.vw / 100;
                chassis_cmd_data.gyro_vw = temp.gyro_vw;
                chassis_cmd_data.yaw_angle = -temp.yaw_angle;
                chassis_cmd_data.yaw_feedback = temp.yaw_feedback;

                first_order_filter_cali(&yaw_ctrl_first_order_filter, chassis_cmd_data.yaw_angle);
                first_order_filter_cali(&yaw_info_first_order_filter, chassis_cmd_data.yaw_feedback);
//                average_add(&yaw_ctrl_average_filter, yaw_ctrl_first_order_filter.out);
                chassis_cmd_data.chassis_online = temp.chassis_online;
            }
//            else if(uart_buffer[1] == REFEREE)
//            {
//
//            }
        }
    }
}

void STR_Motor_Reverse_Handler(STR_MOTOR_t *motor) {
    fp32 boundary = motor->STR_Reverse_Ecd;
    // 是否处理目标值数据
    int flag = 0;
    if(fabs(motor->motor_data.PID_Angle_target - boundary) > QUARTER_ROUND_ECD) {
        if(fabs(motor->motor_data.PID_Angle_target - boundary) > ROUND_ECD - 227) {
            // 当这次旋转编码值和上一次旋转编码值差值取在整盘值左右(这里约为±10°)时，判定为越过码盘边界，这时不对目标值做处理，避免其产生额外的旋转
            flag = 0;
        } else if(motor->motor_data.PID_Angle_target > boundary) {
            flag = 1;
        } else if(motor->motor_data.PID_Angle_target < boundary) {
            flag = 1;
        }
    }
    // 处理目标值数据
    if(flag) {
        motor->motor_data.PID_Angle_target = STR_Encoder_Limit(motor->motor_data.PID_Angle_target + HALF_ROUND_ECD);
        // 更新真实舵向
        motor->Move.Dir = motor->motor_data.PID_Angle_target;
        motor->Move.Dir_last = motor->Move.Dir;
        // 置反舵位1
        motor->tra_reverse = 1;
    }
}

void STR_Reverse_Handler() {
    STR_Motor_Reverse_Handler(&str.STR_Axis[STR_LF]);
    STR_Motor_Reverse_Handler(&str.STR_Axis[STR_LB]);
    STR_Motor_Reverse_Handler(&str.STR_Axis[STR_RF]);
    STR_Motor_Reverse_Handler(&str.STR_Axis[STR_RB]);
}

//串口中断函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);//读取UART6-SR 和UART6-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart1.hdmarx); //使能dma_rx

        Chassis_Cmd_Update(&usart1_buff[0]);

        detect_handle(DETECT_GYRO_UART);

//        memset(&usart1_buff[0],0,CHASSIS_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart1.hdmarx, CHASSIS_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart1.hdmarx); //使能DMARx

    }
//    HAL_UART_IRQHandler(&huart1);

}

void chassis_power_ctrl()
{

}

_Noreturn void STR_ctrl(void const * pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    usart1_init(&usart1_buff[0], CHASSIS_BUFFER_SIZE);

    while(1)
    {
        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
        STR_Set_Mode();
        STR_Offset_Init();
        if(prev_ctrl_mode != str.ctrl_mode)
        {
            prev_ctrl_mode = str.ctrl_mode;
            STR_PID_Init();
            STR_First_Angle();
        }
        chassis_device_offline_handle();
//        chassis_power_ctrl();
        switch(str.ctrl_mode)
        {
            case CHASSIS_AUTO:
                STR_UART_Ctrl();
                STR_Output();
                break;
            case CHASSIS_SPEED:
                STR_Remote_Ctrl();
                STR_Output();
                break;
            case CHASSIS_GYRO:
                STR_GYRO_Ctrl();
                STR_Output();
                break;
            case CHASSIS_RELAX:
                STR_Relax_Handle();
                break;
            default:
                break;
        }

        xTaskResumeAll();

        vTaskDelay(1);
    }
}
