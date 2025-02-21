//
// Created by Jackson on 2023/11/7.
//

#ifndef REMOTE_CUBE_SENTRY_RIGHT_GIMBAL_H
#define REMOTE_CUBE_SENTRY_RIGHT_GIMBAL_H
/*  Include   */


#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "user_lib.h"
#include "protocol_shaob.h"
#include "filter.h"
#include "Detection.h"
#include "packet.h"
#include "Referee.h"
#include "bsp_can.h"
/*   define   */

#define GIMBAL_TASK_INIT_TIME 3000
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

#define RC_TO_YAW 0.0005f
#define RC_TO_PITCH 0.0003f
#define MAX_ABS_ANGLE (15)
#define MIN_ABS_ANGLE (-14)
#define MAX_RELA_ANGLE (15)
#define MIN_RELA_ANGLE (-14)
//云台转动速度系数
//小云台没有跟底盘直接连接，不知道需不需要云台转动速度系数，先放着
//#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
//#define GIMBAL_RC_MOVE_RATIO_YAW 2.0f
//#define GIMBAL_YAW_PATROL_SPEED 0.4f
//#define GIMBAL_PITCH_PATROL_SPEED 0.2f
//#define GIMBAL_PITCH_PATROL_ANGLE_UP 3.0f
//#define GIMBAL_PITCH_PATROL_ANGLE_DOWN (-10.0f)

#define RIGHT_GIMBAL_YAW_ANGLE_PID_KP     10.0f
#define RIGHT_GIMBAL_YAW_ANGLE_PID_KI     0.0f
#define RIGHT_GIMBAL_YAW_ANGLE_PID_KD     0.0f
#define RIGHT_GIMBAL_YAW_ANGLE_MAX_OUT    10000.f
#define RIGHT_GIMBAL_YAW_ANGLE_MAX_IOUT   60.f

#define RIGHT_GIMBAL_YAW_SPEED_PID_KP     10.f
#define RIGHT_GIMBAL_YAW_SPEED_PID_KI     0.0f
#define RIGHT_GIMBAL_YAW_SPEED_PID_KD     0.0f
#define RIGHT_GIMBAL_YAW_SPEED_MAX_OUT    30000.f
#define RIGHT_GIMBAL_YAW_SPEED_MAX_IOUT   2000.f

#define RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KP    10.0f
#define RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KI     0.01f
#define RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_PID_KD     20.0f
#define RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_OUT    360.f
#define RIGHT_GIMBAL_YAW_RELATIVE_ANGLE_MAX_IOUT   60.f

#define RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KP     30.f
#define RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KI     0.8f
#define RIGHT_GIMBAL_YAW_RELATIVE_SPEED_PID_KD     0.0f
#define RIGHT_GIMBAL_YAW_RELATIVE_SPEED_MAX_OUT    20000.f
#define RIGHT_GIMBAL_YAW_RELATIVE_SPEED_MAX_IOUT   1000.f


#define RIGHT_GIMBAL_PITCH_ANGLE_PID_KP   20.f
#define RIGHT_GIMBAL_PITCH_ANGLE_PID_KI   0.01f
#define RIGHT_GIMBAL_PITCH_ANGLE_PID_KD   180.f
#define RIGHT_GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
#define RIGHT_GIMBAL_PITCH_ANGLE_MAX_IOUT 80.f

#define RIGHT_GIMBAL_PITCH_SPEED_PID_KP   80.0f
#define RIGHT_GIMBAL_PITCH_SPEED_PID_KI   1.5f
#define RIGHT_GIMBAL_PITCH_SPEED_PID_KD   0.f
#define RIGHT_GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
#define RIGHT_GIMBAL_PITCH_SPEED_MAX_IOUT 2000.f

//#define GIMBAL_PITCH_ANGLE_PID_KP   12.f
//#define GIMBAL_PITCH_ANGLE_PID_KI   1.0f
//#define GIMBAL_PITCH_ANGLE_PID_KD   80.f
//#define GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
//#define GIMBAL_PITCH_ANGLE_MAX_IOUT 5.f
//
//#define GIMBAL_PITCH_SPEED_PID_KP   180.0f
//#define GIMBAL_PITCH_SPEED_PID_KI   0.5f
//#define GIMBAL_PITCH_SPEED_PID_KD   180.f
//#define GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
//#define GIMBAL_PITCH_SPEED_MAX_IOUT 2000.f
/*   结构体和枚举   */


typedef enum{
    RIGHT_GIMBAL_RELAX=0,
    RIGHT_GIMBAL_BACK,
    RIGHT_GIMBAL_ACTIVE,
    RIGHT_GIMBAL_PATROL,
    RIGHT_GIMBAL_AUTO
}right_gimbal_mode_e;

typedef struct {
    fp32 absolute_angle_set;
    fp32 absolute_angle_get;
}right_gimbal_angle;

typedef struct {
    right_gimbal_mode_e mode;
    right_gimbal_mode_e last_mode;

    right_gimbal_motor_t motor_gimbal[2];

    right_gimbal_angle  yaw;
    right_gimbal_angle  pitch;
//  因为用编码器所以不需要这些变量
//    fp32 absolute_gyro_yaw;
//    fp32 absolute_gyro_pitch;
    fp32 absolute_ecd_yaw;
    fp32 absolute_ecd_pitch;

    bool_t right_yaw_is_back;
    bool_t right_pitch_is_back;

}right_gimbal_t;

_Noreturn extern void right_gimbal_task(void const*pvParameters);
extern uint8_t outpost_state;
extern uint8_t critical_value;


#endif //REMOTE_CUBE_SENTRY_RIGHT_GIMBAL_H
