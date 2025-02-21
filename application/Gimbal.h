//
// Created by xhuanc on 2021/10/13.
//

#ifndef _GIMBAL_H_
#define _GIMBAL_H_
/*      Include     */


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
/*      define     */

#define GIMBAL_TASK_INIT_TIME 3000
#define YAW_CHANNEL 2
#define GIMBAL_RC_TO_YAW 0.0005
#define GIMBAL_OFFSET_ANGLE 30

//#define GIMBAL_YAW_ANGLE_PID_KP     0.06f
//#define GIMBAL_YAW_ANGLE_PID_KI     0.00001f
//#define GIMBAL_YAW_ANGLE_PID_KD     12.0f
//#define GIMBAL_YAW_ANGLE_MAX_OUT    100.0f
//#define GIMBAL_YAW_ANGLE_MAX_IOUT   18.0f
//
//#define GIMBAL_YAW_SPEED_PID_KP    1.8f
//#define GIMBAL_YAW_SPEED_PID_KI    0.00002f
//#define GIMBAL_YAW_SPEED_PID_KD    0.0f
//#define GIMBAL_YAW_SPEED_MAX_OUT   15.0f
//#define GIMBAL_YAW_SPEED_MAX_IOUT  1.0f



#define GIMBAL_YAW_ANGLE_PID_KP     0.35f
#define GIMBAL_YAW_ANGLE_PID_KI     0.00f
#define GIMBAL_YAW_ANGLE_PID_KD     12.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT    15.0f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   1.0f

#define GIMBAL_YAW_SPEED_PID_KP    1.5f
#define GIMBAL_YAW_SPEED_PID_KI    0.0015f
#define GIMBAL_YAW_SPEED_PID_KD    7.0f
#define GIMBAL_YAW_SPEED_MAX_OUT   25.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT  10.0f

#define GIMBAL_YAW_TORQUE_PARAM    1/18

#define CHASSIS_SPIN_RATIO -0.02f

typedef enum
{
    YAW=0,
    PITCH=1,
}gimbal_motor_index_e;

typedef enum
{
    GIMBAL_INIT,
    GIMBAL_RELAX,
    GIMBAL_AUTO,
    GIMBAL_GYRO,
}gimbal_mode_e;

typedef struct {
    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

    pid_t DM_speed_loop;
    pid_t DM_angle_loop;
    fp32 DM_angle_ref;
    fp32 DM_angle_get;
    fp32 DM_speed_ref;

    fp32 yaw_offset;
    float kd;
//    DM_Motor yaw_motor;
//    xiaomi_motor_t xiaomi_yaw_motor;
//    encoder_t encoder_yaw;
}gimbal_t;
_Noreturn extern void gimbal_task(void const*pvParameters);
static void Gimbal_Set_Mode();
static void Gimbal_Init();
static void Gimbal_Relax_Handle();
static void Gimbal_Yaw_Handle();
static void Gimbal_Followed_Handle();
static void Gimbal_Output();
static void Gimbal_Angle_Update();
void Gimbal_Speed_Ref_Set(fp32 speed_ref);
fp32 YAW_Relative_Angle();
void gimbal_device_offline_handle();

#endif
