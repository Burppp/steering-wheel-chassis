//
// Created by xhuanc on 2021/11/2.
//

#ifndef _LAUNCHER_H_
#define _LAUNCHER_H_

#include <stdbool.h>
#include "Gimbal.h"
#include "cmsis_os.h"
#include "remote.h"
#include "bsp_laser.h"
#include "filter.h"
#include "Referee.h"
#include "protocol_shaob.h"
#include "tim.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
/******************define******************/
//后期根据射速限制来修改摩擦轮的最大转速
#define FIRE_SPEED_MAX (-7500) //7500 28
#define TRIGGER_CONTINUES_SPEED (-4500)
#define TRIGGER_REVERSE_SPEED 1500
#define DEGREE_45_TO_ENCODER (-36863)
#define DEGREE_BARRLE_TO_ENCODER (-58158)
#define HEAT_MAX 200

//连发准备计时完成
#define CONTINUES_SHOOT_TIMING_COMPLETE() (HAL_GetTick()-continue_shoot_time>2000)
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>2000)
#define BARRLE_INIT_JUDGE() (HAL_GetTick()-barrle_init_block_time>2000)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<200)
//摩擦轮转速PID
#define SHOOT_FIRE_L_PID_KP 30
#define SHOOT_FIRE_L_PID_KI 0.1f
#define SHOOT_FIRE_L_PID_KD 0.f
#define SHOOT_FIRE_L_PID_MAX_OUT    16000
#define SHOOT_FIRE_L_PID_MAX_IOUT   0

#define SHOOT_FIRE_R_PID_KP 30
#define SHOOT_FIRE_R_PID_KI 0.1f
#define SHOOT_FIRE_R_PID_KD 0.f
#define SHOOT_FIRE_R_PID_MAX_OUT    16000
#define SHOOT_FIRE_R_PID_MAX_IOUT   0

//拨弹电机角度环PID
#define SHOOT_TRI_ANGLE_PID_KP 30
#define SHOOT_TRI_ANGLE_PID_KI 0.f
#define SHOOT_TRI_ANGLE_PID_KD 0.0f
#define SHOOT_TRI_ANGLE_PID_MAX_OUT 1000
#define SHOOT_TRI_ANGLE_PID_MAX_IOUT 0

//拨弹电机速度环PID
#define SHOOT_TRI_SPEED_PID_KP  10
#define SHOOT_TRI_SPEED_PID_KI  0.0f
#define SHOOT_TRI_SPEED_PID_KD  0.f
#define SHOOT_TRI_SPEED_PID_MAX_OUT 31000
#define SHOOT_TRI_SPEED_PID_MAX_IOUT 2000


//轮换枪管电机角度环PID
#define SHOOT_CNG_ANGLE_PID_KP 1
#define SHOOT_CNG_ANGLE_PID_KI 3.f
#define SHOOT_CNG_ANGLE_PID_KD 0.1f
#define SHOOT_CNG_ANGLE_PID_MAX_OUT 1000
#define SHOOT_CNG_ANGLE_PID_MAX_IOUT 0

//轮换枪管电机速度环PID
#define SHOOT_CNG_SPEED_PID_KP  5
#define SHOOT_CNG_SPEED_PID_KI  0.0f
#define SHOOT_CNG_SPEED_PID_KD  3.0f
#define SHOOT_CNG_SPEED_PID_MAX_OUT 14000
#define SHOOT_CNG_SPEED_PID_MAX_IOUT 0

//轮换枪管
#define LEFT_BARREL_PWM 500   //左枪管对应的PWM值
#define RIGHT_BARREL_PWM 2500 //右枪管对应的PWM值
#define STAGE_1_WAITING_TIME 100 //等待摩擦轮时间
#define STAGE_2_WAITING_TIME 500 //等待枪管轮换时间
/******************struct&enum******************/
typedef enum {
    FIRE_LEFT=0,
    FIRE_RIGHT,
    TRIGGER,
    BARRLE
}launcher_motor_e;

typedef enum {
    TURN_LEFT=0,
    TURN_RIGHT
}barrel_mode;

typedef enum{
    Fire_OFF=0,
    Fire_ON=1,
}fire_mode;

typedef enum{
    SHOOT_CLOSE=0,
    SHOOT_SINGLE,
    SHOOT_ING,
    SHOOT_CONTINUES,
}trigger_cmd;

typedef enum{
    CHANGE_SINGLE=0,
    CHANGE_ING,
    CHANGE_OK,
}barrle_cmd;

typedef struct {
    fire_mode fire_mode;//摩擦轮状态

    fire_mode fire_last_mode;//摩擦轮上一次状态

    trigger_cmd trigger_cmd;    //发射机构单发还是

    barrle_cmd barrle_cmd;

    launcher_motor_t motor_launcher[4];

    trigger_cmd trigger_down_cmd;

}launcher_t;

extern void launcher_init();
extern void launcher_mode_set();
extern void launcher_control();
extern void launcher_relax_handle();



#endif
