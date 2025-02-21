//#ifndef _CHASSIS_H_
//#define _CHASSIS_H_
//
///*include*/
//#include "struct_typedef.h"
//#include "bsp_can.h"
//#include "PID.h"
//#include "remote.h"
//#include "user_lib.h"
//#include "cmsis_os.h"
//#include "ramp.h"
#include "Gimbal.h"
//#include "bsp_buzzer.h"
//#include "math.h"
//#include "Referee.h"
//#include "Detection.h"
#include "protocol_shaob.h"
//#include "packet.h"
//
//
///*define*/
//
////任务开始空闲一段时间
//
//#define CHASSIS_TASK_INIT_TIME 157
//
//#define CHASSIS_Y_CHANNEL 0
//
//#define CHASSIS_X_CHANNEL 1
//
//#define CHASSIS_Z_CHANNEL 4
//
//#define CHASSIS_MODE_CHANNEL 0
//
//#define CHASSIS_CONTROL_TIME_MS 2
//
////底盘旋转跟随PID
//#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.3f
//#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 4.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.8f
//
//#define chassis_start_buzzer buzzer_on  (31, 19999)
//#define chassis_buzzer_off   buzzer_off()            //buzzer off，关闭蜂鸣器
//
//#define MAX_CHASSIS_VX_SPEED 2.8f
//#define MAX_CHASSIS_VY_SPEED 2.8f
//#define MAX_CHASSIS_VW_SPEED 0.9f
//#define CHASSIS_SWING_SPEED 1.0f
//
//#define MAX_CHASSIS_DEGREE 0.2f
//
//#define STEERING_WHEEL_MAX_VX 2.8f      // m/s
//#define STEERING_WHEEL_MAX_VY 2.8f      // m/s
//#define STEERING_WHEEL_MAX_VW 0.9f      // rad/s
//#define STEERING_WHEEL_MAX_DEGREE 0.2  // degree
//#define STEERING_WHEEL_DR16_RC_TO_CHASSIS_VX (STEERING_WHEEL_MAX_VX/660.0f)
//#define STEERING_WHEEL_DR16_RC_TO_CHASSIS_VY (STEERING_WHEEL_MAX_VY/660.0f)
//#define STEERING_WHEEL_DR16_RC_TO_CHASSIS_VW (STEERING_WHEEL_MAX_VW/660.0f)
//
//#define STEERING_WHEEL_DR16_RC_TO_CHASSIS_CONTROL_YAW (STEERING_WHEEL_MAX_DEGREE/660.0f)
//
//
//#define CHASSIS_CURRENT_LIMIT_TOTAL 20000
//#define CHASSIS_CURRENT_LIMIT_40W 7300
//#define CHASSIS_POWER_BUFF 60
//
//
////底盘机械信息 /m
//#define Wheel_axlespacing 0.448f //H
//#define Wheel_spacing 0.391f //W
//#define GIMBAL_OFFSET 0
//#define PERIMETER 0.47414f //zhou chang /m
//#define M3508_DECELE_RATIO 1.0f/19.0f
//#define M3508_MAX_RPM 8000
//#define TREAD 480 //lun ju
//
//#define STEERING_WHEEL_V_SPEED_LIMIT 3.0f    // m/s
//#define STEERING_WHEEL_W_SPEED_LIMIT 0.9f    // rad/s
//
//#define WHEEL_MOTO_RATE 0.00041591f
//#define STEERING_WHEEL_WHEELBASE 0.32        // m
//#define STEERING_WHEEL_TRACK_WIDTH 0.4       // m
//#define STEERING_WHEEL_WHEEL_RADIUS 0.07     // m
//
//#define STEERING_WHEEL_STEERING_MOTOR_OFFSET_0 7605
//#define STEERING_WHEEL_STEERING_MOTOR_OFFSET_1 6154
//#define STEERING_WHEEL_STEERING_MOTOR_OFFSET_2 4873
//#define STEERING_WHEEL_STEERING_MOTOR_OFFSET_3 4040
//
//#define STEERING_WHEEL_ROTATE_KP 0.04
//#define STEERING_WHEEL_ROTATE_KI 0.000006
//#define STEERING_WHEEL_ROTATE_KD 0.05
//#define STEERING_WHEEL_ROTATE_OUT 1.5
//#define STEERING_WHEEL_ROTATE_IOUT 0.8
//
//#define STEERING_WHEEL_CHASSIS_KP 20.0
//#define STEERING_WHEEL_CHASSIS_KI 0.001
//#define STEERING_WHEEL_CHASSIS_KD 40
//#define STEERING_WHEEL_CHASSIS_OUT 11000.0
//#define STEERING_WHEEL_CHASSIS_IOUT 4000.0
//
//#define STEERING_WHEEL_STEER_ANGLE_KP 40
//#define STEERING_WHEEL_STEER_ANGLE_KI 0.02
//#define STEERING_WHEEL_STEER_ANGLE_KD 5.0
//#define STEERING_WHEEL_STEER_ANGLE_OUT 150.0
//#define STEERING_WHEEL_STEER_ANGLE_IOUT 30.0
//
//#define STEERING_WHEEL_STEER_SPEED_KP 60.0
//#define STEERING_WHEEL_STEER_SPEED_KI 0.1
//#define STEERING_WHEEL_STEER_SPEED_KD 20.0
//#define STEERING_WHEEL_STEER_SPEED_OUT 28000.0
//#define STEERING_WHEEL_STEER_SPEED_IOUT 8000.0
//
//#define LF_0_WHEEL_OUTWARD_ECD_FROM 1400
//#define LF_0_WHEEL_OUTWARD_ECD_TO 5300
//#define RF_1_WHEEL_OUTWARD_ECD_FROM 0
//#define RF_1_WHEEL_OUTWARD_ECD_TO 4000
//
//#define RB_2_WHEEL_OUTWARD_ECD_FROM 6800
//#define RB_2_WHEEL_OUTWARD_ECD_TO 2700
//#define LB_3_WHEEL_OUTWARD_ECD_FROM 6100
//#define LB_3_WHEEL_OUTWARD_ECD_TO 2000
//
////枚举 结构体
//typedef enum
//{
//    CHASSIS_RELAX,
//    CHASSIS_AUTO,
//    CHASSIS_DEBUG
//} chassis_mode_e;
//
//typedef enum
//{
//    INWARD,
//    OUTWARD
//}steering_status_e;
//
//typedef struct {
//    fp32 power_buff;
//    fp32 limit_k;
//    fp32 total_current;
//    fp32 total_current_limit;
//
//}chassis_power_limit_t;
//
//typedef struct {
//    float wheel_v;      // m/s
//    float wheel_angle;  //rad
//    int32_t speed_vector;
//} steering_velocity_vector_t;
//
//typedef struct {
//    steering_velocity_vector_t velocity_vector;
//    steering_velocity_vector_t velocity_vector_last;
//} steering_control_t;
//
//typedef struct
//{
//    steering_status_e steering_wheel_status[4];
//
//    chassis_mode_e mode;
//    chassis_mode_e last_mode;
//    chassis_motor_t motor_chassis[4];
//    steering_motor_t motor_steering[4];
//    pid_t chassis_vw_pid;
//    pid_t rotate_pid;
//    fp32 vx;
//    fp32 vy;
//    fp32 vw;
//    float plane_yaw;
//    float plane_v;
//    float control_yaw;
//    float chassis_yaw;
//    float circle_rad;
//    chassis_power_limit_t chassis_power_limit;
//    steering_control_t wheel[4];
//    steering_control_t rotate[4];
//} chassis_t;
//
//
//
//
////函数声明
//_Noreturn extern void chassis_task(void const *pvParameters);
//
//extern void chassis_feedback_update();
////
//#endif
//
