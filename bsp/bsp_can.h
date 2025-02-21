#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "PID.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;

    int32_t total_ecd;   //电机旋转的总编码器数值
    int32_t round_count;
    uint16_t offset_ecd;//电机的校准编码值

} motor_measure_t;



#define STEERING_WHEEL_STEER_ANGLE_KP 40
#define STEERING_WHEEL_STEER_ANGLE_KI 0.02
#define STEERING_WHEEL_STEER_ANGLE_KD 5.0
#define STEERING_WHEEL_STEER_ANGLE_OUT 150.0
#define STEERING_WHEEL_STEER_ANGLE_IOUT 30.0

#define STEERING_WHEEL_STEER_SPEED_KP 60.0
#define STEERING_WHEEL_STEER_SPEED_KI 0.1
#define STEERING_WHEEL_STEER_SPEED_KD 20.0
#define STEERING_WHEEL_STEER_SPEED_OUT 28000.0
#define STEERING_WHEEL_STEER_SPEED_IOUT 8000.0

/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125f     //        360  /8192
#define MOTOR_RAD_TO_ANGLE 57.29577951308238f // 360/2PI
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#define ANGLE_TO_ECD 22.75555
#define MOTOR_RAD_TO_ECD 1303.797316049191101
#define MOTOR_NOT_BLOCKED 0
#define MOTOR_IS_BLOCKED 1

#define MOTOR_OFFLINE 0
#define MOTOR_ONLINE 1

#define MOTOR_CHECK_TIME 200

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

typedef enum{
    CAN_DJI_MOTOR_0x200_ID = 0x200,//C620/C610 id=1~4 (0x201~0x204)
    CAN_DJI_MOTOR_0x1FF_ID = 0x1FF,//C620/C610 id=5~8 (0x205~0x208);GM6020 id=1~4 (0x205~0x208)
    CAN_DJI_MOTOR_0x2FF_ID = 0x2FF,//GM6020 id=5~7 (0x209~0x20B)
}DJI_MOTOR_ID;

typedef enum{
    //左手电机为CAN1
     CAN_CHASSIS_3508_MOTOR_LF=0x202,
     CAN_CHASSIS_3508_MOTOR_LB=0x203,
     CAN_STEER_MOTOR_LF=0x205,
     CAN_STEER_MOTOR_LB=0x208,

//    右手电机为CAN2
     CAN_CHASSIS_3508_MOTOR_RF=0x201,
     CAN_CHASSIS_3508_MOTOR_RB=0x204,
     CAN_STEER_MOTOR_RF=0x206,
     CAN_STEER_MOTOR_RB=0x207,

    //0X1FF对应的电机ID(CAN1)
    CAN_GIMBAL_6020_YAW=0x205,
    CAN_GIMBAL_XIAOMI_YAW = 0x64,
    CAN_LAUNCHER_2006_TRIGGER=0x206,

    //0X200对应的电机ID(CAN2)

    //0X1FF对应的电机ID(CAN2)
    CAN_LAUNCHER_3508_FIRE_R=0X205,
    CAN_GIMBAL_6020_PITCH=0x206,
    CAN_LAUNCHER_3508_FIRE_L=0X207,
    CAN_LAUNCHER_2006_BARREL=0x208,

}CAN_ID;

typedef struct {
    uint8_t blocked_flag;
    uint8_t real_time_block_flag;
    uint32_t check_blocked_start_time;
    uint32_t check_blocked_continue_time;
    float blocked_check_ratio; //堵转判定比率（低于电机设定速度）
} motor_blocked_t;
typedef struct {
    uint8_t status_flag;
    uint32_t status_update_time;
    uint32_t status_check_time;
} motor_status_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t speed_p;
    fp32 set_rpm;
    int16_t give_current;
}chassis_motor_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t speed_p;
    pid_t angle_p;
    fp32 set_speed;
    fp32 get_angle;
    fp32 set_angle;
    int16_t give_current;
}steering_motor_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t angle_p;
    pid_t relative_angle_p;
    pid_t speed_p;
    pid_t relative_speed_p;

    fp32 relative_angle_get;
    fp32 relative_angle_set; //°
    fp32 relative_gyro_get;

    fp32 gyro_set;  //转速设置
    int16_t give_current; //最终电流值
}gimbal_motor_t;

typedef struct
{
    motor_measure_t  motor_info;
    pid_t angle_p;
    pid_t relative_angle_p;
    pid_t speed_p;
    pid_t relative_speed_p;

    fp32 relative_angle_get;
    fp32 relative_angle_set;
    fp32 relative_gyro_set;

    fp32 ecd_set;
    int16_t give_current;
}right_gimbal_motor_t;

typedef struct
{
    motor_measure_t  motor_info;
    pid_t angle_p;
    pid_t relative_angle_p;
    pid_t speed_p;
    pid_t relative_speed_p;

    fp32 relative_angle_get;
    fp32 relative_angle_set;
    fp32 relative_gyro_set;

    fp32 ecd_set;
    int16_t give_current;
}left_gimbal_motor_t;


typedef struct
{
    motor_measure_t motor_info;
    pid_t angle_p;
    pid_t speed_p;

    fp32 rpm_set;
    int16_t give_current;

}launcher_motor_t;

typedef struct
{
    uint32_t id:8;
    uint32_t data:16;
    uint32_t mode:5;
    uint32_t res:3;
}xiaomi_motor_t;

typedef struct
{
    float angle; //减去中值的角度 360
    int16_t speed_rpm;//转速
    float angular_v;//角速度
    float temperate;
    float offset_angle;
    float real_angle;//原始角度
    float last_angle;
} encoder_t;

typedef struct
{
    uint8_t mode;//底盘模式
    fp32 vx;
    fp32 vy;
    fp32 vw;//大yaw旋转速度
    fp32 gyro_vw;//陀螺速度
    fp32 yaw_angle;//yaw控制角度
}chassis_ctrl_info_t;

typedef  struct
{
    float chassis_power;
    uint16_t buffer_energy;
} __packed chassis_power_t;

#pragma pack(1)
typedef struct
{
    uint8_t mode;//底盘模式
    uint8_t chassis_online;
    int16_t vx;
    int16_t vy;
    int16_t vw;//大yaw旋转速度
    int16_t gyro_vw;//陀螺速度
    fp32 yaw_angle;//yaw控制角度
    fp32 yaw_feedback;
} chassis_cmd_cp;
#pragma pack()

typedef struct
{
//    chassis_ctrl_info_t ctrl_data;
//    chassis_power_t referee_data;
//    fp32 yaw_feedback;//陀螺仪数据

    uint8_t mode;//底盘模式
    uint8_t chassis_online;//底盘是否在线
    fp32 vx;
    fp32 vy;
    fp32 vw;//大yaw旋转速度
    fp32 gyro_vw;//陀螺速度
    fp32 yaw_angle;//yaw控制角度
    fp32 yaw_feedback;//yaw陀螺仪数据
}chassis_cmd;

typedef struct
{
    int p_int,v_int,t_int;
    float position,velocity,torque;
    uint8_t  Tx_Data[8];												//数据发送存储
    uint8_t  RxData[8];												//数据接收存储
    CAN_RxHeaderTypeDef Rx_pHeader;
}CANx_t;

typedef struct
{
    int p_int,v_int,t_int;
    float position,velocity,torque;
    uint8_t  Tx_Data[8];												//数据发送存储
    uint8_t  RxData[8];												//数据接收存储
    CAN_RxHeaderTypeDef Rx_pHeader;
}DM_Motor;

typedef struct
{
    fp32 voltage_input;
    fp32 voltage_cap;
    fp32 current_input;
    fp32 power_target;
}Power_feedback;

extern CANx_t can_1,can_2;
extern DM_Motor yaw_motor;

extern void get_can_msg(CAN_TYPE can_type,uint32_t *can_id, uint8_t * can_ms);
extern void chassis_motor_decode(chassis_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void chassis_steering_motor_decode(steering_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void motor_blocked_check_init(motor_measure_t *measure, uint32_t blocked_continue_time, float blocked_check_ratio);
extern void motor_status_init(motor_measure_t *measure);
extern void dji_motor_init(motor_measure_t *measure, uint32_t blocked_continue_time, float blocked_check_ratio);
extern void dji_steering_motor_init(motor_measure_t *measure, uint32_t blocked_continue_time, float blocked_check_ratio,uint16_t offset_ecd);
extern void dji_motor_decode(motor_measure_t *motor,const uint8_t *data);
extern motor_measure_t *get_motor_measure_point(uint8_t group, uint8_t i);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd);
extern void gimbal_motor_decode(gimbal_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void launcher_motor_decode(launcher_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void can_send_dji_motor(CAN_TYPE can_type, DJI_MOTOR_ID CMD_ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void can_send_xiaomi_motor(fp32 speed);
extern void DM_enable(void);
extern void Speed_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID,fp32 _vel);
extern int float_to_uint(float x, float x_min, float x_max, int bits);
extern void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);

extern void encoder_decode(encoder_t *encoder,const uint8_t *data);
extern void yaw_encoder_decode(encoder_t *encoder,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void DM_MotorDecode(DM_Motor *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void CAN_cmd_cap(uint16_t power_limit);
extern void Cap_Decode(uint8_t can_type,uint32_t can_id,uint8_t * can_msg);

#endif
