#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include <stdbool.h>

#include "object_dictionary.h"
#include "pid.h"
#include "controller_loop_param.h"

#define CHASSIS_CAN hcan1

#define CAN_SDO_READ_CTRL_BYTE 0x40
#define CAN_SDO_WRITE_CTRL_BYTE 0x00

#define CAN_SEND_1_BYTE    0x2F
#define CAN_SEND_2_BYTE    0x2B
#define CAN_SEND_4_BYTE    0x23
#define CAN_SEND_SUCCESS   0x60
#define CAN_FAILED    0x80

#define CAN_RECV_1_BYTE 0x4F
#define CAN_RECV_2_BYTE 0x4B
#define CAN_RECV_4_BYTE 0x43

#define SDO_SEND_OFFSET 0x600
#define SDO_RECV_OFFSET 0x580

typedef enum
{
    PULSE_TRAIN_CONTROL = 0xFC,
    POSITION_CONTROL = 0x01,
    SPEED_CONTROL = 0x03,
    TORQUE_CONTROL = 0x04,
    HOMING_MODE = 0x06,
    INTERPOLATION_MODE = 0x07,
    CSP = 0x08,
    CSV = 0x09,
    CST = 0x0A
}MODE_ENUM;

typedef enum
{
    LF_STEER_ID = 0x00,
    LF_DRIVE_ID = 0x01,
    RF_STEER_ID = 0x02,
    RF_DRIVE_ID = 0x03,
    LB_STEER_ID = 0x04,
    LB_DRIVE_ID = 0x05,
    RB_STEER_ID = 0x06,
    RB_DRIVE_ID = 0x07
}ID_ENUM;

typedef struct
{
    float F_dir;
    float XY_dir;
    float Z_dir;
    float XY_Z_Angle;
    float prev_Z_Angle;
    float XYZ_Speed;
    float Dir;
    float Dir_last;
    bool JUST_SPIN;
}Move_t;

typedef struct
{
    ID_ENUM motor_id;
    MODE_ENUM mode;
    bool is_enable;
    int32_t position_feedback;
    float speed_feedback;
    int16_t torque_feedback; //percent

    PID_TypeDef position_loop;
    PID_TypeDef speed_loop;

    int32_t position_ref;
    int32_t speed_ref;
    int16_t torque_ref;

    int32_t position_feedback_last;
    int32_t speed_feedback_last;

    ObjectDictionary* od;
    float offset;

    float move_mid_angle;
    float spin_mid_angle;
    Move_t Move;
    bool tra_reverse;
    float tra_reverse_ecd;
}Motor_t;

extern void can_filter_init(void);

void motor_init(Motor_t* motor, ID_ENUM id);

bool motor_enable(uint8_t motor_id);
bool motor_mode(int8_t mode, uint8_t motor_id);
bool motor_setSpeed(int32_t speed_ref, uint8_t motor_id);
bool motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id);
bool motor_setPos(int32_t position_ref, uint8_t motor_id);
bool motor_encoderDataReset(uint8_t value, uint8_t motor_id);
bool motor_setTorque(int16_t torque_ref, uint8_t motor_id);

bool motor_readRealPosition(uint8_t motor_id);
bool motor_readRealVelocity(uint8_t motor_id);
bool motor_readRealTorque(uint8_t motor_id);

bool motor_TPDO_config(uint8_t motor_id);
bool motor_TPDO_enable(uint8_t ctrl, uint8_t motor_id);

bool SDO_Send(uint8_t motor_id, uint8_t ctrl, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType);
bool PDO_Send(uint8_t ctrl, uint8_t motor_id);

bool SDO_Read(uint8_t motor_id, uint16_t index, uint8_t subindex, OD_DataType dataType);
bool SDO_Write(uint8_t motor_id, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType);

#endif
