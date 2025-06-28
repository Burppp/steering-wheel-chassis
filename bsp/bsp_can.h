#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

#include "main.h"
#include "object_dictionary.h"

#define CHASSIS_CAN hcan1

#define CAN_SDO_READ_CTRL_BYTE 0x40
#define CAN_SDO_WRITE_CTRL_BYTE 0x00

#define CAN_SEND_1_BYTE    0x2F
#define CAN_SEND_2_BYTE    0x2B
#define CAN_SEND_4_BYTE    0x23

typedef enum
{
    POSITION_CONTROL = 0x01,
    SPEED_CONTROL = 0x03,
    TORQUE_CONTROL = 0x04,
    HOMING_MODE = 0x06,
    INTERPOLATION_MODE = 0x07,
    CSP = 0x08,
    CSV = 0x09,
    CST = 0x0A,
    PULSE_TRAIN_CONTROL = -4
}MODE_ENUM;

extern void can_filter_init(void);

void motor_enable(uint8_t motor_id);
void motor_mode(int8_t mode, uint8_t motor_id);
void motor_setSpeed(int32_t speed_ref, uint8_t motor_id);
void motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id);
void motor_setPos(int32_t position_ref, uint8_t motor_id);
void motor_encoderDataReset(uint8_t value, uint8_t motor_id);

void motor_TPDO_config(uint8_t motor_id);
void motor_TPDO_enable(uint8_t ctrl, uint8_t motor_id);

bool SDO_Send(uint8_t motor_id, uint8_t ctrl, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType);
bool PDO_Send(uint8_t ctrl, uint8_t motor_id);

bool SDO_Read(uint8_t motor_id, uint16_t index, uint8_t subindex, OD_DataType dataType);
bool SDO_Write(uint8_t motor_id, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType);

#endif
