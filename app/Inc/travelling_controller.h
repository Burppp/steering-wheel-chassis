#ifndef TRAVELLING_CONTROLLER_H
#define TRAVELLING_CONTROLLER_H

#include "steering_controller.h"

typedef struct
{
    ChassisMode_t mode;
    Motor_t TRA_Axis[WHEEL_CNT];
    float speed_max;
    Vector_t vector;
    int8_t head_or_tail;
}ChassisTravelling_t;

void travellingCtrl_task(void const * pvParameters) __attribute__((noreturn));

#endif
