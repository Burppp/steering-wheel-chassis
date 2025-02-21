//
// Created by Lumos on 2024/3/26.
//

#ifndef REMOTE_CUBE_SENTRY_CAP_H
#define REMOTE_CUBE_SENTRY_CAP_H

#include "struct_typedef.h"

typedef enum
{
    CHASSIS_ONLINE,
    CHASSIS_OFFLINE
}chassis_status;

#define CAP_TASK_INIT_TIME 357
#define CHASSIS_ONLINE_POWER 95;
#define CHASSIS_OFFLINE_POWER 0

extern void cap_task(void const *pvParameters);
//extern uint16_t chassis_detect();

#endif //REMOTE_CUBE_SENTRY_CAP_H
