#ifndef __DATA_UPDATE_H
#define __DATA_UPDATE_H

#include "cmsis_os.h"
#include "string.h"
#include "bsp_can.h"
#include "packet_param.h"

void real_position_update(void);
void real_velocity_update(void);
void real_torque_update(void);

void dataUpdate_task(void const * argument);

#endif
