//
// Created by nobody_knows on 23-3-13.
//

#ifndef REMOTE_CUBE_SENTRY_CAN_HARDWARES_H
#define REMOTE_CUBE_SENTRY_CAN_HARDWARES_H

#include "struct_typedef.h"
#include "can_common.h"
#define MAX_CAN_COMM_COUNT 10 // 注意均衡负载,一条总线上不要挂载过多的外设
static uint8_t idx = 0;
typedef struct
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_date[8];
}CAN_MSG;
extern void can_filter_init(void);
void register_can_comm_instance(CANCommInstance *ins);
#endif //REMOTE_CUBE_SENTRY_CAN_HARDWARES_H
