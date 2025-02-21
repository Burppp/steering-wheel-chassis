//
// Created by Lumos on 2024/3/26.
//

#include "Cap.h"
#include "cmsis_os.h"
#include "Referee.h"
#include "bsp_can.h"
#include "Detection.h"

extern chassis_cmd chassis_cmd_data;
chassis_status chassisStatus = CHASSIS_OFFLINE;
uint16_t target_power;
void cap_task(void const *pvParameters)
{
    vTaskDelay(CAP_TASK_INIT_TIME);
    while (1)
    {
        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
        target_power = 0;
        target_power = CHASSIS_ONLINE_POWER;
        CAN_cmd_cap(target_power);
        xTaskResumeAll();
        vTaskDelay(100);
    }
}