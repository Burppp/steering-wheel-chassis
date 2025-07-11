#include "data_update.h"

extern Motor_t motor_list[8];
bool is_init = false;

void real_position_update(void)
{
//    motor_readRealPosition(LF_STEER_ID);
//    motor_readRealPosition(RF_STEER_ID);
//    motor_readRealPosition(LB_STEER_ID);
//    motor_readRealPosition(RB_STEER_ID);
    motor_readRealPosition(LF_DRIVE_ID);
//    motor_readRealPosition(RF_DRIVE_ID);
//    motor_readRealPosition(LB_DRIVE_ID);
//    motor_readRealPosition(RB_DRIVE_ID);
}

void real_velocity_update(void)
{
//    motor_readRealVelocity(LF_STEER_ID);
//    motor_readRealVelocity(RF_STEER_ID);
//    motor_readRealVelocity(LB_STEER_ID);
//    motor_readRealVelocity(RB_STEER_ID);
    motor_readRealVelocity(LF_DRIVE_ID);
//    motor_readRealVelocity(RF_DRIVE_ID);
//    motor_readRealVelocity(LB_DRIVE_ID);
//    motor_readRealVelocity(RB_DRIVE_ID);
}

void real_torque_update(void)
{
//    motor_readRealTorque(LF_STEER_ID);
//    motor_readRealTorque(RF_STEER_ID);
//    motor_readRealTorque(LB_STEER_ID);
//    motor_readRealTorque(RB_STEER_ID);
    motor_readRealTorque(LF_DRIVE_ID);
//    motor_readRealTorque(RF_DRIVE_ID);
//    motor_readRealTorque(LB_DRIVE_ID);
//    motor_readRealTorque(RB_DRIVE_ID);
}

void od2motor(void)
{
    for(int i = 0; i < 8; i++)
    {
        OD_Entry* entry = OD_GetEntry(motor_list[i].od, OD_INDEX_ACTUAL_POSITION, OD_SUBINDEX_DEFAULT);
        if(entry != NULL)
        {
            memcpy(&motor_list[i].position_feedback, entry->data, sizeof(int32_t));
            if(motor_list[i].position_feedback != 0 && !is_init)
            {
                is_init = true;
                motor_list[i].position_loop.ref = motor_list[i].position_feedback;
            }
        }

        entry = OD_GetEntry(motor_list[i].od, OD_INDEX_ACTUAL_VELOCITY, OD_SUBINDEX_DEFAULT);
        if(entry != NULL)
        {
            int32_t v_fdb = 0;
            memcpy(&v_fdb, entry->data, sizeof(int32_t));
            motor_list[i].speed_feedback = (float)v_fdb / STR_RPM_TO_DEC;
        }

        entry = OD_GetEntry(motor_list[i].od, OD_INDEX_ACTUAL_CURRENT, OD_SUBINDEX_DEFAULT);
        if(entry != NULL)
        {
            memcpy(&motor_list[i].torque_feedback, entry->data, sizeof(int16_t));
        }
    }
}

void dataUpdate_task(void const * argument)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    while(1)
    {
        vTaskSuspendAll();

        real_position_update();
        real_velocity_update();
        real_torque_update();
        od2motor();

        xTaskResumeAll();
        vTaskDelay(2);
    }
}
