#include "cmsis_os.h"

#include "bsp_can.h"


void steeringCtrl_task(void const * pvParameters)
{
    while(1)
    {
        osDelay(100);
    }
}
