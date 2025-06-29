#include "cmsis_os.h"

#include "bsp_can.h"


void travellingCtrl_task(void const * pvParameters)
{
    while(1)
    {
        vTaskSuspendAll();

        xTaskResumeAll();
        vTaskDelay(1);
    }
}
