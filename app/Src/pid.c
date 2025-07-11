#include "pid.h"

void pid_init(PID_TypeDef* pid, float kp, float ki, float kd, float output_max, float output_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_max = integral_max;
}

void pid_calc(PID_TypeDef* pid)
{
    pid->error = pid->ref - pid->feedback;
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->error_last;
    pid->pout = pid->kp * pid->error;
    pid->iout = pid->ki * pid->integral;
    if(pid->iout > pid->integral_max)
    {
        pid->iout = pid->integral_max;
    }
    pid->dout = pid->kd * pid->derivative;
    pid->output = pid->pout + pid->iout + pid->dout;
    if(pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if(pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }
    pid->error_last = pid->error;
}

void pid_loop_calc(PID_TypeDef* pid, float max_value, float min_value)
{
    float gap, mid;
    mid = (max_value - min_value) / 2;
    gap = pid->ref - pid->feedback;
    if(gap>=mid)
    {
        while(gap < min_value || gap > max_value)
            gap -= max_value - min_value;
        pid->ref = 0;
        pid->feedback = -gap;
        pid_calc(pid);
    }
    else if(gap<=-mid)
    {
        while(gap < min_value || gap > max_value)
            gap += max_value - min_value;
        pid->ref = 0;
        pid->feedback = -gap;
        pid_calc(pid);
    }
    else
    {
        pid_calc(pid);
    }
}
