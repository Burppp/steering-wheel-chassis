#ifndef __PID_H
#define __PID_H

typedef struct
{
    float ref;
    float feedback;
    float error;
    float error_last;
    float derivative;
    float integral;

    float kp;
    float ki;
    float kd;

    float output;

    float output_max;
    float output_min;
    float integral_max;
    float integral_min;
    float integral_last;
}PID_TypeDef;

void pid_init(PID_TypeDef* pid, float kp, float ki, float kd, float output_max, float output_min, float integral_max, float integral_min);
void pid_calc(PID_TypeDef* pid);
void pid_loop_calc(PID_TypeDef* pid, float max_value, float min_value);

#endif
