#ifndef MOTOR_H
#define MOTOR_H

#include "car_config.h"

/* 速度环 PID */
typedef struct {
    float Kp, Ki, Kd;
    float target;
    float measure;
    float error;
    float pre_error;
    float integral;
    float output;
} PID_t;

/* 方向环 PI */
typedef struct {
    float Kp, Ki;
    float target;
    float measure;
    float error, pre_error, integral;
    float output;
} DirPID_t;

extern PID_t pid_L;
extern PID_t pid_R;
extern DirPID_t dir_pid;

float PID_Calc(PID_t *p, float measure);
float DirPID_Calc(DirPID_t *p, float measure);
void Motor_Output(float out_L, float out_R);

#endif /* MOTOR_H */
