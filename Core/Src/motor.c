#include "main.h"
#include "motor.h"

/* 左右轮速度环 */
PID_t pid_L = { .Kp = 2.5f, .Ki = 0.01f, .Kd = 0.0f };
PID_t pid_R = { .Kp = 2.5f, .Ki = 0.01f, .Kd = 0.0f };

/* 方向环 */
DirPID_t dir_pid = { .Kp = 2.5f, .Ki = 0.0f, .target = 0.0f };

float PID_Calc(PID_t *p, float measure)
{
    float kp;

    p->measure   = measure;
    p->pre_error = p->error;
    p->error     = p->target - p->measure;
    p->integral += p->error;
    if (p->integral >  INTEG_LIMIT) p->integral =  INTEG_LIMIT;
    if (p->integral < -INTEG_LIMIT) p->integral = -INTEG_LIMIT;

    kp = p->Kp;
    if (p->measure < KP_LOW_SPEED && p->target > p->measure)
    {
        float ratio = 1.0f - p->measure / KP_LOW_SPEED;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        kp = p->Kp * (1.0f + (KP_BOOST - 1.0f) * ratio);
    }

    p->output = kp * p->error
              + p->Ki * p->integral
              + p->Kd * (p->error - p->pre_error);

    /* 输出限幅 + 抗 windup */
    if (p->output >  PWM_MAX) { p->output =  PWM_MAX; if (p->error > 0) p->integral -= p->error; }
    if (p->output < -PWM_MAX) { p->output = -PWM_MAX; if (p->error < 0) p->integral -= p->error; }
    return p->output;
}

float DirPID_Calc(DirPID_t *p, float measure)
{
    p->measure   = measure;
    p->pre_error = p->error;
    p->error     = p->target - p->measure;

    p->integral += p->error;
    if (p->integral >  INTEG_LIMIT) p->integral =  INTEG_LIMIT;
    if (p->integral < -INTEG_LIMIT) p->integral = -INTEG_LIMIT;

    p->output = p->Kp * p->error + p->Ki * p->integral;
    if (p->output >  TURN_MAX) p->output =  TURN_MAX;
    if (p->output < -TURN_MAX) p->output = -TURN_MAX;
    return p->output;
}

void Motor_Output(float out_L, float out_R)
{
    if (out_L >  PWM_MAX) out_L =  PWM_MAX;
    if (out_L < -PWM_MAX) out_L = -PWM_MAX;
    if (out_R >  PWM_MAX) out_R =  PWM_MAX;
    if (out_R < -PWM_MAX) out_R = -PWM_MAX;

    /* 死区补偿：只在目标比实测快时抬到最小值 */
    if (out_L > 0.0f && out_L < MOTOR_MIN_DUTY_L && pid_L.target > pid_L.measure) out_L = MOTOR_MIN_DUTY_L;
    if (out_L < 0.0f && out_L > -MOTOR_MIN_DUTY_L && pid_L.target > pid_L.measure) out_L = -MOTOR_MIN_DUTY_L;
    if (out_R > 0.0f && out_R < MOTOR_MIN_DUTY_R && pid_R.target > pid_R.measure) out_R = MOTOR_MIN_DUTY_R;
    if (out_R < 0.0f && out_R > -MOTOR_MIN_DUTY_R && pid_R.target > pid_R.measure) out_R = -MOTOR_MIN_DUTY_R;

    /* 前进时不允许 PID 过冲把电机拉到反转 */
    if (pid_L.target >= 0.0f && out_L < 0.0f) out_L = 0.0f;
    if (pid_R.target >= 0.0f && out_R < 0.0f) out_R = 0.0f;

    if (out_L >= 0) {
        TIM1->CCR1 = (uint32_t)out_L;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else {
        TIM1->CCR1 = (uint32_t)(-out_L);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }

    if (out_R >= 0) {
        TIM1->CCR2 = (uint32_t)out_R;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        TIM1->CCR2 = (uint32_t)(-out_R);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
}
