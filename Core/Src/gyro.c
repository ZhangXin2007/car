/**
 * @file    gyro.c
 * @brief   BMI270 陀螺仪采样层（原来在 main.c 的 Gyro_Calibrate 与中断采样块）
 */
#include "gyro.h"
#include "main.h"
#include "car_config.h"
#include "dodo_BMI270.h"

#define GYRO_CALIB_NUM      500U   /* 约 0.5 s，一次 1 ms */

static float   gyro_z_bias = 0.0f;   /* 静态零偏，开机标定得到 */
static float   gyro_z_filt = 0.0f;   /* 去零偏 + 低通后的角速度，°/s */
static uint8_t gyro_ok = 0U;         /* 1 = 初始化成功 */

uint8_t Gyro_Init(void)
{
    float    sum = 0.0f;
    uint16_t i;

    if (dodo_BMI270_init() != 0)
        return 0U;

    gyro_ok = 1U;

    /* 扣掉静态零漂，否则角速度会一直往一边偏 */
    for (i = 0U; i < GYRO_CALIB_NUM; i++)
    {
        dodo_BMI270_get_data();
        sum += BMI270_gyro_transition(BMI270_gyro_z);
        HAL_Delay(1);
    }

    gyro_z_bias = sum / (float)GYRO_CALIB_NUM;
    gyro_z_filt = 0.0f;
    return 1U;
}

void Gyro_Task(void)
{
    float raw;

    dodo_BMI270_get_data();
    raw = BMI270_gyro_transition(BMI270_gyro_z) - gyro_z_bias;
    gyro_z_filt += GYRO_ALPHA * (raw - gyro_z_filt);
}

float Gyro_GetZ(void)
{
    return gyro_z_filt;
}

uint8_t Gyro_IsOk(void)
{
    return gyro_ok;
}
