/**
 * @file    sensor.c
 * @brief   12 路光电扫描（原来内联在 main.c 的 Control_Update 里）
 *
 * 只依赖 multiplexer 驱动，不碰电机/状态机，方便以后脱机喂数据做测试。
 */
#include "sensor.h"
#include "multiplexer.h"

#define SENSOR_CENTER       5.5f   /* 12 路的中线位置：(0+11)/2 */
#define SENSOR_CH_NUM       12

void Sensor_Scan(LineSample *s)
{
    uint16_t mux = 0U;
    uint16_t mask = 0U;
    float    pos = 0.0f;
    uint8_t  cnt = 0U;

    MUX_get_value(&mux);

    for (int i = 0; i < SENSOR_CH_NUM; i++)
    {
        if (MUX_GET_CHANNEL(mux, i))
        {
            pos += (float)i;
            cnt++;
            mask |= (uint16_t)(1U << i);
        }
    }

    s->mask = mask;
    s->cnt  = cnt;
    s->bias = (cnt > 0U) ? (pos / (float)cnt) : 0.0f;
    s->error = (cnt > 0U) ? (s->bias - SENSOR_CENTER) : 0.0f;

    /* 六边形入口用的原始信号，和原来 main.c 里同名变量一致 */
    s->center = (uint8_t)(MUX_GET_CHANNEL(mux, 5) && MUX_GET_CHANNEL(mux, 6));
    s->right  = (uint8_t)MUX_GET_CHANNEL(mux, 11);
    s->left   = (uint8_t)MUX_GET_CHANNEL(mux, 0);
}
