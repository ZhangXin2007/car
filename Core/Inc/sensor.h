#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

/* ============================================================
 * 12 路光电扫描结果
 *
 * 通道编号约定（和硬件一致）：
 *   ch0 在最左，ch11 在最右，ch5/ch6 是车身中线两侧。
 *   bias = 压线通道号的加权平均；error = bias - 5.5。
 *   error > 0 表示线偏右，方向环应往右打（turn < 0）。
 *
 * mask 不是原始多路复用字：bit i 对应通道 i（方便调试打印）。
 * ============================================================ */
typedef struct {
    uint16_t mask;      /* bit(i) = 通道 i 压线 */
    uint8_t  cnt;       /* 压线通道数 */
    uint8_t  center;    /* ch5 与 ch6 同时压线（六边形入口条件） */
    uint8_t  right;     /* ch11 */
    uint8_t  left;      /* ch0 */
    float    bias;      /* 压线通道号平均；无压线为 0 */
    float    error;     /* bias - 5.5；无压线为 0 */
} LineSample;

/* 扫描一次 12 路光电。方向环 100 Hz 调用，内部是 74HC4067 轮询 + 微秒级等待。 */
void Sensor_Scan(LineSample *s);

#endif /* SENSOR_H */
