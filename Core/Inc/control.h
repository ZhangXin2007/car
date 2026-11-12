#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "special.h"   /* HexDebug */

/* ============================================================
 * 1 kHz 控制层（原 main.c 的 Control_Update 及其内部状态）
 *
 * 调用关系：
 *   TIM2 更新中断 -> Timebase_1kHz() -> Control_Update()
 *   主循环按键     -> Control_Start()
 *   主循环串口命令 -> Control_StartSpin() / Control_StartStraight()
 *   主循环调试打印 -> Control_GetDebug() / Control_GetHexDebug()
 * ============================================================ */

/* 1 kHz 控制入口：陀螺 200 Hz、方向环/速度环 100 Hz、电机输出。 */
void Control_Update(void);

/* 发车：清积分、复位 kick，从第一拍开始脉冲 kick。 */
void Control_Start(void);

/* 1 = 正在跑（发车后、停车前）。 */
uint8_t Control_IsRunning(void);

/* 串口命令 '2'：原地转 360°。 */
void Control_StartSpin(void);

/* 串口命令 '3'：脱线直线 1 m。 */
void Control_StartStraight(void);

/* 主循环用的调试快照，避免在中断里碰串口。 */
typedef struct {
    float    speed_cmd;
    float    speed_L, speed_R;   /* PID 实际使用的轮速，含符号 */
    int16_t  dL, dR;             /* 10 ms 编码器增量 */
    float    target_L, target_R;
    float    gyro_z;             /* 滤波后的角速度，°/s */
    uint8_t  run;                /* 1 = 正在跑 */
} ControlDebug;

void Control_GetDebug(ControlDebug *d);

/* 六边形入口调试快照（转发 special.c 的 Hex_GetDebug）。 */
void Control_GetHexDebug(HexDebug *d);

#endif /* CONTROL_H */
