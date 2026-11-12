#ifndef CAR_STATE_H
#define CAR_STATE_H

#include <stdint.h>

/* ============================================================
 * 小车跨拍状态模型
 *
 * 以前这些变量都是 main.c 里的 file-static，谁都能读写；
 * 现在集中成一个结构体，实例由 control.c 私有持有（static CarState car）。
 * 这样状态迁移只能通过 control 模块的接口发生，调试时也只有一个快照来源。
 * ============================================================ */

/* 起步 kick 状态机 */
typedef struct {
    uint8_t  done;           /* 0 = 还在 kick，1 = 已起来 */
    uint8_t  kick_on;        /* 当前脉冲是 on 还是 off */
    uint16_t kick_phase;     /* 当前 on/off 剩余 10 ms 拍数 */
    uint16_t kick_total;     /* 已 kick 的总拍数 */
    uint16_t settle_timer;   /* 起步后只给前馈的拍数 */
    uint16_t stall_cnt;      /* 连续接近 0 速度的拍数 */
    uint8_t  stall_retry;    /* 卡住重试次数 */
} LaunchState;

/* 单轮开环动作（原地转 360° / 脱线直线 1 m）的共享状态 */
typedef struct {
    uint8_t  mode;           /* 1 = 该动作正在执行 */
    float    yaw;            /* 本次动作的陀螺积分，° */
    float    speed;          /* 直线动作的斜坡速度目标 */
    uint32_t dist;           /* 直线动作已走 counts */
    uint16_t ticks;          /* 100 Hz 拍数 */
} ManeuverState;

typedef struct {
    volatile uint8_t run_flag;  /* 0 = 停车，1 = 发车 */
    float    speed_cmd;         /* 方向环给出的基础速度 */
    uint16_t lost_cnt;          /* 丢线计数，100 Hz */

    LaunchState   launch;
    ManeuverState spin;
    ManeuverState straight;
} CarState;

#endif /* CAR_STATE_H */
