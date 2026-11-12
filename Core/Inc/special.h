#ifndef SPECIAL_H
#define SPECIAL_H

#include <stdint.h>

/* 通用过弯锁存状态 */
typedef struct {
    uint8_t  state;          /* 0=正常巡线，1=过弯保持 */
    float    dir;            /* +1 左转，-1 右转 */
    float    yaw;            /* 本次过弯陀螺积分 */
    uint16_t ticks;          /* 保持拍数，100 Hz */
    uint16_t cooldown;       /* 出弯冷却，100 Hz */
    uint16_t ramp_ticks;     /* 出弯速度斜坡 */
} CornerState;

/* 六边形入口状态 */
typedef struct {
    uint8_t  right_prev;
    uint16_t right_off_cnt;
    uint8_t  left_prev;
    uint16_t left_off_cnt;

    float    dir;            /* -1 右转，+1 左转 */
    uint16_t event_cnt;
    uint16_t arm_ticks;
    uint16_t preturn_ticks;
    uint16_t turn_ticks;
    uint32_t dist;
    uint16_t evt_seq;

    uint8_t  dbg_event;
    uint32_t dbg_dist;
    uint8_t  dbg_right;
    uint8_t  dbg_left;
    uint8_t  dbg_center;
    uint16_t dbg_mask;
} HexState;

typedef struct {
    float    turn;
    float    speed_cmd;
    uint8_t  have_control;
} HexOutput;

typedef struct {
    uint16_t evt_seq;
    uint8_t  event;
    uint32_t event_dist;
    uint8_t  right;
    uint8_t  left;
    uint8_t  center;
    uint16_t mask;
    uint16_t event_cnt;
    uint16_t off_r;
    uint16_t off_l;
    uint32_t dist;
    uint16_t arm;
} HexDebug;

void Corner_Reset(CornerState *c);
uint8_t Corner_IsActive(const CornerState *c);

/* 返回最终 turn；speed_cmd 由函数写入。
   pid_turn 是方向环 PID + 二次项、限幅后的值。 */
float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint8_t gyro_ok,
                    float gyro_z,
                    float *speed_cmd);

void Hex_Reset(HexState *h);
void Hex_AddDistance(HexState *h, uint32_t counts);
HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask);
void Hex_GetDebug(const HexState *h, HexDebug *d);

#endif /* SPECIAL_H */
