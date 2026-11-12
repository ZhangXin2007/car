#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ============================================================
 * 小车参数集中配置
 * 说明：这里只放“可调参数”，不放硬件句柄和函数实现。
 * 后续 main.c / motor.c / special.c / timebase.c 共用本文件。
 * ============================================================ */

/* ---- PWM / 速度环基础 ---- */
#define PWM_MAX                 6000.0f
#define SPEED_DIV               10U      /* 1 kHz / 10 = 100 Hz 速度环 */
#define INTEG_LIMIT             30000.0f

/* ---- 直道 / 方向环 ---- */
#define BASE_SPEED              40.0f    /* 脉冲/10 ms */
#define TURN_MAX                25.0f
#define CURVE_SLOW              0.5f
#define MIN_CURVE_SPEED         20.0f
#define DIR_QUAD                0.21f

/* ---- 通用过弯锁存 ---- */
#define CORNER_ENTER_ERR        3.0f
#define CORNER_ENTER_TURN       8.0f
#define CORNER_SPEED            25.0f
#define CORNER_YAW_EXIT         70.0f
#define CORNER_MIN_TICKS        12U      /* 120 ms */
#define CORNER_MAX_TICKS        40U      /* 400 ms */
#define CORNER_COOLDOWN         20U      /* 200 ms */
#define CORNER_RAMP_TICKS       15U      /* 150 ms */

/* ---- 六边形入口 ---- */
#define COUNTS_PER_CM           435.7f
#define HEX_CLEAR_CM            0.0f
#define HEX_CLEAR_COUNTS        ((uint32_t)(COUNTS_PER_CM * HEX_CLEAR_CM))
#define HEX_OFF_TICKS           1U
#define HEX_PRE_TURN_TICKS      20U
#define HEX_PRE_TURN            16.0f
#define HEX_PRE_SPEED           20.0f
#define HEX_TURN_TICKS          15U
#define HEX_ARM_TIMEOUT         500U

/* ---- 原地转 360°（90/180/270/360 分段） ---- */
#define SPIN_TARGET_DEG         360.0f
#define SPIN_STOP_DEG           358.0f
#define SPIN_KP                 0.6f
#define SPIN_MAX                35.0f
#define SPIN_MIN                30.0f
#define SPIN_KICK               42.0f
#define SPIN_KICK_TICKS         18U
#define SPIN_TIMEOUT            1000U

/* ---- 脱线直线 1 m ---- */
#define STRAIGHT_TARGET_COUNTS  43570U
#define STRAIGHT_SPEED          60.0f
#define STRAIGHT_MIN_SPEED      20.0f
#define STRAIGHT_DECEL_COUNTS   2178U
#define STRAIGHT_KP             1.0f
#define STRAIGHT_KD             0.1f
#define STRAIGHT_TURN_MAX       10.0f
#define STRAIGHT_TIMEOUT        1000U

/* ---- 前馈 / 死区 ---- */
#define FF_GAIN_L               14.5f
#define FF_GAIN_R               15.4f
#define MOTOR_MIN_DUTY_L        150.0f
#define MOTOR_MIN_DUTY_R        169.0f

/* ---- 起步 kick ---- */
#define KICK_L                  390.0f
#define KICK_R                  428.0f
#define KICK_ON_TICKS           6U
#define KICK_OFF_TICKS          6U
#define KICK_MAX_TICKS          60U
#define KICK_EXIT_SPEED         4.0f
#define SETTLE_TICKS            15U

/* ---- 卡死恢复 ---- */
#define STALL_SPEED             2.0f
#define STALL_TICKS             30U
#define STALL_RETRIES           3U

/* ---- 丢线 / 编码器保护 ---- */
#define LOST_STOP_TICKS         150U
#define ENC_DELTA_LIMIT         300

/* ---- 低速增益调度 ---- */
#define KP_LOW_SPEED            10.0f
#define KP_BOOST                2.5f

/* ---- 陀螺 ---- */
#define GYRO_ALPHA              0.3f

#endif /* CAR_CONFIG_H */
