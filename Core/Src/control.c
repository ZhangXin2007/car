/**
 * @file    control.c
 * @brief   1 kHz 控制层：传感器 -> 策略 -> 左右目标速度 -> 电机输出
 *
 * 拆分前这些代码全部内联在 main.c 的 Control_Update() 里（约 260 行）。
 * 现在按职责分成：
 *   Control_Direction()  扫描 -> 误差 -> 方向环 -> 过弯/六边形仲裁 -> 左右目标
 *   Control_Speed()      编码器增量 -> 前馈 + 速度环 PID -> Motor_Output
 * 以及五个特殊动作：起步 kick、卡死恢复、原地转 360°、脱线直线 1 m、丢线停车。
 *
 * 【行为等价】本次只做搬运，不改任何状态迁移、限幅与调用顺序：
 *   - 方向环/速度环仍是每 10 个 1 kHz 拍执行一次，且对齐在同一拍；
 *   - 丢线停车 / 卡死停车仍会中断本拍后续逻辑（等价于原来的 return）；
 *   - 未发车时仍每 1 ms 强制写 0 并复位 kick。
 *
 * 设计备忘（0/40 方案、FF/死区标定、kick 微调）与已知隐患清单：
 * 见工程根目录 CONTROL_NOTES.md。
 */
#include "main.h"
#include "control.h"
#include "car_config.h"
#include "car_state.h"
#include "gyro.h"
#include "motor.h"
#include "sensor.h"
#include "special.h"

/* htim3 / htim4 定义在 main.c。main.h 没有 extern 声明，这里显式引用。 */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* ---- 跨拍状态（实例私有，只在本文件可见） ---- */
static CarState car;
static CornerState corner;
static HexState hex;

/* ---- 编码器 ---- */
static int16_t last_L = 0, last_R = 0;
static int16_t last_valid_dL = 0, last_valid_dR = 0;

/* ---- 调试快照：中断写、主循环读，只做显示用 ---- */
static volatile int16_t dbg_dL = 0, dbg_dR = 0;
static volatile float   dbg_speed_L = 0.0f, dbg_speed_R = 0.0f;

/* ================= 停车 / 发车 ================= */

static void Launch_Reset(void)
{
    car.launch.done = 0U;
    car.launch.kick_on = 0U;
    car.launch.kick_phase = 0U;
    car.launch.kick_total = 0U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;
}

/* 停车：清运行标志 + 复位目标和积分。
   复位目标是为了避免"停车那一刻目标停在左转状态，下次发车直接窜出去"。
   幂等：丢线、卡死、动作完成都走这里。 */
static void Car_Stop(void)
{
    car.run_flag = 0U;
    pid_L.target = BASE_SPEED;
    pid_R.target = BASE_SPEED;
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    car.speed_cmd = 0.0f;
    Launch_Reset();
    car.lost_cnt = 0U;
    Corner_Reset(&corner);
    Hex_Reset(&hex);

    car.spin.mode = 0U;
    car.spin.yaw = 0.0f;
    car.spin.ticks = 0U;

    car.straight.mode = 0U;
    car.straight.yaw = 0.0f;
    car.straight.speed = 0.0f;
    car.straight.dist = 0U;
    car.straight.ticks = 0U;
}

void Control_Start(void)
{
    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    car.speed_cmd = BASE_SPEED;
    Launch_Reset();
    car.run_flag = 1U;
}

uint8_t Control_IsRunning(void)
{
    return (car.run_flag != 0U) ? 1U : 0U;
}

/* ================= 原地转 360° ================= */

void Control_StartSpin(void)
{
    car.run_flag = 1U;
    car.launch.done = 1U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    car.speed_cmd = 0.0f;

    car.spin.yaw = 0.0f;
    car.spin.ticks = 0U;
    car.straight.mode = 0U;
    car.spin.mode = 1U;
}

/* 方向环 100 Hz 调用：陀螺积分 + 角度闭环 */
static void Spin_Update(void)
{
    float yaw_abs;
    float err;
    float cmd;

    if (!Gyro_IsOk())
    {
        Car_Stop();
        return;
    }

    car.spin.yaw += Gyro_GetZ() * 0.01f;   /* 100 Hz，dt = 10 ms */
    ++car.spin.ticks;

    yaw_abs = (car.spin.yaw >= 0.0f) ? car.spin.yaw : -car.spin.yaw;

    if (yaw_abs >= SPIN_STOP_DEG || car.spin.ticks >= SPIN_TIMEOUT)
    {
        Car_Stop();
        return;
    }

    err = SPIN_TARGET_DEG - yaw_abs;

    if (car.spin.ticks <= SPIN_KICK_TICKS)
    {
        /* 前 180 ms 硬踹一下，打破静摩擦 */
        cmd = SPIN_KICK;
    }
    else
    {
        cmd = SPIN_KP * err;
        if (cmd > SPIN_MAX) cmd = SPIN_MAX;
        if (cmd < SPIN_MIN) cmd = SPIN_MIN;
    }

    pid_L.target = -cmd;   /* 左轮反转 */
    pid_R.target =  cmd;   /* 右轮正转 */
}

/* ================= 脱线直线 1 m ================= */

void Control_StartStraight(void)
{
    car.run_flag = 1U;
    car.launch.done = 1U;
    car.launch.settle_timer = 0U;
    car.launch.stall_cnt = 0U;
    car.launch.stall_retry = 0U;

    pid_L.integral = 0.0f;
    pid_R.integral = 0.0f;
    dir_pid.integral = 0.0f;
    pid_L.target = 0.0f;
    pid_R.target = 0.0f;
    car.speed_cmd = 0.0f;

    car.spin.mode = 0U;
    car.straight.yaw = 0.0f;
    car.straight.speed = 0.0f;
    car.straight.dist = 0U;
    car.straight.ticks = 0U;
    car.straight.mode = 1U;
}

/* 方向环 100 Hz 调用：陀螺航向保持 + 编码器距离 */
static void Straight_Update(void)
{
    float remain;
    float desired;
    float corr = 0.0f;

    ++car.straight.ticks;

    if (car.straight.dist >= STRAIGHT_TARGET_COUNTS ||
        car.straight.ticks >= STRAIGHT_TIMEOUT)
    {
        Car_Stop();
        return;
    }

    if (Gyro_IsOk())
        car.straight.yaw += Gyro_GetZ() * 0.01f;

    remain = (float)STRAIGHT_TARGET_COUNTS - (float)car.straight.dist;
    if (remain < 0.0f) remain = 0.0f;

    desired = STRAIGHT_SPEED;
    if (remain < (float)STRAIGHT_DECEL_COUNTS)
    {
        desired = STRAIGHT_MIN_SPEED +
                  (STRAIGHT_SPEED - STRAIGHT_MIN_SPEED) *
                  (remain / (float)STRAIGHT_DECEL_COUNTS);
    }

    if (car.straight.speed < desired) car.straight.speed += 0.5f;
    else                             car.straight.speed -= 0.5f;
    if (car.straight.speed < 0.0f) car.straight.speed = 0.0f;

    if (Gyro_IsOk())
    {
        corr = STRAIGHT_KP * (0.0f - car.straight.yaw)
             - STRAIGHT_KD * Gyro_GetZ();
        if (corr >  STRAIGHT_TURN_MAX) corr =  STRAIGHT_TURN_MAX;
        if (corr < -STRAIGHT_TURN_MAX) corr = -STRAIGHT_TURN_MAX;
    }

    pid_L.target = car.straight.speed - corr;
    pid_R.target = car.straight.speed + corr;
    if (pid_L.target < 0.0f) pid_L.target = 0.0f;
    if (pid_R.target < 0.0f) pid_R.target = 0.0f;
}

/* ================= 方向环 ================= */

/* 返回 1 = 本拍已丢线停车，Control_Update 必须立刻退出（等价于原来的 return）。 */
static uint8_t Control_Direction(void)
{
    LineSample line;
    HexOutput  hex_out;
    float      pid_turn = 0.0f;
    float      speed_local;
    float      turn;
    uint8_t    have_control;
    uint8_t    corner_was_active;

    /* 特殊动作只更新目标，其余巡线逻辑全部跳过 */
    if (car.spin.mode)
    {
        Spin_Update();
        return 0U;
    }
    if (car.straight.mode)
    {
        Straight_Update();
        return 0U;
    }

    /* 1) 12 路 -> 偏差 */
    Sensor_Scan(&line);

    /* 2) 方向环 PID + 二次项，先算出来交给 special 决定是否锁存 */
    if (line.cnt > 0U)
    {
        pid_turn = DirPID_Calc(&dir_pid, line.error);
        pid_turn += DIR_QUAD * (-line.error) * ((line.error >= 0.0f) ? line.error : -line.error);
        if (pid_turn >  TURN_MAX) pid_turn =  TURN_MAX;
        if (pid_turn < -TURN_MAX) pid_turn = -TURN_MAX;
    }

    /* 3) 丢线保护：过弯锁存期间不因线暂时消失而停 */
    if (line.cnt == 0U && !Corner_IsActive(&corner))
    {
        if (++car.lost_cnt >= LOST_STOP_TICKS)
        {
            Car_Stop();
            return 1U;
        }
    }
    else
    {
        car.lost_cnt = 0U;
    }

    /* 4) 六边形检测先跑，会取消通用过弯锁存 */
    hex_out = Hex_Update(&hex, &corner,
                         line.center, line.right, line.left, line.mask);

    corner_was_active = Corner_IsActive(&corner);

    speed_local = BASE_SPEED;
    turn = Corner_Update(&corner, pid_turn, line.error, line.cnt,
                         Gyro_IsOk(), Gyro_GetZ(), &speed_local);
    have_control = (line.cnt > 0U || corner_was_active ||
                    Corner_IsActive(&corner)) ? 1U : 0U;
    car.speed_cmd = speed_local;

    /* 六边形满舵覆盖通用过弯 */
    if (hex_out.have_control)
    {
        turn = hex_out.turn;
        car.speed_cmd = hex_out.speed_cmd;
        have_control = 1U;
    }

    /* 5) 差速 -> 左右目标速度（过弯/正常统一在这里写） */
    if (have_control)
    {
        pid_L.target = car.speed_cmd - turn;
        pid_R.target = car.speed_cmd + turn;
        if (pid_L.target < 0.0f) pid_L.target = 0.0f;
        if (pid_R.target < 0.0f) pid_R.target = 0.0f;
    }

    return 0U;
}

/* ================= 速度环 ================= */

/* 返回 1 = 连续卡死已停车，Control_Update 必须立刻退出（等价于原来的 return）。 */
static uint8_t Control_Speed(void)
{
    int16_t now_L;
    int16_t now_R;
    int16_t dL;
    int16_t dR;
    int32_t dL_abs;
    int32_t dR_abs;
    float   speed_L;
    float   speed_R;

    now_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    now_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    dL = (int16_t)(now_L - last_L);
    dR = (int16_t)(now_R - last_R);
    last_L = now_L;
    last_R = now_R;

    /* 坏采样保护：超过 ENC_DELTA_LIMIT 就沿用上一次有效值 */
    if (dL > ENC_DELTA_LIMIT || dL < -ENC_DELTA_LIMIT)
        dL = last_valid_dL;
    else
        last_valid_dL = dL;

    if (dR > ENC_DELTA_LIMIT || dR < -ENC_DELTA_LIMIT)
        dR = last_valid_dR;
    else
        last_valid_dR = dR;

    dbg_dL = dL;
    dbg_dR = dR;

    /* 距离累计：左右轮平均路径，单位 counts */
    dL_abs = (dL < 0) ? -(int32_t)dL : (int32_t)dL;
    dR_abs = (dR < 0) ? -(int32_t)dR : (int32_t)dR;
    Hex_AddDistance(&hex, (uint32_t)((dL_abs + dR_abs) / 2));
    car.straight.dist += (uint32_t)((dL_abs + dR_abs) / 2);

    /* PID 实际使用的速度：保持当前符号（左轮反转计数，所以取负） */
    speed_L = -(float)dL;          /* TIM3 左 */
    speed_R =  (float)dR;          /* TIM4 右 */

    /* 停车推车时也更新，供主循环打印 */
    dbg_speed_L = speed_L;
    dbg_speed_R = speed_R;

    /* 未发车只采样、不跑 PID，避免停车时积分乱积。
       电机由 Control_Update 末尾的 !run_flag 分支强制写 0。 */
    if (car.run_flag)
    {
        /* speed_cmd 由方向环按 turn 更新（弯道降速），这里不要再覆盖 */

        if (!car.launch.done && !car.spin.mode && !car.straight.mode)
        {
            /* 阶段 1：短脉冲 kick：60 ms on / 60 ms off，
               直到两轮都 > 4，或总时长 600 ms。 */
            if (speed_L > KICK_EXIT_SPEED && speed_R > KICK_EXIT_SPEED)
            {
                car.launch.done = 1U;
                car.launch.settle_timer = SETTLE_TICKS;
            }
            else if (car.launch.kick_total < KICK_MAX_TICKS)
            {
                if (car.launch.kick_phase == 0U)
                {
                    car.launch.kick_on = (car.launch.kick_on == 0U) ? 1U : 0U;
                    car.launch.kick_phase = (car.launch.kick_on != 0U) ? KICK_ON_TICKS : KICK_OFF_TICKS;
                }
                --car.launch.kick_phase;
                ++car.launch.kick_total;

                if (car.launch.kick_on != 0U) Motor_Output(KICK_L, KICK_R);
                else                          Motor_Output(0.0f, 0.0f);
            }
            else
            {
                car.launch.done = 1U;
                car.launch.settle_timer = SETTLE_TICKS;
            }
        }

        /* 中途卡住：速度接近 0 持续 300 ms，重新脉冲 kick。
           spin / straight 模式不要走这个逻辑，否则会两轮正转把车往前踹。 */
        if (car.launch.done && !car.spin.mode && !car.straight.mode)
        {
            if (speed_L < STALL_SPEED && speed_R < STALL_SPEED)
            {
                if (++car.launch.stall_cnt >= STALL_TICKS)
                {
                    car.launch.stall_cnt = 0U;
                    if (car.launch.stall_retry < STALL_RETRIES)
                    {
                        ++car.launch.stall_retry;
                        car.launch.done = 0U;
                        car.launch.kick_on = 0U;
                        car.launch.kick_phase = 0U;
                        car.launch.kick_total = 0U;
                        car.launch.settle_timer = 0U;
                        Motor_Output(0.0f, 0.0f);
                    }
                    else
                    {
                        /* 连续 3 次还起不来，停车保护 */
                        Car_Stop();
                        return 1U;
                    }
                }
            }
            else
            {
                car.launch.stall_cnt = 0U;
                if (speed_L > 10.0f && speed_R > 10.0f)
                    car.launch.stall_retry = 0U;
            }
        }

        if (car.launch.done)
        {
            /* 如果已经明显超速，跳过 FF 保持，直接交给 PID 平滑降输出。
               不要用"超过阈值就 0 占空比"的硬刹车，否则 CCR 会在 0 和前馈值
               之间每 10 ms 跳，看起来就是"非常抖动"。 */
            if (car.launch.settle_timer > 0U &&
                speed_L < BASE_SPEED + 5.0f &&
                speed_R < BASE_SPEED + 5.0f)
            {
                --car.launch.settle_timer;
                Motor_Output(FF_GAIN_L * pid_L.target, FF_GAIN_R * pid_R.target);
            }
            else
            {
                Motor_Output(FF_GAIN_L * pid_L.target + PID_Calc(&pid_L, speed_L),
                             FF_GAIN_R * pid_R.target + PID_Calc(&pid_R, speed_R));
            }
        }
    }

    return 0U;
}

/* ================= 1 kHz 调度 ================= */

void Control_Update(void)
{
    static uint16_t div_gyro = 0U;
    static uint16_t div_dir = 0U;
    static uint16_t div_speed = 0U;

    /* 陀螺仪采样：200 Hz（每 5 ms）。
       放中断里是为了周期严格；放主循环会被 printf 阻塞抖到好几毫秒。 */
    if (++div_gyro >= 5U)
    {
        div_gyro = 0U;
        Gyro_Task();
    }

    /* 方向环：100 Hz（10 ms） */
    if (++div_dir >= 10U)
    {
        div_dir = 0U;
        if (Control_Direction())
            return;
    }

    /* 速度环：100 Hz（10 ms） */
    if (++div_speed >= SPEED_DIV)
    {
        div_speed = 0U;
        if (Control_Speed())
            return;
    }

    /* 未发车：电机停转。
       注意：这里不要再刷新 last_L/last_R！
       它们只在 10 ms 采样里更新，所以不发车也能看到真实的推车速度。 */
    if (!car.run_flag)
    {
        Motor_Output(0.0f, 0.0f);
        car.speed_cmd = 0.0f;
        Launch_Reset();               /* 下次发车重新 kick */
        return;
    }
}

/* ================= 调试快照 ================= */

void Control_GetDebug(ControlDebug *d)
{
    d->speed_cmd = car.speed_cmd;
    d->speed_L   = dbg_speed_L;
    d->speed_R   = dbg_speed_R;
    d->dL        = dbg_dL;
    d->dR        = dbg_dR;
    d->target_L  = pid_L.target;
    d->target_R  = pid_R.target;
    d->gyro_z    = Gyro_GetZ();
    d->run       = car.run_flag;
}

void Control_GetHexDebug(HexDebug *d)
{
    Hex_GetDebug(&hex, d);
}
