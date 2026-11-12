#include "special.h"
#include "car_config.h"

void Corner_Reset(CornerState *c)
{
    c->state = 0U;
    c->dir = 1.0f;
    c->yaw = 0.0f;
    c->ticks = 0U;
    c->cooldown = 0U;
    c->ramp_ticks = 0U;
}

uint8_t Corner_IsActive(const CornerState *c)
{
    return (c->state != 0U) ? 1U : 0U;
}

float Corner_Update(CornerState *c,
                    float pid_turn,
                    float error,
                    int cnt,
                    uint8_t gyro_ok,
                    float gyro_z,
                    float *speed_cmd)
{
    /* ---- 过弯保持 ---- */
    if (c->state != 0U)
    {
        if (gyro_ok)
            c->yaw += gyro_z * 0.01f;

        ++c->ticks;

        *speed_cmd = CORNER_SPEED;

        float abs_yaw = (c->yaw >= 0.0f) ? c->yaw : -c->yaw;
        float abs_err = (error >= 0.0f) ? error : -error;

        uint8_t line_back = (cnt > 0 && abs_err < 2.5f);
        uint8_t yaw_done  = (gyro_ok && abs_yaw >= CORNER_YAW_EXIT);
        uint8_t time_done = (c->ticks >= CORNER_MAX_TICKS);
        uint8_t can_exit  = (c->ticks >= CORNER_MIN_TICKS);

        if (can_exit &&
            (yaw_done ||
             (line_back && (gyro_ok ? (abs_yaw >= 45.0f) : (c->ticks >= 20U))) ||
             time_done))
        {
            c->state = 0U;
            c->cooldown = CORNER_COOLDOWN;
            c->ramp_ticks = CORNER_RAMP_TICKS;
        }

        return c->dir * TURN_MAX;
    }

    /* ---- 正常巡线 ---- */
    if (c->cooldown > 0U)
        --c->cooldown;

    if (cnt == 0)
    {
        *speed_cmd = BASE_SPEED;
        return pid_turn;
    }

    /* 大误差/大 turn 进入过弯锁存 */
    {
        float abs_err  = (error >= 0.0f) ? error : -error;
        float abs_turn = (pid_turn >= 0.0f) ? pid_turn : -pid_turn;

        if ((abs_err > CORNER_ENTER_ERR || abs_turn > CORNER_ENTER_TURN)
            && c->cooldown == 0U)
        {
            c->state = 1U;
            c->dir = (pid_turn >= 0.0f) ? 1.0f : -1.0f;
            if (abs_turn < 0.5f)
                c->dir = (error > 0.0f) ? -1.0f : 1.0f;

            c->yaw = 0.0f;
            c->ticks = 0U;
            c->ramp_ticks = 0U;

            *speed_cmd = CORNER_SPEED;
            return c->dir * TURN_MAX;
        }
    }

    /* 正常弯道降速 + 出弯速度斜坡 */
    {
        float abs_turn = (pid_turn >= 0.0f) ? pid_turn : -pid_turn;

        *speed_cmd = BASE_SPEED - CURVE_SLOW * abs_turn;
        if (*speed_cmd < MIN_CURVE_SPEED)
            *speed_cmd = MIN_CURVE_SPEED;

        if (c->ramp_ticks > 0U)
        {
            --c->ramp_ticks;

            float t = (float)(CORNER_RAMP_TICKS - c->ramp_ticks) /
                      (float)CORNER_RAMP_TICKS;
            float ramp_speed = CORNER_SPEED +
                               (BASE_SPEED - CORNER_SPEED) * t;

            if (*speed_cmd > ramp_speed)
                *speed_cmd = ramp_speed;
        }
    }

    return pid_turn;
}

void Hex_Reset(HexState *h)
{
    h->right_prev = 0U;
    h->right_off_cnt = 0U;
    h->left_prev = 0U;
    h->left_off_cnt = 0U;

    h->dir = -1.0f;
    h->event_cnt = 0U;
    h->arm_ticks = 0U;
    h->preturn_ticks = 0U;
    h->turn_ticks = 0U;
    h->dist = 0U;
    h->evt_seq = 0U;

    h->dbg_event = 0U;
    h->dbg_dist = 0U;
    h->dbg_right = 0U;
    h->dbg_left = 0U;
    h->dbg_center = 0U;
    h->dbg_mask = 0U;
}

void Hex_AddDistance(HexState *h, uint32_t counts)
{
    h->dist += counts;
}

HexOutput Hex_Update(HexState *h,
                     CornerState *corner,
                     uint8_t center,
                     uint8_t right,
                     uint8_t left,
                     uint16_t mask)
{
    HexOutput out = { 0.0f, 0.0f, 0U };

    h->dbg_right = right;
    h->dbg_left = left;
    h->dbg_center = center;
    h->dbg_mask = mask;

    if (h->event_cnt == 1U)
    {
        if (h->arm_ticks < 60000U) ++h->arm_ticks;
        if (h->arm_ticks > HEX_ARM_TIMEOUT) h->event_cnt = 0U;
    }

    /* 右侧 ch11 上升沿 */
    if (right)
    {
        if (!h->right_prev && h->right_off_cnt >= HEX_OFF_TICKS)
        {
            if (h->event_cnt == 0U)
            {
                if (center)
                {
                    h->event_cnt = 1U;
                    h->arm_ticks = 0U;
                    h->dist = 0U;
                    h->preturn_ticks = 0U;
                    h->turn_ticks = 0U;
                    h->dir = -1.0f;
                    corner->state = 0U;

                    h->dbg_event = 1U;
                    h->dbg_dist = h->dist;
                    ++h->evt_seq;
                }
            }
            else if (h->event_cnt == 1U &&
                     h->dir < 0.0f &&
                     h->dist >= HEX_CLEAR_COUNTS)
            {
                h->event_cnt = 2U;
                h->preturn_ticks = 0U;
                h->turn_ticks = HEX_TURN_TICKS;
                corner->state = 0U;

                h->dbg_event = 2U;
                h->dbg_dist = h->dist;
                ++h->evt_seq;
            }
        }
        h->right_off_cnt = 0U;
    }
    else
    {
        if (h->right_off_cnt < 60000U) ++h->right_off_cnt;
    }
    h->right_prev = right;

    /* 左侧 ch0 上升沿 */
    if (left)
    {
        if (!h->left_prev && h->left_off_cnt >= HEX_OFF_TICKS)
        {
            if (h->event_cnt == 0U)
            {
                if (center)
                {
                    h->event_cnt = 1U;
                    h->arm_ticks = 0U;
                    h->dist = 0U;
                    h->preturn_ticks = 0U;
                    h->turn_ticks = 0U;
                    h->dir = 1.0f;
                    corner->state = 0U;

                    h->dbg_event = 1U;
                    h->dbg_dist = h->dist;
                    ++h->evt_seq;
                }
            }
            else if (h->event_cnt == 1U &&
                     h->dir > 0.0f &&
                     h->dist >= HEX_CLEAR_COUNTS)
            {
                h->event_cnt = 2U;
                h->preturn_ticks = 0U;
                h->turn_ticks = HEX_TURN_TICKS;
                corner->state = 0U;

                h->dbg_event = 3U;
                h->dbg_dist = h->dist;
                ++h->evt_seq;
            }
        }
        h->left_off_cnt = 0U;
    }
    else
    {
        if (h->left_off_cnt < 60000U) ++h->left_off_cnt;
    }
    h->left_prev = left;

    /* 满舵覆盖，优先级高于通用过弯 */
    if (h->turn_ticks > 0U)
    {
        out.turn = h->dir * TURN_MAX;
        out.speed_cmd = CORNER_SPEED;
        out.have_control = 1U;

        --h->turn_ticks;
        if (h->turn_ticks == 0U) h->event_cnt = 0U;
    }
    else if (h->preturn_ticks > 0U)
    {
        out.turn = h->dir * HEX_PRE_TURN;
        out.speed_cmd = HEX_PRE_SPEED;
        out.have_control = 1U;
        --h->preturn_ticks;
    }

    return out;
}

void Hex_GetDebug(const HexState *h, HexDebug *d)
{
    d->evt_seq = h->evt_seq;
    d->event = h->dbg_event;
    d->event_dist = h->dbg_dist;
    d->right = h->dbg_right;
    d->left = h->dbg_left;
    d->center = h->dbg_center;
    d->mask = h->dbg_mask;
    d->event_cnt = h->event_cnt;
    d->off_r = h->right_off_cnt;
    d->off_l = h->left_off_cnt;
    d->dist = h->dist;
    d->arm = h->arm_ticks;
}
