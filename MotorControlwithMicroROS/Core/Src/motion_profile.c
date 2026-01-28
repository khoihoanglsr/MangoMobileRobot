#include "motion_profile.h"
#include <math.h>

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float signf_nonzero(float x) {
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

void MP_Init(MP_State_t* st, float init_pos, float init_vel) {
    if (!st) return;
    st->pos_ref = init_pos;
    st->vel_ref = init_vel;
}

MP_Output_t MP_StepVelocity(const MP_Config_t* cfg, MP_State_t* st, float v_des)
{
    MP_Output_t out = {0};

    if (!cfg || !st || cfg->dt <= 0.0f || cfg->amax <= 0.0f || cfg->vmax <= 0.0f) {
        out.pos_next = st ? st->pos_ref : 0.0f;
        out.vel_next = st ? st->vel_ref : 0.0f;
        out.acc_used = 0.0f;
        out.reached  = 0;
        return out;
    }

    // Clamp desired velocity
    v_des = clampf(v_des, -cfg->vmax, cfg->vmax);

    float v = st->vel_ref;
    float dv = v_des - v;

    // Acceleration-limited ramp
    float a_cmd;
    float max_dv = cfg->amax * cfg->dt;

    if (fabsf(dv) <= max_dv) {
        a_cmd = dv / cfg->dt;  // exactly reach v_des this step
        v = v_des;
        out.reached = 1;
    } else {
        a_cmd = signf_nonzero(dv) * cfg->amax;
        v = v + a_cmd * cfg->dt;
        out.reached = 0;
    }

    // Safety clamp
    v = clampf(v, -cfg->vmax, cfg->vmax);

    // Integrate position reference using updated velocity
    float p = st->pos_ref + v * cfg->dt;

    st->vel_ref = v;
    st->pos_ref = p;

    out.vel_next = v;
    out.pos_next = p;
    out.acc_used = a_cmd;

    return out;
}

MP_Output_t MP_StepPosition(const MP_Config_t* cfg, MP_State_t* st, float pos_des)
{
    MP_Output_t out = {0};

    if (!cfg || !st || cfg->dt <= 0.0f || cfg->amax <= 0.0f || cfg->vmax <= 0.0f) {
        out.pos_next = st ? st->pos_ref : 0.0f;
        out.vel_next = st ? st->vel_ref : 0.0f;
        out.acc_used = 0.0f;
        out.reached  = 0;
        return out;
    }

    float p = st->pos_ref;
    float v = st->vel_ref;

    float dist = pos_des - p;                // remaining distance
    float dir  = (fabsf(dist) < 1e-12f) ? 1.0f : signf_nonzero(dist);

    // If already close enough and slow enough -> snap to target
    if (fabsf(dist) <= cfg->pos_eps && fabsf(v) <= cfg->vel_eps) {
        st->pos_ref = pos_des;
        st->vel_ref = 0.0f;
        out.pos_next = pos_des;
        out.vel_next = 0.0f;
        out.acc_used = 0.0f;
        out.reached  = 1;
        return out;
    }

    // Stopping distance needed to brake to 0: d_stop = v^2 / (2*amax)
    float d_stop = (v * v) / (2.0f * cfg->amax);

    // Decide accelerate or decelerate:
    // - If we are close enough that we must brake -> decelerate opposite to current velocity (or toward stopping)
    // - Else accelerate toward target direction
    float a_cmd = 0.0f;

    // If moving away from target, immediately accelerate back toward target
    if (v * dir < -cfg->vel_eps) {
        a_cmd = dir * cfg->amax;
    } else {
        // Need to brake if remaining distance is smaller than stopping distance
        if (fabsf(dist) <= d_stop) {
            // Brake to stop at target
            if (fabsf(v) > cfg->vel_eps) {
                a_cmd = -signf_nonzero(v) * cfg->amax;
            } else {
                // Very small v, just push toward target
                a_cmd = dir * cfg->amax;
            }
        } else {
            // Plenty of distance -> accelerate toward target
            a_cmd = dir * cfg->amax;
        }
    }

    // Integrate velocity
    float v_next = v + a_cmd * cfg->dt;
    v_next = clampf(v_next, -cfg->vmax, cfg->vmax);

    // Integrate position (using new velocity)
    float p_next = p + v_next * cfg->dt;

    // Anti-overshoot snap:
    // If we crossed the target this step, snap to target and stop.
    float dist_next = pos_des - p_next;
    if ((dist * dist_next) < 0.0f) { // sign changed => crossed
        p_next = pos_des;
        v_next = 0.0f;
        a_cmd  = 0.0f;
        out.reached = 1;
    } else {
        out.reached = 0;
    }

    st->pos_ref = p_next;
    st->vel_ref = v_next;

    out.pos_next = p_next;
    out.vel_next = v_next;
    out.acc_used = a_cmd;

    return out;
}
