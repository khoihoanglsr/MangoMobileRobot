/*
 * encoder_speed.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */
#include "encoder_speed.h"
#include "robot_params.h"
#include <math.h>

// ===== Local variables =====
static int32_t prev_ext_1 = 0;
static int32_t prev_ext_2 = 0;

void EncoderSpeed_Init(void)
{
    prev_ext_1 = Encoder_GetCount(ENCODER_1);
    prev_ext_2 = Encoder_GetCount(ENCODER_2);
}

// --------------------------------------
//  Quãng đường tính từ lúc Reset
// --------------------------------------
float Encoder_GetPosition_m(EncoderId id)
{
    int32_t count = Encoder_GetCount(id);

    float rev = (float)count / TICKS_PER_REV;
    float dist = rev * WHEEL_CIRC_M;

    return dist;
}

// --------------------------------------
//   Tốc độ m/s (gọi mỗi vòng lặp)
// --------------------------------------
float Encoder_GetSpeed_mps(EncoderId id, float dt)
{
    int32_t now = Encoder_GetCount(id);
    int32_t prev = (id == ENCODER_1 ? prev_ext_1 : prev_ext_2);

    int32_t delta = now - prev;

    if (id == ENCODER_1) prev_ext_1 = now;
    else                 prev_ext_2 = now;

    float rev_per_sec = ((float)delta / dt) / TICKS_PER_REV;
    float speed_mps   = rev_per_sec * WHEEL_CIRC_M;

    return speed_mps;
}
