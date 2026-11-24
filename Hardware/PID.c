#include "PID.h"

static float Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float WithOffset(float output, float offset)
{
    if (output > 0.0f) {
        return output + offset;
    }
    if (output < 0.0f) {
        return output - offset;
    }
    return output;
}

void PID_Init(PID_t *p)
{
    *p = (PID_t){0};
}

void PID_Update(PID_t *p)
{
    float d_raw;
    float i_step;
    float i_next;
    float i_min = p->ILimit > 0.0f ? -p->ILimit : p->OutMin;
    float i_max = p->ILimit > 0.0f ? p->ILimit : p->OutMax;
    float without_i;
    float candidate;

    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    if (p->Error0 > -p->DeadBand && p->Error0 < p->DeadBand) {
        p->Error0 = 0.0f;
    }
    p->POut = p->Kp * p->Error0;
    d_raw = -p->Kd * (p->Actual - p->Actual1);
    p->DOut = p->DAlpha * p->DOut1 + (1.0f - p->DAlpha) * d_raw;
    p->DOut1 = p->DOut;
    p->Actual1 = p->Actual;

    if (p->Ki == 0.0f) {
        p->IOut = 0.0f;
    }
    i_step = p->Ki * p->Error0;
    i_next = Clamp(p->IOut + i_step, i_min, i_max);
    without_i = p->FeedForward + p->POut + p->DOut;
    candidate = WithOffset(without_i + i_next, p->OutOffset);

    /* Freeze integration only when it would drive a saturated output farther. */
    if (!((candidate > p->OutMax && i_step > 0.0f) ||
          (candidate < p->OutMin && i_step < 0.0f))) {
        p->IOut = i_next;
    }
    p->Out = Clamp(WithOffset(without_i + p->IOut, p->OutOffset),
                   p->OutMin, p->OutMax);
}
