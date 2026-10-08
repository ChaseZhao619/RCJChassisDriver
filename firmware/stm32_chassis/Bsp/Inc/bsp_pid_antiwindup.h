#ifndef BSP_PID_ANTIWINDUP_H
#define BSP_PID_ANTIWINDUP_H

static inline float BspPid_NextIntegral(float integral, float error, float dt,
                                        float integral_limit, float feedforward,
                                        float kp, float ki, float kd, float derivative,
                                        float output_limit)
{
    float candidate = integral + error * dt;
    float output;
    if (candidate > integral_limit) candidate = integral_limit;
    if (candidate < -integral_limit) candidate = -integral_limit;
    output = feedforward + kp * error + ki * candidate + kd * derivative;
    if ((output_limit > 0.0f) &&
        ((output > output_limit && error > 0.0f) ||
         (output < -output_limit && error < 0.0f)))
        return integral;
    return candidate;
}

#endif
