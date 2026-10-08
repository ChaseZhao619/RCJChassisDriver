#include "soccer_predict.h"

#include <math.h>

void SoccerBall_Update(SoccerBallEstimate *ball, float dt_s,
                       unsigned char observed, float x_m, float y_m)
{
    float ex, ey;
    if ((ball == 0) || !(dt_s > 0.0f) || (dt_s > 0.1f)) return;
    if (!ball->initialized) {
        if (!observed || !isfinite(x_m) || !isfinite(y_m)) return;
        ball->x_m = x_m;
        ball->y_m = y_m;
        ball->vx_m_s = ball->vy_m_s = 0.0f;
        ball->age_s = 0.0f;
        ball->initialized = 1U;
        return;
    }
    ball->x_m += ball->vx_m_s * dt_s;
    ball->y_m += ball->vy_m_s * dt_s;
    ball->age_s += dt_s;
    if (!observed || !isfinite(x_m) || !isfinite(y_m)) return;
    ex = x_m - ball->x_m;
    ey = y_m - ball->y_m;
    if (hypotf(ex, ey) > 1.0f) {
        ball->x_m = x_m;
        ball->y_m = y_m;
        ball->vx_m_s = ball->vy_m_s = 0.0f;
    } else {
        ball->x_m += 0.65f * ex;
        ball->y_m += 0.65f * ey;
        ball->vx_m_s += 0.08f * ex / dt_s;
        ball->vy_m_s += 0.08f * ey / dt_s;
    }
    ball->age_s = 0.0f;
}

AppLqrReference SoccerBall_Intercept(const SoccerBallEstimate *ball,
                                     const AppLqrState *robot)
{
    AppLqrReference ref = {0};
    float distance, lead;
    if ((ball == 0) || (robot == 0) || !ball->initialized || ball->age_s > 0.5f)
        return ref;
    distance = hypotf(ball->x_m - robot->x_m, ball->y_m - robot->y_m);
    lead = distance / APP_LQR_VELOCITY_LIMIT_M_S;
    if (lead > 1.5f) lead = 1.5f;
    ref.x_m = ball->x_m + lead * ball->vx_m_s;
    ref.y_m = ball->y_m + lead * ball->vy_m_s;
    ref.yaw_rad = robot->yaw_rad;
    ref.track_x = ref.track_y = 1U;
    return ref;
}
