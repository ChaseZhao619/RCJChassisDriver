#ifndef SOCCER_PREDICT_H
#define SOCCER_PREDICT_H

#include "app_lqr.h"

typedef struct {
    float x_m, y_m, vx_m_s, vy_m_s;
    float age_s;
    unsigned char initialized;
} SoccerBallEstimate;

/* Host-only synthetic 2D ball observations; no BE1732 distance inference. */
void SoccerBall_Update(SoccerBallEstimate *ball, float dt_s,
                       unsigned char observed, float x_m, float y_m);
AppLqrReference SoccerBall_Intercept(const SoccerBallEstimate *ball,
                                     const AppLqrState *robot);

#endif
