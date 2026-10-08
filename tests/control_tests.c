#include "app_control_safety.h"
#include "app_lqr.h"
#include "app_velocity_command.h"
#include "soccer_predict.h"
#include "bsp_pid_antiwindup.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static float absolute(float x) { return x < 0.0f ? -x : x; }

static void step_robot(AppLqrState *state, AppLqrOutput command)
{
    const float ts = 0.01f;
    AppLqr_ModelStep(&state->x_m, &state->vx_m_s, command.vx_m_s, ts, 0.12f);
    AppLqr_ModelStep(&state->y_m, &state->vy_m_s, command.vy_m_s, ts, 0.12f);
    AppLqr_ModelStep(&state->yaw_rad, &state->wz_rad_s, command.wz_rad_s, ts, 0.10f);
    state->yaw_rad = AppLqr_WrapRadians(state->yaw_rad);
}

static void test_control_basics(void)
{
    AppLqrState state = {0};
    AppLqrReference ref = {0};
    AppLqrOutput out;
    float forward, left, yaw;
    assert(AppVelocity_Crc16Ccitt((const uint8_t *)"123456789", 9U) == 0x29B1U);
    assert(AppVelocity_Parse(" 100 -50 0.5", &forward, &left, &yaw));
    assert(forward == 100.0f && left == -50.0f && yaw == 0.5f);
    assert(!AppVelocity_Parse(" 650 650 0", &forward, &left, &yaw));
    assert(!AppVelocity_Parse(" 10 0 nan", &forward, &left, &yaw));
    assert(!AppVelocity_Parse(" 10 0 1 extra", &forward, &left, &yaw));
    {
        float p = 0.0f, v = 0.0f;
        AppLqr_ModelStep(&p, &v, 1.0f, 0.01f, 0.12f);
        assert(absolute(v - (1.0f - expf(-0.01f / 0.12f))) < 0.000001f);
        assert(p > 0.0f && p < 0.01f);
    }
    assert(absolute(AppLqr_WrapRadians(3.14159265f + 0.01f) +
                    3.14159265f - 0.01f) < 0.0001f);
    assert(AppControlSafety_IsFresh(5U, 0xfffffff0U, 30U));
    assert(!AppControlSafety_IsFresh(500U, 100U, 300U));
    assert(BspPid_NextIntegral(5.0f, 100.0f, 0.01f, 100.0f,
                               0.0f, 2.0f, 1.0f, 0.0f, 0.0f, 50.0f) == 5.0f);
    assert(BspPid_NextIntegral(5.0f, -100.0f, 0.01f, 100.0f,
                               0.0f, 2.0f, 1.0f, 0.0f, 0.0f, 50.0f) == 5.0f);
    assert(BspPid_NextIntegral(5.0f, -1.0f, 0.01f, 100.0f,
                               0.0f, 2.0f, 1.0f, 0.0f, 0.0f, 50.0f) < 5.0f);
    ref.x_m = 1.0f;
    ref.track_x = 0U;
    ref.track_yaw = 1U;
    ref.yaw_rad = 0.01f;
    out = AppLqr_Calculate(&state, &ref, 0.65f);
    assert(absolute(out.wz_rad_s - 0.03867701f) < 0.000001f);
    ref.track_x = 1U;
    ref.track_yaw = 0U;
    ref.yaw_rad = 0.0f;
    out = AppLqr_Calculate(&state, &ref, 0.35f);
    assert(out.vx_m_s > 0.0f && out.vx_m_s <= 0.35001f);
    assert(out.vy_m_s == 0.0f);
    ref.x_m = 0.01f;
    out = AppLqr_Calculate(&state, &ref, 0.65f);
    assert(absolute(out.vx_m_s - 0.04838423f) < 0.000001f);
    ref.x_m = 1.0f;
    {
        AppLqrOutput mixed = {0.65f, 0.65f, 2.0f};
        mixed = AppLqr_LimitOmni(mixed, 0.0f, 0.65f, 0.096f, -0.08f, 0.0f);
        assert(absolute(mixed.vx_m_s) +
               absolute(mixed.vy_m_s - 0.08f * mixed.vx_m_s) +
               0.096f * absolute(mixed.wz_rad_s) <= 0.65001f);
    }
    state.yaw_rad = -3.13f;
    ref.yaw_rad = 3.13f;
    ref.track_yaw = 1U;
    out = AppLqr_Calculate(&state, &ref, 0.35f);
    assert(out.wz_rad_s < 0.0f && absolute(out.wz_rad_s) < 0.2f);
    state.vx_m_s = NAN;
    out = AppLqr_Calculate(&state, &ref, 0.35f);
    assert(out.vx_m_s == 0.0f && out.wz_rad_s == 0.0f);
}

static void compare_goal(FILE *trace, int lqr)
{
    AppLqrState state = {0};
    AppLqrReference ref = {0};
    float max_x = 0.0f, effort = 0.0f;
    int k;
    ref.x_m = 1.0f;
    ref.track_x = 1U;
    for (k = 0; k < 600; k++) {
        AppLqrOutput out = {0};
        if (lqr) {
            out = AppLqr_Calculate(&state, &ref, 0.65f);
            out = AppLqr_LimitOmni(out, state.yaw_rad, 0.65f, 0.096f, -0.08f, 0.0f);
        }
        else {
            out.vx_m_s = 3.0f * (ref.x_m - state.x_m);
            if (out.vx_m_s > 0.65f) out.vx_m_s = 0.65f;
            if (out.vx_m_s < -0.65f) out.vx_m_s = -0.65f;
        }
        effort += absolute(out.vx_m_s) * 0.01f;
        step_robot(&state, out);
        if (state.x_m > max_x) max_x = state.x_m;
        fprintf(trace, "goal,%s,%.2f,%.5f,0,1,0,%.5f\n",
                lqr ? "lqr" : "baseline", k * 0.01, state.x_m, out.vx_m_s);
    }
    assert(absolute(1.0f - state.x_m) < 0.025f);
    printf("goal %s: final_error=%.4f m overshoot=%.4f m command_integral=%.4f m\n",
           lqr ? "lqr" : "baseline", absolute(1.0f - state.x_m),
           fmaxf(0.0f, max_x - 1.0f), effort);
}

static void soccer_case(FILE *trace, const char *name, int scenario)
{
    AppLqrState robot = {0};
    SoccerBallEstimate ball = {0};
    AppLqrOutput command = {0};
    float bx = 1.0f, by = 0.25f;
    unsigned int last_renew_ms = 0U;
    unsigned char motion_enabled = 1U;
    int k;
    for (k = 0; k < 800; k++) {
        float t = k * 0.01f;
        unsigned char observed = ((k % 3) == 0);
        unsigned int now_ms = (unsigned int)(k * 10);
        unsigned char link_available = !((scenario == 4) && (t >= 2.0f) && (t < 2.7f));
        AppLqrReference ref;
        if (scenario == 1) bx = 1.0f + 0.10f * t;
        if (scenario == 2 && t >= 2.0f && t < 2.7f) observed = 0U;
        if (scenario == 3 && t >= 3.0f) { bx = 1.7f; by = -0.35f; }
        SoccerBall_Update(&ball, 0.01f, observed, bx, by);
        ref = SoccerBall_Intercept(&ball, &robot);
        if (!ref.track_x) {
            ref.x_m = robot.x_m;
            ref.y_m = robot.y_m;
        }
        command = AppLqr_Calculate(&robot, &ref, 0.65f);
        command = AppLqr_LimitOmni(command, robot.yaw_rad, 0.65f, 0.096f,
                                    -0.08f, 0.0f);
        if ((scenario == 4) && (k == 300) && link_available) {
            motion_enabled = 1U;
            last_renew_ms = now_ms;
        }
        if ((k % 10 == 0) && link_available && motion_enabled) last_renew_ms = now_ms;
        if (!AppControlSafety_IsFresh(now_ms, last_renew_ms, 300U)) motion_enabled = 0U;
        if (!motion_enabled) {
            command.vx_m_s = command.vy_m_s = command.wz_rad_s = 0.0f;
        }
        if ((scenario == 4) && (k == 250))
            assert(command.vx_m_s == 0.0f && command.vy_m_s == 0.0f);
        step_robot(&robot, command);
        fprintf(trace, "%s,lqr,%.2f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                name, t, robot.x_m, robot.y_m, bx, by, command.vx_m_s);
    }
    printf("soccer %s: final_ball_distance=%.4f m\n", name,
           hypotf(robot.x_m - bx, robot.y_m - by));
    assert(hypotf(robot.x_m - bx, robot.y_m - by) < 0.35f);
}

int main(void)
{
    FILE *trace = fopen("control_trace.csv", "w");
    SoccerBallEstimate ball = {0};
    assert(trace != NULL);
    fprintf(trace, "scenario,controller,t_s,robot_x_m,robot_y_m,ball_x_m,ball_y_m,cmd_vx_m_s\n");
    test_control_basics();
    SoccerBall_Update(&ball, 0.01f, 1U, 1.0f, 0.0f);
    SoccerBall_Update(&ball, 0.01f, 1U, 1.1f, 0.0f);
    assert(ball.vx_m_s > 0.0f);
    SoccerBall_Update(&ball, 0.01f, 0U, 0.0f, 0.0f);
    assert(ball.age_s > 0.0f);
    SoccerBall_Update(&ball, 0.01f, 1U, 3.0f, 0.0f);
    assert(ball.vx_m_s == 0.0f);
    ball.age_s = 0.51f;
    {
        AppLqrState robot = {0};
        assert(SoccerBall_Intercept(&ball, &robot).track_x == 0U);
    }
    compare_goal(trace, 0);
    compare_goal(trace, 1);
    soccer_case(trace, "stationary", 0);
    soccer_case(trace, "moving", 1);
    soccer_case(trace, "occlusion", 2);
    soccer_case(trace, "jump", 3);
    soccer_case(trace, "link_loss", 4);
    fclose(trace);
    return 0;
}
