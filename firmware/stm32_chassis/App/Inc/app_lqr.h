#ifndef APP_LQR_H
#define APP_LQR_H

#ifdef __cplusplus
extern "C" {
#endif

/* All coordinates are world-frame SI units; yaw is wrapped to [-pi, pi). */
typedef struct {
    float x_m, y_m, yaw_rad;
    float vx_m_s, vy_m_s, wz_rad_s;
} AppLqrState;

typedef struct {
    float x_m, y_m, yaw_rad;
    float vx_m_s, vy_m_s, wz_rad_s;
    unsigned char track_x, track_y, track_yaw;
} AppLqrReference;

typedef struct {
    float vx_m_s, vy_m_s, wz_rad_s;
} AppLqrOutput;

#define APP_LQR_PERIOD_MS 10U
#define APP_LQR_VELOCITY_LIMIT_M_S 0.65f
#define APP_LQR_YAW_RATE_LIMIT_RAD_S 2.0f

float AppLqr_WrapRadians(float angle);
AppLqrOutput AppLqr_Calculate(const AppLqrState *state,
                              const AppLqrReference *reference,
                              float translation_limit_m_s);
AppLqrOutput AppLqr_LimitOmni(AppLqrOutput command, float yaw_rad,
                              float wheel_limit_m_s, float rotation_radius_m,
                              float forward_to_left, float left_to_forward);
void AppLqr_ModelStep(float *position, float *velocity,
                      float command, float period_s, float tau_s);

#ifdef __cplusplus
}
#endif
#endif
