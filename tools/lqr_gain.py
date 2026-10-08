#!/usr/bin/env python3
"""Calculate nominal 2-state discrete LQR gains without external packages."""

import argparse
import math


def gains(period, tau, q_position, q_velocity, r):
    decay = math.exp(-period / tau)
    a = ((1.0, tau * (1.0 - decay)), (0.0, decay))
    b = (period - tau * (1.0 - decay), 1.0 - decay)
    p = [[q_position, 0.0], [0.0, q_velocity]]
    for _ in range(100000):
        pb = [sum(p[i][j] * b[j] for j in range(2)) for i in range(2)]
        denom = r + sum(b[i] * pb[i] for i in range(2))
        pa = [[sum(p[i][k] * a[k][j] for k in range(2)) for j in range(2)] for i in range(2)]
        bpa = [sum(b[i] * pa[i][j] for i in range(2)) for j in range(2)]
        next_p = [[(q_position if i == j == 0 else q_velocity if i == j == 1 else 0.0)
                   + sum(a[k][i] * pa[k][j] for k in range(2))
                   - bpa[i] * bpa[j] / denom for j in range(2)] for i in range(2)]
        delta = max(abs(next_p[i][j] - p[i][j]) for i in range(2) for j in range(2))
        p = next_p
        if delta < 1e-12:
            break
    return [sum(b[i] * sum(p[i][k] * a[k][j] for k in range(2)) for i in range(2)) /
            (r + sum(b[i] * sum(p[i][k] * b[k] for k in range(2)) for i in range(2)))
            for j in range(2)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--period", type=float, default=0.01)
    parser.add_argument("--tau-xy", type=float, default=0.12)
    parser.add_argument("--tau-yaw", type=float, default=0.10)
    parser.add_argument("--q-xy-pos", type=float, default=25.0)
    parser.add_argument("--q-yaw-pos", type=float, default=16.0)
    parser.add_argument("--q-velocity", type=float, default=1.0)
    parser.add_argument("--r", type=float, default=1.0)
    args = parser.parse_args()
    if min(args.period, args.tau_xy, args.tau_yaw, args.q_xy_pos,
           args.q_yaw_pos, args.q_velocity, args.r) <= 0:
        parser.error("all model and cost parameters must be positive")
    print("xy position, velocity:", *[f"{v:.8f}" for v in
          gains(args.period, args.tau_xy, args.q_xy_pos, args.q_velocity, args.r)])
    print("yaw position, velocity:", *[f"{v:.8f}" for v in
          gains(args.period, args.tau_yaw, args.q_yaw_pos, args.q_velocity, args.r)])
