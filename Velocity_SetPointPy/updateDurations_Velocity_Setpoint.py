from computeDirection import compute_direction
from updateDurationsMinimizeTotalTime import update_durations_minimize_total_time
def update_durations_velocity_setpoint(vel_sp, state_a, state_v, max_jerk, max_accel, max_velocity):
    if vel_sp > max_velocity:
        vel_sp = max_velocity
    if vel_sp < -max_velocity:
        vel_sp = -max_velocity
    
    direction = compute_direction(vel_sp, state_a, state_v, max_jerk)
    
    if direction != 0:
        T1, T2, T3 = update_durations_minimize_total_time(direction, max_jerk, max_accel, vel_sp, state_a, state_v)
    else:
        T1 = 0
        T2 = 0
        T3 = 0
    
    return T1, T2, T3, direction

def evaluate_poly(jerk, a0, v0, x0, t, d):
    jt = d * jerk
    t2 = t * t
    t3 = t2 * t

    j = jt
    a = a0 + jt * t
    v = v0 + a0 * t + 0.5 * jt * t2
    p = x0 + v0 * t + 0.5 * a0 * t2 + (1 / 6) * jt * t3

    return p, v, a, j


def update_traj(dt, time_stamp, tt1, tt2, tt3, a0, v0, p0, direction, max_jerk):
    time_stamp += dt
    t_remain = time_stamp
    time = time_stamp
    t1 = min(t_remain, tt1)
    p, v, a, j = evaluate_poly(max_jerk, a0, v0, p0, t1, direction)
    t_remain -= t1
    
    if t_remain > 0:
        t2 = min(t_remain, tt2)
        p, v, a, j = evaluate_poly(0, a, v, p, t2, 0)
        t_remain -= t2
    
    if t_remain > 0:
        t3 = min(t_remain, tt3)
        p, v, a, j = evaluate_poly(max_jerk, a, v, p, t3, -direction)
        t_remain -= t3
    
    if t_remain > 0:
        p, v, a, j = evaluate_poly(0, 0, v, p, t_remain, 0)
    
    return p, v, a, j, time