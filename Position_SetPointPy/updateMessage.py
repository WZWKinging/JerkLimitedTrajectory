from other import compute_direction,evaluate_poly
from computeT import compute_t4,compute_t1_1,compute_t3,compute_t2_1,compute_t5_1,compute_t7,compute_t6_1

def update_traj(dt, timestamp, T1, T2, T3, T4, T5, T6, T7, a0, v0, p0, direction_acc, direction_dec, max_jerk):
    timestamp = timestamp + dt
    p = 0
    a = 0
    v = 0
    j = 0
    
    if timestamp < (T1 + T2 + T3):
        # update acceleration trajectory
        p, v, a, j = update_traj_acc(timestamp, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk)
    
    if (T1 + T2 + T3) <= timestamp < (T1 + T2 + T3 + T4):
        # update cruise trajectory
        p, v, a, j = update_traj_acc(T1 + T2 + T3, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk)
        p, v, a, j = update_traj_cruise(timestamp - (T1 + T2 + T3), T4, 0, v, p, 0, max_jerk)
    
    if timestamp >= (T1 + T2 + T3 + T4):
        # update deceleration trajectory
        p, v, a, j = update_traj_acc(T1 + T2 + T3, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk)
        p, v, a, j = update_traj_cruise(T4, T4, 0, v, p, 0, max_jerk)
        p, v, a, j = update_traj_acc(timestamp - (T1 + T2 + T3 + T4), T5, T6, T7, a, v, p, direction_dec, max_jerk)
    
    return p, v, a, j, timestamp


def update_traj_acc(timestamp, t1, t2, t3, a0, v0, p0, direction, max_jerk):
    t_remain = timestamp
    
    t1_new = min(t_remain, t1)
    p, v, a, j = evaluate_poly(max_jerk, a0, v0, p0, t1_new, direction)
    t_remain = t_remain - t1_new
    
    if t_remain > 0:
        t2_new = min(t_remain, t2)
        p, v, a, j = evaluate_poly(0, a, v, p, t2_new, 0)
        t_remain = t_remain - t2_new
    
    if t_remain > 0:
        t3_new = min(t_remain, t3)
        p, v, a, j = evaluate_poly(max_jerk, a, v, p, t3_new, -direction)
        t_remain = t_remain - t3_new
    
    if t_remain > 0:
        p, v, a, j = evaluate_poly(0, 0, v, p, t_remain, 0)
    
    return p, v, a, j


def update_traj_cruise(timestamp, t4, a0, v0, p0, direction, max_jerk):
    t_remain = timestamp
    
    t4_new = min(t_remain, t4)
    p, v, a, j = evaluate_poly(max_jerk, a0, v0, p0, t4_new, direction)
    t_remain = t_remain - t4_new
    
    if t_remain > 0:
        p, v, a, j = evaluate_poly(0, 0, v, p, t_remain, 0)
    
    return p, v, a, j


def update_traj_dec(timestamp, t5, t6, t7, a0, v0, p0, direction, max_jerk):
    t_remain = timestamp
    
    t5_new = min(t_remain, t5)
    p, v, a, j = evaluate_poly(max_jerk, a0, v0, p0, t5_new, direction)
    t_remain = t_remain - t5_new
    
    if t_remain > 0:
        t6_new = min(t_remain, t6)
        p, v, a, j = evaluate_poly(0, a, v, p, t6_new, 0)
        t_remain = t_remain - t6_new
    
    if t_remain > 0:
        t7_new = min(t_remain, t7)
        p, v, a, j = evaluate_poly(max_jerk, a, v, p, t7_new, -direction)
        t_remain = t_remain - t7_new
    
    if t_remain > 0:
        p, v, a, j = evaluate_poly(0, 0, v, p, t_remain, 0)
    
    return p, v, a, j

def update_durations_position_setpoint(vel_sp, state_a, state_v, max_jerk, max_accel, max_velocity, p0, pt):
    p_acc, v_acc, a_acc, j_acc, delt_p_acc = 0, 0, 0, 0, 0
    p_dec, v_dec, a_dec, j_dec, delt_p_dec = 0, 0, 0, 0, 0
    deltpp = 0
    delt_p = pt - p0
    
    if vel_sp > max_velocity:
        vel_sp = max_velocity
    if vel_sp < -max_velocity:
        vel_sp = -max_velocity
    
    direction_acc = compute_direction(vel_sp, state_a, state_v, max_jerk)
    
    if direction_acc != 0:
        T1, T2, T3, delt_p_acc, p_acc, v_acc, a_acc, j_acc = update_durations_minimize_total_time_acceleration(direction_acc, max_jerk, max_accel, vel_sp, state_a, state_v, p0)
    else:
        T1, T2, T3 = 0, 0, 0
    
    state_a = 0
    state_v = vel_sp
    vel_sp = 0
    
    direction_dec = compute_direction(vel_sp, state_a, state_v, max_jerk)
    
    if direction_dec != 0:
        T5, T6, T7, delt_p_dec, p_dec, v_dec, a_dec, j_dec = update_durations_minimize_total_time_deceleration(direction_dec, max_jerk, max_accel, vel_sp, a_acc, v_acc, p_acc)
    else:
        T5, T6, T7 = 0, 0, 0
    
    T4 = compute_t4(delt_p_acc, delt_p_dec, state_v, delt_p)
    
    delt_p_cruise = 0
    p_cruise, v_cruise, a_cruise, j_cruise = update_traj_cruise(T4, T4, 0, v_acc, p_acc, 0, max_jerk)
    delt_p_cruise = p_cruise - p_acc
    
    deltpp = delt_p_acc + delt_p_dec + delt_p_cruise
    
    return T1, T2, T3, T4, T5, T6, T7, direction_acc, direction_dec, deltpp


def update_durations_minimize_total_time_acceleration(direction, max_jerk, max_accel, vel_sp, state_a, state_v, state_p):
    jerk_max_t1 = direction * max_jerk
    delta_v = vel_sp - state_v
    
    T1 = compute_t1_1(state_a, delta_v, jerk_max_t1, max_accel)
    T3 = compute_t3(T1, state_a, jerk_max_t1)
    T2 = compute_t2_1(T1, T3, state_a, delta_v, jerk_max_t1)
    
    delt_p_acc = 0
    p, v, a, j = update_traj_acc(T1 + T2 + T3, T1, T2, T3, state_a, state_v, state_p, direction, max_jerk)
    delt_p_acc = p - state_p
    
    return T1, T2, T3, delt_p_acc, p, v, a, j

def update_durations_minimize_total_time_deceleration(direction, max_jerk, max_accel, vel_sp, state_a, state_v, state_p):
    jerk_max_t5 = direction * max_jerk
    delta_v = vel_sp - state_v
    
    T5 = compute_t5_1(state_a, delta_v, jerk_max_t5, max_accel)
    T7 = compute_t7(T5, state_a, jerk_max_t5)
    T6 = compute_t6_1(T5, T7, state_a, delta_v, jerk_max_t5)
    
    delt_p_dec = 0
    p, v, a, j = update_traj_dec(T5 + T6 + T7, T5, T6, T7, state_a, state_v, state_p, direction, max_jerk)
    delt_p_dec = p - state_p
    
    return T5, T6, T7, delt_p_dec, p, v, a, j
