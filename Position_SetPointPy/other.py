

def compute_vel_at_zero_acc(state_v, state_a, max_jerk):
    vel_zero_acc = state_v
    
    if abs(state_a) > 0:
        j_zero_acc = -1 if state_a < 0 else 1 * max_jerk
        t_zero_acc = -state_a / j_zero_acc
        vel_zero_acc = state_v + state_a * t_zero_acc + 0.5 * j_zero_acc * t_zero_acc**2
    
    return vel_zero_acc

def evaluate_poly(jerk, a0, v0, x0, t, d):
    jt = d * jerk
    t2 = t * t
    t3 = t2 * t

    j = jt
    a = a0 + jt * t
    v = v0 + a0 * t + 0.5 * jt * t2
    p = x0 + v0 * t + 0.5 * a0 * t2 + (1 / 6) * jt * t3

    return p, v, a, j

def compute_direction(vel_sp, state_a, state_v, max_jerk):
    vel_zero_acc = compute_vel_at_zero_acc(state_v, state_a, max_jerk)
    
    direction = 1 if (vel_sp - vel_zero_acc) > 0 else -1 if (vel_sp - vel_zero_acc) < 0 else 0
    
    if direction == 0:
        direction = 1 if state_a > 0 else -1 if state_a < 0 else 0
    
    return direction

