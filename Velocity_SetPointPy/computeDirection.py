
def compute_vel_at_zero_acc(state_v, state_a, max_jerk):
    vel_zero_acc = state_v
    
    if abs(state_a) > 1.192093e-007:
        j_zero_acc = -1 if state_a < 0 else 1 * max_jerk
        t_zero_acc = -state_a / j_zero_acc
        vel_zero_acc = state_v + state_a * t_zero_acc + 0.5 * j_zero_acc * t_zero_acc * t_zero_acc
    
    return vel_zero_acc

def compute_direction(vel_sp, state_a, state_v, max_jerk):
    vel_zero_acc = compute_vel_at_zero_acc(state_v, state_a, max_jerk)
    print("vel_sp", vel_sp)
    print("vel_zero_acc", vel_zero_acc)
    direction = (vel_sp - vel_zero_acc) / abs(vel_sp - vel_zero_acc) if vel_sp != vel_zero_acc else state_a / abs(state_a)
    
    if direction == 0:
        direction = state_a / abs(state_a)
    
    return direction
