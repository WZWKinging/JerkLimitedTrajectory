from computeT import compute_t1_1
from computeT import compute_t3
from computeT import compute_t2_1
def update_durations_minimize_total_time(direction, max_jerk, max_accel, vel_sp, state_a, state_v):
    jerk_max_t1 = direction * max_jerk
    delta_v = vel_sp - state_v
    
    T1 = compute_t1_1(state_a, delta_v, jerk_max_t1, max_accel)
    T3 = compute_t3(T1, state_a, jerk_max_t1)
    T2 = compute_t2_1(T1, T3, state_a, delta_v, jerk_max_t1)
    
    return T1, T2, T3



