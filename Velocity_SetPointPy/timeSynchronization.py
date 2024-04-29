from computeT import compute_t1_2
from computeT import compute_t3
from computeT import compute_t2_2

def update_durations_given_total_time(t123, direction, max_jerk, max_accel, vel_sp, state_a, state_v):
    jerk_max_t1 = direction * max_jerk
    delta_v = vel_sp - state_v
    
    T1 = compute_t1_2(t123, state_a, delta_v, jerk_max_t1, max_accel)
    T3 = compute_t3(T1, state_a, jerk_max_t1)
    T2 = compute_t2_2(t123, T1, T3)
    
    return T1, T2, T3


def time_synchronization(state, n_traj):
    desired_time = 0
    longest_traj_index = 0
    longest_time = 0
    
    for i in range(n_traj):
        T123 = state[i].T1 + state[i].T2 + state[i].T3
        if T123 > desired_time:
            desired_time = T123
            longest_traj_index = i
    
    longest_time = desired_time
    
    if desired_time > 1.192093e-007:
        for i in range(n_traj):
            if i != longest_traj_index:
                state[i].T1, state[i].T2, state[i].T3 = update_durations_given_total_time(desired_time, state[i].direction, state[i].j_max, state[i].a_max, state[i].v_sp, state[i].a, state[i].v)
    
    return state, longest_time




