from binarySearchUpdate import binary_search_update_durations_given_total_time
def time_synchronization(time_stamp, state, n_traj):
    desired_time = 0
    longest_traj_index = 0

    for i in range(n_traj):
        T1234567 = state[i].T1 + state[i].T2 + state[i].T3 + state[i].T4 + state[i].T5 + state[i].T6 + state[i].T7
        if T1234567 > desired_time:
            desired_time = T1234567
            longest_traj_index = i

    longest_time = desired_time

    if desired_time > 0:
        for i in range(n_traj):
            if i != longest_traj_index:
                state[i].T1, state[i].T2, state[i].T3, state[i].T4, state[i].T5, state[i].T6, state[i].T7, state[i].direction_acc, state[i].direction_dec, state[i].v_m= binary_search_update_durations_given_total_time(desired_time, state[i].j_max, state[i].a0, state[i].a_max, state[i].v0, state[i].v_max, state[i].p0, state[i].p_sp)

    return state, longest_time