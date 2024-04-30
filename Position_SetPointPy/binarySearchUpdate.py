from updateMessage import update_durations_position_setpoint
def binary_search_update_durations_given_total_time(t1234567, j_max, a0, a_max, v0, v_max, p0, pt):
    delt_p = pt - p0
    if delt_p == 0:
        v_now = -1 if v0 < 0 else 1 * v_max
    else:
        v_now = 1 if delt_p > 0 else -1 * v_max
    
    v_s = 0
    v_e = v_now
    
    meet_the_set_point = 0
    delt_p_thr = 0.000001
    delt_t_thr = 0.01
    delt_vel_dec = 0.01
    
    iter_counter = 1
    max_iter = 100
    
    while meet_the_set_point != 1:
        T1, T2, T3, T4, T5, T6, T7, dir_acc, dir_dec, deltpp = update_durations_position_setpoint(v_now, a0, v0, j_max, a_max, v_max, p0, pt)
        
        direction_acc = dir_acc
        direction_dec = dir_dec
        total_t1234567 = T1 + T2 + T3 + T4 + T5 + T6 + T7
        
        if abs(deltpp - delt_p) <= delt_p_thr and iter_counter < max_iter:
            if abs(total_t1234567 - t1234567) < delt_t_thr:
                meet_the_set_point = 1
                break
            else:
                if T1 < 0.0001 and T2 < 0.0001 and T3 < 0.0001:
                    meet_the_set_point = 1
                    break
                
                meet_the_set_point = 0
                v_now = v_now - (1 if v_now < 0 else -1) * delt_vel_dec
                iter_counter += 1
                continue
        else:
            if iter_counter >= max_iter:
                meet_the_set_point = 1
                break
            else:
                meet_the_set_point = 0
        
        if abs(deltpp) > abs(delt_p):
            v_s = 0
            v_e = v_now
            v_now = (v_s + v_e) / 2
        else:
            if abs(deltpp) < abs(delt_p):
                v_s = v_now
                v_e = 1 if delt_p > 0 else -1 * v_max
                v_now = (v_s + v_e) / 2
        
        iter_counter += 1
    
    v_m = v_now
    
    return T1, T2, T3, T4, T5, T6, T7, direction_acc, direction_dec, v_m

def binary_search_update_durations_minimize_total_time(j_max, a0, a_max, v0, v_max, p0, pt):
    delt_p = pt - p0
    if delt_p == 0:
        v_now = -1 if v0 < 0 else 1 * v_max
    else:
        v_now = 1 if delt_p > 0 else -1 * v_max
    
    v_s = 0
    v_e = v_now
    
    meet_the_set_point = 0
    delt_p_thr = 0.000001
    
    iter_counter = 1
    max_iter = 100
    
    while meet_the_set_point != 1:
        T1, T2, T3, T4, T5, T6, T7, dir_acc, dir_dec, deltpp = update_durations_position_setpoint(v_now, a0, v0, j_max, a_max, v_max, p0, pt)
        
        direction_acc = dir_acc
        direction_dec = dir_dec
        
        if abs(deltpp - delt_p) <= delt_p_thr and iter_counter < max_iter:
            meet_the_set_point = 1
            break
        else:
            if iter_counter >= max_iter:
                meet_the_set_point = 1
                break
            else:
                meet_the_set_point = 0
        
        if abs(deltpp) > abs(delt_p):
            v_s = 0
            v_e = v_now
            v_now = (v_s + v_e) / 2
        else:
            if abs(deltpp) < abs(delt_p):
                v_s = v_now
                v_e = 1 if delt_p > 0 else -1 * v_max
                v_now = (v_s + v_e) / 2
        
        iter_counter += 1
    
    v_m = v_now
    
    return T1, T2, T3, T4, T5, T6, T7, direction_acc, direction_dec, v_m
