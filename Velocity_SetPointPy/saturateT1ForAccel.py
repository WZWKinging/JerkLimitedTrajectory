def saturate_t1_for_accel(a0, j_max, t1, a_max):
    accel_t1 = a0 + j_max * t1
    t1_new = t1
    
    if accel_t1 > a_max:
        t1_new = (a_max - a0) / j_max
    else:
        if accel_t1 < -a_max:
            t1_new = (-a_max - a0) / j_max
    
    return t1_new
