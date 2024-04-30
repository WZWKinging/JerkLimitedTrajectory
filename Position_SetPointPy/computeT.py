import math
from saturateMessage import saturate_t1_for_accel
def compute_t1_1(a0, v3, j_max, a_max):
    delta = 2 * a0 * a0 + 4 * j_max * v3
    
    if delta < 0:
        t1 = 0
        return t1
    
    sqrt_delta = math.sqrt(delta)
    t1_plus = (-a0 + 0.5 * sqrt_delta) / j_max
    t1_minus = (-a0 - 0.5 * sqrt_delta) / j_max
    
    t3_plus = a0 / j_max + t1_plus
    t3_minus = a0 / j_max + t1_minus
    
    t1 = 0
    
    if t1_plus >= 0 and t3_plus >= 0:
        t1 = t1_plus
    else:
        if t1_minus >= 0 and t3_minus >= 0:
            t1 = t1_minus
            
    t1 = saturate_t1_for_accel(a0, j_max, t1, a_max)
    
    t1 = max(t1, 0)
    
    return t1

def compute_t1_2(t123, a0, v3, j_max, a_max):
    a = -j_max
    b = j_max * t123 - a0
    delta = t123 * t123 * j_max * j_max + 2 * t123 * a0 * j_max - a0 * a0 - 4 * j_max * v3

    if delta < 0:
        t1 = 0
        return t1

    sqrt_delta = math.sqrt(delta)
    denominator_inv = 1 / (2 * a)
    t1_plus = max((-b + sqrt_delta) * denominator_inv, 0)
    t1_minus = max((-b - sqrt_delta) * denominator_inv, 0)

    t3_plus = a0 / j_max + t1_plus
    t3_minus = a0 / j_max + t1_minus

    t13_plus = t1_plus + t3_plus
    t13_minus = t1_minus + t3_minus

    t1 = 0

    if t13_plus > t123:
        t1 = t1_minus
    else:
        if t13_minus > t123:
            t1 = t1_plus

    t1 = saturate_t1_for_accel(a0, j_max, t1, a_max)

    return t1

def compute_t2_1(t1, t3, a0, v3, j_max):
    t2 = 0
    den = a0 + j_max * t1

    if abs(den) > 0:
        t2 = (-0.5 * t1 * t1 * j_max - t1 * t3 * j_max - t1 * a0 + 0.5 * t3 * t3 * j_max - t3 * a0 + v3) / den

    t2 = max(t2, 0)

    return t2

def compute_t2_2(t123, t1, t3):
    t2 = t123 - t1 - t3
    t2 = max(t2, 0)

    return t2

def compute_t3(t1, a0, j_max):
    t3 = a0 / j_max + t1
    t3 = max(t3, 0)

    return t3

def compute_t4(delt_p_acc, delt_p_dec, v_i, delt_p):
    if v_i!=0:
        t4 = (delt_p - delt_p_acc - delt_p_dec) / v_i
    else:
        t4 = 0
    t4 = max(t4, 0)

    return t4

def compute_t5_1(a0, v3, j_max, a_max):
    delta = 2 * a0 * a0 + 4 * j_max * v3
    
    if delta < 0:
        t5 = 0
        return t5
    
    sqrt_delta = math.sqrt(delta)
    t5_plus = (-a0 + 0.5 * sqrt_delta) / j_max
    t5_minus = (-a0 - 0.5 * sqrt_delta) / j_max
    
    t7_plus = a0 / j_max + t5_plus
    t7_minus = a0 / j_max + t5_minus
    
    t5 = 0
    
    if t5_plus >= 0 and t7_plus >= 0:
        t5 = t5_plus
    else:
        if t5_minus >= 0 and t7_minus >= 0:
            t5 = t5_minus
    
    t5 = saturate_t5_for_accel(a0, j_max, t5, a_max)
    
    t5 = max(t5, 0)
    
    return t5

def saturate_t5_for_accel(a0, j_max, t5, a_max):
    if a0 >= 0:
        if t5 * j_max > a_max:
            t5 = a_max / j_max
    else:
        if t5 * j_max < -a_max:
            t5 = -a_max / j_max
    
    return t5

def compute_t5_2(t567, a0, v3, j_max, a_max):
    a = -j_max
    b = j_max * t567 - a0
    delta = t567 * t567 * j_max * j_max + 2 * t567 * a0 * j_max - a0 * a0 - 4 * j_max * v3
    
    if delta < 0:
        t5 = 0
        return t5
    
    sqrt_delta = math.sqrt(delta)
    denominator_inv = 1 / (2 * a)
    t5_plus = max((-b + sqrt_delta) * denominator_inv, 0)
    t5_minus = max((-b - sqrt_delta) * denominator_inv, 0)
    
    t7_plus = a0 / j_max + t5_plus
    t7_minus = a0 / j_max + t5_minus
    
    t57_plus = t5_plus + t7_plus
    t57_minus = t5_minus + t7_minus
    
    t5 = 0
    
    if t57_plus > t567:
        t5 = t5_minus
    else:
        if t57_minus > t567:
            t5 = t5_plus
    
    t5 = saturate_t5_for_accel(a0, j_max, t5, a_max)
    
    return t5

def saturate_t5_for_accel(a0, j_max, t5, a_max):
    if a0 >= 0:
        if t5 * j_max > a_max:
            t5 = a_max / j_max
    else:
        if t5 * j_max < -a_max:
            t5 = -a_max / j_max
    
    return t5

def compute_t6_1(t5, t7, a0, v3, j_max):
    t6 = 0
    den = a0 + j_max * t5
    
    if abs(den) > 0:
        t6 = (-0.5 * t5 * t5 * j_max - t5 * t7 * j_max - t5 * a0 + 0.5 * t7 * t7 * j_max - t7 * a0 + v3) / den
    
    t6 = max(t6, 0)
    
    return t6

def compute_t6_2(t567, t5, t7):
    t6 = t567 - t5 - t7
    t6 = max(t6, 0)
    
    return t6

def compute_t7(t5, a0, j_max):
    t7 = a0 / j_max + t5
    t7 = max(t7, 0)
    
    return t7
