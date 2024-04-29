import math
from saturateT1ForAccel import saturate_t1_for_accel
def compute_t1_1(a0, v3, j_max, a_max):
    delta = 2 * a0 * a0 + 4 * j_max * v3
    
    if delta < 0:
        T1 = 0
        return T1
    
    sqrt_delta = math.sqrt(delta)
    t1_plus = (-a0 + 0.5 * sqrt_delta) / j_max
    t1_minus = (-a0 - 0.5 * sqrt_delta) / j_max
    
    t3_plus = a0 / j_max + t1_plus
    t3_minus = a0 / j_max + t1_minus
    
    T1 = 0
    
    if t1_plus >= 0 and t3_plus >= 0:
        T1 = t1_plus
    else:
        if t1_minus >= 0 and t3_minus >= 0:
            T1 = t1_minus
    
    T1 = saturate_t1_for_accel(a0, j_max, T1, a_max)
    
    T1 = max(T1, 0)
    
    return T1

def compute_t1_2(t123, a0, v3, j_max, a_max):
    a = -j_max
    b = j_max * t123 - a0
    delta = t123**2 * j_max**2 + 2 * t123 * a0 * j_max - a0**2 - 4 * j_max * v3

    if delta < 0:
        T1 = 0
        return T1

    sqrt_delta = math.sqrt(delta)
    denominator_inv = 1 / (2 * a)
    t1_plus = max((-b + sqrt_delta) * denominator_inv, 0)
    t1_minus = max((-b - sqrt_delta) * denominator_inv, 0)

    t3_plus = a0 / j_max + t1_plus
    t3_minus = a0 / j_max + t1_minus

    t13_plus = t1_plus + t3_plus
    t13_minus = t1_minus + t3_minus

    T1 = 0

    if t13_plus > t123:
        T1 = t1_minus
    else:
        if t13_minus > t123:
            T1 = t1_plus

    T1 = saturate_t1_for_accel(a0, j_max, T1, a_max)
    
    return T1




def compute_t2_1(t1, t3, a0, v3, j_max):
    T2 = 0
    den = a0 + j_max * t1
    
    if abs(den) > 1.192093e-007:
        T2 = (-0.5 * t1 * t1 * j_max - t1 * t3 * j_max - t1 * a0 + 0.5 * t3 * t3 * j_max - t3 * a0 + v3) / den
    
    T2 = max(T2, 0)
    
    return T2

def compute_t2_2(t123, t1, t3):
    T2 = t123 - t1 - t3
    T2 = max(T2, 0)
    
    return T2


def compute_t3(t1, a0, j_max):
    t3 = a0 / j_max + t1
    t3 = max(t3, 0)
    
    return t3