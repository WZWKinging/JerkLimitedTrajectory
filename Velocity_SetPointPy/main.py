# -*- coding: utf-8 -*-
# 绘制图形
import math
import matplotlib.pyplot as plt
from updateDurations_Velocity_Setpoint import update_durations_velocity_setpoint,update_traj
from timeSynchronization import time_synchronization
class State:
    def __init__(self):
        self.p0 = 0.0
        self.v0 = 0.0
        self.a0 = 0.0
        self.j0 = 0.0
        self.p = 0.0
        self.v = 0.0
        self.a = 0.0
        self.j = 0.0
        self.p_sp = 0.0
        self.v_sp = 0.0
        self.a_sp = 0.0
        self.v_max = 0.0
        self.a_max = 0.0
        self.j_max = 0.0
        self.T1 = 0
        self.T2 = 0
        self.T3 = 0
        self.T4 = 0
        self.T5 = 0
        self.T6 = 0
        self.T7 = 0
        self.log_p = []
        self.log_v = []
        self.log_a = []
        self.log_j = []
        self.log_t = []
        self.log_v_sp = []
        self.direction = 0
        self.timestamp = 0

# 创建三个状态对象
state1 = State()
state1.p0 = 10
state1.v0 = 1
state1.a0 = 0
state1.j0 = 0.0
state1.p = 10
state1.v = 1
state1.a = 0
state1.j = 0
state1.p_sp = 0
state1.v_sp = 3
state1.a_sp = 0.0
state1.v_max = 5.0
state1.a_max = 2.0
state1.j_max = 4.0
state1.T1 = 0
state1.T2 = 0
state1.T3 = 0
state1.T4 = 0
state1.T5 = 0
state1.T6 = 0
state1.T7 = 0
state1.direction = 0
state1.timestamp = 0

state2 = State()
state2.p0 = 0.0
state2.v0 = 0.0
state2.a0 = 0.0
state2.j0 = 0.0
state2.p = 0
state2.v = 0
state2.a = 0
state2.j = 0
state2.p_sp = 0.0
state2.v_sp = -5
state2.a_sp = 0.0
state2.v_max = 5.0
state2.a_max = 5.0
state2.j_max = 10.0
state2.T1 = 0
state2.T2 = 0
state2.T3 = 0
state2.T4 = 0
state2.T5 = 0
state2.T6 = 0
state2.T7 = 0
state2.direction = 0
state2.timestamp = 0

state3 = State()
state3.p0 = 0.0
state3.v0 = 0.0
state3.a0 = 0.0
state3.j0 = 0.0
state3.p = 0
state3.v = 0
state3.a = 0
state3.j = 0
state3.p_sp = 0.0
state3.v_sp = 3.0
state3.a_sp = 0.0
state3.v_max = 5.0
state3.a_max = 2.0
state3.j_max = 5.0
state3.T1 = 0
state3.T2 = 0
state3.T3 = 0
state3.T4 = 0
state3.T5 = 0
state3.T6 = 0
state3.T7 = 0
state3.direction = 0
state3.timestamp = 0

dt = 0.0025
n_index = 1

state = [state1, state2, state3]

# 更新状态变量的持续时间和方向
for i in range(3):
    state[i].T1, state[i].T2, state[i].T3, state[i].direction = update_durations_velocity_setpoint(state[i].v_sp, state[i].a, state[i].v, state[i].j_max, state[i].a_max, state[i].v_max)

# 时间同步
state, longest_time = time_synchronization(state, 3)
n_steps = math.ceil(longest_time / dt)

# 更新轨迹并记录日志
for i in range(n_steps):
    for k in range(3):
        state[k].timestamp = 0
        state[k].p, state[k].v, state[k].a, state[k].j, state[k].timestamp = update_traj(dt, state[k].timestamp, state[k].T1, state[k].T2, state[k].T3, state[k].a, state[k].v, state[k].p, state[k].direction, state[k].j_max)
        state[k].log_p.append(state[k].p)
        state[k].log_v.append(state[k].v)
        state[k].log_a.append(state[k].a)
        state[k].log_j.append(state[k].j)
        state[k].log_t.append(n_index * dt)
        state[k].log_v_sp.append(state[k].v_sp)
        state[k].T1, state[k].T2, state[k].T3, state[k].direction = update_durations_velocity_setpoint(state[k].v_sp, state[k].a, state[k].v, state[k].j_max, state[k].a_max, state[k].v_max)

    state, longest_time = time_synchronization(state, 3)
    n_index += 1



plt.figure(1)
plt.plot(state[0].log_t, state[0].log_p, 'r')
plt.plot(state[0].log_t, state[0].log_v, 'g')
plt.plot(state[0].log_t, state[0].log_a, 'b')
plt.plot(state[0].log_t, state[0].log_j, 'm')
plt.plot(state[0].log_t, state[0].log_v_sp, 'k')
plt.legend(['pos_x', 'vel_x', 'acc_x', 'jerk_x'])
plt.grid(True)

plt.figure(2)
plt.plot(state[1].log_t, state[1].log_p, 'r')
plt.plot(state[1].log_t, state[1].log_v, 'g')
plt.plot(state[1].log_t, state[1].log_a, 'b')
plt.plot(state[1].log_t, state[1].log_j, 'm')
plt.plot(state[1].log_t, state[1].log_v_sp, 'k')
plt.legend(['pos_y', 'vel_y', 'acc_y', 'jerk_y'])
plt.grid(True)

plt.figure(3)
plt.plot(state[2].log_t, state[2].log_p, 'r')
plt.plot(state[2].log_t, state[2].log_v, 'g')
plt.plot(state[2].log_t, state[2].log_a, 'b')
plt.plot(state[2].log_t, state[2].log_j, 'm')
plt.plot(state[2].log_t, state[2].log_v_sp, 'k')
plt.legend(['pos_z', 'vel_z', 'acc_z', 'jerk_z'])
plt.grid(True)

plt.show()
