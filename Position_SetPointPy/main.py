import math
import matplotlib.pyplot as plt
from time_synchronization import time_synchronization
from binarySearchUpdate import binary_search_update_durations_minimize_total_time
from updateMessage import update_traj
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
        self.direction_acc = 0
        self.direction_dec = 0
        self.timestamp = 0
        self.v_m = 0.0

local_time = 0
states = [State() for _ in range(4)]

states[0].p0 = 0.0
states[0].v0 = 1.0
states[0].a0 = 0.0
states[0].j0 = 0.0
states[0].p_sp = 25.0
states[0].v_sp = 10.0
states[0].a_sp = 0.0
states[0].v_max = 10.0
states[0].a_max = 2.0
states[0].j_max = 15.0

states[1].p0 = 0.0
states[1].v0 = 0.0
states[1].a0 = 0.0
states[1].j0 = 0.0
states[1].p_sp = 4.0
states[1].v_sp = 0.0
states[1].a_sp = 0.0
states[1].v_max = 5.0
states[1].a_max = 5.0
states[1].j_max = 10.0

states[2].p0 = 0.0
states[2].v0 = 0.0
states[2].a0 = 0.0
states[2].j0 = 0.0
states[2].p_sp = 3.0
states[2].v_sp = 0.0
states[2].a_sp = 0.0
states[2].v_max = 5.0
states[2].a_max = 2.0
states[2].j_max = 5.0

states[3].p = 0

dt = 0.0025
n_index = 1

for state in states:
    state.T1, state.T2, state.T3, state.T4, state.T5, state.T6, state.T7, state.direction_acc, state.direction_dec, state.v_m = binary_search_update_durations_minimize_total_time(state.j_max, state.a0, state.a_max, state.v0, state.v_max, state.p0, state.p_sp)

states, longest_time = time_synchronization(local_time, states, 3)
n_step = math.ceil(longest_time / dt)

for i in range(1, n_step + 1):
    for k in range(3):
        states[k].p, states[k].v, states[k].a, states[k].j, states[k].timestamp = update_traj(dt, states[k].timestamp,
                                                                                         states[k].T1, states[k].T2, states[k].T3, states[k].T4, states[k].T5, states[k].T6, states[k].T7,
                                                                                         states[k].a0, states[k].v0, states[k].p0, states[k].direction_acc, states[k].direction_dec, states[k].j_max)
        states[k].log_p.append(states[k].p)
        states[k].log_v.append(states[k].v)
        states[k].log_a.append(states[k].a)
        states[k].log_j.append(states[k].j)
        states[k].log_t.append(n_index * dt)
        states[k].log_v_sp.append(states[k].v_sp)

    states[3].log_p.append(math.sqrt(pow(states[0].p, 2) + pow(states[1].p, 2)))
    states[3].log_v.append(math.sqrt(pow(states[0].v, 2) + pow(states[1].v, 2)))

    n_index += 1


# Plotting for state 1
plt.figure(1)
plt.plot(states[0].log_t, states[0].log_p, 'r', label='pos_x')
plt.plot(states[0].log_t, states[0].log_v, 'g', label='vel_x')
plt.plot(states[0].log_t, states[0].log_a, 'b', label='acc_x')
plt.plot(states[0].log_t, states[0].log_j, 'm', label='jerk_x')
plt.legend()
plt.grid()
plt.show()

# Plotting for state 2
plt.figure(2)
plt.plot(states[1].log_t, states[1].log_p, 'r', label='pos_y')
plt.plot(states[1].log_t, states[1].log_v, 'g', label='vel_y')
plt.plot(states[1].log_t, states[1].log_a, 'b', label='acc_y')
plt.plot(states[1].log_t, states[1].log_j, 'm', label='jerk_y')
plt.legend()
plt.grid()
plt.show()

# Plotting for state 3
plt.figure(3)
plt.plot(states[2].log_t, states[2].log_p, 'r', label='pos_z')
plt.plot(states[2].log_t, states[2].log_v, 'g', label='vel_z')
plt.plot(states[2].log_t, states[2].log_a, 'b', label='acc_z')
plt.plot(states[2].log_t, states[2].log_j, 'm', label='jerk_z')
plt.legend()
plt.grid()
plt.show()

# Plotting for state 4 (if needed)
# plt.figure(4)
# plt.plot(state[0].log_t, state[3].log_p, 'r', label='pos')
# plt.plot(state[0].log_t, state[3].log_v, 'g', label='vel')
# plt.legend()
# plt.grid()
# plt.show()
