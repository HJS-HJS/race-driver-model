import numpy as np
import matplotlib.pyplot as plt

from utils.utils import *
from utils.path_optimizer import shortest_path, min_curve_path, optimize

track = "cb"
ref_path, lenght, center, file_path = gps_path(track)
bound_in = ref_path - lenght
bound_out = ref_path + lenght
H_S , B_S = shortest_path(ref_path, lenght)
H_C , B_C = min_curve_path(ref_path, lenght)

# epsilon
e_set = [0.00, 0.15, 0.30, 0.60, 1.00]
# e = 0.00   # min curve path
# e = 1.00   # shortest path

_file_path = file_path + '_track'

bound_in = np.vstack((bound_in, bound_in[0]))
bound_out = np.vstack((bound_out, bound_out[0]))

plt.figure()
plt.plot(bound_in[:,0],  bound_in[:,1],  label="bound_in" , color="blue")
plt.plot(bound_out[:,0], bound_out[:,1], label="bound_out", color="blue")
plt.grid(True)
plt.gca().set_aspect('equal')
plt.title("Track: " + track)
plt.savefig(_file_path + '.png', dpi=400)

for e in e_set:
    H = (1 - e) * H_C + e * H_S
    B = (1 - e) * B_C + e * B_S

    alpha = optimize(H, B)
    course_race = ref_path + lenght * alpha[:, np.newaxis]

    _file_path = file_path + '_' + str(int(e*100))
    save_path(course_race, center, _file_path + '.csv')

    course_race = np.vstack((course_race, course_race[0]))
    plt.figure()
    plt.plot(bound_in[:,0],  bound_in[:,1],  label="bound_in" , color="blue")
    plt.plot(bound_out[:,0], bound_out[:,1], label="bound_out", color="blue")
    plt.plot(course_race[:,0], course_race[:,1], label="optimized path", color="orange", linewidth = 3)
    plt.grid(True)
    plt.gca().set_aspect('equal')
    plt.legend()
    plt.title("Track: " + track + ", e: " + str(int(e*100)) + "%")
    plt.savefig(_file_path + '.png', dpi=400)
    plt.show()
