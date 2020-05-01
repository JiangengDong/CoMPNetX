#! /bin/python2

import pickle
import numpy as np

esc_dict = {}
for e in range(0, 19):
    env_no = "env_" + str(e)
    with open("../data/result7/esc_dict_tbmpnet_env%d.p"%e, "rb") as f:
        esc_dict[env_no] = pickle.load(f)[env_no]

with open("../data/result7/esc_dict_tbmpnet.p", "wb") as f:
    pickle.dump(esc_dict, f)

with open("../data/result7/esc_dict_tbmpnet.p", "rb") as f:
    esc_dict = pickle.load(f)


obj_invalid = 0
obj_fail = 0
obj_success = 0
path_times = []
for e in range(0, 19):
    env_no = "env_" + str(e)
    if env_no not in esc_dict.keys():
        continue
    for s in range(110, 120):  # 30
        s_no = "s_" + str(s)
        if s_no not in esc_dict[env_no].keys():
            continue

        isValid = True
        path_time = 0
        obj_order = esc_dict[env_no][s_no]["obj_order"]
        for obj in obj_order:
            obj_time = esc_dict[env_no][s_no][obj]["time_pick_place"]
            if obj_time == -1:
                obj_invalid += 1
                isValid = False
            elif obj_time == 1000:
                obj_fail += 1
                isValid = False
            else:
                obj_success += 1
                path_time += obj_time
        if isValid:
            path_times.append(path_time)


print "Number of invalid objects: ", obj_invalid
print "Number of failed objects: ", obj_fail
print "Number of successful objects: ", obj_success
print "Number of successful paths: ", len(path_times)
print "Mean of path time: ", np.mean(path_times)
print "Stdev of path time: ", np.sqrt(np.cov(path_times))