#! /bin/python2

import pickle 
import numpy as np

# esc_dict = {}

# filenames = [("../data/result5/esc_dict_atlasmpnet_env0.p", (0, )), 
#             ("../data/result5/esc_dict_atlasmpnet_env1.p", (1, )), 
#             ("../data/result5/esc_dict_atlasmpnet_env2-3.p", (2, 3)), 
#             ("../data/result5/esc_dict_atlasmpnet_env4.p", (4, )), 
#             ("../data/result5/esc_dict_atlasmpnet_env5.p", (5, )), 
#             ("../data/result5/esc_dict_atlasmpnet_env6-8.p", (6, 7, 8)), 
#             ("../data/result5/esc_dict_atlasmpnet_env9.p", (9, )), 
#             ("../data/result5/esc_dict_atlasmpnet_env10-18.p", (10, 11, 12, 13, 14, 15, 16, 17, 18))]

# for filename, envs in filenames:
#     with open(filename, "rb") as f:
#         esc_dict_temp = pickle.load(f)
#     for e in envs:
#         env_no = "env_" + str(e)
#         esc_dict[env_no] = esc_dict_temp[env_no]

# # fill invalid time with -1
# for e in range(0, 19):
#     env_no = "env_" + str(e)
#     if env_no not in esc_dict.keys():
#         continue
#     for s in range(110, 120):  # 30
#         s_no = "s_" + str(s)
#         if s_no not in esc_dict[env_no].keys():
#             continue

#         obj_order = esc_dict[env_no][s_no]["obj_order"]
#         for obj in obj_order:
#             if "time_pick_place" not in esc_dict[env_no][s_no][obj].keys():
#                 esc_dict[env_no][s_no][obj]["time_pick_place"] = -1

# with open("../data/result5/esc_dict_atlasmpnet.p", "wb") as f:
#     pickle.dump(esc_dict, f)

with open("../data/result5/esc_dict_atlasmpnet.p", "rb") as f:
    esc_dict = pickle.load(f)


obj_invalid = 0
obj_fail = 0
obj_success = 0
for e in range(0, 19):
    env_no = "env_" + str(e)
    if env_no not in esc_dict.keys():
        continue
    for s in range(110, 120):  # 30
        s_no = "s_" + str(s)
        if s_no not in esc_dict[env_no].keys():
            continue

        obj_order = esc_dict[env_no][s_no]["obj_order"]
        for obj in obj_order:
            obj_time = esc_dict[env_no][s_no][obj]["time_pick_place"]
            if obj_time == -1:
                obj_invalid += 1
            elif obj_time == 1000:
                obj_fail += 1
            else:
                obj_success += 1

print obj_invalid, obj_fail, obj_success