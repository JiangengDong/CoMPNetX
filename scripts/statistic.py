#! /bin/python2

import pickle
import numpy as np
import csv
import os


def merge_pickle(folder, env_range, scene_range):
    esc_dict = {}
    for e in env_range:
        env_no = "env_" + str(e)
        with open(os.path.join(folder, "env%d.p" % e), "rb") as f:
            esc_dict[env_no] = pickle.load(f)[env_no]

    with open(os.path.join(folder, "esc_dict.p"), "wb") as f:
        pickle.dump(esc_dict, f)


def generate_csv(folder, env_range, scene_range, obj_order):
    with open(os.path.join(folder, "esc_dict.p"), "rb") as f:
        esc_dict = pickle.load(f)

    obj_line = {obj:0 for obj in obj_order}
    with open(os.path.join(folder, "result.csv"), "wb") as csvfile:
        csv_writer = csv.DictWriter(csvfile, fieldnames=("env", "scene") + obj_order)
        csv_writer.writeheader()
        for e in env_range:
            env_no = "env_" + str(e)
            if env_no not in esc_dict.keys():
                continue
            obj_line["env"] = e
            for s in scene_range:  # 30
                s_no = "s_" + str(s)
                if s_no not in esc_dict[env_no].keys():
                    continue
                obj_line["scene"] = s
                for obj in obj_order:
                    obj_time = esc_dict[env_no][s_no][obj]["time_pick_place"]
                    if obj_time == 1000:
                        obj_time = np.inf
                    if obj_time == -1:
                        obj_time = np.nan
                    obj_line[obj] = obj_time
                csv_writer.writerow(obj_line)


def main(folder, task):
    if task == "bartender":
        env_range = list(range(0, 19))
        scene_range = list(range(110, 120))
        obj_order = ("juice", "fuze_bottle", "coke_can", "plasticmug", "teakettle")
    elif task == "kitchen":
        env_range = list(range(0, 70))
        scene_range = list(range(27, 31))
        obj_order = ("juice", "fuze_bottle", "coke_can", "plasticmug", "pitcher", "mugblack")
    elif task == "new kitchen":
        env_range = list(range(0, 70))
        scene_range = list(range(27, 31))
        obj_order = ("juice", "fuze_bottle", "coke_can", "plasticmug", "pitcher", "mugblack")
    else:
        raise NotImplementedError

    merge_pickle(folder, env_range, scene_range)

    generate_csv(folder, env_range, scene_range, obj_order)


if __name__ == "__main__":
    folder = "data/result/result25"
    task = "new kitchen"
    main(folder, task)