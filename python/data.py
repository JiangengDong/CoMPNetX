#!/usr/bin/env python3
import yaml
import h5py
import torch
from typing import Dict, Tuple
import numpy as np
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm
import sys

JOINT_RANGE = np.array([6.1083, 2.668, 3.4033, 3.194, 6.118, 3.6647, 6.118])
KITCHEN_TSR_RANGE = np.array([3.6086, 2.62952, 2.0635998, 6.284, 6.284, 6.284])
BARTENDER_TSR_RANGE = np.array([2.73688, 2.07038677, 2.028624, 6.284, 6.284, 6.284])


def load_train_data(env: str, use_text: bool, use_reach: bool, use_tsr_config: bool) -> Dict[str, np.ndarray]:
    assert env in ["bartender", "kitchen"]
    TSR_RANGE = KITCHEN_TSR_RANGE if env == "kitchen" else BARTENDER_TSR_RANGE

    with open("data/dataset/description.yaml", "r") as f:
        description = yaml.load(f, Loader=yaml.CLoader)
    groups = description[env]["train"]
    if env == "bartender":
        objs = ["fuze_bottle", "juice", "coke_can", "plasticmug", "teakettle"]
    else:
        objs = ["fuze_bottle", "juice", "coke_can", "mugred", "mugblack", "pitcher", "door"]

    f_voxel = h5py.File("data/dataset/{}_voxel.hdf5".format(env), "r")
    f_path = h5py.File("data/dataset/{}_path.hdf5".format(env), "r")
    if use_text:
        f_task_embedding = h5py.File("data/dataset/{}_text_embedding.hdf5".format(env), "r")
    else:
        f_task_embedding = h5py.File("data/dataset/{}_ntp_embedding.hdf5".format(env), "r")
    f_tsr = h5py.File("data/dataset/{}_tsr_path.hdf5".format(env), "r")

    # calculate total size and preallocate space
    inputs_length = 0
    config_width = 7 if not use_tsr_config else 13
    voxels_length = 0
    voxel_shape = (32, 32, 32) if env == "kitchen" else (33, 33, 33)
    task_embedding_width = 4096 if use_text else 270
    for group in groups:
        paths = f_path["pick_place"][group]
        for obj in objs:
            T = paths[obj].shape[0] - 1
            inputs_length += T
            voxels_length += 1

        if use_reach:
            paths = f_path["reach"][group]
            for obj in objs:
                T = paths[obj].shape[0] - 1
                inputs_length += T
                voxels_length += 1

    result = {
        "inputs": np.zeros((inputs_length, 2*config_width), dtype=np.float32),
        "outputs": np.zeros((inputs_length, config_width), dtype=np.float32),
        "distances": np.zeros((inputs_length,), dtype=np.float32),
        "voxel_idxs": np.zeros((inputs_length,), dtype=np.int32),
        "task_embedding_idxs": np.zeros((inputs_length,), dtype=np.int32),
        "voxels": np.zeros((voxels_length, *voxel_shape), dtype=np.float32),
        "task_embeddings": np.zeros((voxels_length, task_embedding_width), dtype=np.float32),
    }

    inputs_offset = 0
    voxels_offset = 0
    for group in tqdm(groups, desc="Load dataset from disk"):
        voxels = f_voxel[group]

        # process pick_place path
        paths = f_path["pick_place"][group]
        task_embeddings = f_task_embedding["pick_place"][group]
        tsrs = f_tsr["pick_place"]["config"][group]
        distances = f_tsr["pick_place"]["distance"][group]
        for obj in objs:
            voxel = voxels[obj]
            task_embedding = task_embeddings[obj]
            path = np.divide(paths[obj], JOINT_RANGE)
            tsr = np.divide(tsrs[obj], TSR_RANGE)
            distance = distances[obj]
            if use_tsr_config:
                path = np.concatenate((path, tsr), axis=1)
            else:
                path = np.array(path)

            assert path.shape[0] == tsr.shape[0] and tsr.shape[0] == distance.shape[0]
            T = path.shape[0] - 1
            goal = path[-1]
            goals = np.tile(goal, (T, 1))
            starts = path[:-1]
            nexts = path[1:]
            result["inputs"][inputs_offset:inputs_offset+T] = np.concatenate((starts, goals), axis=1)
            result["outputs"][inputs_offset:inputs_offset+T] = nexts
            result["distances"][inputs_offset:inputs_offset+T] = distance[:-1]
            result["voxel_idxs"][inputs_offset:inputs_offset+T] = voxels_offset
            result["task_embedding_idxs"][inputs_offset:inputs_offset+T] = voxels_offset
            result["voxels"][voxels_offset] = voxel
            result["task_embeddings"][voxels_offset] = task_embedding

            voxels_offset += 1
            inputs_offset += T

        if use_reach:
            paths = f_path["reach"][group]
            task_embeddings = f_task_embedding["reach"][group]
            tsrs = f_tsr["reach"]["config"][group]
            distances = f_tsr["reach"]["distance"][group]
            for obj in objs:
                voxel = voxels[obj]
                task_embedding = task_embeddings[obj]
                path = np.divide(paths[obj], JOINT_RANGE)
                tsr = np.divide(tsrs[obj], TSR_RANGE)
                distance = distances[obj]
                if use_tsr_config:
                    path = np.concatenate((path, tsr), axis=1)
                else:
                    path = np.array(path)

                assert path.shape[0] == tsr.shape[0] and tsr.shape[0] == distance.shape[0]
                T = path.shape[0] - 1
                goal = path[-1]
                goals = np.tile(goal, (T, 1))
                starts = path[:-1]
                nexts = path[1:]
                result["inputs"][inputs_offset:inputs_offset+T] = np.concatenate((starts, goals), axis=1)
                result["outputs"][inputs_offset:inputs_offset+T] = nexts
                result["distances"][inputs_offset:inputs_offset+T] = distance[:-1]
                result["voxel_idxs"][inputs_offset:inputs_offset+T] = voxels_offset
                result["task_embedding_idxs"][inputs_offset:inputs_offset+T] = voxels_offset
                result["voxels"][voxels_offset] = voxel
                result["task_embeddings"][voxels_offset] = task_embedding

                voxels_offset += 1
                inputs_offset += T

    assert voxels_offset == voxels_length
    assert inputs_offset == inputs_length

    f_voxel.close()
    f_path.close()
    f_task_embedding.close()
    f_tsr.close()

    return result


class CoMPNetXDataset(Dataset):
    def __init__(self, env: str, use_text: bool, use_reach: bool, use_tsr_config: bool, use_manifold_distance: bool) -> None:
        super().__init__()
        data = load_train_data(env, use_text, use_reach, use_tsr_config)
        self.inputs = data["inputs"]
        self.outputs = data["outputs"]
        self.distances = data["distances"] if use_manifold_distance else None
        self.voxel_idxs = data["voxel_idxs"]
        self.task_embedding_idxs = data["task_embedding_idxs"]
        self.voxels = data["voxels"]
        self.task_embeddings = data["task_embeddings"]

        self.size = self.inputs.shape[0]
        self.use_manifold_distance = use_manifold_distance

    def __getitem__(self, index: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        result = [self.inputs[index],
                  self.outputs[index],
                  self.voxels[self.voxel_idxs[index]],
                  self.task_embeddings[self.task_embedding_idxs[index]]] + ([self.distances[index]] if self.use_manifold_distance else [])
        return tuple(torch.as_tensor(value) for value in result)

    def __len__(self) -> int:
        return self.size


if __name__ == "__main__":
    np.set_printoptions(threshold=sys.maxsize)
    result = load_train_data("bartender", use_text=False, use_reach=False, use_tsr_config=True)
    print(np.max(result["outputs"] * np.concatenate((JOINT_RANGE, BARTENDER_TSR_RANGE)), axis=0))
    print(np.min(result["outputs"] * np.concatenate((JOINT_RANGE, BARTENDER_TSR_RANGE)), axis=0))
