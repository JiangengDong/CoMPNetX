#!/usr/bin/env python2

"""
Copyright (c) 2020, University of California, San Diego
All rights reserved.

Author: Jiangeng Dong <jid103@ucsd.edu>
        Ahmed Qureshi <a1qureshi@ucsd.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# This file is used to test our algorithm on the following environments.
#   1. Bartender
#   1. Kitchen

from __future__ import absolute_import, division, print_function

import csv
import os
import pickle
import sys
import time
from argparse import ArgumentParser
from multiprocessing import Process

import h5py
import numpy as np
import openravepy as orpy
import rospkg
import yaml
from tqdm import tqdm

from OMPLInterface import (OMPLInterface, PlannerParameter, RPY2Transform, TSRChain)


def loadTestData(env):
    assert env in ["bartender", "kitchen"]

    with open("data/dataset/description.yaml", "r") as f:
        description = yaml.load(f, Loader=yaml.CLoader)
    groups = description[env]["test"]
    f_setup = h5py.File("data/dataset/{}_setup.hdf5".format(env), "r")

    result = {}
    for group in tqdm(groups, desc="Load dataset from disk"):
        result[group] = {}
        result[group]["obj_order"] = np.array(f_setup[group]["obj_order"])
        for obj_name in result[group]["obj_order"]:
            obj_data = f_setup[group][obj_name]
            if obj_name == "door":
                result[group][obj_name] = {
                    key: np.array(obj_data[key]) for key in ("initial_config",
                                                             "start_config", "goal_config",
                                                             "start_trans", "goal_trans",
                                                             "tsr_base", "tsr_bound0", "tsr_offset0", "tsr_bound1", "tsr_offset1")
                }
            else:
                result[group][obj_name] = {
                    key: np.array(obj_data[key]) for key in ("initial_config",
                                                             "start_config", "goal_config",
                                                             "start_trans", "goal_trans",
                                                             "tsr_bound", "tsr_offset")
                }
        for obj_name in ("tray", "recyclingbin"):
            result[group][obj_name] = {"start_trans": np.array(f_setup[group][obj_name]["start_trans"])}
    return result


def setOpenRAVE(visible=False, coll_checker="ode"):
    orEnv = orpy.Environment()
    if visible:
        orEnv.SetViewer('qtcoin')
        viewer = orEnv.GetViewer()
        viewer.SetCamera(np.array([[0.83316667, 0.27865777, -0.4776852, 2.19543743],
                                   [0.55220156, -0.46622736, 0.69116242, -2.45863342],
                                   [-0.03011214, -0.839632, -0.54232035, 2.58929443],
                                   [0.,          0.,         0.,          1.]]))
    orEnv.Reset()
    orEnv.SetDebugLevel(orpy.DebugLevel.Info)
    colchecker = orpy.RaveCreateCollisionChecker(orEnv, coll_checker)
    orEnv.SetCollisionChecker(colchecker)
    return orEnv


def loadTable(orEnv):
    table1 = orpy.RaveCreateKinBody(orEnv, '')
    table1.SetName('table1')
    table1.InitFromBoxes(np.array([[1.0, 0.4509, 0.5256, 0.2794, 0.9017, 0.5256]]), True)
    orEnv.Add(table1, True)

    table2 = orpy.RaveCreateKinBody(orEnv, '')
    table2.SetName('table2')
    table2.InitFromBoxes(np.array([[0.3777, -0.7303, 0.5256, 0.9017, 0.2794, 0.5256]]), True)
    orEnv.Add(table2, True)


def loadCabinet(orEnv):
    cabinet = orEnv.ReadRobotURI(os.path.abspath("./data/openrave_model/cabinets.robot.xml"))
    orEnv.Add(cabinet, True)
    cab_rot = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    T0_cab = RPY2Transform(0, 0, -np.pi / 2, 0.85, 1.28, 0)
    cabinet.SetTransform(np.array(T0_cab[0:3][:, 0:4]))
    cabinet.SetActiveDOFs([3])
    return cabinet


def setCabinet(cabinet, joint_value):
    cabinet.SetActiveDOFs([3])
    cabinet.SetActiveDOFValues([joint_value])


def loadObjects(orEnv):
    model_folder = os.path.abspath("./data/openrave_model")

    def loadFromFolder(name):
        return orEnv.ReadKinBodyXMLFile(os.path.join(model_folder, name))

    obj_dict = {
        "tray": loadFromFolder("tray.kinbody.xml"),
        "recyclingbin": loadFromFolder('recyclingbin.kinbody.xml'),
        "juice": loadFromFolder('juice_bottle.kinbody.xml'),
        "fuze_bottle": loadFromFolder('fuze_bottle.kinbody.xml'),
        "coke_can": loadFromFolder('coke_can.kinbody.xml'),
        "mugblack": loadFromFolder('mugblack.kinbody.xml'),
        "plasticmug": loadFromFolder('plasticmug.kinbody.xml'),
        "pitcher": loadFromFolder('pitcher.kinbody.xml'),
        "teakettle": loadFromFolder('teakettle.kinbody.xml'),
        "mugred": loadFromFolder('mugred.kinbody.xml')
    }

    return obj_dict


def loadBaxter(orEnv):
    baxter_path = rospkg.RosPack().get_path("baxter_description")
    urdf_path = os.path.join(baxter_path, "urdf", "baxter_sym.urdf")
    srdf_path = os.path.join(baxter_path, "urdf", "baxter_new.srdf")
    module = orpy.RaveCreateModule(orEnv, 'urdf')
    name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
    robot = orEnv.GetRobot(name)

    T0_baxter = RPY2Transform(0, 0, 0, 0.20, 0.15, 0.9242)
    robot.SetTransform(np.array(T0_baxter[0:3][:, 0:4]))

    # set initial configuration
    arm0dofs = [2, 3, 4, 5, 6, 7, 8]
    arm1dofs = [10, 11, 12, 13, 14, 15, 16]
    arm0initvals = [-0.386, 1.321, -0.06, 0.916, -0.349, -0.734, -1.84]
    arm1initvals = [0.216, 1.325, 0.173, 0.581, 0.490, -0.3883, 1.950]

    robot.SetActiveDOFs(arm0dofs + arm1dofs)
    robot.SetActiveDOFValues(arm0initvals + arm1initvals)
    robot.SetActiveManipulator(1)

    # open hands
    handdof = np.ones([2])
    robot.SetActiveDOFs([1, 9])
    robot.SetActiveDOFValues(handdof)
    return robot


def resetBaxter(robot):
    arm1dofs = [10, 11, 12, 13, 14, 15, 16]
    arm1initvals = [0.216, 1.325, 0.173, 0.581, 0.490, -0.3883, 1.950]
    robot.SetActiveDOFs(arm1dofs)
    robot.SetActiveDOFValues(arm1initvals)
    robot.WaitForController(0)


def initScene(orEnv, robot, cabinet, obj_dict, setup_dict):
    resetBaxter(robot)
    if "door" in setup_dict.keys():
        setCabinet(cabinet, setup_dict["door"]["start_trans"])
    for obj_name in ("recyclingbin", "tray",
                     "juice", "fuze_bottle", "coke_can",
                     "plasticmug", "teakettle",
                     "mugred", "mugblack", "pitcher"):
        if obj_name not in setup_dict.keys():
            continue
        obj = obj_dict[obj_name]
        obj_transform = setup_dict[obj_name]["start_trans"]
        orEnv.Add(obj)
        obj.SetTransform(np.array(obj_transform[0:3][:, 0:4]))
    time.sleep(0.1)


def cleanupObject(orEnv, obj_name, obj, setup_dict):
    if obj_name == "door":
        setCabinet(obj, setup_dict["goal_trans"])
    elif obj_name in ("juice", "fuze_bottle", "coke_can"):
        orEnv.Remove(obj)
    elif obj_name in ("mugblack", "mugred", "plasticmug", "pitcher", "teakettle"):
        goal_transform = setup_dict["goal_trans"]
        obj.SetTransform(np.array(goal_transform[0:3][:, 0:4]))
    else:
        raise NotImplementedError
    time.sleep(0.1)


def test_per_scene(scene_name, scene_setup, param, args):
    # preparation
    orEnv = setOpenRAVE(args.visible, "ode")
    loadTable(orEnv)
    cabinet = None if args.env == "bartender" else loadCabinet(orEnv)
    obj_dict = loadObjects(orEnv)
    robot = loadBaxter(orEnv)
    planner = OMPLInterface(orEnv, robot, loglevel=args.log_level)

    result_dict = {}
    # initialize scene. Put all the objects to their start position.
    print("\n========== scene name: %s ==========" % scene_name)
    initScene(orEnv, robot, cabinet, obj_dict, scene_setup)
    result_dict[scene_name] = {}

    obj_names = scene_setup["obj_order"]
    print("Object order: %s" % str(obj_names))
    for obj_name in obj_names:
        print("Planning for %s ..." % obj_name)
        obj_setup = scene_setup[obj_name]
        obj = obj_dict[obj_name] if obj_name != "door" else cabinet

        # unpack values
        start_config = obj_setup["start_config"]
        goal_config = obj_setup["goal_config"]
        start_trans = obj_setup["start_trans"]
        goal_trans = obj_setup["goal_trans"]
        # go to start position
        robot.SetActiveDOFValues(start_config)
        if obj_name != "door":
            robot.Grab(obj)
        robot.WaitForController(0)
        # show the start and goal
        if args.visible:
            robot.SetActiveDOFValues(start_config)
            robot.WaitForController(0)
            if obj_name == "door":
                setCabinet(cabinet, start_trans)
            time.sleep(2)
            robot.SetActiveDOFValues(goal_config)
            robot.WaitForController(0)
            if obj_name == "door":
                setCabinet(cabinet, goal_trans)
            time.sleep(2)
            robot.SetActiveDOFValues(start_config)
            robot.WaitForController(0)
            if obj_name == "door":
                setCabinet(cabinet, start_trans)
            time.sleep(0.1)
        # TSR parameters
        param.clearTSRChains()
        if obj_name != "door":
            T0_w = obj_setup["goal_trans"]
            Tw_e = obj_setup["tsr_offset"]
            Bw = obj_setup["tsr_bound"]
            param.addTSRChain(TSRChain().addTSR(T0_w, Tw_e, Bw))
        else:
            T0_w = obj_setup["tsr_base"]
            Tw_e_0 = obj_setup["tsr_offset0"]
            Bw_0 = obj_setup["tsr_bound0"]
            Tw_e_1 = obj_setup["tsr_offset1"]
            Bw_1 = obj_setup["tsr_bound1"]
            param.addTSRChain(TSRChain(mimic_body_name="cabinets", mimic_body_index=[3]).addTSR(T0_w, Tw_e_0, Bw_0).addTSR(T0_w, Tw_e_1, Bw_1))
        # voxel and task representation embeddings
        param.mpnet_parameter.voxel_dataset = "/%s/%s" % (scene_name, obj_name)
        param.mpnet_parameter.ohot_dataset = "/%s/%s" % (scene_name, obj_name)
        # plan
        resp, t_time, traj = planner.solve(start_config, goal_config, param)
        time.sleep(1)   # I don't know why, but removing this line may cause segment fault, so be careful
        robot.WaitForController(0)
        # show the result
        if args.visible and resp is True:
            robot.GetController().SetPath(traj)
            cabinet.GetController().SetPath(traj)
            robot.WaitForController(0)
            cabinet.WaitForController(0)
        # go to goal position
        robot.ReleaseAllGrabbed()
        robot.WaitForController(0)
        robot.SetActiveDOFValues(goal_config)
        cleanupObject(orEnv, obj_name, obj, obj_setup)
        # save result and clean up
        result_dict[scene_name][obj_name] = t_time

    if not os.path.exists(os.path.join(args.result_dir, "temp")):
        os.makedirs(os.path.join(args.result_dir, "temp"))
    result_filename = os.path.join(args.result_dir, "temp", "result_{}.p".format(scene_name))
    with open(result_filename, "wb") as f:
        pickle.dump(result_dict, f)


def mergeResults(work_dir, scene_names):
    all_dict = {}
    for scene_name in scene_names:
        result_filename = os.path.join(work_dir, "temp", "result_{}.p".format(scene_name))
        if not os.path.exists(result_filename):
            print(result_filename)
            continue

        with open(result_filename, "rb") as f:
            result_dict = pickle.load(f)
        all_dict[scene_name] = result_dict[scene_name]

    return all_dict


def saveResultCSV(filename, cols, result_dict):
    fieldnames = ["scene_name"] + list(cols)
    with open(filename, "wb") as f:
        csv_writer = csv.DictWriter(f, fieldnames=fieldnames)
        csv_writer.writeheader()
        for (scene_name, scene_result) in result_dict.items():
            row = {"scene_name": scene_name}
            row.update(scene_result)
            csv_writer.writerow(row)


def printStatistics(filename, cols):
    data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=cols)
    inf_mask = np.isinf(data)
    nan_mask = np.isnan(data)
    success_mask = np.logical_and(np.logical_not(inf_mask), np.logical_not(nan_mask))

    success_count = success_mask.sum()
    valid_count = success_mask.sum() + inf_mask.sum()
    accuracy = float(success_count) / float(valid_count)
    print("accuracy:  ", accuracy, " (", success_count, "/", valid_count, ")")

    success_row_mask = np.logical_and.reduce(success_mask, 1)
    success_rows = data[success_row_mask]
    average_time = success_rows.sum(axis=1).mean(axis=0)
    print("mean time: ", average_time)


def test(args):
    all_setup_dict = loadTestData(args.env)

    param = PlannerParameter()
    param.solver_parameter.type = args.algorithm
    param.solver_parameter.time = 25 if args.env == "bartender" else 60
    param.solver_parameter.range = 0.05
    param.constraint_parameter.type = args.space
    param.constraint_parameter.tolerance = 1e-3
    param.constraint_parameter.delta = 0.05
    if args.space == "proj":
        pass
    elif args.space == "atlas":
        param.atlas_parameter.rho = 1.5
        param.atlas_parameter.exploration = 0.9
        param.atlas_parameter.epsilon = 0.01
    elif args.space in ("tangent-bundle", "tangent_bundle", "tb"):
        param.atlas_parameter.rho = 2.0
        param.atlas_parameter.exploration = 0.9
        param.atlas_parameter.epsilon = 0.01
    # MPNet parameters
    param.mpnet_parameter.pnet_path = os.path.join(args.work_dir, "torchscript", "pnet.pt")
    param.mpnet_parameter.dnet_path = os.path.join(args.work_dir, "torchscript", "dnet.pt") if args.use_dnet else ""
    param.mpnet_parameter.voxel_path = os.path.join(args.work_dir, "embedding", "voxel.hdf5")
    param.mpnet_parameter.ohot_path = os.path.join(args.work_dir, "embedding", "task_embedding.hdf5")
    param.mpnet_parameter.predict_tsr = args.use_tsr

    # write settings to a file
    with open(os.path.join(args.result_dir, "args.xml"), "w") as f:
        f.write("<parameter>\n")
        f.write(str(param))
        f.write("</parameter>\n")
    with open(os.path.join(args.result_dir, "args.yaml"), "w") as f:
        yaml.dump(args.__dict__, f)

    # run each test in a separate process
    for (scene_name, scene_setup) in all_setup_dict.items():
        p = Process(target=lambda: test_per_scene(scene_name, scene_setup, param, args))
        p.start()
        p.join()

    # collect results, merge into a CSV, and print statistics
    result_dict = mergeResults(args.result_dir, all_setup_dict.keys())
    result_csv_path = os.path.join(args.result_dir, "result_{}_{}_{}.csv".format(args.algorithm, args.space, ("dnet" if args.use_dnet else "no-dnet")))
    if args.env == "bartender":
        cols = ["fuze_bottle", "juice", "coke_can", "plasticmug", "teakettle"]
        col_indices = [1, 2, 3, 4, 5]
    elif args.env == "kitchen":
        cols = ["fuze_bottle", "juice", "coke_can", "mugred", "mugblack", "pitcher", "door"]
        col_indices = [1, 2, 3, 4, 5, 6, 7]
    else:
        cols = []
        col_indices = []
    saveResultCSV(result_csv_path, cols, result_dict)
    printStatistics(result_csv_path, col_indices)


def get_args():
    parser = ArgumentParser(description="A all-in-one script to test RRTConnect, CoMPNet and CoMPNetX algorithm.")
    parser.add_argument("-l", "--log_level", type=int, choices=(0, 1, 2, 3, 4), default=2, help="Set log level. Lower level generates more logs.")
    parser.add_argument("-v", "--visible", action="store_true", help="Show a 3D visualization of the planning.")
    parser.add_argument("-d", "--work_dir", required=True,
                        help="Output directory during training. Setting, data and models will be read from this directory automatically.")
    parser.add_argument("-s", "--space", choices=("proj", "atlas", "tb"), default="atlas", help="Constraint-adherence method to use.")
    parser.add_argument("-a", "--algorithm", choices=("compnetx", "rrtconnect"), default="compnetx",
                        help="Select an algorithm. Choose `compnetx` for both CoMPNet and CoMPNetX, and the exact algorithm will be selected according to the settings in the work directory.")
    parser.add_argument("-p", "--use_dnet", action="store_true", help="Use neural projector.")
    return parser.parse_args()


def process_args(args):
    if sys.version_info[0] != 2:
        print("This script only works under python2, because of the limitation of OpenRAVE. Exit.")
        exit()
    if not os.path.exists(args.work_dir):
        print("%s does not exist. Exit. " % args.work_dir)
        exit()

    with open(os.path.join(args.work_dir, "args.yaml"), "r") as f:
        training_args = yaml.load(f, Loader=yaml.CLoader)
    args.env = training_args["env"]
    args.use_text = training_args["use_text"]
    args.use_tsr = training_args["use_tsr"]
    args.torchscript_dir = training_args["torchscript_dir"]
    args.embedding_dir = training_args["embedding_dir"]

    args.result_dir = os.path.join(args.work_dir, "result")
    if not os.path.exists(args.result_dir):
        os.makedirs(args.result_dir)

    if args.use_dnet and not os.path.exists(os.path.join(args.torchscript_dir, "dnet.pt")):
        print("Cannot find neural projector. use_dnet is set to false.")
        args.use_dnet = False

    return args


def print_args(args):
    print("Test will run with the following arguments: ")
    keys = list(args.__dict__.keys())
    keys.sort()
    for key in keys:
        value = args.__dict__[key]
        print("\t%s: %s" % (key, value))


if __name__ == "__main__":
    args = process_args(get_args())
    print_args(args)
    test(args)
