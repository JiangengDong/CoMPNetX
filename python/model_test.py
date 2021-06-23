#!/usr/bin/env python2
# This file is used to test our algorithm on the following environments.
#   1. Bartender
#   1. Kitchen
# The data storage for each environments are so different that we have to use some inelegent way to process the data.

from __future__ import absolute_import, division, print_function

import numpy as np
import os
import pickle
import rospkg
import time
from argparse import ArgumentParser
from multiprocessing import Process
import h5py
import yaml
from tqdm import tqdm

import openravepy as orpy

from OMPLInterface import RPY2Transform, OMPLInterface, PlannerParameter, TSRChain


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
        "plasticmug": loadFromFolder('mugred.kinbody.xml'),
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
        setCabinet(obj, setup_dict["door"]["goal_trans"])
    elif obj_name in ("juice", "fuze_bottle", "coke_can"):
        orEnv.Remove(obj)
    elif obj_name in ("mugblack", "mugred", "plasticmug", "pitcher", "teakettle"):
        goal_transform = setup_dict["goal_trans"]
        obj.SetTransform(np.array(goal_transform[0:3][:, 0:4]))
    else:
        raise NotImplementedError
    time.sleep(0.1)


def test(args):
    all_setup_dict = loadTestData(args.env)

    param = PlannerParameter()
    param.solver_parameter.type = args.algorithm
    param.solver_parameter.time = 300
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
    param.mpnet_parameter.dnet_path = os.path.join(args.work_dir, "torchscript", "dnet.pt") if False else ""  # TODO: add use_dnet
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

    # preparation
    orEnv = setOpenRAVE(args.visible, "ode")
    loadTable(orEnv)
    cabinet = None if args.env == "bartender" else loadCabinet(orEnv)
    obj_dict = loadObjects(orEnv)
    robot = loadBaxter(orEnv)
    planner = OMPLInterface(orEnv, robot, loglevel=args.log_level)

    # main loop
    result_dict = {}
    for (scene_name, scene_setup) in all_setup_dict.items():
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
            if obj_name == "door":  # door uses a different setting, so skip it here
                cleanupObject(orEnv, obj_name, obj, obj_setup)
                continue

            # unpack values
            T0_w = obj_setup["goal_trans"]
            Tw_e = obj_setup["tsr_offset"]
            Bw = obj_setup["tsr_bound"]
            startik = obj_setup["start_config"]
            goalik = obj_setup["goal_config"]
            # go to start position
            robot.SetActiveDOFValues(startik)
            robot.Grab(obj)
            robot.WaitForController(0)
            # show the start and goal
            if args.visible:
                robot.SetActiveDOFValues(startik)
                robot.WaitForController(0)
                time.sleep(2)
                robot.SetActiveDOFValues(goalik)
                robot.WaitForController(0)
                time.sleep(2)
                robot.SetActiveDOFValues(startik)
                robot.WaitForController(0)
                time.sleep(0.1)
            # plan
            param.clearTSRChains()
            param.addTSRChain(TSRChain().addTSR(T0_w, Tw_e, Bw))
            param.mpnet_parameter.voxel_dataset = "/%s/%s" % (scene_name, obj_name)
            param.mpnet_parameter.ohot_dataset = "/%s/%s" % (scene_name, obj_name)
            resp, t_time, traj = planner.solve(startik, goalik, param)
            time.sleep(1)   # I don't know why, but removing this line may cause segment fault, so be careful
            robot.WaitForController(0)
            # show the result
            if args.visible and resp is True:
                robot.GetController().SetPath(traj)
                robot.WaitForController(0)
            # go to goal position
            robot.ReleaseAllGrabbed()
            robot.WaitForController(0)
            robot.SetActiveDOFValues(goalik)
            cleanupObject(orEnv, obj_name, obj, obj_setup)
            # save result and clean up
            result_dict[scene_name][obj_name] = t_time

        with open(os.path.join(args.result_dir, "result.yaml"), "w") as f:
            yaml.dump(result_dict, f)


def get_args():
    parser = ArgumentParser(description="A all-in-one script to test the CoMPNet and CoMPNetX algorithm.")
    parser.add_argument("--log_level", choices=(0, 1, 2, 3, 4), default=2, help="Lower level generates more logs")
    parser.add_argument("--visible", action="store_true")
    parser.add_argument("--space", choices=("proj", "atlas", "tb"), default="proj")
    parser.add_argument("--work_dir", default="./data/experiments/exp1",
                        help="Training's output directory. Setting, data and models will be read from this directory automatically.")
    parser.add_argument("--algorithm", choices=("compnetx", "rrtconnect"), default="compnetx")
    return parser.parse_args()


def process_args(args):
    if not os.path.exists(args.work_dir):
        print("%s does not exist. Exit. " % args.work_dir)
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
