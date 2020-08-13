#!/usr/bin/env python2
# This file is used to test our algorithm on the following environments.
#   1. Bartender (not implemented yet)
#   1. Kitchen
#   1. kitchen-v2
# The data storage for each environments are so different that we have to use some inelegent way to process the data.

import numpy as np
import os
import pickle
import rospkg
import time
from multiprocessing import Process

import openravepy as orpy

from OMPLInterface import RPY2Transform, OMPLInterface, PlannerParameter, TSRChain


def setOpenRAVE(visible=False, coll_checker="fcl"):
    orEnv = orpy.Environment()
    if visible:
        orEnv.SetViewer('qtcoin')
    orEnv.Reset()
    orEnv.SetDebugLevel(orpy.DebugLevel.Info)
    colchecker = orpy.RaveCreateCollisionChecker(orEnv, 'fcl') if coll_checker == "fcl" else orpy.RaveCreateCollisionChecker(orEnv, 'ode')
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
        "mugred": loadFromFolder('mugred.kinbody.xml'),
        "pitcher": loadFromFolder('pitcher.kinbody.xml'),
        "teakettle": loadFromFolder('teakettle.kinbody.xml')
    }
    obj_dict["plasticmug"] = obj_dict["mugred"]
    obj_dict["mugblack2"] = obj_dict["mugred"]
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


def getObjectSetupsFactory(task):
    # The most dirty function in this file. Hardcoded branch-_-
    def getObjectSetupsBartender(scene_setup, env_setup):
        scene_setup_target = env_setup["targets"]
        obj_setups = {
            "recyclingbin": {
                "start": {
                    "transform": scene_setup_target["recyclingbin"]["T0_w2"]
                }
            },
            "tray": {
                "start": {
                    "transform": scene_setup_target["tray"]["T0_w2"]
                }
            },
            "juice": {
                "start": {
                    "transform": scene_setup["juice"]["T0_w"],
                    "ik": scene_setup["juice"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_target["juice"]["T0_w2"],
                    "ik": scene_setup_target["juice"]["rgconf"]
                },
                "offset": scene_setup["juice"]["Tw_e"],
                "bound": scene_setup_target["juice"]["Bw2"],
                "bound_reach": scene_setup["juice"]["Bw"],
                "initial": {
                    "ik": scene_setup["juice"]["riconf"]
                },
            },
            "fuze_bottle": {
                "start": {
                    "transform": scene_setup["fuze_bottle"]["T0_w"],
                    "ik": scene_setup["fuze_bottle"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_target["fuze_bottle"]["T0_w2"],
                    "ik": scene_setup_target["fuze_bottle"]["rgconf"]
                },
                "offset": scene_setup["fuze_bottle"]["Tw_e"],
                "bound": scene_setup_target["fuze_bottle"]["Bw2"],
                "bound_reach": scene_setup["fuze_bottle"]["Bw"],
                "initial": {
                    "ik": scene_setup["fuze_bottle"]["riconf"]
                },
            },
            "coke_can": {
                "start": {
                    "transform": scene_setup["coke_can"]["T0_w"],
                    "ik": scene_setup["coke_can"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_target["coke_can"]["T0_w2"],
                    "ik": scene_setup_target["coke_can"]["rgconf"]
                },
                "offset": scene_setup["coke_can"]["Tw_e"],
                "bound": scene_setup_target["coke_can"]["Bw2"],
                "bound_reach": scene_setup["coke_can"]["Bw"],
                "initial": {
                    "ik": scene_setup["coke_can"]["riconf"]
                },
            },
            "plasticmug": {
                "start": {
                    "transform": scene_setup["plasticmug"]["T0_w"],
                    "ik": scene_setup["plasticmug"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_target["plasticmug"]["T0_w2"],
                    "ik": scene_setup_target["plasticmug"]["rgconf"]
                },
                "offset": scene_setup["plasticmug"]["Tw_e"],
                "bound": scene_setup_target["plasticmug"]["Bw2"],
                "bound_reach": scene_setup["plasticmug"]["Bw"],
                "initial": {
                    "ik": scene_setup["plasticmug"]["riconf"]
                },
            },
            "teakettle": {
                "start": {
                    "transform": scene_setup["teakettle"]["T0_w"],
                    "ik": scene_setup["teakettle"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_target["teakettle"]["T0_w2"],
                    "ik": scene_setup_target["teakettle"]["rgconf"]
                },
                "offset": scene_setup["teakettle"]["Tw_e"],
                "bound": scene_setup_target["teakettle"]["Bw2"],
                "bound_reach": scene_setup["teakettle"]["Bw"],
                "initial": {
                    "ik": scene_setup["teakettle"]["riconf"]
                },
            }
        }
        return obj_setups

    def getObjectSetupsKitchen(scene_setup, env_setup):
        scene_setup_initial = scene_setup["initial"]
        obj_setups = {
            "recyclingbin": {
                "start": {
                    "transform": scene_setup_initial["recyclingbin"]["T0_w2"]
                }
            },
            "tray": {
                "start": {
                    "transform": scene_setup["tray"]["T0_w2"]
                }
            },
            "door": {
                "start": {
                    "value": scene_setup_initial["door"]["door_start"],
                    "ik": None  # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                },
                "initial": {
                    "ik": None
                },
                "goal": {
                    "value": scene_setup_initial["door"]["door_end"],
                    "ik": None  # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                },
                "offset": None,   # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                "bound": None    # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
            },
            "mugblack": {
                "start": {
                    "transform": scene_setup_initial["mugblack"]["T0_w0"],
                    "ik": scene_setup_initial["mugblack"]["rsconf"]
                },
                "initial": {
                    "ik": scene_setup_initial["mugblack"]["riconf"]
                },
                "goal": {
                    "transform": scene_setup["mugblack"]["T0_w2"],
                    "ik": scene_setup["mugblack"]["rgconf"]
                },
                "offset": scene_setup["mugblack"]["Tw_e"],
                "bound": scene_setup["mugblack"]["Bw2"],
                "bound_reach": scene_setup_initial["mugblack"]["Bw"]
            },
            "mugred": {
                "start": {
                    "transform": scene_setup_initial["plasticmug"]["T0_w0"],
                    "ik": scene_setup_initial["plasticmug"]["rsconf"]
                    # "ik": np.array([1.44845044, 0.70520101, 0.26510866, -0.48933351, -1.77352356, 0.39498159, 0.09028754])
                },
                "initial": {
                    "ik": scene_setup["mugblack"]["rgconf"]
                },
                "goal": {
                    "transform": scene_setup["plasticmug"]["T0_w2"],
                    "ik": scene_setup["plasticmug"]["rgconf"]
                    # "ik": np.array([1.4753715, 0.61263517, 1.30811848, 0.14469196, -0.04560525, -1.15197346, -1.89523285])
                },
                "offset": scene_setup["plasticmug"]["Tw_e"],
                "bound": scene_setup["plasticmug"]["Bw2"],
                "bound_reach": scene_setup_initial["plasticmug"]["Bw"]
            },
            "pitcher": {
                "start": {
                    "transform": scene_setup["pitcher"]["T0_w"],
                    "ik": scene_setup["pitcher"]["rsconf"]
                },
                "initial": {
                    "ik": scene_setup["pitcher"]["riconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["pitcher"]["T0_w2"],
                    "ik": scene_setup_initial["pitcher"]["rgconf"]
                },
                "offset": scene_setup["pitcher"]["Tw_e"],
                "bound": scene_setup_initial["pitcher"]["Bw2"],
                "bound_reach": scene_setup["pitcher"]["Bw"]
            },
            "juice": {
                "start": {
                    "transform": scene_setup["juice"]["T0_w"],
                    "ik": scene_setup["juice"]["rsconf"]
                },
                "initial": {
                    "ik": scene_setup["juice"]["riconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["juice"]["T0_w2"],
                    "ik": scene_setup_initial["juice"]["rgconf"]
                },
                "offset": scene_setup["juice"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi]),
                "bound_reach": scene_setup["juice"]["Bw"]
            },
            "fuze_bottle": {
                "start": {
                    "transform": scene_setup["fuze_bottle"]["T0_w"],
                    "ik": scene_setup["fuze_bottle"]["rsconf"]
                },
                "initial": {
                    "ik": scene_setup["fuze_bottle"]["riconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["fuze_bottle"]["T0_w2"],
                    "ik": scene_setup_initial["fuze_bottle"]["rgconf"]
                },
                "offset": scene_setup["fuze_bottle"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi]),
                "bound_reach": scene_setup["fuze_bottle"]["Bw"]
            },
            "coke_can": {
                "start": {
                    "transform": scene_setup["coke_can"]["T0_w"],
                    "ik": scene_setup["coke_can"]["rsconf"]
                },
                "initial": {
                    "ik": scene_setup["coke_can"]["riconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["coke_can"]["T0_w2"],
                    "ik": scene_setup_initial["coke_can"]["rgconf"]
                },
                "offset": scene_setup["coke_can"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi]),
                "bound_reach": scene_setup["coke_can"]["Bw"]
            },
        }
        obj_setups["plasticmug"] = obj_setups["mugred"]
        obj_setups["mugblack2"] = obj_setups["mugred"]
        return obj_setups

    def getObjectSetupsKitchenv2(scene_setup, env_setup):
        scene_setup_initial = scene_setup["initial"]
        obj_setups = {
            "recyclingbin": {
                "start": {
                    "transform": scene_setup_initial["recyclingbin"]["T0_w2"]
                }
            },
            "tray": {
                "start": {
                    "transform": scene_setup["tray"]["T0_w2"]
                }
            },
            "door": {
                "start": {
                    "value": scene_setup_initial["door"]["door_start"],
                    "ik": None  # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                },
                "goal": {
                    "value": scene_setup_initial["door"]["door_end"],
                    "ik": None  # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                },
                "offset": None,   # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
                "bound": None    # TODO: This field is not used now, so it is ignored here. Change it to the exact value.
            },
            "mugblack": {
                "start": {
                    "transform": scene_setup_initial["mugblack"]["T0_w0"],
                    "ik": scene_setup_initial["mugblack"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup["mugblack"]["T0_w2"],
                    "ik": scene_setup["mugblack"]["rgconf"]
                },
                "offset": scene_setup["mugblack"]["Tw_e"],
                "bound": scene_setup["mugblack"]["Bw2"]
            },
            "mugred": {
                "start": {
                    "transform": scene_setup["mugblack2"]["T0_w0"],
                    "ik": scene_setup["mugblack2"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["mugblack2"]["T0_w2"],
                    "ik": scene_setup_initial["mugblack2"]["rgconf"]
                },
                "offset": scene_setup["mugblack2"]["Tw_e"],
                "bound": scene_setup_initial["mugblack2"]["Bw2"]
            },
            "pitcher": {
                "start": {
                    "transform": scene_setup["pitcher"]["T0_w"],
                    "ik": scene_setup["pitcher"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup["pitcher_target"]["T0_w2"],
                    "ik": scene_setup["pitcher_target"]["rgconf"]
                },
                "offset": scene_setup["pitcher_target"]["Tw_e"],
                "bound": scene_setup["pitcher_target"]["Bw2"]
            },
            "juice": {
                "start": {
                    "transform": scene_setup["juice"]["T0_w"],
                    "ik": scene_setup["juice"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["juice"]["T0_w2"],
                    "ik": scene_setup_initial["juice"]["rgconf"]
                },
                "offset": scene_setup["juice"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi])
            },
            "fuze_bottle": {
                "start": {
                    "transform": scene_setup["fuze_bottle"]["T0_w"],
                    "ik": scene_setup["fuze_bottle"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["fuze_bottle"]["T0_w2"],
                    "ik": scene_setup_initial["fuze_bottle"]["rgconf"]
                },
                "offset": scene_setup["fuze_bottle"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi])
            },
            "coke_can": {
                "start": {
                    "transform": scene_setup["coke_can"]["T0_w"],
                    "ik": scene_setup["coke_can"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["coke_can"]["T0_w2"],
                    "ik": scene_setup_initial["coke_can"]["rgconf"]
                },
                "offset": scene_setup["coke_can"]["Tw_e"],
                "bound": np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                 -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi])
            },
        }
        obj_setups["plasticmug"] = obj_setups["mugred"]
        obj_setups["mugblack2"] = obj_setups["mugred"]
        return obj_setups

    if task == "bartender":
        return getObjectSetupsBartender
    elif task == "kitchen":
        return getObjectSetupsKitchen
    elif task == "kitchen-v2":
        return getObjectSetupsKitchenv2
    else:
        raise NotImplementedError



def initSceneFactory(task):
    def initSceneBartender(orEnv, robot, cabinet, obj_dict, obj_setups):
        resetBaxter(robot)
        for obj_name in ("recyclingbin", "tray", 'coke_can', 'juice', 'fuze_bottle', 'plasticmug', 'teakettle'):
            obj = obj_dict[obj_name]
            obj_transform = obj_setups[obj_name]["start"]["transform"]
            orEnv.Add(obj)
            obj.SetTransform(np.array(obj_transform[0:3][:, 0:4]))
        time.sleep(0.1)

    def initSceneKitchen(orEnv, robot, cabinet, obj_dict, obj_setups):
        resetBaxter(robot)
        setCabinet(cabinet, obj_setups["door"]["start"]["value"])
        for obj_name in ("recyclingbin", "tray", "juice", "fuze_bottle", "coke_can", "mugblack", "mugred", "pitcher"):
            obj = obj_dict[obj_name]
            obj_transform = obj_setups[obj_name]["start"]["transform"]
            orEnv.Add(obj)
            obj.SetTransform(np.array(obj_transform[0:3][:, 0:4]))
        time.sleep(0.1)

    if task == "bartender":
        return initSceneBartender
    elif task == "kitchen":
        return initSceneKitchen
    elif task == "kitchen-v2":
        return initSceneKitchen
    else:
        raise NotImplementedError


def cleanupObject(orEnv, obj_name, obj, obj_setup):
    if obj_name == "door":
        door_goal = obj_setup["goal"]["value"]
        obj.SetActiveDOFValues([door_goal])
        print("----------open " + obj_name)
    elif obj_name in ("juice", "fuze_bottle", "coke_can"):
        orEnv.Remove(obj)
        print("--------remove " + obj_name)
    elif obj_name in ("mugblack", "mugblack2", "mugred", "plasticmug", "pitcher", "teakettle"):
        goal_transform = obj_setup["goal"]["transform"]
        obj.SetTransform(np.array(goal_transform[0:3][:, 0:4]))
        print("----------move " + obj_name)
    else:
        raise NotImplementedError
    time.sleep(0.1)


def main(env_range=None):
    # settings
    visible = False
    coll_checker = "fcl"
    task = "bartender"
    output_folder = "data/result/tsr_value_5"
    message = "Shrink pitcher."

    # env_range = list(range(0, 5))
    # scene_range = list(range(0, 5))
    # setup_file = "data/experiment_setup/esc_dict_door5_5_p0.9.p"
    env_range = env_range or list(range(0, 70))
    scene_range = list(range(0, 120))
    setup_file = "data/experiment_setup/esp_dict_rkitchen_70_30.p"

    param = PlannerParameter()
    param.solver_parameter.type = "rrtconnect"
    param.solver_parameter.time = 180
    param.constraint_parameter.type = "atlas"
    param.constraint_parameter.tolerance = 1e-3
    param.constraint_parameter.delta = 0.05
    param.atlas_parameter.rho = 2.0
    param.atlas_parameter.exploration = 0.9
    param.atlas_parameter.epsilon = 0.01
    # write settings to a file
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    with open(os.path.join(output_folder, "settings.xml"), "w") as f:
        f.write("<task>%s</task>\n" % task)
        f.write("<collision_checker>%s</collision_checker>\n" % coll_checker)
        f.write("<message>%s</message>\n" % message)
        f.write("<parameter>\n")
        f.write(str(param))
        f.write("</parameter>\n")

    # preparation
    orEnv = setOpenRAVE(visible, coll_checker)
    loadTable(orEnv)
    cabinet = None if task == "bartender" else loadCabinet(orEnv)
    obj_dict = loadObjects(orEnv)
    robot = loadBaxter(orEnv)
    planner = OMPLInterface(orEnv, robot, loglevel=2)
    initScene = initSceneFactory(task)
    getObjectSetups = getObjectSetupsFactory(task)

    # main loop
    for e in env_range:
        with open("data/experiment_setup/esp_split/env%d" % e, "rb") as f:
            setup_dict = pickle.load(f)
        env_key = "env_%d" % e
        if env_key not in setup_dict.keys():
            continue
        env_setup = setup_dict[env_key]
        time_dict = {}

        for s in scene_range:
            scene_key = "s_%d" % s
            if scene_key not in env_setup.keys():
                continue
            scene_setup = env_setup[scene_key]
            time_dict[scene_key] = {}

            # initialize scene. Put all the objects to their start position.
            print("\n==========env_no: %d====s_no: %d==========" % (e, s))
            obj_setups = getObjectSetups(scene_setup, env_setup)
            initScene(orEnv, robot, cabinet, obj_dict, obj_setups)

            obj_names = scene_setup["obj_order"]
            print("Object order: %s" % str(obj_names))
            for obj_name in obj_names:
                print("Planning for %s ..." % obj_name)
                if "path_pick_place" not in scene_setup[obj_name].keys() or "path_pick_place" not in scene_setup[obj_name].keys():
                    print("Cannot find path.")
                    continue
                obj_setup = obj_setups[obj_name]
                obj = obj_dict[obj_name] if obj_name != "door" else cabinet
                # unpack values
                T0_w = obj_setup["goal"]["transform"] if obj_name != "door" else None
                Tw_e = obj_setup["offset"]
                Bw = obj_setup["bound"]
                startik = obj_setup["start"]["ik"]
                initik = obj_setup["initial"]["ik"]
                goalik = obj_setup["goal"]["ik"]

                isObjectValid = all([element is not None for element in [T0_w, Tw_e, Bw, startik, goalik]])
                if isObjectValid:
                    path_pick_place = scene_setup[obj_name]["path_pick_place"]
                    path_reach = scene_setup[obj_name]["path_reach"]
                    robot.SetActiveDOFValues(startik)
                    robot.Grab(obj)
                    robot.WaitForController(0)
                    if visible:
                        robot.SetActiveDOFValues(path_pick_place[0])
                        robot.WaitForController(0)
                        time.sleep(2)
                        robot.SetActiveDOFValues(path_pick_place[-1])
                        robot.WaitForController(0)
                        time.sleep(2)
                        robot.SetActiveDOFValues(path_reach[0])
                        robot.WaitForController(0)
                        time.sleep(0.1)
                        robot.SetActiveDOFValues(path_reach[-1])
                        robot.WaitForController(0)
                        time.sleep(0.1)
                    # plan
                    param.clearTSRChains()
                    param.addTSRChain(TSRChain().addTSR(T0_w, Tw_e, Bw))
                    # resp, t_time, traj = planner.solve(startik, goalik, param)
                    resp, t_time, traj = False, np.nan, None
                    tsr_pick_place, dist_pick_place = planner.distanceToManifold(path_pick_place, param, startik, goalik)
                    print("max dist pick place: %f" % max(dist_pick_place))
                    time.sleep(0.1)

                    T0_w = obj_setup["start"]["transform"]
                    Bw = obj_setup["bound_reach"]
                    param.clearTSRChains()
                    param.addTSRChain(TSRChain().addTSR(T0_w, Tw_e, Bw))
                    tsr_reach = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]*len(path_reach)
                    _, dist_reach = planner.distanceToManifold(path_reach, param, initik, startik)
                    print("max dist reach: %f" % max(dist_reach))
                    time.sleep(0.1)
                    robot.WaitForController(0)

                    if visible and resp is True:
                        robot.GetController().SetPath(traj)
                        robot.WaitForController(0)
                    robot.ReleaseAllGrabbed()
                    robot.WaitForController(0)
                    robot.SetActiveDOFValues(goalik)

                    time_dict[scene_key][obj_name] = {"tsr_pick_place": tsr_pick_place, "tsr_reach": tsr_reach, "dist_pick_place": dist_pick_place, "dist_reach": dist_reach}

                elif obj_name == "door":
                    T0_w0 = scene_setup["initial"]["door"]["T0_w0"]
                    Tw0_e = scene_setup["initial"]["door"]["Tw0_e"]
                    Tw1_e = scene_setup["initial"]["door"]["Tw1_e"]
                    Bw0 = scene_setup["initial"]["door"]["Bw0"]
                    Bw1 = scene_setup["initial"]["door"]["Bw1"]

                    startik = scene_setup["initial"]["door"]["rsconf"]
                    goalik = scene_setup["initial"]["door"]["rgconf"]
                    initik = scene_setup["initial"]["door"]["riconf"]

                    path_pick_place = scene_setup["initial"]["door"]["path_pick_place"]
                    path_reach = scene_setup["initial"]["door"]["path_reach"]

                    param.clearTSRChains()
                    param.addTSRChain(TSRChain(mimic_body_name="cabinets", mimic_body_index=[3])
                                        .addTSR(T0_w0, Tw0_e, Bw0)
                                        .addTSR(np.eye(4), Tw1_e, Bw1))
                    tsr_pick_place, dist_pick_place = planner.distanceToManifold(path_pick_place, param, startik, goalik)
                    print("max dist pick place: %f" % max(dist_pick_place))
                    time.sleep(0.1)
                    tsr_reach, dist_reach = planner.distanceToManifold(path_reach, param, initik, startik)
                    print("max dist reach: %f" % max(dist_reach))
                    time.sleep(0.1)
                    time_dict[scene_key]["door"] = {"tsr_pick_place": tsr_pick_place, "tsr_reach": tsr_reach, "dist_pick_place": dist_pick_place, "dist_reach": dist_reach}

                cleanupObject(orEnv, obj_name, obj, obj_setup)
        with open(os.path.join(output_folder, "env%d.p" % e), "wb") as f:
            pickle.dump({env_key: time_dict}, f)


if __name__ == "__main__":
    for e in range(13, 20):
        while True:
            p = Process(target=main, args=([e, ], ))
            p.start()
            p.join()
            print(p.exitcode)
            if p.exitcode==0:
                break
