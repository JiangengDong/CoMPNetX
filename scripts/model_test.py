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

import openravepy as orpy

from OMPLInterface import RPY2Transform, OMPLInterface, PlannerParameter, TSRChain


def setOpenRAVE(visible=False, coll_checker="fcl"):
    orEnv = orpy.Environment()
    if visible:
        orEnv.SetViewer('qtcoin')
    orEnv.Reset()
    orEnv.SetDebugLevel(orpy.DebugLevel.Info)
    colchecker = orpy.RaveCreateCollisionChecker(orEnv, 'fcl_') if coll_checker == "fcl" else orpy.RaveCreateCollisionChecker(orEnv, 'ode')
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


def loadObjects(orEnv, task):
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
        "teakettle": loadFromFolder('teakettle.kinbody.xml')
    }

    if task == "kitchen":
        obj_dict["plasticmug"] = loadFromFolder('mugred.kinbody.xml')
    elif task == "bartender":
        obj_dict["plasticmug"] = loadFromFolder('plasticmug.kinbody.xml')
    else:
        pass

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
                "bound": scene_setup_target["juice"]["Bw2"]
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
                "bound": scene_setup_target["fuze_bottle"]["Bw2"]
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
                "bound": scene_setup_target["coke_can"]["Bw2"]
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
                "bound": scene_setup_target["plasticmug"]["Bw2"]
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
                "bound": scene_setup_target["teakettle"]["Bw2"]
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
                    "transform": scene_setup_initial["plasticmug"]["T0_w0"],
                    "ik": scene_setup_initial["plasticmug"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup["plasticmug"]["T0_w2"],
                    "ik": scene_setup["plasticmug"]["rgconf"]
                },
                "offset": scene_setup["plasticmug"]["Tw_e"],
                "bound": scene_setup["plasticmug"]["Bw2"]
            },
            "pitcher": {
                "start": {
                    "transform": scene_setup["pitcher"]["T0_w"],
                    "ik": scene_setup["pitcher"]["rsconf"]
                },
                "goal": {
                    "transform": scene_setup_initial["pitcher"]["T0_w2"],
                    "ik": scene_setup_initial["pitcher"]["rgconf"]
                },
                "offset": scene_setup["pitcher"]["Tw_e"],
                "bound": scene_setup_initial["pitcher"]["Bw2"]
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
    else:
        raise NotImplementedError



def initSceneFactory(task):
    def initSceneBartender(orEnv, robot, cabinet, obj_dict, obj_setups):
        resetBaxter(robot)
        for obj_name in ("recyclingbin", "tray", "juice", "fuze_bottle", "coke_can", "plasticmug", "teakettle"):
            obj = obj_dict[obj_name]
            obj_transform = obj_setups[obj_name]["start"]["transform"]
            orEnv.Add(obj)
            obj.SetTransform(np.array(obj_transform[0:3][:, 0:4]))
        time.sleep(0.1)

    def initSceneKitchen(orEnv, robot, cabinet, obj_dict, obj_setups):
        resetBaxter(robot)
        setCabinet(cabinet, obj_setups["door"]["start"]["value"])
        for obj_name in ("recyclingbin", "tray", "juice", "fuze_bottle", "coke_can", "mugblack", "plasticmug", "pitcher"):
            obj = obj_dict[obj_name]
            obj_transform = obj_setups[obj_name]["start"]["transform"]
            orEnv.Add(obj)
            obj.SetTransform(np.array(obj_transform[0:3][:, 0:4]))
        time.sleep(0.1)

    if task == "bartender":
        return initSceneBartender
    elif task == "kitchen":
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


def main():
    # settings
    visible = False
    coll_checker = "fcl"
    task = "kitchen"
    output_folder = "data/result/result38"
    message = "Test mpnet"

    if task == "kitchen":
        env_range = list(range(0, 70))
        scene_range = list(range(27, 30))
        setup_file = "data/experiment_setup/esc_dict_door70_30.p"
    elif task == "bartender":
        env_range = list(range(0, 19))
        scene_range = list(range(110, 120))
        setup_file = "data/experiment_setup/esc_dict20_120.p"
    else:
        env_range = scene_range = setup_file = None

    param = PlannerParameter()
    param.solver_parameter.type = "rrtconnect"
    param.solver_parameter.time = 180
    param.constraint_parameter.type = "atlas"
    param.constraint_parameter.tolerance = 1e-3
    param.constraint_parameter.delta = 0.05
    param.atlas_parameter.rho = 1.5
    param.atlas_parameter.exploration = 0.5
    param.atlas_parameter.epsilon = 0.01
    if param.solver_parameter.type == "mpnet":
        if task == "kitchen":
            param.mpnet_parameter.pnet_path = "data/pytorch_model/cmpnet_annotated_gpu_rpp_newdoor4.pt"
            param.mpnet_parameter.dnet_path = "data/pytorch_model/dnet4_annotated_gpu_newdoor.pt"
        elif task == "bartender":
            param.mpnet_parameter.pnet_path = "data/pytorch_model/ctpnet_annotated_gpu4.pt"
            param.mpnet_parameter.dnet_path = "data/pytorch_model/dnet_annotated_gpu.pt"
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
    obj_dict = loadObjects(orEnv, task)
    robot = loadBaxter(orEnv)
    planner = OMPLInterface(orEnv, robot, loglevel=2)
    initScene = initSceneFactory(task)
    getObjectSetups = getObjectSetupsFactory(task)
    with open(setup_file, "rb") as f:
        setup_dict = pickle.load(f)

    # main loop
    for e in env_range:
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
                obj_setup = obj_setups[obj_name]
                obj = obj_dict[obj_name] if obj_name != "door" else cabinet
                # unpack values
                T0_w = obj_setup["goal"]["transform"] if obj_name != "door" else None
                Tw_e = obj_setup["offset"]
                Bw = obj_setup["bound"]
                startik = obj_setup["start"]["ik"]
                goalik = obj_setup["goal"]["ik"]

                isObjectValid = all([element is not None for element in [T0_w, Tw_e, Bw, startik, goalik]])
                if isObjectValid:
                    robot.SetActiveDOFValues(startik)
                    robot.Grab(obj)
                    robot.WaitForController(0)
                    if visible:
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
                    if param.solver_parameter.type == "mpnet":
                        if task == "kitchen":
                            param.mpnet_parameter.ohot_path = "data/pytorch_model/seen_reps_txt_door_new4/e_%d_s_%d_%s_pp_ohot.csv" % (e, s, obj_name)
                            param.mpnet_parameter.voxel_path = "data/pytorch_model/seen_reps_txt_door_new4/e_%d_s_%d_%s_voxel.csv" % (e, s, obj_name)
                        elif task == "bartender":
                            param.mpnet_parameter.ohot_path = "data/pytorch_model/seen_reps_txt4/e_%d_s_%d_%s_pp_ohot.csv" % (e, s, obj_name)
                            param.mpnet_parameter.voxel_path = "data/pytorch_model/seen_reps_txt4/e_%d_s_%d_%s_voxel.csv" % (e, s, obj_name)
                    resp, t_time, traj = planner.solve(startik, goalik, param)
                    time.sleep(1)
                    robot.WaitForController(0)

                    if visible and resp is True:
                        robot.GetController().SetPath(traj)
                        robot.WaitForController(0)
                    robot.ReleaseAllGrabbed()
                    robot.WaitForController(0)
                    robot.SetActiveDOFValues(goalik)

                    time_dict[scene_key][obj_name] = {"time_pick_place": t_time}

                cleanupObject(orEnv, obj_name, obj, obj_setup)
        with open(os.path.join(output_folder, "env%d.p" % e), "wb") as f:
            pickle.dump({env_key: time_dict}, f)


if __name__ == "__main__":
    main()
