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
    colchecker = orpy.RaveCreateCollisionChecker(orEnv, 'fcl') if coll_checker == "fcl" else orpy.RaveCreateCollisionChecker(orEnv, 'ode')
    orEnv.SetCollisionChecker(colchecker)
    return orEnv


def setEnv(orEnv):
    table1 = orpy.RaveCreateKinBody(orEnv, '')
    table1.SetName('table1')
    table1.InitFromBoxes(np.array([[1.0, 0.4509, 0.5256, 0.2794, 0.9017, 0.5256]]), True)
    orEnv.Add(table1, True)

    table2 = orpy.RaveCreateKinBody(orEnv, '')
    table2.SetName('table2')
    table2.InitFromBoxes(np.array([[0.3777, -0.7303, 0.5256, 0.9017, 0.2794, 0.5256]]), True)
    orEnv.Add(table2, True)


def loadCabinet(orEnv):
    cabinet = orEnv.ReadRobotURI('objects/furniture/my_cabinets.robot.xml')
    orEnv.Add(cabinet, True)
    cab_rot = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    T0_cab = RPY2Transform(0, 0, -np.pi / 2, 0.85, 1.28, 0)
    cabinet.SetTransform(np.array(T0_cab[0:3][:, 0:4]))
    cabinet.SetActiveDOFs([3])
    return cabinet


def loadObjects(orEnv):
    return {
        "tray": orEnv.ReadKinBodyXMLFile('objects/household/tray.kinbody.xml'),
        "recyclingbin": orEnv.ReadKinBodyXMLFile('objects/household/recyclingbin.kinbody.xml'),
        "juice": orEnv.ReadKinBodyXMLFile('objects/household/juice_bottle.kinbody.xml'),
        "fuze_bottle": orEnv.ReadKinBodyXMLFile('objects/household/fuze_bottle.kinbody.xml'),
        "coke_can": orEnv.ReadKinBodyXMLFile('objects/household/coke_can.kinbody.xml'),
        "mugblack": orEnv.ReadKinBodyXMLFile('objects/household/mugblack.kinbody.xml'),
        "plasticmug": orEnv.ReadKinBodyXMLFile('objects/household/mug2.kinbody.xml'),
        "pitcher": orEnv.ReadKinBodyXMLFile('objects/household/pitcher3.kinbody.xml'),
        "teakettle": orEnv.ReadKinBodyXMLFile('objects/household/teakettle.kinbody.xml')
    }


def placeObjects(orEnv, task, obj_dict, scene_config, cabinet):
    if task == "kitchen" or task == "new kitchen":
        scene_initial = scene_config["initial"]
        # cabinet
        cabinet.SetActiveDOFs([3])
        cabinet.SetActiveDOFValues([scene_initial["door"]["door_start"]])
        # trash
        trash = obj_dict["recyclingbin"]
        orEnv.Add(trash)
        T0_object = scene_initial["recyclingbin"]["T0_w2"]
        trash.SetTransform(np.array(T0_object[0:3][:, 0:4]))
        # black mug
        mugblack = obj_dict["mugblack"]
        orEnv.Add(mugblack)
        T0_object = scene_initial["mugblack"]["T0_w0"]
        mugblack.SetTransform(np.array(T0_object[0:3][:, 0:4]))
        # red mug
        mugred = obj_dict["plasticmug"]
        orEnv.Add(mugred)
        T0_object = scene_config["plasticmug"]["T0_w0"]
        mugred.SetTransform(np.array(T0_object[0:3][:, 0:4]))
        # pticher
        pitcher = obj_dict["pitcher"]
        orEnv.Add(pitcher)
        T0_object = scene_config["pitcher"]["T0_w"]
        pitcher.SetTransform(np.array(T0_object[0:3][:, 0:4]))

        obj_names = ("tray", "juice", "fuze_bottle", "coke_can")
        # place the other objects
        for obj_name in obj_names:
            obj = obj_dict[obj_name]
            orEnv.Add(obj)
            if "T0_w2" in scene_config[obj_name].keys():
                T0_object = scene_config[obj_name]["T0_w2"]
            else:
                T0_object = scene_config[obj_name]["T0_w"]
            obj.SetTransform(np.array(T0_object[0:3][:, 0:4]))

    else:
        raise NotImplementedError


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


def taskSpecificConfig(task):
    if task == "bartender":
        env_range = list(range(0, 19))
        scene_range = list(range(110, 120))
        config_file = ""
    elif task == "kitchen":
        env_range = list(range(0, 66))
        scene_range = list(range(27, 31))
        config_file = ""
    elif task == "new kitchen":
        env_range = list(range(0, 5))
        scene_range = list(range(0, 5))
        config_file = "data/experiment_setup/esc_dict_newkitchen_5_5.p"
    else:
        raise NotImplementedError
    return env_range, scene_range, config_file


def main():
    # settings
    visible = True
    coll_checker = "ode"
    task = "new kitchen"
    output_folder = "data/result/result13"
    message = "Exchange red mug's start and goal."
    param = PlannerParameter()
    param.solver_parameter.type = "rrtconnect"
    param.solver_parameter.time = 120
    # param.constraint_parameter.type="projection"
    param.atlas_parameter.rho = 1.5
    # param.atlas_parameter.exploration = 0.9
    # param.atlas_parameter.epsilon = 0.02

    # write settings to a file
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    with open(os.path.join(output_folder, "settings.txt"), "w") as f:
        f.write("Task: %s\n" % task)
        f.write("Collision Checker: %s\n" % coll_checker)
        f.write("Other message: %s\n" % message)
        f.write("Parameters: \n")
        f.write(str(param))
        f.write("\n")

    # preparation
    orEnv = setOpenRAVE(visible, coll_checker)
    setEnv(orEnv)
    cabinet = None if task == "bartender" else loadCabinet(orEnv)
    obj_dict = loadObjects(orEnv)
    robot = loadBaxter(orEnv)
    planner = OMPLInterface(orEnv, robot, loglevel=2)
    env_range, scene_range, config_file = taskSpecificConfig(task)
    with open(config_file, "rb") as f:
        config_dict = pickle.load(f)

    # main loop
    for e in env_range:
        env_key = "env_%d" % e
        if env_key not in config_dict.keys():
            continue
        env_config = config_dict[env_key]
        time_dict = {}

        for s in scene_range:
            scene_key = "s_%d" % s
            if scene_key not in env_config.keys():
                continue
            scene_config = env_config[scene_key]
            scene_initial = scene_config["initial"]
            time_dict[scene_key] = {}

            print("==========env_no: %d====s_no: %d==========" % (e, s))
            resetBaxter(robot)
            placeObjects(orEnv, task, obj_dict, scene_config, cabinet)
            obj_names = scene_config["obj_order"]
            print("Object order: %s" % str(obj_names))

            for obj_name in obj_names:
                print("Planning for %s ..." % obj_name)

                # skip door directly
                if obj_name == "door":
                    door_end = scene_initial["door"]["door_end"]
                    cabinet.SetActiveDOFValues([door_end])
                    print("----------open " + obj_name)
                    T0_w = Tw_e = Bw = startik = goalik = None
                    time.sleep(0.1)
                    continue

                elif obj_name in ("juice", "fuze_bottle", "coke_can"):
                    T0_w = scene_initial[obj_name]["T0_w2"]
                    Tw_e = scene_config[obj_name]["Tw_e"]
                    Bw = np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                  -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi])
                    startik = scene_config[obj_name]["rsconf"]
                    goalik = scene_initial[obj_name]["rgconf"]

                elif obj_name == "mugblack":
                    T0_w = scene_config[obj_name]["T0_w2"]
                    Tw_e = scene_config[obj_name]["Tw_e"]
                    Bw = scene_config[obj_name]["Bw2"]
                    startik = scene_initial[obj_name]["rsconf"]
                    goalik = scene_config[obj_name]["rgconf"]

                elif obj_name == "plasticmug":
                    T0_w = scene_initial[obj_name]["T0_w2"]
                    Tw_e = scene_config[obj_name]["Tw_e"]
                    Bw = scene_initial[obj_name]["Bw2"]
                    startik = scene_config[obj_name]["rsconf"]
                    goalik = scene_initial[obj_name]["rgconf"]

                elif obj_name == "pitcher":
                    T0_w = scene_config["pitcher_target"]["T0_w2"]
                    Tw_e = scene_config["pitcher_target"]["Tw_e"]
                    Bw = scene_config["pitcher_target"]["Bw2"]
                    startik = scene_config["pitcher"]["rsconf"]
                    goalik = scene_config["pitcher_target"]["rgconf"]

                else:
                    T0_w = Tw_e = Bw = startik = goalik = None

                # go to start pos
                robot.SetActiveDOFValues(startik)
                robot.Grab(obj_dict[obj_name])
                robot.WaitForController(0)
                if visible:
                    robot.SetActiveDOFValues(startik)
                    robot.WaitForController(0)
                    time.sleep(1)
                    robot.SetActiveDOFValues(goalik)
                    robot.WaitForController(0)
                    time.sleep(1)
                    robot.SetActiveDOFValues(startik)
                    robot.WaitForController(0)
                # plan
                param.clearTSRChains()
                param.addTSRChain(TSRChain().addTSR(T0_w, Tw_e, Bw))
                resp, t_time, traj = planner.solve(startik, goalik, param)
                time.sleep(0.1)
                robot.WaitForController(0)

                if visible and resp is True:
                    robot.GetController().SetPath(traj)
                    robot.WaitForController(0)
                time_dict[scene_key][obj_name] = {"time_pick_place": t_time}

                # clean up
                robot.ReleaseAllGrabbed()
                robot.WaitForController(0)
                robot.SetActiveDOFValues(goalik)
                if obj_name in ("mugblack", "plasticmug", "pitcher"):
                    obj_dict[obj_name].SetTransform(np.array(T0_w[0:3][:, 0:4]))
                    print("----------move " + obj_name)
                else:
                    orEnv.Remove(obj_dict[obj_name])
                    print("--------remove " + obj_name)
                time.sleep(0.1)
            print("\n")
        dict_temp = {env_key: time_dict}
        with open(os.path.join(output_folder, "env%d.p" % e), "wb") as f:
            pickle.dump(dict_temp, f)


if __name__ == "__main__":
    main()
