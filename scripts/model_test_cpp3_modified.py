# coding: utf-8

import numpy as np
import os
import pickle
import rospkg
import time

import openravepy as orpy

from OMPLInterface import RPY2Transform, OMPLInterface, PlannerParameter, TSRChain


def setup_init_sc(orEnv, targObj, obj_names, e_no, s_no, esc_dict, cabinets):
    env_no = "env_" + str(e_no)
    sc_no = "s_" + str(s_no)
    env_info = esc_dict[env_no]["initial"]
    # door
    cabinets.SetActiveDOFs([3])
    door_start = env_info["door"]["door_start"]
    door_end = env_info["door"]["door_end"]
    cabinets.SetActiveDOFValues([door_start])
    # bin
    trash = env_info["recyclingbin"]
    orEnv.Add(targObj[1])
    T0_object = trash["T0_w2"]
    targObj[1].SetTransform(np.array(T0_object[0:3][:, 0:4]))

    # mugblack
    name = "mugblack"
    idx = obj_names.index(name)
    mug = env_info[name]
    orEnv.Add(targObj[idx])
    T0_object = mug["T0_w0"]
    targObj[idx].SetTransform(np.array(T0_object[0:3][:, 0:4]))

    # mugred
    name = "plasticmug"
    idx = obj_names.index(name)
    mug = env_info[name]
    orEnv.Add(targObj[idx])
    T0_object = mug["T0_w0"]
    targObj[idx].SetTransform(np.array(T0_object[0:3][:, 0:4]))
    names = ["tray", "juice", "fuze_bottle", "coke_can", "pitcher"]
    scene = esc_dict[env_no][sc_no]
    for i in range(0, len(names)):
        idx = obj_names.index(names[i])
        orEnv.Add(targObj[idx])
        if "T0_w2" in scene[names[i]].keys():
            T0_object = scene[names[i]]["T0_w2"]
        else:
            T0_object = scene[names[i]]["T0_w"]
        targObj[idx].SetTransform(np.array(T0_object[0:3][:, 0:4]))


####################

orEnv = orpy.Environment()
orEnv.SetViewer('qtcoin')
orEnv.Reset()
orEnv.SetDebugLevel(orpy.DebugLevel.Info)
# colchecker = orpy.RaveCreateCollisionChecker(orEnv, 'ode')
# orEnv.SetCollisionChecker(colchecker)

#### tables & objects placement #####
tables = []
table1 = orpy.RaveCreateKinBody(orEnv, '')
table1.SetName('table1')
table1.InitFromBoxes(np.array([[1.0, 0.4509, 0.5256, 0.2794, 0.9017, 0.5256]]), True)
orEnv.Add(table1, True)
tables.append(table1)

table2 = orpy.RaveCreateKinBody(orEnv, '')
table2.SetName('table2')
table2.InitFromBoxes(np.array([[0.3777, -0.7303, 0.5256, 0.9017, 0.2794, 0.5256]]), True)
orEnv.Add(table2, True)
tables.append(table2)

cabinets = orEnv.ReadRobotURI('objects/furniture/my_cabinets.robot.xml')
orEnv.Add(cabinets, True)
cab_rot = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
T0_cab = RPY2Transform(0, 0, -np.pi / 2, 0.85, 1.28, 0)
cabinets.SetTransform(np.array(T0_cab[0:3][:, 0:4]))
cabinets.SetActiveDOFs([3])

obj_names = ["tray", "recyclingbin", "juice", "fuze_bottle", "coke_can", "mugblack", "plasticmug", "pitcher"]
targobject = []
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/tray.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/recyclingbin.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/juice_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/fuze_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/coke_can.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/mugblack.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/mug2.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/pitcher3.kinbody.xml'))

########## load robot and place ###
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

with open("../data/esc_dict_door100_30.p", "rb") as f:
    esc_dict = pickle.load(f)
param = PlannerParameter()
param.solver_parameter.type = "mpnet"
param.solver_parameter.time = 180
param.constraint_parameter.type="atlas"
param.atlas_parameter.rho = 1.5
param.atlas_parameter.exploration = 0.9
param.atlas_parameter.epsilon = 0.02

planner = OMPLInterface(orEnv, robot, loglevel=2)
for i in range(0, 66):
    env_no = "env_" + str(i)
    if env_no not in esc_dict.keys():
        continue
    for j in range(27, 31):
        sc_no = "s_" + str(j)
        if sc_no not in esc_dict[env_no].keys():
            continue
        rgconf = esc_dict[env_no][sc_no]["mugblack"]["rgconf"]
        esc_dict[env_no][sc_no]["initial"]["plasticmug"]["riconf"] = rgconf
### when loading files
for e in range(0, 66):
    env_no = "env_" + str(e)
    if env_no not in esc_dict.keys():
        continue
    for s in range(27, 31):
        s_no = "s_" + str(s)
        if s_no not in esc_dict[env_no].keys():
            continue
        robot.SetActiveDOFs(arm1dofs)
        robot.SetActiveDOFValues(arm1initvals)
        numTSRChainMimicDOF = 1

        setup_init_sc(orEnv, targobject, obj_names, e, s, esc_dict, cabinets)
        print("==========env_no: %d====s_no: %d==========" % (e, s))
        obj_order = esc_dict[env_no][s_no]["obj_order"]
        print("Object order: %s" % str(obj_order))

        # main loop
        idx = None
        names1 = ("juice", "fuze_bottle", "coke_can", "pitcher")
        names2 = ("mugblack", "plasticmug")
        for i in range(0, len(obj_order)):
            print("Planning for %s ..." % obj_order[i])
            param.clearTSRChains()
            if obj_order[i] in names1:
                T0_w2 = esc_dict[env_no][s_no]["initial"][obj_order[i]]["T0_w2"]
                Tw_e = esc_dict[env_no][s_no][obj_order[i]]["Tw_e"]
                Bw2 = esc_dict[env_no][s_no]["initial"][obj_order[i]]["Bw2"] if obj_order[i] == "pitcher" else np.mat([-1000., 1000., -1000., 1000., -1000, 1000,
                                                                                                                       -np.pi, np.pi, -np.pi, np.pi, -np.pi, np.pi])

                initdofvals = esc_dict[env_no][s_no][obj_order[i]]["riconf"]
                startik = esc_dict[env_no][s_no][obj_order[i]]["rsconf"]
                goalik = esc_dict[env_no][s_no]["initial"][obj_order[i]]["rgconf"]

                param.addTSRChain(TSRChain().addTSR(T0_w2, Tw_e, Bw2))
            elif obj_order[i] == "door":
                T0_w0 = esc_dict[env_no][s_no]["initial"]["door"]["T0_w0"]
                Tw0_e = esc_dict[env_no][s_no]["initial"]["door"]["Tw0_e"]
                Bw0 = esc_dict[env_no][s_no]["initial"]["door"]["Bw0"]
                door_init = esc_dict[env_no][s_no]["initial"]["door"]["door_start"]
                door_end = esc_dict[env_no][s_no]["initial"]["door"]["door_end"]

                Tw1_e = esc_dict[env_no][s_no]["initial"]["door"]["Tw1_e"]
                Bw1 = esc_dict[env_no][s_no]["initial"]["door"]["Bw1"]

                initdofvals = esc_dict[env_no][s_no]["initial"]["door"]["riconf"]
                startik = esc_dict[env_no][s_no]["initial"]["door"]["rsconf"]
                goalik = esc_dict[env_no][s_no]["initial"]["door"]["rgconf"]

                param.addTSRChain(TSRChain(mimic_body_name="cabinets", mimic_body_index=[3])
                                  .addTSR(T0_w0, Tw0_e, Bw0)
                                  .addTSR(np.eye(4), Tw1_e, Bw1))

            elif obj_order[i] in names2:
                T0_w2 = esc_dict[env_no][s_no][obj_order[i]]["T0_w2"]
                Tw_e = esc_dict[env_no][s_no][obj_order[i]]["Tw_e"]
                Bw2 = esc_dict[env_no][s_no][obj_order[i]]["Bw2"]

                initdofvals = esc_dict[env_no][s_no][obj_order[i - 1]]["rgconf"] if obj_order[i] == "plasticmug" else esc_dict[env_no][s_no]["initial"][obj_order[i]]["riconf"]
                startik = esc_dict[env_no][s_no]["initial"][obj_order[i]]["rsconf"]
                goalik = esc_dict[env_no][s_no][obj_order[i]]["rgconf"]

                param.addTSRChain(TSRChain().addTSR(T0_w2, Tw_e, Bw2))
            else:
                startik = goalik = initdofvals = None

            robot.SetActiveDOFValues(startik)
            if obj_order[i] == "door":
                # resp, t_time, traj = planner.solve(startik, goalik, param)
                resp = None
                t_time = -1
                traj = None
            else:
                robot.Grab(targobject[obj_names.index(obj_order[i])])
                resp, t_time, traj = planner.solve(startik, goalik, param)

            if resp is True:
                print("Found a solution for %s after %f seconds." % (obj_order[i], t_time))
                if obj_order[i] != "door":
                    # robot.GetController().SetPath(traj)
                    robot.WaitForController(0)
                    pass
                else:
                    # robot.GetController().SetPath(traj)
                    # cabinets.GetController().SetPath(traj)
                    robot.WaitForController(0)
                    # cabinets.WaitForController(0)
                    pass
            elif resp is False:
                print("Failed to find a solution.")
                robot.WaitForController(0)
                t_time = 1e3
            elif resp is None:
                print("Start or goal is invalid!")
                robot.WaitForController(0)
                t_time = -1
            if obj_order[i] != "door":
                esc_dict[env_no][s_no][obj_order[i]].update({"time_pick_place": t_time})

            time.sleep(0.02)
            robot.ReleaseAllGrabbed()
            robot.WaitForController(0)
            robot.SetActiveDOFs(arm1dofs)
            robot.SetActiveDOFValues(goalik)

            if obj_order[i] == "mugblack" or obj_order[i] == "plasticmug" or obj_order[i] == "pitcher":
                idx = obj_names.index(obj_order[i])
                targobject[idx].SetTransform(np.array(T0_w2[0:3][:, 0:4]))
                print("----------move " + obj_order[i])
                idx = None
            elif obj_order[i] == "door":
                cabinets.SetActiveDOFValues([door_end])
                print("----------open " + obj_order[i])
                idx = None
            else:
                idx = obj_names.index(obj_order[i])
                orEnv.Remove(targobject[idx])
                print("----------remove " + obj_order[i])
                idx = None
        print("\n")
    output_name = "../data/result2/esp_dict_door100_30_tbrrt_env%d.p" % e
    esc_dict_temp = {env_no:esc_dict[env_no]}
    with open(output_name, "wb") as f:
        pickle.dump(esc_dict_temp, f)
