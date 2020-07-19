# coding: utf-8

import numpy as np
import os
import pickle
import rospkg
import time

import openravepy as orpy

from OMPLInterface import OMPLInterface, TSRChain, PlannerParameter, RPY2Transform


def setup_esc(orEnv, targObj, obj_names, e_no, s_no, esc_dict):
    env_no = "env_" + str(e_no)
    sc_no = "s_" + str(s_no)
    targets = esc_dict[env_no]["targets"]
    scene = esc_dict[env_no][sc_no]
    for i in range(0, 2):
        orEnv.Add(targObj[i])
        T0_object = targets[obj_names[i]]["T0_w2"]
        targObj[i].SetTransform(np.array(T0_object[0:3][:, 0:4]))

    for i in range(2, len(targObj)):
        orEnv.Add(targObj[i])
        T0_object = scene[obj_names[i]]["T0_w"]
        targObj[i].SetTransform(np.array(T0_object[0:3][:, 0:4]))


orEnv = orpy.Environment()
orEnv.SetViewer('qtcoin')
orEnv.Reset()
orEnv.SetDebugLevel(orpy.DebugLevel.Info)

#### tables & objects placement #####
tables = []
table1 = orpy.RaveCreateKinBody(orEnv, '')
table1.SetName('table1')
table1.InitFromBoxes(np.array([[1.0, 0.4509, 0.5256, 0.2794, 0.9017, 0.5256]]), True)
orEnv.Add(table1, True)

table2 = orpy.RaveCreateKinBody(orEnv, '')
table2.SetName('table2')
table2.InitFromBoxes(np.array([[0.3777, -0.7303, 0.5256, 0.9017, 0.2794, 0.5256]]), True)
orEnv.Add(table2, True)

tables.append(table1)
tables.append(table2)

obj_names = ["tray", "recyclingbin", "juice", "fuze_bottle", "coke_can", "plasticmug", "teakettle"]
targobject = []
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/tray.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/recyclingbin.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/juice_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/fuze_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/coke_can.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/mug2.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('objects/household/teakettle.kinbody.xml'))

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
activedofs = arm0dofs + arm1dofs
arm0vals = np.r_[-0.386, 1.321, -0.06, 0.916, -0.349, -0.734, -1.84]
arm1vals = np.r_[0.216, 1.325, 0.173, 0.581, 0.490, -0.3883, 1.950]
initdofvals = np.r_[-0.386, 1.321, -0.06, 0.916, -0.349, -0.734, -1.84, 0.216, 1.325, 0.173, 0.581, 0.490, -0.3883, 1.950]

robot.SetActiveDOFs(activedofs)
robot.SetActiveDOFValues(initdofvals)

# open hands
handdof = np.r_[(1 * np.ones([1, 2]))[0]]
robot.SetActiveDOFs([1, 9])
robot.SetActiveDOFValues(handdof)
robot.SetActiveManipulator(1)

### when loading files
esc_dict = pickle.load(open("data/experiment_setup/esc_dict20_120.p", "rb"))
ompl_planner = OMPLInterface(orEnv, robot, loglevel=2)
planner_parameter = PlannerParameter()
planner_parameter.solver_parameter.type = "mpnet"
planner_parameter.constraint_parameter.type = "tangent_bundle"
planner_parameter.solver_parameter.time = 25

e_start = int(input("Start from env: "))
for e in range(e_start, 19):
    for s in range(110, 120):  # 30
        env_no = "env_" + str(e)
        s_no = "s_" + str(s)
        if s_no not in esc_dict[env_no].keys():
            continue

        # Move to initial position
        robot.SetActiveDOFs(arm1dofs)
        robot.SetActiveDOFValues(arm1vals)

        setup_esc(orEnv, targobject, obj_names, e, s, esc_dict)
        print("==========env_no: %d====s_no: %d==========" % (e, s))

        obj_order = esc_dict[env_no][s_no]["obj_order"]
        print("Object order: %s" % str(obj_order))

        for i in range(0, len(obj_order)):
            print("Planning for %s ..." % obj_order[i])
            T0_w = esc_dict[env_no][s_no][obj_order[i]]["T0_w"]
            T0_w2 = esc_dict[env_no]["targets"][obj_order[i]]["T0_w2"]
            Bw2 = esc_dict[env_no]["targets"][obj_order[i]]["Bw2"]
            Tw_e = esc_dict[env_no][s_no][obj_order[i]]["Tw_e"]
            Bw = esc_dict[env_no][s_no][obj_order[i]]["Bw"]

            initdofvals = esc_dict[env_no][s_no][obj_order[i]]["riconf"]
            startik = esc_dict[env_no][s_no][obj_order[i]]["rsconf"]
            goalik = esc_dict[env_no]["targets"][obj_order[i]]["rgconf"]

            robot.SetActiveDOFs(arm1dofs)
            robot.SetActiveDOFValues(startik)
            robot.Grab(targobject[obj_names.index(obj_order[i])])
            time.sleep(0.05)  # draw the scene

            try:
                planner_parameter.clearTSRChains().addTSRChain(TSRChain(manipulator_index=1).addTSR(T0_w2, Tw_e, Bw2))
                planner_parameter.mpnet_parameter.pnet_path= "data/pytorch_model/ctpnet_annotated_gpu4.pt"
                planner_parameter.mpnet_parameter.dnet_path= "data/pytorch_model/dnet_annotated_gpu.pt"
                planner_parameter.mpnet_parameter.voxel_path = "data/pytorch_model/seen_reps_txt4/e_%d_s_%d_%s_voxel.csv" % (e, s, obj_order[i])
                planner_parameter.mpnet_parameter.ohot_path = "data/pytorch_model/seen_reps_txt4/e_%d_s_%d_%s_pp_ohot.csv" % (e, s, obj_order[i])

                resp, t_time, traj = ompl_planner.solve(startik, goalik, planner_parameter)
                time.sleep(0.1)
                robot.WaitForController(0)

            except Exception as e2:
                print(e2)
                time.sleep(0.1)
                t_time = np.nan

            finally:
                robot.SetActiveDOFs(arm1dofs)
                robot.SetActiveDOFValues(goalik)
                robot.ReleaseAllGrabbed()
                robot.WaitForController(0)
                time.sleep(0.05)

                esc_dict[env_no][s_no][obj_order[i]].update({"time_pick_place": t_time})

                # clean up
                idx = obj_names.index(obj_order[i])
                if obj_order[i] == "teakettle" or obj_order[i] == "plasticmug":
                    T0_object = T0_w2
                    targobject[idx].SetTransform(np.array(T0_object[0:3][:, 0:4]))
                    print("----------move " + obj_order[i])
                else:
                    orEnv.Remove(targobject[idx])
                    print("--------remove " + obj_order[i])
                time.sleep(0.05)
        print("")
    with open("../data/result29/env%d.p"%e, "wb") as f:
        pickle.dump({env_no:esc_dict[env_no]}, f)
