#!//bin/env python
# This script requires the models that are shipped with OpenRAVE to be in your
# OPENRAVE_DATA path. This is true by default, but may not be true if you set
# your OPENRAVE_DATA environmental variable. Notably, this includes if you
# source a Catkin workspace that includes the openrave_catkin package.
#
# If so, you should explicitly add this directory to your path:
#
#    export OPENRAVE_DATA="${OPENRAVE_DATA}:/usr/share/openrave-0.9/data"
#
# This path assumes that you installed OpenRAVE to /usr. You may need to alter
# the command to match your acutal install destination.

import os
import numpy as np

import rospkg
import openravepy as orpy

from utils import SerializeTransform, RPY2Transform, quat2Transform, pause
from plannerParameters import PlannerParameters


class BaxterDoorOpeningProblem:
    def __init__(self):
        # setup for OpenRAVE environment
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(orpy.DebugLevel.Info)

        # load kitchen scene, delete herb robot, and load baxter robot
        self.env.Load('scenes/intelkitchen_robotized_herb2.env.xml')
        rp = rospkg.RosPack()
        baxter_path = rp.get_path('baxter_description')
        urdf_path = os.path.join(baxter_path, "urdf/baxter_sym.urdf")
        srdf_path = os.path.join(baxter_path, "urdf/baxter_new.srdf")
        or_urdf = orpy.RaveCreateModule(self.env, 'urdf')
        with self.env:
            self.env.Remove(self.env.GetRobot("BarrettWAM"))
            name = or_urdf.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))

        self.baxter = self.env.GetRobot(name)
        self.left_arm = self.baxter.GetManipulator("left_hand_eef")
        self.right_arm = self.baxter.GetManipulator("right_hand_eef")
        self.left_arm_indices = [2, 3, 4, 5, 6, 7, 8]
        self.right_arm_indices = [10, 11, 12, 13, 14, 15, 16]

        self.kitchen = self.env.GetKinBody("kitchen")

        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        self.cbirrt = orpy.RaveCreateProblem(self.env, "CBiRRT")
        self.env.LoadProblem(self.cbirrt, name)

        self.traj = orpy.RaveCreateTrajectory(self.env, '')

        self.gotoInitialPose()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.env.GetViewer().quitmainloop()
        self.env.Destroy()

    def inverseKinematic(self, manipulator, pose):
        with self.env, self.baxter:
            if manipulator is self.left_arm:
                self.baxter.SetActiveDOFs(self.left_arm_indices)
            elif manipulator is self.right_arm:
                self.baxter.SetActiveDOFs(self.right_arm_indices)
            else:
                raise Exception("Invalid manipulator!")
            self.baxter.SetActiveManipulator(manipulator)
            q_str = self.cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm %d %s' %
                                            (self.baxter.GetActiveManipulatorIndex(), SerializeTransform(pose)))
        return [float(q) for q in q_str.split()]

    def setLeftArmConfig(self, leftarm_q):
        with self.env:
            prev_active_dofs = self.baxter.GetActiveDOFIndices()
            self.baxter.SetActiveDOFs(self.left_arm_indices)
            self.baxter.SetActiveDOFValues(leftarm_q)
            self.baxter.SetActiveDOFs(prev_active_dofs)

    def setRightArmConfig(self, rightarm_q):
        with self.env:
            prev_active_dofs = self.baxter.GetActiveDOFIndices()
            self.baxter.SetActiveDOFs(self.right_arm_indices)
            self.baxter.SetActiveDOFValues(rightarm_q)
            self.baxter.SetActiveDOFs(prev_active_dofs)

    def setArmsConfig(self, leftarm_q, rightarm_q):
        self.setLeftArmConfig(leftarm_q)
        self.setRightArmConfig(rightarm_q)

    def setHandsConfig(self, hands_q):
        with self.env:
            prev_active_dofs = self.baxter.GetActiveDOFIndices()
            self.baxter.SetActiveDOFs([1, 9])
            self.baxter.SetActiveDOFValues(hands_q)
            self.baxter.SetActiveDOFs(prev_active_dofs)

    def openHands(self):
        self.setHandsConfig([1, 1])

    def setPlannerParameters(self, start_config, goal_config):
        params = orpy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.baxter)
        params.SetInitialConfig(start_config)
        params.SetGoalConfig(goal_config)
        # TODO: the extra parameter is fixed now. Make it more flexible.
        # TODO: change cpp code of TSRRobot to support relative body and manipulator index
        params.SetExtraParameters(
            """<solver_parameters type="2" time="10" range="0.05"/>
               <constraint_parameters type="1" tolerance="0.001" max_iter="50" delta="0.05" lambda="2"/>
               <atlas_parameters exploration="0.5" epsilon="0.01" rho="0.08" alpha="0.45" max_charts="500" using_bias="1" separate="0"/>
               <tsr_chain purpose="0 0 1" mimic_body_name="kitchen" mimic_body_index="6">
                    <tsr manipulator_index="0" relative_body_name="NULL" 
                        T0_w="-1  0  0  -0  -1  0  0  0  1  1.818  0.0266  0.884" 
                        Tw_e="1  0  0  0  1  0  0  0  1  0.3829  0.0816  0.2801" 
                        Bw="0  0  0  0  0  0  0  0  0  0  0 3.1415926" />
                    <tsr manipulator_index="0" relative_body_name="NULL"
                        T0_w="1  0  0  0  1  0  0  0  1  0  0  0"
                        Tw_e="0  0  1  -1  0  0  0  -1  0  0  0.19500  0"
                        Bw="0  0  0  0  0  0  0  0  0  0  -1.57  0" />
               </tsr_chain>""")
        return params

    def gotoInitialPose(self):
        baxter_pose = np.array(RPY2Transform(0, 0, np.pi / 2, 0, 0, 0) * RPY2Transform(0, 0, 0, -0.8, -1.2217, 0.9614))
        self.baxter.SetTransform(baxter_pose)

        self.baxter.SetActiveDOFs(self.right_arm_indices)
        self.baxter.SetActiveManipulator(self.right_arm)

        joint_val = [0, 0.75, 0, -0.55, 0, 1.26, 0, 0, 0.75, 0, -0.55, 0, 1.26, 0]
        self.setArmsConfig(joint_val[:7], joint_val[7:])
        self.openHands()

    def solve(self, arm_start_pose, arm_goal_pose):
        right_start = self.inverseKinematic(self.right_arm, arm_start_pose)
        right_goal = self.inverseKinematic(self.right_arm, arm_goal_pose)
        params = self.setPlannerParameters(right_start, right_goal)
        with self.env, self.baxter:
            self.planner.InitPlan(self.baxter, params)
            self.planner.PlanPath(self.traj)
        orpy.planningutils.RetimeTrajectory(self.traj)
        return self.traj

    def display(self):
        self.baxter.GetController().SetPath(self.traj)
        self.baxter.WaitForController(0)


def main():
    hinge_start_pose = np.mat([[-1, 0, 0, 1.818],
                               [0, -1, 0, 0.0266],
                               [0, 0, 1, 0.884],
                               [0, 0, 0, 1]])
    handle_start_pose = np.mat([[-1, 0, 0, 1.4351],
                                [0, -1, 0, -0.055],
                                [0, 0, 1, 1.1641],
                                [0, 0, 0, 1]])
    arm_start_pose = np.mat([[0, 1, 0, 1.4351],
                             [0, 0, 1, -0.25],
                             [1, 0, 0, 1.1641],
                             [0, 0, 0, 1]])
    handle_hinge_offset = hinge_start_pose.I * handle_start_pose
    hand_handle_offset = handle_start_pose.I * arm_start_pose

    Thinge_rot = RPY2Transform(0, 0, np.pi / 2, 0, 0, 0)
    Thandle_rot = RPY2Transform(0, 0, -np.pi / 4, 0, 0, 0)

    hinge_goal_pose = hinge_start_pose * Thinge_rot

    arm_goal_pose = hinge_goal_pose * handle_hinge_offset * Thandle_rot * hand_handle_offset

    problem = BaxterDoorOpeningProblem()
    problem.solve(arm_start_pose, arm_goal_pose)
    problem.display()
    pause()


if __name__ == '__main__':
    main()
    # TODO: multiple TSRChains for different manipulators
    # TODO: multiple TSRChains for the same manipulators to choose from
