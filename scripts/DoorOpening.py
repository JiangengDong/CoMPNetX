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

import numpy as np
import sys

import openravepy as orpy

from utils import SerializeTransform, RPY2Transform, quat2Transform
from plannerParameters import PlannerParameters


class DoorOpeningProblem:
    def __init__(self, initial_config):
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(orpy.DebugLevel.Info)
        self.env.Load('environments/intelkitchen_robotized_herb2.env.xml')
        self.robot = self.env.GetRobot("BarrettWAM")
        self.manipulator_left = self.robot.GetManipulator('left_wam')
        self.manipulator_right = self.robot.GetManipulator('right_wam')
        self.kitchen = self.env.GetKinBody("kitchen")

        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        self.cbirrt = orpy.RaveCreateProblem(self.env, "CBiRRT")
        self.env.LoadProblem(self.cbirrt, "BarrettWAM")

        self.traj = orpy.RaveCreateTrajectory(self.env, '')

        self.setArmsConfig(initial_config)  # initial config of arms
        self.setRightHandConfig([2, 2, 2])
        self.robot.SetTransform(np.array([[-0.0024, -1, 0, 1.2996],  # initial transformation of robot
                                          [1, -0.0024, 0, -0.6217],
                                          [0, 0, 1, 0]]))

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.env.GetViewer().quitmainloop()
        self.env.Destroy()

    def inverseKinematic(self, manipulator, pose):
        with self.env, self.robot:
            self.robot.SetActiveDOFs(manipulator.GetArmIndices())
            self.robot.SetActiveManipulator(manipulator)
            q_str = self.cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm %d %s' %
                                            (self.robot.GetActiveManipulatorIndex(), SerializeTransform(pose)))
        return [float(q) for q in q_str.split()]

    def setLeftArmConfig(self, leftarm_q):
        with self.env:
            prev_active_dofs = self.robot.GetActiveDOFIndices()
            self.robot.SetActiveDOFs(self.manipulator_left.GetArmIndices())
            self.robot.SetActiveDOFValues(leftarm_q)
            self.robot.SetActiveDOFs(prev_active_dofs)

    def setRightArmConfig(self, rightarm_q):
        with self.env:
            prev_active_dofs = self.robot.GetActiveDOFIndices()
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveDOFValues(rightarm_q)
            self.robot.SetActiveDOFs(prev_active_dofs)

    def setArmsConfig(self, arms_q):
        self.setRightArmConfig(arms_q[:7])
        self.setLeftArmConfig(arms_q[7:])

    def setRightHandConfig(self, righthand_q):
        with self.env:
            prev_active_dofs = self.robot.GetActiveDOFIndices()
            self.robot.SetActiveDOFs([7, 8, 9])
            self.robot.SetActiveDOFValues(righthand_q)
            self.robot.SetActiveDOFs(prev_active_dofs)

    def setPlannerParameters(self, initial_config, goal_config):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)

        params = orpy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(initial_config)
        params.SetGoalConfig(goal_config)
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
                        Tw_e="-1  0  0  0  0  -1  0  -1  0  0  0.155 0"
                        Bw="0  0  0  0  0  0  0  0  0  0  -1.57  0" />
               </tsr_chain>""")
        return params

    def solve(self, arm_initial_pose, arm_goal_pose):
        right_initial = self.inverseKinematic(self.manipulator_right, arm_initial_pose)
        right_goal = self.inverseKinematic(self.manipulator_right, arm_goal_pose)
        params = self.setPlannerParameters(right_initial, right_goal)
        with self.env, self.robot:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)
            self.planner.InitPlan(self.robot, params)
            self.planner.PlanPath(self.traj)
        orpy.planningutils.RetimeTrajectory(self.traj)
        return self.traj

    def display(self):
        self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
        self.robot.SetActiveManipulator(self.manipulator_right)
        self.robot.GetController().SetPath(self.traj)
        self.robot.WaitForController(0)


def main():
    arm_initial_config = [3.68, -1.9, -0.0000, 2.2022, -0.0000, 0.0000, -2.1,
                          2.6, -1.9, -0.0000, 2.2022, -3.14, 0.0000, -1.0]

    hinge_start_pose = np.mat([[-1, 0, 0, 1.818],
                               [0, -1, 0, 0.0266],
                               [0, 0, 1, 0.884],
                               [0, 0, 0, 1]])
    handle_start_pose = np.mat([[-1, 0, 0, 1.4351],
                                [0, -1, 0, -0.055],
                                [0, 0, 1, 1.1641],
                                [0, 0, 0, 1]])
    arm_start_pose = np.mat([[1, 0, 0, 1.4351],
                             [0, 0, 1, -0.21],
                             [0, -1, 0, 1.1641],
                             [0, 0, 0, 1]])
    handle_hinge_offset = hinge_start_pose.I * handle_start_pose
    hand_handle_offset = handle_start_pose.I * arm_start_pose

    Thinge_rot = RPY2Transform(0, 0, np.pi / 3, 0, 0, 0)
    Thandle_rot = RPY2Transform(0, 0, -np.pi / 4, 0, 0, 0)

    hinge_goal_pose = hinge_start_pose * Thinge_rot

    arm_goal_pose = hinge_goal_pose * handle_hinge_offset * Thandle_rot * hand_handle_offset

    problem = DoorOpeningProblem(arm_initial_config)
    problem.solve(arm_start_pose, arm_goal_pose)
    problem.display()
    print "Press enter to exit..."
    # sys.stdin.readline()


if __name__ == '__main__':
    main()
