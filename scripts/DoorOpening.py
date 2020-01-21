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
import time

import openravepy as orpy

from OMPLInterface import OMPLInterface, TSRChainParameter, PlannerParameter
from utils import SerializeTransform, RPY2Transform, pause


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

        self.planner = OMPLInterface(self.env, self.robot)
        self.cbirrt = orpy.RaveCreateProblem(self.env, "CBiRRT")
        self.env.LoadProblem(self.cbirrt, "BarrettWAM")

        self.traj = orpy.RaveCreateTrajectory(self.env, '')

        self.setArmsConfig(initial_config)  # initial config of arms
        self.setRightHandConfig([2, 2, 2])
        self.robot.SetTransform(np.array([[-0.0024, -1, 0, 1.2996],  # initial transformation of robot
                                          [1, -0.0024, 0, -0.6217],
                                          [0, 0, 1, 0]]))
        time.sleep(1)

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

    def solve(self, arm_start_pose, arm_goal_pose, T0_w, Tw_e, Bw):
        self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
        start_config = self.inverseKinematic(self.manipulator_right, arm_start_pose)
        goal_config = self.inverseKinematic(self.manipulator_right, arm_goal_pose)
        planner_parameter = PlannerParameter().addTSRChain(TSRChainParameter(mimic_body_name="kitchen",
                                                                             mimic_body_index=(6,)).addTSR(T0_w, Tw_e, Bw))
        planner_parameter.constraint_parameter.tolerance = 1e-3
        status, time, self.traj = self.planner.solve(start_config, goal_config, planner_parameter)
        return self.traj

    def display(self):
        self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
        self.robot.SetActiveManipulator(self.manipulator_right)
        self.robot.GetController().SetPath(self.traj)
        self.kitchen.GetController().SetPath(self.traj)
        self.robot.WaitForController(0)
        self.kitchen.WaitForController(0)


def main():
    hand_initial_config = [3.68, -1.9, -0.0000, 2.2022, -0.0000, 0.0000, -2.1,
                           2.6, -1.9, -0.0000, 2.2022, -3.14, 0.0000, -1.0]

    hinge_start_pose = np.mat([[-1, 0, 0, 1.818],
                               [0, -1, 0, 0.0266],
                               [0, 0, 1, 0.884],
                               [0, 0, 0, 1]])
    handle_start_pose = np.mat([[-1, 0, 0, 1.4351],
                                [0, -1, 0, -0.055],
                                [0, 0, 1, 1.1641],
                                [0, 0, 0, 1]])
    hand_start_pose = np.mat([[1, 0, 0, 1.4351],
                              [0, 0, 1, -0.21],
                              [0, -1, 0, 1.1641],
                              [0, 0, 0, 1]])
    handle_hinge_offset = hinge_start_pose.I * handle_start_pose
    hand_handle_offset = handle_start_pose.I * hand_start_pose
    hand_hinge_offset = hinge_start_pose.I * hand_start_pose

    Thinge_rot = RPY2Transform(0, 0, np.pi / 3, 0, 0, 0)
    Thandle_rot = RPY2Transform(0, 0, 0, 0, 0, 0)

    hinge_goal_pose = hinge_start_pose * Thinge_rot
    hand_goal_pose = hinge_goal_pose * hand_hinge_offset

    hinge_bound = np.array([[0, 0],
                            [0, 0],
                            [0, 0],
                            [0, 0],
                            [0, 0],
                            [0, 3.1415928]])
    handle_bound = np.array([[0, 0],
                             [0, 0],
                             [0, 0],
                             [0, 0],
                             [0, 0],
                             [-1.58, 0]])

    problem = DoorOpeningProblem(hand_initial_config)
    problem.solve(hand_start_pose, hand_goal_pose, hinge_start_pose, hand_hinge_offset, hinge_bound)
    problem.display()
    pause()


if __name__ == '__main__':
    main()
