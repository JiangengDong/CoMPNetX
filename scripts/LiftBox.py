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

import openravepy as orpy

from utils import SerializeTransform
from OMPLInterface import OMPLInterface, TSRChainParameter, PlannerParameter


class LiftingBoxProblem:
    def __init__(self, arm_initial_config, box_initial_pose):
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(orpy.DebugLevel.Info)

        self.env.Load('robots/herb2_padded.robot.xml')
        self.robot = self.env.GetRobot("Herb2")
        self.manipulator_left = self.robot.GetManipulator('left_wam')
        self.manipulator_right = self.robot.GetManipulator('right_wam')
        self.setHandsConfig(arm_initial_config[7:], arm_initial_config[:7])

        self.box = self.env.ReadKinBodyXMLFile('objects/misc/liftingbox.kinbody.xml')
        self.env.AddKinBody(self.box)
        self.box.SetTransform(np.array(box_initial_pose))

        self.planner = OMPLInterface(self.env, self.robot)
        self.cbirrt = orpy.RaveCreateProblem(self.env, "CBiRRT")
        self.env.LoadProblem(self.cbirrt, "Herb2")

        self.traj = orpy.RaveCreateTrajectory(self.env, '')
        self._start_config = None
        self._goal_config = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.env.GetViewer().quitmainloop()
        self.env.Destroy()

    def inverseKinematic(self, manipulator, pose):
        with self.env, self.robot:
            self.robot.SetActiveDOFs(manipulator.GetArmIndices())
            self.robot.SetActiveManipulator(manipulator)
            q_str = self.cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm %d %s' %
                                            (self.robot.GetActiveManipulatorIndex(), SerializeTransform(pose)))
        return [float(theta) for theta in q_str.split()]

    def setHandsConfig(self, lefthand_q, righthand_q):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_left.GetArmIndices())
            self.robot.SetActiveDOFValues(lefthand_q)
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveDOFValues(righthand_q)

    def display(self):
        self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
        self.robot.SetActiveManipulator(self.manipulator_right)

        self.robot.SetActiveDOFValues(self._start_config)
        self.robot.Grab(self.box)

        self.robot.GetController().SetPath(self.traj)
        self.robot.WaitForController(0)

        self.robot.ReleaseAllGrabbed()
        self.robot.WaitForController(0)

    def solve(self, arm_start_pose, arm_goal_pose, T0_w, Tw_e, Bw):
        self._start_config = self.inverseKinematic(self.manipulator_right, arm_start_pose)
        self._goal_config = self.inverseKinematic(self.manipulator_right, arm_goal_pose)
        planner_parameter = PlannerParameter().addTSRChain(TSRChainParameter().addTSR(T0_w, Tw_e, Bw))
        status, time, self.traj = self.planner.solve(self._start_config, self._goal_config, planner_parameter)
        return self.traj


def main():
    arm_initial_config = [3.68, -1.9, -0.0000, 2.2022, -0.0000, 0.0000, 0.0000,
                          2.6, -1.9, -0.0000, 2.2022, -0.0001, 0.0000, 0.0000]
    box_initial_pose = np.mat([[1, 0, 0, 0.6923],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0.5689],
                               [0, 0, 0, 1]])
    box_goal_pose = np.mat([[1, 0, 0, 0.6923],
                            [0, 1, 0, 0],
                            [0, 0, 1, 1.3989],
                            [0, 0, 0, 1]])

    righthand_offset = np.mat([[1, 0, 0, 0],
                               [0, 0, 1, -0.285],
                               [0, -1, 0, 0],
                               [0, 0, 0, 1]])

    hand_initial_pose = box_initial_pose * righthand_offset
    hand_goal_pose = box_goal_pose * righthand_offset

    bound = np.mat([[-1000, 1000],
                    [-1000, 1000],
                    [-1000, 1000],
                    [0, 0],
                    [0, 0],
                    [-3, 3]])

    problem = LiftingBoxProblem(arm_initial_config, box_initial_pose)
    problem.solve(hand_initial_pose, hand_goal_pose, box_initial_pose, righthand_offset, bound)
    problem.display()
    print "Press enter to exit..."
    # sys.stdin.readline()


if __name__ == '__main__':
    main()
