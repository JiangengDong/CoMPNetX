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

from TransformMatrix import SerializeTransform
from plannerParameters import PlannerParameters


class LiftingBoxProblem:
    def __init__(self, initial_config):
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(orpy.DebugLevel.Info)
        self.env.Load('scenes/herb2_liftingbox.env.xml')
        self.robot = self.env.GetRobot("Herb2")
        self.manipulator_left = self.robot.GetManipulator('left_wam')
        self.manipulator_right = self.robot.GetManipulator('right_wam')
        self.box = self.env.GetKinBody("box")

        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        self.cbirrt = orpy.RaveCreateProblem(self.env, "CBiRRT")
        self.env.LoadProblem(self.cbirrt, "Herb2")

        self.traj = orpy.RaveCreateTrajectory(self.env, '')

        self.setHandsConfig(initial_config[7:], initial_config[:7])

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

    def calculateHandsConfig(self, box_pose):
        lefthand_offset = np.mat([[1, 0, 0, 0],
                                  [0, 0, -1, 0.305],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])
        righthand_offset = np.mat([[1, 0, 0, 0],
                                   [0, 0, 1, -0.285],
                                   [0, -1, 0, 0],
                                   [0, 0, 0, 1]])
        lefthand_pose = box_pose * lefthand_offset
        righthand_pose = box_pose * righthand_offset
        lefthand_q = self.inverseKinematic(self.manipulator_left, lefthand_pose)
        righthand_q = self.inverseKinematic(self.manipulator_right, righthand_pose)
        return lefthand_q, righthand_q

    def setHandsConfig(self, lefthand_q, righthand_q):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_left.GetArmIndices())
            self.robot.SetActiveDOFValues(lefthand_q)
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveDOFValues(righthand_q)

    def grabBox(self):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)
            self.robot.Grab(self.box)

    def releaseBox(self):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)
            self.robot.Release(self.box)

    def liftBox(self):
        self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
        self.robot.SetActiveManipulator(self.manipulator_right)
        self.robot.GetController().SetPath(self.traj)
        self.robot.WaitForController(0)

    def setPlannerParameters(self, initial_config, goal_config):
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)

        params = orpy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(initial_config)
        params.SetGoalConfig(goal_config)
        # TODO: the extra parameter is fixed now. Make it more flexible.
        # TODO: change cpp code of TSRRobot to support relative body and manipulator index
        params.SetExtraParameters(
            """<solver_parameters type="2" time="5" range="0.05"/>
               <constraint_parameters type="1" tolerance="0.01" max_iter="50" delta="0.05" lambda="2"/>
               <atlas_parameters exploration="0.5" epsilon="0.05" rho="0.20" alpha="0.45" max_charts="500" using_bias="0" separate="0"/>
               <tsr_chain purpose="0 0 1" mimic_body_name="NULL">
               <tsr manipulator_index="0" relative_body_name="NULL" 
                                             T0_w="1 0 0 0 1  0 0 0 1 0.6923      0 0" 
                                             Tw_e="1 0 0 0 0 -1 0 1 0      0 -0.285 0" 
                                             Bw="-1000 1000 -1000 1000 -1000 1000 0 0 0 0 -3 3" />
               </tsr_chain>""")
        return params

    def solve(self, box_initial_pose, box_goal_pose):
        self.box.SetTransform(np.array(box_initial_pose[:3, :]))
        left_initial, right_initial = self.calculateHandsConfig(box_initial_pose)
        left_goal, right_goal = self.calculateHandsConfig(box_goal_pose)
        params = self.setPlannerParameters(right_initial, right_goal)
        with self.env, self.robot:
            self.robot.SetActiveDOFs(self.manipulator_right.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator_right)
            self.planner.InitPlan(self.robot, params)
            self.planner.PlanPath(self.traj)
        orpy.planningutils.RetimeTrajectory(self.traj)
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

    problem = LiftingBoxProblem(arm_initial_config)
    problem.solve(box_initial_pose, box_goal_pose)
    # problem.grabBox()
    problem.liftBox()
    # problem.releaseBox()


if __name__ == '__main__':
    main()
    # TODO: multiple TSRs for different manipulators
