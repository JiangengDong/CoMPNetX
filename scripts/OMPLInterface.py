import openravepy as orpy

import utils


class OMPLInterface:
    template_string = """<solver_parameters type="2" time="20" range="0.05"/>
               <constraint_parameters type="1" tolerance="0.001" max_iter="50" delta="0.01" lambda="2"/>
               <atlas_parameters exploration="0.5" epsilon="0.01" rho="1" alpha="0.2" max_charts="500" using_bias="1" separate="0"/>
               <tsr_chain purpose="0 0 1">
                    <tsr manipulator_index="0" relative_body_name="NULL" T0_w="%s" Tw_e="%s" Bw="%s" />
               </tsr_chain>"""

    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        assert self.planner != None
        self.planner.SendCommand("SetLogLevel 0")

    def solve(self, start_config, goal_config, T0_w, Tw_e, Bw):
        dof = self.robot.GetActiveDOF()
        assert len(start_config) == dof
        assert len(goal_config) == dof

        params = orpy.Planner.PlannerParameters()

        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(start_config)
        params.SetGoalConfig(goal_config)
        params.SetExtraParameters(OMPLInterface.template_string % (utils.SerializeTransform(T0_w), utils.SerializeTransform(Tw_e), utils.SerializeBound(Bw)))

        with self.env, self.robot:
            if (self.planner.InitPlan(self.robot, params)):
                traj = orpy.RaveCreateTrajectory(self.env, '')
                status = self.planner.PlanPath(traj)
                if status == orpy.PlannerStatus.HasSolution:
                    resp = True
                    time = float(self.planner.SendCommand("GetPlanningTime"))
                    orpy.planningutils.RetimeTrajectory(traj)
                else:
                    resp = False
                    time = -1
            else:
                return None, None, None
        return resp, time, traj
