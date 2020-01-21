import numpy as np
from collections import Iterable

import openravepy as orpy

import utils


class OMPLInterface:
    def __init__(self, env, robot, loglevel=2, visualize_sample=False):
        self.env = env
        self.robot = robot
        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        assert self.planner != None
        self.planner.SendCommand("SetLogLevel %d" % loglevel)
        self.visualize_sample = visualize_sample

    def solve(self, start_config, goal_config, planner_params):
        dof = self.robot.GetActiveDOF()
        assert len(start_config) == dof
        assert len(goal_config) == dof

        params = orpy.Planner.PlannerParameters()

        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(start_config)
        params.SetGoalConfig(goal_config)
        params.SetExtraParameters(str(planner_params))

        if self.visualize_sample is False:
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
        else:
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


class SolverParameter(object):
    types = ("rrt", "rrtstar", "rrtconnect")
    Template_str = """<solver_parameters type="%d" time="%d" range="%f"/>"""

    def __init__(self, type="rrtconnect", time=120, range=0.05):
        self._type = 2
        self._time = 120
        self._range = 0.05

        self.type = type
        self.time = time
        self.range = range

    def __str__(self):
        return SolverParameter.Template_str % (self._type, self._time, self._range)

    @property
    def type(self):
        return SolverParameter.types[self._type]

    @type.setter
    def type(self, value):
        assert value in SolverParameter.types
        self._type = SolverParameter.types.index(value)

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._time = value

    @property
    def range(self):
        return self._range

    @range.setter
    def range(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._range = value


class ConstraintParameter(object):
    types = ("projection", "atlas", "tangent_bundle")
    Template_str = """<constraint_parameters type="%d" tolerance="%f" max_iter="%d" delta="%f" lambda="%f"/>"""

    def __init__(self, type="atlas", tolerance=1e-3, max_iter=50, delta=0.05, lambd=2.0):
        self._type = 1
        self._tolerance = 1e-4
        self._max_iter = 50
        self._delta = 0.05
        self._lambda = 2.0

        self.type = type
        self.tolerance = tolerance
        self.max_iter = max_iter
        self.delta = delta
        self.lambd = lambd

    def __str__(self):
        return ConstraintParameter.Template_str % (self._type, self._tolerance, self._max_iter, self._delta, self._lambda)

    @property
    def type(self):
        return ConstraintParameter.types[self._type]

    @type.setter
    def type(self, value):
        assert value in ConstraintParameter.types
        self._type = ConstraintParameter.types.index(value)

    @property
    def tolerance(self):
        return self._tolerance

    @tolerance.setter
    def tolerance(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._tolerance = value

    @property
    def max_iter(self):
        return self._max_iter

    @max_iter.setter
    def max_iter(self, value):
        assert type(value) == int and value > 0
        self._max_iter = value

    @property
    def delta(self):
        return self._delta

    @delta.setter
    def delta(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._delta = value

    @property
    def lambd(self):
        return self._lambda

    @lambd.setter
    def lambd(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._lambda = value


class AtlasParameter(object):
    Template_str = """<atlas_parameters exploration="%f" epsilon="%f" rho="%f" alpha="%f" max_charts="%d" using_bias="%d" separate="%d"/>"""

    def __init__(self, exploration=0.5,
                 rho=0.5,
                 epsilon=0.01,
                 alpha=np.pi / 16,
                 max_chart=5000,
                 using_bias=True,
                 separate=True):
        self._exploration = 0.5
        self._rho = 0.75
        self._epsilon = 0.01
        self._alpha = np.pi / 8
        self._max_chart = 5000
        self._using_bias = 1
        self._separate = 1

        self.exploration = exploration
        self.rho = rho
        self.epsilon = epsilon
        self.alpha = alpha
        self.max_chart = max_chart
        self.using_bias = using_bias
        self.separate = separate

    def __str__(self):
        return AtlasParameter.Template_str % (self._exploration, self._epsilon, self._rho, self._alpha, self._max_chart, self._using_bias, self._separate)

    @property
    def exploration(self):
        return self._exploration

    @exploration.setter
    def exploration(self, value):
        assert type(value) == float or type(value) == int and 0 < value < 1
        self._exploration = value

    @property
    def rho(self):
        return self._rho

    @rho.setter
    def rho(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._rho = value

    @property
    def epsilon(self):
        return self._epsilon

    @epsilon.setter
    def epsilon(self, value):
        assert type(value) == float or type(value) == int and value > 0
        self._epsilon = value

    @property
    def alpha(self):
        return self._alpha

    @alpha.setter
    def alpha(self, value):
        assert type(value) == float or type(value) == int and 0 < value < np.pi / 2
        self._alpha = value

    @property
    def max_chart(self):
        return self._max_chart

    @max_chart.setter
    def max_chart(self, value):
        assert type(value) == int and value > 0
        self._max_chart = value

    @property
    def using_bias(self):
        return True if self._using_bias == 1 else False

    @using_bias.setter
    def using_bias(self, value):
        assert type(value) == bool
        self._using_bias = 1 if value else 0

    @property
    def separate(self):
        return True if self._separate == 1 else False

    @separate.setter
    def separate(self, value):
        assert type(value) == bool
        self._separate = 1 if value else 0


class TSRParameter(object):
    Template_str = """<tsr T0_w="%s" Tw_e="%s" Bw="%s"/>"""

    def __init__(self, T0_w=np.eye(4), Tw_e=np.eye(4), Bw=np.zeros((6, 2))):
        self._T0_w = np.eye(4)
        self._Tw_e = np.eye(4)
        self._Bw = np.zeros((6, 2))

        self.T0_w = T0_w
        self.Tw_e = Tw_e
        self.Bw = Bw

    def __str__(self):
        return TSRParameter.Template_str % (utils.SerializeTransform(self._T0_w), utils.SerializeTransform(self._Tw_e), utils.SerializeBound(self._Bw))

    @property
    def T0_w(self):
        return self._T0_w

    @T0_w.setter
    def T0_w(self, value):
        temp = np.array(value)
        assert (temp.dtype == np.float64 or temp.dtype == np.int64) and (temp.shape == (4, 4) or temp.shape == (3, 4))
        self._T0_w = temp

    @property
    def Tw_e(self):
        return self._Tw_e

    @Tw_e.setter
    def Tw_e(self, value):
        temp = np.array(value)
        assert (temp.dtype == np.float64 or temp.dtype == np.int64) and (temp.shape == (4, 4) or temp.shape == (3, 4))
        self._Tw_e = temp

    @property
    def Bw(self):
        return self._Bw

    @Bw.setter
    def Bw(self, value):
        temp = np.array(value)
        assert (temp.dtype == np.float64 or temp.dtype == np.int64) and temp.size == 12
        self._Bw = temp


class TSRChainParameter(object):
    purposes = ("constraint",)
    Template_str = """<tsr_chain purpose="%d" manipulator_index="%d" relative_body_name="%s" relative_link_name="%s" mimic_body_name="%s" mimic_body_index="%s">%s</tsr_chain>"""

    def __init__(self,
                 purpose="constraint",
                 manipulator_index=1,
                 relative_body_name="NULL",
                 relative_link_name="NULL",
                 mimic_body_name="NULL",
                 mimic_body_index=()):
        self._purpose = 0
        self._manipulator_index = 1
        self._relative_body_name = "NULL"
        self._relative_link_name = "NULL"
        self._mimic_body_name = "NULL"
        self._mimic_body_index = []
        self.TSRs = []

        self.purpose = purpose
        self.manipulator_index = manipulator_index
        self.relative_body_name = relative_body_name
        self.relative_link_name = relative_link_name
        self.mimic_body_name = mimic_body_name
        self.mimic_body_index = mimic_body_index

    def __str__(self):
        TSR_str = "".join([str(tsr) for tsr in self.TSRs])
        mimic_indices_str = " ".join([str(ind) for ind in self._mimic_body_index])
        return TSRChainParameter.Template_str % (self._purpose, self._manipulator_index, self._relative_body_name, self._relative_link_name, self._mimic_body_name, mimic_indices_str, TSR_str)

    @property
    def purpose(self):
        return TSRChainParameter.purposes[self._purpose]

    @purpose.setter
    def purpose(self, value):
        assert value in TSRChainParameter.purposes
        self._purpose = TSRChainParameter.purposes.index(value)

    @property
    def manipulator_index(self):
        return self._manipulator_index

    @manipulator_index.setter
    def manipulator_index(self, value):
        assert type(value) == int and value >= 0
        self._manipulator_index = value

    @property
    def relative_body_name(self):
        return self._relative_body_name

    @relative_body_name.setter
    def relative_body_name(self, value):
        assert type(value) == str
        self._relative_body_name = value

    @property
    def relative_link_name(self):
        return self._relative_link_name

    @relative_link_name.setter
    def relative_link_name(self, value):
        assert type(value) == str
        self._relative_link_name = value

    @property
    def mimic_body_name(self):
        return self._mimic_body_name

    @mimic_body_name.setter
    def mimic_body_name(self, value):
        assert type(value) == str
        self._mimic_body_name = value

    @property
    def mimic_body_index(self):
        return self._mimic_body_index

    @mimic_body_index.setter
    def mimic_body_index(self, value):
        assert isinstance(value, Iterable)
        self._mimic_body_index = list(value)

    def addTSR(self, T0_w, Tw_e, Bw):
        self.TSRs.append(TSRParameter(T0_w, Tw_e, Bw))
        return self

    def clearTSRs(self):
        self.TSRs = []
        return self


class PlannerParameter(object):
    def __init__(self):
        self._solver_parameter = SolverParameter()
        self._contraint_parameter = ConstraintParameter()
        self._atlas_parameter = AtlasParameter()
        self.TSRChains = []

    def __str__(self):
        tsrchain_str = "".join([str(tsrchain) for tsrchain in self.TSRChains])
        return str(self._solver_parameter) + str(self._contraint_parameter) + str(self._atlas_parameter) + tsrchain_str

    @property
    def solver_parameter(self):
        return self._solver_parameter

    @property
    def constraint_parameter(self):
        return self._contraint_parameter

    @property
    def atlas_parameter(self):
        return self._atlas_parameter

    def addTSRChain(self, tsrchain):
        assert isinstance(tsrchain, TSRChainParameter)
        self.TSRChains.append(tsrchain)
        return self

    def clearTSRChains(self):
        self.TSRChains = []
        return self
