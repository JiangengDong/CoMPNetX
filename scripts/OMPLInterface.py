# encoding: utf-8

import os
import numpy as np
import sys
import time
from collections import Iterable

import openravepy as orpy


def SerializeTransform(tm):
    flatten_tm = np.array(tm[0:3, 0:4]).T.flatten()
    return ' '.join(['%.5f' % v for v in flatten_tm])


def SerializeBound(bw):
    flatten_bw = np.array(bw).flatten()
    return ' '.join(['%.5f' % v for v in flatten_bw])


def RPY2Transform(psi, theta, phi, x, y, z):
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    cTheta = np.cos(theta)
    sTheta = np.sin(theta)
    cPhi = np.cos(phi)
    sPhi = np.sin(phi)
    return np.mat([[cTheta * cPhi, sPsi * sTheta * cPhi - cPsi * sPhi, cPsi * sTheta * cPhi + sPsi * sPhi, x],
                   [cTheta * sPhi, sPsi * sTheta * sPhi + cPsi * cPhi, cPsi * sTheta * sPhi - sPsi * cPhi, y],
                   [-sTheta, sPsi * cTheta, cPsi * cTheta, z],
                   [0, 0, 0, 1]])


def quat2Transform(a, b, c, d, x, y, z):
    # a+bi+cj+dk
    assert 1 - 1e-4 < a ** 2 + b ** 2 + c ** 2 + d ** 2 < 1 + 1e-4
    return np.mat([[a ** 2 + b ** 2 - c ** 2 - d ** 2, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c, x],
                   [2 * b * c + 2 * a * d, a ** 2 - b ** 2 + c ** 2 - d ** 2, 2 * c * d - 2 * a * b, y],
                   [2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a * 2 - b ** 2 - c ** 2 + d ** 2, z],
                   [0, 0, 0, 1]])


def pause():
    print "Press enter to continue..."
    sys.stdin.readline()


class SolverParameter(object):
    types = ("rrt", "rrtstar", "rrtconnect", "mpnet")
    Template_str = """<solver_parameters type="%d" time="%d" range="%f"/>\n"""

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
    Template_str = """<constraint_parameters type="%d" tolerance="%f" max_iter="%d" delta="%f" lambda="%f"/>\n"""

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
    Template_str = """<atlas_parameters exploration="%f" epsilon="%f" rho="%f" alpha="%f" max_charts="%d" using_bias="%d" separate="%d"/>\n"""

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
    Template_str = """<tsr T0_w="%s" Tw_e="%s" Bw="%s"/>\n"""

    def __init__(self, T0_w=np.eye(4), Tw_e=np.eye(4), Bw=np.zeros((6, 2))):
        self._T0_w = np.eye(4)
        self._Tw_e = np.eye(4)
        self._Bw = np.zeros((6, 2))

        self.T0_w = T0_w
        self.Tw_e = Tw_e
        self.Bw = Bw

    def __str__(self):
        return TSRParameter.Template_str % (SerializeTransform(self._T0_w), SerializeTransform(self._Tw_e), SerializeBound(self._Bw))

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


class TSRChain(object):
    purposes = ("constraint",)
    Template_str = """<tsr_chain purpose="%d" manipulator_index="%d" relative_body_name="%s" relative_link_name="%s" mimic_body_name="%s" mimic_body_index="%s">\n%s</tsr_chain>\n"""

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
        return TSRChain.Template_str % (self._purpose, self._manipulator_index, self._relative_body_name, self._relative_link_name, self._mimic_body_name, mimic_indices_str, TSR_str)

    @property
    def purpose(self):
        return TSRChain.purposes[self._purpose]

    @purpose.setter
    def purpose(self, value):
        assert value in TSRChain.purposes
        self._purpose = TSRChain.purposes.index(value)

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


class MPNetParameter(object):
    Template_str = """<mpnet pnet_path="%s" dnet_path="%s" voxel_path="%s" ohot_path="%s" dnet_threshold="%s" dnet_coeff="%s"/>\n"""

    def __init__(self, pnet_path="", dnet_path="", voxel_path="", ohot_path="", dnet_threshold=0.3, dnet_coeff=0.4):
        self._pnet_path = ""
        self._dnet_path = ""
        self._voxel_path = ""
        self._ohot_path = ""
        self._dnet_threshold = 0.3
        self._dnet_coeff = 0.4

        self.pnet_path = pnet_path
        self.dnet_path = dnet_path
        self.voxel_path = voxel_path
        self.ohot_path = ohot_path
        self.dnet_threshold = dnet_threshold
        self.dnet_coeff = dnet_coeff

    def __str__(self):
        return MPNetParameter.Template_str % (self._pnet_path, self._dnet_path, self._voxel_path, self._ohot_path, self._dnet_threshold, self._dnet_coeff)

    @property
    def pnet_path(self):
        return self._pnet_path

    @pnet_path.setter
    def pnet_path(self, value):
        if value:
            pnet_path = os.path.abspath(value)
            if not os.path.exists(pnet_path):
                raise IOError("%s not found" % value)
            self._pnet_path = pnet_path

    @property
    def dnet_path(self):
        return self._dnet_path

    @dnet_path.setter
    def dnet_path(self, value):
        if value:
            dnet_path = os.path.abspath(value)
            if not os.path.exists(dnet_path):
                raise IOError("%s not found" % value)
            self._dnet_path = dnet_path

    @property
    def voxel_path(self):
        return self._voxel_path

    @voxel_path.setter
    def voxel_path(self, value):
        if value:
            voxel_path = os.path.abspath(value)
            if not os.path.exists(voxel_path):
                raise IOError("%s not found" % value)
            self._voxel_path = voxel_path

    @property
    def ohot_path(self):
        return self._ohot_path

    @ohot_path.setter
    def ohot_path(self, value):
        if value:
            ohot_path = os.path.abspath(value)
            if not os.path.exists(ohot_path):
                raise IOError("%s not found" % value)
            self._ohot_path = ohot_path

    @property
    def dnet_threshold(self):
        return self._dnet_threshold

    @dnet_threshold.setter
    def dnet_threshold(self, value):
        assert value > 0
        self._dnet_threshold = value

    @property
    def dnet_coeff(self):
        return self._dnet_coeff

    @dnet_coeff.setter
    def dnet_coeff(self, value):
        assert value > 0
        self._dnet_coeff = value


class PlannerParameter(object):
    def __init__(self):
        self._solver_parameter = SolverParameter()
        self._contraint_parameter = ConstraintParameter()
        self._atlas_parameter = AtlasParameter()
        self.TSRChains = []
        self._mpnet_parameter = MPNetParameter()

    def __str__(self):
        tsrchain_str = "".join([str(tsrchain) for tsrchain in self.TSRChains])
        return str(self._solver_parameter) + str(self._contraint_parameter) + str(self._atlas_parameter) + "<tsr_chains>\n" + tsrchain_str + "</tsr_chains>\n" + str(self._mpnet_parameter)

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
        assert isinstance(tsrchain, TSRChain)
        self.TSRChains.append(tsrchain)
        return self

    def clearTSRChains(self):
        self.TSRChains = []
        return self

    @property
    def mpnet_parameter(self):
        return self._mpnet_parameter


class EmptyContext(object):
    def __init__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, trace):
        pass


class OMPLInterface:
    def __init__(self, env, robot, loglevel=2):
        self.env = env
        self.robot = robot
        self.planner = orpy.RaveCreatePlanner(self.env, 'AtlasMPNet')
        assert self.planner != None
        self.planner.SendCommand("SetLogLevel %d" % loglevel)

    def solve(self, start_config, goal_config, planner_params):
        assert isinstance(planner_params, PlannerParameter)
        dof = self.robot.GetActiveDOF()
        assert len(start_config) == dof
        assert len(goal_config) == dof

        params = orpy.Planner.PlannerParameters()

        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(start_config)
        params.SetGoalConfig(goal_config)
        params.SetExtraParameters(str(planner_params))

        with self.env, self.robot:
        # with EmptyContext():
            if not self.planner.InitPlan(self.robot, params):
                print("Start or goal is invalid!")
                return None, np.nan, None

            traj = orpy.RaveCreateTrajectory(self.env, '')
            status = self.planner.PlanPath(traj)
            if status == orpy.PlannerStatus.HasSolution:
                t_time = float(self.planner.SendCommand("GetPlanningTime"))
                orpy.planningutils.RetimeTrajectory(traj)
                print("Found a solution after %f seconds." % t_time)
                return True, t_time, traj
            else:
                print("Failed to find a solution. ")
                return False, np.inf, None

    def distanceToManifold(self, configs, planner_params, startik, goalik):
        assert isinstance(planner_params, PlannerParameter)
        dof = self.robot.GetActiveDOF()

        params = orpy.Planner.PlannerParameters()

        params.SetRobotActiveJoints(self.robot)
        params.SetInitialConfig(startik)
        params.SetGoalConfig(goalik)
        params.SetExtraParameters(str(planner_params))

        # with self.env, self.robot:
        with EmptyContext():
            if not self.planner.InitPlan(self.robot, params):
                print("Start or goal is invalid!")
                return None

            commandStr = " ".join(["GetDistanceToManifold"] + ["%f"]*dof)

            def singleDistance(config):
                s = self.planner.SendCommand(commandStr % tuple(config))
                s_split = [float(val) for val in s.split(" ")]
                vals = s_split[:-1]
                dist = s_split[-1]
                if len(vals) == 4:
                    vals.append(0.0)
                    vals.append(0.0)
                    vals[5] = vals[3]
                    vals[3] = 0.0
                return vals, dist

            all_list = [singleDistance(config) for config in configs]
            val_list = [vals for (vals, dist) in all_list]
            dist_list = [dist for (vals, dist) in all_list]

        return val_list, dist_list


