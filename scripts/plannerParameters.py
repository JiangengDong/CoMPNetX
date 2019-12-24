import xml.etree.ElementTree as ET
from math import pi


class SolverParameters(object):
    def __init__(self):
        self._element = ET.fromstring("""<planner_parameters time="20.0" range="0.2"/>""")

    def __str__(self):
        return ET.tostring(self._element)

    @property
    def time(self):
        return float(self._element.get("time", None))

    @time.setter
    def time(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("time", str(value))

    @property
    def range(self):
        return float(self._element.get("range", None))

    @range.setter
    def range(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("range", str(value))


class ConstraintParameters(object):
    def __init__(self):
        self._element = ET.fromstring("""<constraint_parameters tolerance="0.01" max_iter="50" delta="0.05" lambda="2"/>""")

    def __str__(self):
        return ET.tostring(self._element)

    @property
    def tolerance(self):
        return float(self._element.get("tolerance", None))

    @tolerance.setter
    def tolerance(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("tolerance", str(value))

    @property
    def max_iter(self):
        return float(self._element.get("max_iter", None))

    @max_iter.setter
    def max_iter(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("max_iter", str(value))

    @property
    def delta(self):
        return float(self._element.get("delta", None))

    @delta.setter
    def delta(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("delta", str(value))

    @property
    def lambd(self):
        return float(self._element.get("lambda", None))

    @lambd.setter
    def lambd(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("lambda", str(value))


class AtlasParameters(object):
    def __init__(self):
        self._element = ET.fromstring("""<atlas_parameters exploration="0.5" epsilon="0.05" rho="0.08" alpha="0.392699" max_charts="500" using_bias="0" using_tb="0" separate="0"/>""")

    def __str__(self):
        return ET.tostring(self._element)

    @property
    def exploration(self):
        return float(self._element.get("exploration", None))

    @exploration.setter
    def exploration(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("exploration", str(value))

    @property
    def epsilon(self):
        return float(self._element.get("epsilon", None))

    @epsilon.setter
    def epsilon(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("epsilon", str(value))

    @property
    def rho(self):
        return float(self._element.get("rho", None))

    @rho.setter
    def rho(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("rho", str(value))

    @property
    def alpha(self):
        return float(self._element.get("alpha", None))

    @alpha.setter
    def alpha(self, value):
        assert type(value) is float or type(value) is int
        self._element.set("alpha", str(value))

    @property
    def max_chart(self):
        return float(self._element.get("max_chart", None))

    @max_chart.setter
    def max_chart(self, value):
        assert type(value) is int
        self._element.set("max_chart", str(value))

    @property
    def using_bias(self):
        return float(self._element.get("using_bias", None))

    @using_bias.setter
    def using_bias(self, value):
        assert type(value) is bool
        self._element.set("using_bias", "1" if value else "0")

    @property
    def using_tb(self):
        return float(self._element.get("using_tb", None))

    @using_tb.setter
    def using_tb(self, value):
        assert type(value) is bool
        self._element.set("using_tb", "1" if value else "0")

    @property
    def separate(self):
        return float(self._element.get("separate", None))

    @separate.setter
    def separate(self, value):
        assert type(value) is bool
        self._element.set("separate", "1" if value else "0")

class PlannerParameters:
    def __init__(self):
        template_strings = ["""<tsr_chain purpose="0 0 1" mimic_body_name="NULL">
                               <tsr manipulator_index="0" relative_body_name="NULL"
                               T0_w="1 0 0 0 1  0 0 0 1 0.6923      0 0"
                               Tw_e="1 0 0 0 0 -1 0 1 0      0 -0.285 0"
                               Bw="-1000 1000 -1000 1000 -1000 1000 0 0 0 0 -4 4" />
                               </tsr_chain>"""]
        self.planner_parameters = SolverParameters()
        self.constraint_parameters = ConstraintParameters()
        self.atlas_parameters = AtlasParameters()

    def __str__(self):
        return str(self.planner_parameters) + '\n' + str(self.constraint_parameters) + '\n' + str(self.atlas_parameters)
