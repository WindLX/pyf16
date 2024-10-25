from typing import Optional, List

class PlaneConstants:
    def __init__(
        self,
        m: float,
        b: float,
        s: float,
        c_bar: float,
        x_cg_r: float,
        x_cg: float,
        h_eng: float,
        j_y: float,
        j_xz: float,
        j_z: float,
        j_x: float,
    ) -> None:
        self.m: float
        self.b: float
        self.s: float
        self.c_bar: float
        self.x_cg_r: float
        self.x_cg: float
        self.h_eng: float
        self.j_y: float
        self.j_xz: float
        self.j_z: float
        self.j_x: float

    @property
    def m(self) -> float:
        pass

    @property
    def b(self) -> float:
        pass

    @property
    def s(self) -> float:
        pass

    @property
    def c_bar(self) -> float:
        pass

    @property
    def x_cg_r(self) -> float:
        pass

    @property
    def x_cg(self) -> float:
        pass

    @property
    def h_eng(self) -> float:
        pass

    @property
    def j_y(self) -> float:
        pass

    @property
    def j_xz(self) -> float:
        pass

    @property
    def j_z(self) -> float:
        pass

    @property
    def j_x(self) -> float:
        pass

class ControlLimit:
    def __init__(
        self,
        thrust_cmd_limit_top: float,
        thrust_cmd_limit_bottom: float,
        thrust_rate_limit: float,
        ele_cmd_limit_top: float,
        ele_cmd_limit_bottom: float,
        ele_rate_limit: float,
        ail_cmd_limit_top: float,
        ail_cmd_limit_bottom: float,
        ail_rate_limit: float,
        rud_cmd_limit_top: float,
        rud_cmd_limit_bottom: float,
        rud_rate_limit: float,
        alpha_limit_top: float,
        alpha_limit_bottom: float,
        beta_limit_top: float,
        beta_limit_bottom: float,
    ) -> None:
        self.thrust_cmd_limit_top: float
        self.thrust_cmd_limit_bottom: float
        self.thrust_rate_limit: float
        self.ele_cmd_limit_top: float
        self.ele_cmd_limit_bottom: float
        self.ele_rate_limit: float
        self.ail_cmd_limit_top: float
        self.ail_cmd_limit_bottom: float
        self.ail_rate_limit: float
        self.rud_cmd_limit_top: float
        self.rud_cmd_limit_bottom: float
        self.rud_rate_limit: float
        self.alpha_limit_top: float
        self.alpha_limit_bottom: float
        self.beta_limit_top: float
        self.beta_limit_bottom: float

    @property
    def thrust_cmd_limit_top(self) -> float:
        pass

    @property
    def thrust_cmd_limit_bottom(self) -> float:
        pass

    @property
    def thrust_rate_limit(self) -> float:
        pass

    @property
    def ele_cmd_limit_top(self) -> float:
        pass

    @property
    def ele_cmd_limit_bottom(self) -> float:
        pass

    @property
    def ele_rate_limit(self) -> float:
        pass

    @property
    def ail_cmd_limit_top(self) -> float:
        pass

    @property
    def ail_cmd_limit_bottom(self) -> float:
        pass

    @property
    def ail_rate_limit(self) -> float:
        pass

    @property
    def rud_cmd_limit_top(self) -> float:
        pass

    @property
    def rud_cmd_limit_bottom(self) -> float:
        pass

    @property
    def rud_rate_limit(self) -> float:
        pass

    @property
    def alpha_limit_top(self) -> float:
        pass

    @property
    def alpha_limit_bottom(self) -> float:
        pass

    @property
    def beta_limit_top(self) -> float:
        pass

    @property
    def beta_limit_bottom(self) -> float:
        pass

class AerodynamicModel:
    def __init__(self, path: str) -> None:
        pass

    def install(self, path: str) -> None:
        pass

    def uninstall(self) -> None:
        pass

    def load_constants(self) -> PlaneConstants:
        pass

    def load_ctrl_limits(self) -> ControlLimit:
        pass

class State:
    def to_list(self) -> List[float]:
        pass

    @property
    def npos(self) -> float:
        pass

    @property
    def epos(self) -> float:
        pass

    @property
    def altitude(self) -> float:
        pass

    @property
    def phi(self) -> float:
        pass

    @property
    def theta(self) -> float:
        pass

    @property
    def psi(self) -> float:
        pass

    @property
    def velocity(self) -> float:
        pass

    @property
    def alpha(self) -> float:
        pass

    @property
    def beta(self) -> float:
        pass

    @property
    def p(self) -> float:
        pass

    @property
    def q(self) -> float:
        pass

    @property
    def r(self) -> float:
        pass

class Control:
    def __init__(
        self, thrust: float, elevator: float, aileron: float, rudder: float
    ) -> None:
        self.thrust: float
        self.elevator: float
        self.aileron: float
        self.rudder: float

    def to_list(self) -> List[float]:
        pass

    @property
    def thrust(self) -> float:
        pass

    @property
    def elevator(self) -> float:
        pass

    @property
    def aileron(self) -> float:
        pass

    @property
    def rudder(self) -> float:
        pass

    @thrust.setter
    def thrust(self, value: float) -> None:
        pass

    @elevator.setter
    def elevator(self, value: float) -> None:
        pass

    @aileron.setter
    def aileron(self, value: float) -> None:
        pass

    @rudder.setter
    def rudder(self, value: float) -> None:
        pass

class StateExtend:
    def to_list(self) -> List[float]:
        pass

    @property
    def nx(self) -> float:
        pass

    @property
    def ny(self) -> float:
        pass

    @property
    def nz(self) -> float:
        pass

    @property
    def mach(self) -> float:
        pass

    @property
    def qbar(self) -> float:
        pass

    @property
    def ps(self) -> float:
        pass

class NelderMeadResult:
    @property
    def x(self) -> List[float]:
        pass

    @property
    def fval(self) -> float:
        pass

    @property
    def iter(self) -> int:
        pass

    @property
    def fun_evals(self) -> int:
        pass

class FlightCondition:
    def __init__(self, value: int) -> None:
        pass

    @property
    def value(self) -> int:
        pass

    @staticmethod
    def wings_level() -> "FlightCondition":
        pass

    @staticmethod
    def turning() -> "FlightCondition":
        pass

    @staticmethod
    def pull_up() -> "FlightCondition":
        pass

    @staticmethod
    def roll() -> "FlightCondition":
        pass

class TrimInit:
    def __init__(self, control: Control, alpha: float) -> None:
        self.control: Control
        self.alpha: float

    @property
    def control(self) -> Control:
        pass

    @property
    def alpha(self) -> float:
        pass

    @control.setter
    def control(self, control: Control) -> None:
        pass

    @alpha.setter
    def alpha(self, value: float) -> None:
        pass

class TrimTarget:
    def __init__(
        self,
        altitude: float,
        velocity: float,
        npos: Optional[float] = None,
        epos: Optional[float] = None,
    ) -> None:
        self.altitude: float
        self.velocity: float
        self.npos: float
        self.epos: float

    @property
    def altitude(self) -> float:
        pass

    @property
    def velocity(self) -> float:
        pass

    @property
    def npos(self) -> float:
        pass

    @property
    def epos(self) -> float:
        pass

    @altitude.setter
    def altitude(self, value: float) -> None:
        pass

    @velocity.setter
    def velocity(self, value: float) -> None:
        pass

    @npos.setter
    def npos(self, value: float) -> None:
        pass

    @epos.setter
    def epos(self, value: float) -> None:
        pass

class TrimOutput:
    def __init__(
        self,
        state: State,
        control: Control,
        state_extend: StateExtend,
        nelder_mead_result: NelderMeadResult,
    ) -> None:
        self.state: State
        self.control: Control
        self.state_extend: StateExtend
        self.nelder_mead_result: NelderMeadResult

    @property
    def state(self) -> State:
        pass

    @property
    def control(self) -> Control:
        pass

    @property
    def state_extend(self) -> StateExtend:
        pass

    @property
    def nelder_mead_result(self) -> NelderMeadResult:
        pass

class NelderMeadOptions:
    def __init__(
        self, max_fun_evals: int, max_iter: int, tol_fun: float, tol_x: float
    ) -> None:
        self.max_fun_evals: int
        self.max_iter: int
        self.tol_fun: float
        self.tol_x: float

    @property
    def max_fun_evals(self) -> int:
        pass

    @property
    def max_iter(self) -> int:
        pass

    @property
    def tol_fun(self) -> float:
        pass

    @property
    def tol_x(self) -> float:
        pass

    @max_fun_evals.setter
    def max_fun_evals(self, value: int) -> None:
        pass

    @max_iter.setter
    def max_iter(self, value: int) -> None:
        pass

    @tol_fun.setter
    def tol_fun(self, value: float) -> None:
        pass

    @tol_x.setter
    def tol_x(self, value: float) -> None:
        pass

class CoreOutput:
    @property
    def state(self) -> State:
        pass

    @property
    def control(self) -> Control:
        pass

    @property
    def state_extend(self) -> StateExtend:
        pass

def trim(
    model: AerodynamicModel,
    trim_target: TrimTarget,
    ctrl_limit: ControlLimit,
    trim_init: Optional[TrimInit] = None,
    flight_condition: Optional[FlightCondition] = None,
    optim_options: Optional[NelderMeadOptions] = None,
) -> TrimOutput:
    pass

class PlaneBlock:
    def __init__(
        self,
        id: str,
        model: AerodynamicModel,
        init: TrimOutput,
        deflection: List[float],
        ctrl_limit: ControlLimit,
    ) -> None:
        pass

    def update(self, control: Control, t: float) -> CoreOutput:
        pass

    def reset(self) -> None:
        pass

    @property
    def state(self) -> CoreOutput:
        pass

    def delete_model(self) -> None:
        pass
