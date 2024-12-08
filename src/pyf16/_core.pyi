from __future__ import annotations
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
    ) -> None: ...
    @property
    def m(self) -> float: ...
    @property
    def b(self) -> float: ...
    @property
    def s(self) -> float: ...
    @property
    def c_bar(self) -> float: ...
    @property
    def x_cg_r(self) -> float: ...
    @property
    def x_cg(self) -> float: ...
    @property
    def h_eng(self) -> float: ...
    @property
    def j_y(self) -> float: ...
    @property
    def j_xz(self) -> float: ...
    @property
    def j_z(self) -> float: ...
    @property
    def j_x(self) -> float: ...

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
    ) -> None: ...
    @property
    def thrust_cmd_limit_top(self) -> float: ...
    @property
    def thrust_cmd_limit_bottom(self) -> float: ...
    @property
    def thrust_rate_limit(self) -> float: ...
    @property
    def ele_cmd_limit_top(self) -> float: ...
    @property
    def ele_cmd_limit_bottom(self) -> float: ...
    @property
    def ele_rate_limit(self) -> float: ...
    @property
    def ail_cmd_limit_top(self) -> float: ...
    @property
    def ail_cmd_limit_bottom(self) -> float: ...
    @property
    def ail_rate_limit(self) -> float: ...
    @property
    def rud_cmd_limit_top(self) -> float: ...
    @property
    def rud_cmd_limit_bottom(self) -> float: ...
    @property
    def rud_rate_limit(self) -> float: ...
    @property
    def alpha_limit_top(self) -> float: ...
    @property
    def alpha_limit_bottom(self) -> float: ...
    @property
    def beta_limit_top(self) -> float: ...
    @property
    def beta_limit_bottom(self) -> float: ...

class AerodynamicModel:
    def __init__(self, path: str) -> None: ...
    def install(self, path: str) -> None: ...
    def uninstall(self) -> None: ...
    def load_constants(self) -> PlaneConstants: ...
    def load_ctrl_limits(self) -> ControlLimit: ...

class State:
    def to_list(self) -> List[float]: ...
    @property
    def npos(self) -> float: ...
    @property
    def epos(self) -> float: ...
    @property
    def altitude(self) -> float: ...
    @property
    def phi(self) -> float: ...
    @property
    def theta(self) -> float: ...
    @property
    def psi(self) -> float: ...
    @property
    def velocity(self) -> float: ...
    @property
    def alpha(self) -> float: ...
    @property
    def beta(self) -> float: ...
    @property
    def p(self) -> float: ...
    @property
    def q(self) -> float: ...
    @property
    def r(self) -> float: ...

class Control:
    def __init__(
        self, thrust: float, elevator: float, aileron: float, rudder: float
    ) -> None: ...
    def to_list(self) -> List[float]: ...
    @property
    def thrust(self) -> float: ...
    @property
    def elevator(self) -> float: ...
    @property
    def aileron(self) -> float: ...
    @property
    def rudder(self) -> float: ...
    @thrust.setter
    def thrust(self, value: float) -> None: ...
    @elevator.setter
    def elevator(self, value: float) -> None: ...
    @aileron.setter
    def aileron(self, value: float) -> None: ...
    @rudder.setter
    def rudder(self, value: float) -> None: ...

class StateExtend:
    def __init__(
        self, nx: float, ny: float, nz: float, mach: float, qbar: float, ps: float
    ) -> None: ...
    def to_list(self) -> List[float]: ...
    @property
    def nx(self) -> float: ...
    @property
    def ny(self) -> float: ...
    @property
    def nz(self) -> float: ...
    @property
    def mach(self) -> float: ...
    @property
    def qbar(self) -> float: ...
    @property
    def ps(self) -> float: ...

class NelderMeadResult:
    @property
    def x(self) -> List[float]: ...
    @property
    def fval(self) -> float: ...
    @property
    def iter(self) -> int: ...
    @property
    def fun_evals(self) -> int: ...

class FlightCondition:
    def __init__(self, value: int) -> None: ...
    @property
    def value(self) -> int: ...
    @staticmethod
    def wings_level() -> "FlightCondition": ...
    @staticmethod
    def turning() -> "FlightCondition": ...
    @staticmethod
    def pull_up() -> "FlightCondition": ...
    @staticmethod
    def roll() -> "FlightCondition": ...

class TrimInit:
    def __init__(self, control: Control, alpha: float) -> None: ...
    @property
    def control(self) -> Control: ...
    @property
    def alpha(self) -> float: ...
    @control.setter
    def control(self, control: Control) -> None: ...
    @alpha.setter
    def alpha(self, value: float) -> None: ...

class TrimTarget:
    def __init__(
        self,
        altitude: float,
        velocity: float,
        npos: Optional[float] = None,
        epos: Optional[float] = None,
    ) -> None: ...
    @property
    def altitude(self) -> float: ...
    @property
    def velocity(self) -> float: ...
    @property
    def npos(self) -> float: ...
    @property
    def epos(self) -> float: ...
    @altitude.setter
    def altitude(self, value: float) -> None: ...
    @velocity.setter
    def velocity(self, value: float) -> None: ...
    @npos.setter
    def npos(self, value: float) -> None: ...
    @epos.setter
    def epos(self, value: float) -> None: ...

class TrimOutput:
    def __init__(
        self,
        state: State,
        control: Control,
        state_extend: StateExtend,
        nelder_mead_result: NelderMeadResult,
    ) -> None: ...
    @property
    def state(self) -> State: ...
    @property
    def control(self) -> Control: ...
    @property
    def state_extend(self) -> StateExtend: ...
    @property
    def nelder_mead_result(self) -> NelderMeadResult: ...
    def to_core_init(self) -> CoreInit: ...

class NelderMeadOptions:
    def __init__(
        self, max_fun_evals: int, max_iter: int, tol_fun: float, tol_x: float
    ) -> None: ...
    @property
    def max_fun_evals(self) -> int: ...
    @property
    def max_iter(self) -> int: ...
    @property
    def tol_fun(self) -> float: ...
    @property
    def tol_x(self) -> float: ...
    @max_fun_evals.setter
    def max_fun_evals(self, value: int) -> None: ...
    @max_iter.setter
    def max_iter(self, value: int) -> None: ...
    @tol_fun.setter
    def tol_fun(self, value: float) -> None: ...
    @tol_x.setter
    def tol_x(self, value: float) -> None: ...

def trim(
    model: AerodynamicModel,
    trim_target: TrimTarget,
    ctrl_limit: ControlLimit,
    trim_init: Optional[TrimInit] = None,
    flight_condition: Optional[FlightCondition] = None,
    optim_options: Optional[NelderMeadOptions] = None,
) -> TrimOutput: ...

class CoreOutput:
    @property
    def state(self) -> State: ...
    @property
    def control(self) -> Control: ...
    @property
    def state_extend(self) -> StateExtend: ...

class CoreInit:
    def __init__(self, state: State, control: Control) -> None: ...
    @property
    def state(self) -> State: ...
    @state.setter
    def state(self, value: State) -> None: ...
    @property
    def control(self) -> Control: ...
    @control.setter
    def control(self, value: Control) -> None: ...

class PlaneBlock:
    def __init__(
        self,
        id: str,
        model: AerodynamicModel,
        init: CoreInit,
        deflection: List[float],
        ctrl_limit: ControlLimit,
    ) -> None: ...
    def update(self, control: Control, t: float) -> CoreOutput: ...
    def reset(self) -> None: ...
    @property
    def state(self) -> CoreOutput: ...
    def delete_model(self) -> None: ...
