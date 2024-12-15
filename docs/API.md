# API Documentation

## Enums

### `SolverType`
An enumeration of solver types:
- `RK1`
- `RK2`
- `RK3`
- `RK4`

## Callable

### `Dynamics`
A callable type representing the dynamics function.

- **Parameters:**
    - `time` (`float`): The current time.
    - `state` (`list`): The current state of the system.
    - `input_` (`list`): The input to the system.

- **Returns:**
    - `list`: The derivative of the state.

## Classes

### `SimpleSolver`
A simple solver class.

- **Methods:**
    - `__init__(solver: SolverType, delta_t: float) -> None`
    - `solve(dynamics: Dynamics, time: float, state: list, input_: list) -> list`
    - `delta_t -> float`
    - `_get_solver_class(solver: SolverType) -> type`

### `PlaneBlock`
A class representing a plane block.

- **Methods:**
    - `__init__(solver: SolverType, delta_t: float, model: AerodynamicModel, init: CoreInit, deflection: List[float], ctrl_limit: ControlLimit) -> None`
    - `update(control: Control, t: float) -> CoreOutput`
    - `reset(init: CoreInit) -> None`
    - `state -> CoreOutput`
    - `state_dot -> State`
    - `delete_model() -> None`
    - `delta_t -> float`
    - `_get_core_class(solver: SolverType) -> type`

### `PlaneConstants`
A class representing plane constants.

- **Methods:**
    - `__init__(m: float, b: float, s: float, c_bar: float, x_cg_r: float, x_cg: float, h_eng: float, j_y: float, j_xz: float, j_z: float, j_x: float) -> None`
    - Properties: `m`, `b`, `s`, `c_bar`, `x_cg_r`, `x_cg`, `h_eng`, `j_y`, `j_xz`, `j_z`, `j_x`

### `ControlLimit`
A class representing control limits.

- **Methods:**
    - `__init__(thrust_cmd_limit_top: float, thrust_cmd_limit_bottom: float, thrust_rate_limit: float, ele_cmd_limit_top: float, ele_cmd_limit_bottom: float, ele_rate_limit: float, ail_cmd_limit_top: float, ail_cmd_limit_bottom: float, ail_rate_limit: float, rud_cmd_limit_top: float, rud_cmd_limit_bottom: float, rud_rate_limit: float, alpha_limit_top: float, alpha_limit_bottom: float, beta_limit_top: float, beta_limit_bottom: float) -> None`
    - Properties: `thrust_cmd_limit_top`, `thrust_cmd_limit_bottom`, `thrust_rate_limit`, `ele_cmd_limit_top`, `ele_cmd_limit_bottom`, `ele_rate_limit`, `ail_cmd_limit_top`, `ail_cmd_limit_bottom`, `ail_rate_limit`, `rud_cmd_limit_top`, `rud_cmd_limit_bottom`, `rud_rate_limit`, `alpha_limit_top`, `alpha_limit_bottom`, `beta_limit_top`, `beta_limit_bottom`

### `AerodynamicModel`
A class representing an aerodynamic model.

- **Methods:**
    - `__init__(path: str) -> None`
    - `install(path: str) -> None`
    - `uninstall() -> None`
    - `load_constants() -> PlaneConstants`
    - `load_ctrl_limits() -> ControlLimit`

### `State`
A class representing the state of the plane.

- **Methods:**
    - `__init__(npos: float, epos: float, altitude: float, phi: float, theta: float, psi: float, velocity: float, alpha: float, beta: float, p: float, q: float, r: float) -> None`
    - `from_list(list: List[float]) -> "State"`
    - `to_list() -> List[float]`
    - Properties: `npos`, `epos`, `altitude`, `phi`, `theta`, `psi`, `velocity`, `alpha`, `beta`, `p`, `q`, `r`

### `Control`
A class representing the control inputs.

- **Methods:**
    - `__init__(thrust: float, elevator: float, aileron: float, rudder: float) -> None`
    - `from_list(list: List[float]) -> "Control"`
    - `to_list() -> List[float]`
    - Properties: `thrust`, `elevator`, `aileron`, `rudder`

### `StateExtend`
A class representing extended state information.

- **Methods:**
    - `__init__(nx: float, ny: float, nz: float, mach: float, qbar: float, ps: float) -> None`
    - `from_list(list: List[float]) -> "StateExtend"`
    - `to_list() -> List[float]`
    - Properties: `nx`, `ny`, `nz`, `mach`, `qbar`, `ps`

### `NelderMeadResult`
A class representing the result of a Nelder-Mead optimization.

- **Properties:**
    - `x`
    - `fval`
    - `iter`
    - `fun_evals`

### `FlightCondition`
A class representing flight conditions.

- **Methods:**
    - `__init__(value: int) -> None`
    - `wings_level() -> "FlightCondition"`
    - `turning() -> "FlightCondition"`
    - `pull_up() -> "FlightCondition"`
    - `roll() -> "FlightCondition"`
    - Properties: `value`

### `TrimInit`
A class representing the initial trim state.

- **Methods:**
    - `__init__(control: Control, alpha: float) -> None`
    - Properties: `control`, `alpha`

### `TrimTarget`
A class representing the target trim state.

- **Methods:**
    - `__init__(altitude: float, velocity: float, npos: Optional[float] = None, epos: Optional[float] = None) -> None`
    - Properties: `altitude`, `velocity`, `npos`, `epos`

### `TrimOutput`
A class representing the output of a trim calculation.

- **Methods:**
    - `__init__(state: State, control: Control, state_extend: StateExtend, nelder_mead_result: NelderMeadResult) -> None`
    - `to_core_init() -> CoreInit`
    - Properties: `state`, `control`, `state_extend`, `nelder_mead_result`

### `NelderMeadOptions`
A class representing options for Nelder-Mead optimization.

- **Methods:**
    - `__init__(max_fun_evals: int, max_iter: int, tol_fun: float, tol_x: float) -> None`
    - Properties: `max_fun_evals`, `max_iter`, `tol_fun`, `tol_x`

### `CoreOutput`
A class representing the core output.

- **Properties:**
    - `state`
    - `control`
    - `state_extend`

### `CoreInit`
A class representing the core initialization.

- **Methods:**
    - `__init__(state: State, control: Control) -> None`
    - Properties: `state`, `control`

## Functions

### `trim`
A function to perform trim calculation.

- **Parameters:**
    - `model: AerodynamicModel`
    - `trim_target: TrimTarget`
    - `ctrl_limit: ControlLimit`
    - `trim_init: Optional[TrimInit] = None`
    - `flight_condition: Optional[FlightCondition] = None`
    - `optim_options: Optional[NelderMeadOptions] = None`

- **Returns:**
    - `TrimOutput`
