# API Documentation

## Classes

### PlaneConstants
Represents the constants of a plane.

#### Methods
- `__init__(m: float, b: float, s: float, c_bar: float, x_cg_r: float, x_cg: float, h_eng: float, j_y: float, j_xz: float, j_z: float, j_x: float) -> None`
- Properties:
    - `m: float`
    - `b: float`
    - `s: float`
    - `c_bar: float`
    - `x_cg_r: float`
    - `x_cg: float`
    - `h_eng: float`
    - `j_y: float`
    - `j_xz: float`
    - `j_z: float`
    - `j_x: float`

### ControlLimit
Represents the control limits of a plane.

#### Methods
- `__init__(thrust_cmd_limit_top: float, thrust_cmd_limit_bottom: float, thrust_rate_limit: float, ele_cmd_limit_top: float, ele_cmd_limit_bottom: float, ele_rate_limit: float, ail_cmd_limit_top: float, ail_cmd_limit_bottom: float, ail_rate_limit: float, rud_cmd_limit_top: float, rud_cmd_limit_bottom: float, rud_rate_limit: float, alpha_limit_top: float, alpha_limit_bottom: float, beta_limit_top: float, beta_limit_bottom: float) -> None`
- Properties:
    - `thrust_cmd_limit_top: float`
    - `thrust_cmd_limit_bottom: float`
    - `thrust_rate_limit: float`
    - `ele_cmd_limit_top: float`
    - `ele_cmd_limit_bottom: float`
    - `ele_rate_limit: float`
    - `ail_cmd_limit_top: float`
    - `ail_cmd_limit_bottom: float`
    - `ail_rate_limit: float`
    - `rud_cmd_limit_top: float`
    - `rud_cmd_limit_bottom: float`
    - `rud_rate_limit: float`
    - `alpha_limit_top: float`
    - `alpha_limit_bottom: float`
    - `beta_limit_top: float`
    - `beta_limit_bottom: float`

### AerodynamicModel
Represents the aerodynamic model of a plane.

#### Methods
- `__init__(path: str) -> None`
- `install(path: str) -> None`
- `uninstall() -> None`
- `load_constants() -> PlaneConstants`
- `load_ctrl_limits() -> ControlLimit`

### State
Represents the state of a plane.

#### Methods
- `to_list() -> List[float]`
- Properties:
    - `npos: float`
    - `epos: float`
    - `altitude: float`
    - `phi: float`
    - `theta: float`
    - `psi: float`
    - `velocity: float`
    - `alpha: float`
    - `beta: float`
    - `p: float`
    - `q: float`
    - `r: float`

### Control
Represents the control inputs of a plane.

#### Methods
- `__init__(thrust: float, elevator: float, aileron: float, rudder: float) -> None`
- `to_list() -> List[float]`
- Properties:
    - `thrust: float`
    - `elevator: float`
    - `aileron: float`
    - `rudder: float`
- Setters:
    - `thrust(value: float) -> None`
    - `elevator(value: float) -> None`
    - `aileron(value: float) -> None`
    - `rudder(value: float) -> None`

### StateExtend
Represents the extended state of a plane.

#### Methods
- `__init__(nx: float, ny: float, nz: float, mach: float, qbar: float, ps: float) -> None`
- `to_list() -> List[float]`
- Properties:
    - `nx: float`
    - `ny: float`
    - `nz: float`
    - `mach: float`
    - `qbar: float`
    - `ps: float`

### NelderMeadResult
Represents the result of a Nelder-Mead optimization.

#### Properties
- `x: List[float]`
- `fval: float`
- `iter: int`
- `fun_evals: int`

### FlightCondition
Represents the flight condition of a plane.

#### Methods
- `__init__(value: int) -> None`
- Properties:
    - `value: int`
- Static Methods:
    - `wings_level() -> FlightCondition`
    - `turning() -> FlightCondition`
    - `pull_up() -> FlightCondition`
    - `roll() -> FlightCondition`

### TrimInit
Represents the initial trim settings of a plane.

#### Methods
- `__init__(control: Control, alpha: float) -> None`
- Properties:
    - `control: Control`
    - `alpha: float`
- Setters:
    - `control(control: Control) -> None`
    - `alpha(value: float) -> None`

### TrimTarget
Represents the target trim settings of a plane.

#### Methods
- `__init__(altitude: float, velocity: float, npos: Optional[float] = None, epos: Optional[float] = None) -> None`
- Properties:
    - `altitude: float`
    - `velocity: float`
    - `npos: Optional[float]`
    - `epos: Optional[float]`
- Setters:
    - `altitude(value: float) -> None`
    - `velocity(value: float) -> None`
    - `npos(value: float) -> None`
    - `epos(value: float) -> None`

### TrimOutput
Represents the output of a trim calculation.

#### Methods
- `__init__(state: State, control: Control, state_extend: StateExtend, nelder_mead_result: NelderMeadResult) -> None`
- Properties:
    - `state: State`
    - `control: Control`
    - `state_extend: StateExtend`
    - `nelder_mead_result: NelderMeadResult`
- `to_core_init() -> CoreInit`

### NelderMeadOptions
Represents the options for a Nelder-Mead optimization.

#### Methods
- `__init__(max_fun_evals: int, max_iter: int, tol_fun: float, tol_x: float) -> None`
- Properties:
    - `max_fun_evals: int`
    - `max_iter: int`
    - `tol_fun: float`
    - `tol_x: float`
- Setters:
    - `max_fun_evals(value: int) -> None`
    - `max_iter(value: int) -> None`
    - `tol_fun(value: float) -> None`
    - `tol_x(value: float) -> None`

### CoreOutput
Represents the core output of a plane.

#### Properties
- `state: State`
- `control: Control`
- `state_extend: StateExtend`

### CoreInit
Represents the core initialization of a plane.

#### Methods
- `__init__(state: State, control: Control) -> None`
- Properties:
    - `state: State`
    - `control: Control`
- Setters:
    - `state(value: State) -> None`
    - `control(value: Control) -> None`

### PlaneBlock
Represents a block of a plane.

#### Methods
- `__init__(id: str, model: AerodynamicModel, init: CoreInit, deflection: List[float], ctrl_limit: ControlLimit) -> None`
- `update(control: Control, t: float) -> CoreOutput`
- `reset() -> None`
- Properties:
    - `state: CoreOutput`
- `delete_model() -> None`

## Functions

### trim
Calculates the trim settings for a plane.

#### Parameters
- `model: AerodynamicModel`
- `trim_target: TrimTarget`
- `ctrl_limit: ControlLimit`
- `trim_init: Optional[TrimInit] = None`
- `flight_condition: Optional[FlightCondition] = None`
- `optim_options: Optional[NelderMeadOptions] = None`

#### Returns
- `TrimOutput`