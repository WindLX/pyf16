use crate::{
    algorithm::nelder_mead::{
        NelderMeadOptions as NelderMeadOptionsBase, NelderMeadResult as NelderMeadResultBase,
    },
    block::PlaneBlock as PlaneBlockBase,
    components::flight::MechanicalModel,
    model::{
        Control as ControlBase, ControlLimit as ControlLimitBase, CoreInit as CoreInitBase,
        CoreOutput as CoreOutputBase, FlightCondition as FlightConditionBase,
        PlaneConstants as PlaneConstantsBase, State as StateBase, StateExtend as StateExtendBase,
    },
    plugin::{AerodynamicModel as AerodynamicModelBase, AsPlugin},
    trim::{
        trim as trim_base, TrimInit as TrimInitBase, TrimOutput as TrimOutputBase,
        TrimTarget as TrimTargetBase,
    },
};
use log::error;
use pyo3::{exceptions::PyValueError, prelude::*};
use std::{cell::RefCell, rc::Rc};

#[pyclass]
struct PlaneConstants(PlaneConstantsBase);

#[pymethods]
impl PlaneConstants {
    #[new]
    fn new(
        m: f64,
        b: f64,
        s: f64,
        c_bar: f64,
        x_cg_r: f64,
        x_cg: f64,
        h_eng: f64,
        j_y: f64,
        j_xz: f64,
        j_z: f64,
        j_x: f64,
    ) -> Self {
        Self(PlaneConstantsBase::new(
            m, b, s, c_bar, x_cg_r, x_cg, h_eng, j_y, j_xz, j_z, j_x,
        ))
    }

    #[getter]
    fn m(&self) -> f64 {
        self.0.m
    }

    #[getter]
    fn b(&self) -> f64 {
        self.0.b
    }

    #[getter]
    fn s(&self) -> f64 {
        self.0.s
    }

    #[getter]
    fn c_bar(&self) -> f64 {
        self.0.c_bar
    }

    #[getter]
    fn x_cg_r(&self) -> f64 {
        self.0.x_cg_r
    }

    #[getter]
    fn x_cg(&self) -> f64 {
        self.0.x_cg
    }

    #[getter]
    fn h_eng(&self) -> f64 {
        self.0.h_eng
    }

    #[getter]
    fn j_y(&self) -> f64 {
        self.0.j_y
    }

    #[getter]
    fn j_xz(&self) -> f64 {
        self.0.j_xz
    }

    #[getter]
    fn j_z(&self) -> f64 {
        self.0.j_z
    }

    #[getter]
    fn j_x(&self) -> f64 {
        self.0.j_x
    }
}

#[pyclass]
struct ControlLimit(ControlLimitBase);

#[pymethods]
impl ControlLimit {
    #[new]
    fn new(
        thrust_cmd_limit_top: f64,
        thrust_cmd_limit_bottom: f64,
        thrust_rate_limit: f64,
        ele_cmd_limit_top: f64,
        ele_cmd_limit_bottom: f64,
        ele_rate_limit: f64,
        ail_cmd_limit_top: f64,
        ail_cmd_limit_bottom: f64,
        ail_rate_limit: f64,
        rud_cmd_limit_top: f64,
        rud_cmd_limit_bottom: f64,
        rud_rate_limit: f64,
        alpha_limit_top: f64,
        alpha_limit_bottom: f64,
        beta_limit_top: f64,
        beta_limit_bottom: f64,
    ) -> Self {
        Self(ControlLimitBase::new(
            thrust_cmd_limit_top,
            thrust_cmd_limit_bottom,
            thrust_rate_limit,
            ele_cmd_limit_top,
            ele_cmd_limit_bottom,
            ele_rate_limit,
            ail_cmd_limit_top,
            ail_cmd_limit_bottom,
            ail_rate_limit,
            rud_cmd_limit_top,
            rud_cmd_limit_bottom,
            rud_rate_limit,
            alpha_limit_top,
            alpha_limit_bottom,
            beta_limit_top,
            beta_limit_bottom,
        ))
    }

    #[getter]
    fn thrust_cmd_limit_top(&self) -> f64 {
        self.0.thrust_cmd_limit_top
    }

    #[getter]
    fn thrust_cmd_limit_bottom(&self) -> f64 {
        self.0.thrust_cmd_limit_bottom
    }

    #[getter]
    fn thrust_rate_limit(&self) -> f64 {
        self.0.thrust_rate_limit
    }

    #[getter]
    fn ele_cmd_limit_top(&self) -> f64 {
        self.0.ele_cmd_limit_top
    }

    #[getter]
    fn ele_cmd_limit_bottom(&self) -> f64 {
        self.0.ele_cmd_limit_bottom
    }

    #[getter]
    fn ele_rate_limit(&self) -> f64 {
        self.0.ele_rate_limit
    }

    #[getter]
    fn ail_cmd_limit_top(&self) -> f64 {
        self.0.ail_cmd_limit_top
    }

    #[getter]
    fn ail_cmd_limit_bottom(&self) -> f64 {
        self.0.ail_cmd_limit_bottom
    }

    #[getter]
    fn ail_rate_limit(&self) -> f64 {
        self.0.ail_rate_limit
    }

    #[getter]
    fn rud_cmd_limit_top(&self) -> f64 {
        self.0.rud_cmd_limit_top
    }

    #[getter]
    fn rud_cmd_limit_bottom(&self) -> f64 {
        self.0.rud_cmd_limit_bottom
    }

    #[getter]
    fn rud_rate_limit(&self) -> f64 {
        self.0.rud_rate_limit
    }

    #[getter]
    fn alpha_limit_top(&self) -> f64 {
        self.0.alpha_limit_top
    }

    #[getter]
    fn alpha_limit_bottom(&self) -> f64 {
        self.0.alpha_limit_bottom
    }

    #[getter]
    fn beta_limit_top(&self) -> f64 {
        self.0.beta_limit_top
    }

    #[getter]
    fn beta_limit_bottom(&self) -> f64 {
        self.0.beta_limit_bottom
    }
}

#[pyclass]
struct AerodynamicModel(AerodynamicModelBase);

#[pymethods]
impl AerodynamicModel {
    // Create a new instance of AerodynamicModel
    #[new]
    fn new(path: String) -> PyResult<Self> {
        let base = AerodynamicModelBase::new(path);
        match base {
            Ok(base) => Ok(Self(base)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    // Initialize the plugin
    fn install(&self, path: String) -> PyResult<()> {
        match self.0.plugin().install(&[path]) {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    // Uninstall the plugin
    fn uninstall(&self) -> PyResult<()> {
        match self.0.plugin().uninstall() {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    fn load_constants(&self) -> PyResult<PlaneConstants> {
        match self.0.load_constants() {
            Ok(c) => Ok(PlaneConstants(c)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    fn load_ctrl_limits(&self) -> PyResult<ControlLimit> {
        match self.0.load_ctrl_limits() {
            Ok(c) => Ok(ControlLimit(c)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }
}

#[pyclass]
struct State(StateBase);

#[pymethods]
impl State {
    fn to_list(&self) -> Vec<f64> {
        self.0.into()
    }

    #[getter]
    fn npos(&self) -> f64 {
        self.0.npos
    }

    #[getter]
    fn epos(&self) -> f64 {
        self.0.epos
    }

    #[getter]
    fn altitude(&self) -> f64 {
        self.0.altitude
    }

    #[getter]
    fn phi(&self) -> f64 {
        self.0.phi
    }

    #[getter]
    fn theta(&self) -> f64 {
        self.0.theta
    }

    #[getter]
    fn psi(&self) -> f64 {
        self.0.psi
    }

    #[getter]
    fn velocity(&self) -> f64 {
        self.0.velocity
    }

    #[getter]
    fn alpha(&self) -> f64 {
        self.0.alpha
    }

    #[getter]
    fn beta(&self) -> f64 {
        self.0.beta
    }

    #[getter]
    fn p(&self) -> f64 {
        self.0.p
    }

    #[getter]
    fn q(&self) -> f64 {
        self.0.q
    }

    #[getter]
    fn r(&self) -> f64 {
        self.0.r
    }
}

#[pyclass]
struct Control(ControlBase);

#[pymethods]
impl Control {
    #[new]
    fn new(thrust: f64, elevator: f64, aileron: f64, rudder: f64) -> Self {
        Self(ControlBase {
            thrust,
            elevator,
            aileron,
            rudder,
        })
    }

    fn to_list(&self) -> Vec<f64> {
        self.0.into()
    }

    #[getter]
    fn thrust(&self) -> f64 {
        self.0.thrust
    }

    #[getter]
    fn elevator(&self) -> f64 {
        self.0.elevator
    }

    #[getter]
    fn aileron(&self) -> f64 {
        self.0.aileron
    }

    #[getter]
    fn rudder(&self) -> f64 {
        self.0.rudder
    }

    #[setter]
    fn set_thrust(&mut self, value: f64) {
        self.0.thrust = value;
    }

    #[setter]
    fn set_elevator(&mut self, value: f64) {
        self.0.elevator = value;
    }

    #[setter]
    fn set_aileron(&mut self, value: f64) {
        self.0.aileron = value;
    }

    #[setter]
    fn set_rudder(&mut self, value: f64) {
        self.0.rudder = value;
    }
}

#[pyclass]
struct StateExtend(StateExtendBase);

#[pymethods]
impl StateExtend {
    #[new]
    fn new(nx: f64, ny: f64, nz: f64, mach: f64, qbar: f64, ps: f64) -> Self {
        Self(StateExtendBase {
            nx,
            ny,
            nz,
            mach,
            qbar,
            ps,
        })
    }

    fn to_list(&self) -> Vec<f64> {
        self.0.into()
    }

    #[getter]
    fn nx(&self) -> f64 {
        self.0.nx
    }

    #[getter]
    fn ny(&self) -> f64 {
        self.0.ny
    }

    #[getter]
    fn nz(&self) -> f64 {
        self.0.nz
    }

    #[getter]
    fn mach(&self) -> f64 {
        self.0.mach
    }

    #[getter]
    fn qbar(&self) -> f64 {
        self.0.qbar
    }

    #[getter]
    fn ps(&self) -> f64 {
        self.0.ps
    }
}


#[pyclass]
struct NelderMeadResult(NelderMeadResultBase);

#[pymethods]
impl NelderMeadResult {
    #[getter]
    fn x(&self) -> Vec<f64> {
        self.0.x.data.clone()
    }

    #[getter]
    fn fval(&self) -> f64 {
        self.0.fval
    }

    #[getter]
    fn iter(&self) -> usize {
        self.0.iter
    }

    #[getter]
    fn fun_evals(&self) -> usize {
        self.0.fun_evals
    }
}

#[pyclass]
struct FlightCondition(FlightConditionBase);

#[pymethods]
impl FlightCondition {
    #[new]
    fn new(value: i32) -> Self {
        match value {
            0 => Self(FlightConditionBase::WingsLevel),
            1 => Self(FlightConditionBase::Turning),
            2 => Self(FlightConditionBase::PullUp),
            3 => Self(FlightConditionBase::Roll),
            _ => panic!("Invalid value for FlightCondition"),
        }
    }

    #[getter]
    fn value(&self) -> i32 {
        match self.0 {
            FlightConditionBase::WingsLevel => 0,
            FlightConditionBase::Turning => 1,
            FlightConditionBase::PullUp => 2,
            FlightConditionBase::Roll => 3,
        }
    }

    #[staticmethod]
    fn wings_level() -> Self {
        Self(FlightConditionBase::WingsLevel)
    }

    #[staticmethod]
    fn turning() -> Self {
        Self(FlightConditionBase::Turning)
    }

    #[staticmethod]
    fn pull_up() -> Self {
        Self(FlightConditionBase::PullUp)
    }

    #[staticmethod]
    fn roll() -> Self {
        Self(FlightConditionBase::Roll)
    }
}

#[pyclass]
struct TrimInit(TrimInitBase);

#[pymethods]
impl TrimInit {
    #[new]
    fn new(control: &Control, alpha: f64) -> Self {
        Self(TrimInitBase {
            control: control.0.clone(),
            alpha,
        })
    }

    #[getter]
    fn control(&self) -> Control {
        Control(self.0.control.clone())
    }

    #[getter]
    fn alpha(&self) -> f64 {
        self.0.alpha
    }

    #[setter]
    fn set_control(&mut self, control: &Control) {
        self.0.control = control.0.clone();
    }

    #[setter]
    fn set_alpha(&mut self, value: f64) {
        self.0.alpha = value;
    }
}

#[pyclass]
struct TrimTarget(TrimTargetBase);

#[pymethods]
impl TrimTarget {
    #[new]
    #[pyo3(signature = (altitude, velocity, npos=None, epos=None))]
    fn new(altitude: f64, velocity: f64, npos: Option<f64>, epos: Option<f64>) -> Self {
        Self(TrimTargetBase {
            altitude,
            velocity,
            npos: npos.unwrap_or_default(),
            epos: epos.unwrap_or_default(),
        })
    }

    #[getter]
    fn altitude(&self) -> f64 {
        self.0.altitude
    }

    #[getter]
    fn velocity(&self) -> f64 {
        self.0.velocity
    }

    #[getter]
    fn npos(&self) -> f64 {
        self.0.npos
    }

    #[getter]
    fn epos(&self) -> f64 {
        self.0.epos
    }

    #[setter]
    fn set_altitude(&mut self, value: f64) {
        self.0.altitude = value;
    }

    #[setter]
    fn set_velocity(&mut self, value: f64) {
        self.0.velocity = value;
    }

    #[setter]
    fn set_npos(&mut self, value: f64) {
        self.0.npos = value;
    }

    #[setter]
    fn set_epos(&mut self, value: f64) {
        self.0.epos = value;
    }
}

#[pyclass]
struct TrimOutput(TrimOutputBase);

#[pymethods]
impl TrimOutput {
    #[new]
    fn new(
        state: &State,
        control: &Control,
        state_extend: &StateExtend,
        nelder_mead_result: &NelderMeadResult,
    ) -> Self {
        Self(TrimOutputBase {
            state: state.0.clone(),
            control: control.0.clone(),
            state_extend: state_extend.0.clone(),
            nelder_mead_result: nelder_mead_result.0.clone(),
        })
    }

    #[getter]
    fn state(&self) -> State {
        State(self.0.state.clone())
    }

    #[getter]
    fn control(&self) -> Control {
        Control(self.0.control.clone())
    }

    #[getter]
    fn state_extend(&self) -> StateExtend {
        StateExtend(self.0.state_extend.clone())
    }

    #[getter]
    fn nelder_mead_result(&self) -> NelderMeadResult {
        NelderMeadResult(self.0.nelder_mead_result.clone())
    }

    fn to_core_init(&self) -> CoreInit {
        let core_init = self.0.clone().into();
        CoreInit(core_init)
    }
}

#[pyclass]
struct NelderMeadOptions(NelderMeadOptionsBase);

#[pymethods]
impl NelderMeadOptions {
    #[new]
    fn new(max_fun_evals: usize, max_iter: usize, tol_fun: f64, tol_x: f64) -> Self {
        Self(NelderMeadOptionsBase {
            max_fun_evals,
            max_iter,
            tol_fun,
            tol_x,
        })
    }

    #[getter]
    fn max_fun_evals(&self) -> usize {
        self.0.max_fun_evals
    }

    #[getter]
    fn max_iter(&self) -> usize {
        self.0.max_iter
    }

    #[getter]
    fn tol_fun(&self) -> f64 {
        self.0.tol_fun
    }

    #[getter]
    fn tol_x(&self) -> f64 {
        self.0.tol_x
    }

    #[setter]
    fn set_max_fun_evals(&mut self, value: usize) {
        self.0.max_fun_evals = value;
    }

    #[setter]
    fn set_max_iter(&mut self, value: usize) {
        self.0.max_iter = value;
    }

    #[setter]
    fn set_tol_fun(&mut self, value: f64) {
        self.0.tol_fun = value;
    }

    #[setter]
    fn set_tol_x(&mut self, value: f64) {
        self.0.tol_x = value;
    }
}

#[pyclass]
struct CoreOutput(CoreOutputBase);

#[pymethods]
impl CoreOutput {
    #[getter]
    fn state(&self) -> State {
        State(self.0.state.clone())
    }

    #[getter]
    fn control(&self) -> Control {
        Control(self.0.control.clone())
    }

    #[getter]
    fn state_extend(&self) -> StateExtend {
        StateExtend(self.0.state_extend.clone())
    }
}

#[pyclass]
struct CoreInit(CoreInitBase);

#[pymethods]
impl CoreInit {
    #[new]
    fn new(state: &State, control: &Control) -> Self {
        Self(CoreInitBase {
            state: state.0.clone(),
            control: control.0.clone(),
        })
    }

    #[getter]
    fn state(&self) -> State {
        State(self.0.state.clone())
    }

    #[getter]
    fn control(&self) -> Control {
        Control(self.0.control.clone())
    }

    #[setter]
    fn set_state(&mut self, state: &State) {
        self.0.state = state.0.clone();
    }

    #[setter]
    fn set_control(&mut self, control: &Control) {
        self.0.control = control.0.clone();
    }
}


#[pyfunction]
#[pyo3(signature = (model, trim_target, ctrl_limit, trim_init=None, flight_condition=None, optim_options=None))]
fn trim(
    model: &AerodynamicModel,
    trim_target: &TrimTarget,
    ctrl_limit: &ControlLimit,
    trim_init: Option<&TrimInit>,
    flight_condition: Option<&FlightCondition>,
    optim_options: Option<&NelderMeadOptions>,
) -> PyResult<TrimOutput> {
    match MechanicalModel::new(&model.0) {
        Ok(m) => {
            let model = Rc::new(RefCell::new(m));
            let res = trim_base(
                model.clone(),
                trim_target.0,
                trim_init.map(|x| x.0),
                ctrl_limit.0,
                flight_condition.map(|x| x.0),
                optim_options.map(|x| x.0),
            );
            match res {
                Ok(o) => Ok(TrimOutput(o)),
                Err(e) => {
                    error!("{}", e);
                    Err(PyValueError::new_err(e.to_string()))
                }
            }
        }
        Err(e) => {
            error!("{}", e);
            return Err(PyValueError::new_err(e.to_string()));
        }
    }
}

#[pyclass]
struct PlaneBlock(PlaneBlockBase);

#[pymethods]
impl PlaneBlock {
    #[new]
    fn new(
        id: String,
        model: &AerodynamicModel,
        init: &CoreInit,
        deflection: Vec<f64>,
        ctrl_limit: &ControlLimit,
    ) -> PyResult<Self> {
        if deflection.len() != 3 {
            return Err(PyValueError::new_err(
                "deflection must have exactly 3 elements",
            ));
        }
        let deflection_array: [f64; 3] = [deflection[0], deflection[1], deflection[2]];
        let plane = PlaneBlockBase::new(&id, &model.0, &init.0, &deflection_array, ctrl_limit.0);
        match plane {
            Ok(p) => Ok(Self(p)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    fn update(&mut self, control: &Control, t: f64) -> PyResult<CoreOutput> {
        match self.0.update(control.0, t) {
            Ok(o) => Ok(CoreOutput(o)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    fn reset(&mut self) {
        self.0.reset();
    }

    #[getter]
    fn state(&self) -> PyResult<CoreOutput> {
        match self.0.state() {
            Ok(o) => Ok(CoreOutput(o)),
            Err(e) => {
                error!("{}", e);
                Err(PyValueError::new_err(e.to_string()))
            }
        }
    }

    fn delete_model(&mut self) {
        self.0.delete_model();
    }
}

pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<AerodynamicModel>()?;
    m.add_class::<PlaneBlock>()?;
    m.add_class::<PlaneConstants>()?;
    m.add_class::<ControlLimit>()?;
    m.add_class::<Control>()?;
    m.add_class::<State>()?;
    m.add_class::<StateExtend>()?;
    m.add_class::<NelderMeadResult>()?;
    m.add_class::<FlightCondition>()?;
    m.add_class::<TrimInit>()?;
    m.add_class::<TrimTarget>()?;
    m.add_class::<TrimOutput>()?;
    m.add_class::<NelderMeadOptions>()?;
    m.add_class::<CoreOutput>()?;
    m.add_function(wrap_pyfunction!(trim, m)?)?;
    Ok(())
}
