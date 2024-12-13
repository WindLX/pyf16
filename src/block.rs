use crate::components::flight::{get_lef, Atmos};
use crate::model::{Control, ControlLimit, CoreOutput, MechanicalModelInput, State, StateExtend};
use crate::plugin::{AerodynamicModel, AsPlugin};
use crate::solver::{ODESolver, VectorODESolver};
use crate::utils::{error::FatalCoreError, Vector};
use crate::{
    components::{
        flight::{disturbance, MechanicalModel},
        group::Actuator,
    },
    model::CoreInit,
};
use log::{debug, trace};
use std::sync::Arc;

pub(crate) struct ControllerBlock<S: ODESolver> {
    actuators: Vec<Actuator<S>>,
    deflection: Vec<f64>,
}

impl<S> ControllerBlock<S>
where
    S: ODESolver,
{
    pub fn new(
        solver: Arc<S>,
        control_init: impl Into<Control>,
        deflection: &[f64; 3],
        control_limit: ControlLimit,
    ) -> Self {
        let control_init: Control = control_init.into();

        trace!(
            "create control block with init: {:?}, deflections: {:?}, ctrl_limit: {:?}",
            control_init,
            deflection,
            control_limit
        );

        let thrust_ac = Actuator::new(
            solver.clone(),
            control_init.thrust,
            control_limit.thrust_cmd_limit_top,
            control_limit.thrust_cmd_limit_bottom,
            control_limit.thrust_rate_limit,
            1.0,
        );
        let elevator_ac = Actuator::new(
            solver.clone(),
            control_init.elevator,
            control_limit.ele_cmd_limit_top,
            control_limit.ele_cmd_limit_bottom,
            control_limit.ele_rate_limit,
            20.2,
        );
        let aileron_ac = Actuator::new(
            solver.clone(),
            control_init.aileron,
            control_limit.ail_cmd_limit_top,
            control_limit.ail_cmd_limit_bottom,
            control_limit.ail_rate_limit,
            20.2,
        );
        let rudder_ac = Actuator::new(
            solver.clone(),
            control_init.rudder,
            control_limit.rud_cmd_limit_top,
            control_limit.rud_cmd_limit_bottom,
            control_limit.rud_rate_limit,
            20.2,
        );
        ControllerBlock {
            actuators: vec![thrust_ac, elevator_ac, aileron_ac, rudder_ac],
            deflection: deflection.to_vec(),
        }
    }

    pub fn update(&mut self, control_input: impl Into<Control>, t: f64) -> Control {
        let mut control_input: Control = control_input.into();

        trace!(
            "update control block with control_input: {:?}, t: {}",
            control_input,
            t
        );

        control_input.thrust = self.actuators[0].update(control_input[0], t);
        for i in 0..4 {
            if i < 3 {
                if self.deflection[i].abs() < 1e-10 {
                    control_input[i + 1] += disturbance(self.deflection[i], t);
                }
            }
            // if control_input[i] < 1e-10 {
            //     let last = self.actuators[i].last();
            //     control_input[i] = self.actuators[i].update(last, t)
            // } else {
            control_input[i] = self.actuators[i].update(control_input[i], t);
            // }
        }
        trace!("correctional control input: \n{}", control_input);
        control_input
    }

    pub fn state(&self) -> Control {
        Control::from([
            self.actuators[0].state(),
            self.actuators[1].state(),
            self.actuators[2].state(),
            self.actuators[3].state(),
        ])
    }

    pub fn reset(&mut self, control: Control) {
        self.actuators[0].reset(control.thrust);
        self.actuators[1].reset(control.elevator);
        self.actuators[2].reset(control.aileron);
        self.actuators[3].reset(control.rudder);
    }
}

pub(crate) struct LeadingEdgeFlapBlock<S: ODESolver> {
    solver: Arc<S>,
    actuator: Actuator<S>,
    // integrator: Integrator,
    // feedback: f64,
    state: f64,
}

impl<S> LeadingEdgeFlapBlock<S>
where
    S: ODESolver,
{
    pub fn new(solver: Arc<S>, altitude: f64, velocity: f64, alpha: f64) -> Self {
        let d_lef = get_lef(altitude, velocity, alpha);

        trace!("LEFBlock: alpha_init: {}, d_lef: {}", alpha, d_lef);
        let actuator = Actuator::new(solver.clone(), d_lef, 25.0, 0.0, 25.0, 1.0 / 0.136);
        // let integrator = Integrator::new(-alpha.to_degrees());

        LeadingEdgeFlapBlock {
            solver: solver.clone(),
            actuator,
            // integrator,
            // feedback: 0.0,
            state: -alpha.to_degrees(),
        }
    }

    pub fn update(&mut self, altitude: f64, velocity: f64, alpha: f64, t: f64) -> f64 {
        trace!(
            "LEFBlock: alpha: {}, altitude: {}, velocity: {}",
            alpha,
            altitude,
            velocity
        );
        let atmos = Atmos::atmos(altitude, velocity);
        let r_1 = atmos.qbar / atmos.ps * 9.05;
        let alpha = alpha.to_degrees();
        // let r_2 = (alpha - self.feedback) * 7.25;

        let dynamics = move |_t: f64, state: f64, alpha: f64| -> f64 {
            let feedback = state + 2.0 * alpha;
            let r_2 = (alpha - feedback) * 7.25;
            r_2
        };
        let r_3 = self.solver.solve(&dynamics, t, self.state, alpha);
        self.state = r_3;
        // let r_3 = self.integrator.integrate(r_2, t);

        let r_4 = r_3 + 2.0 * alpha;
        // self.feedback = r_4;
        let r_5 = r_4 * 1.38;
        let r_6 = self.actuator.update(1.45 + r_5 - r_1, t);

        trace!("LEFBlock: lef: {}", r_6);
        r_6
    }

    pub fn state(&self) -> [f64; 2] {
        [self.actuator.state(), self.state]
    }

    pub fn reset(&mut self, altitude: f64, velocity: f64, alpha: f64) {
        let d_lef = get_lef(altitude, velocity, alpha);
        self.actuator.reset(d_lef);
        self.state = -alpha.to_degrees();
        // self.integrator.reset();
        // self.feedback = 0.0;
    }
}

pub struct PlaneBlock<S: ODESolver + VectorODESolver> {
    start_time: Option<f64>,
    control: ControllerBlock<S>,
    lef: LeadingEdgeFlapBlock<S>,
    // integrator: VectorIntegrator,
    solver: Arc<S>,
    plane: Arc<MechanicalModel>,
    extend: Option<StateExtend>,
    alpha_limit_top: f64,
    alpha_limit_bottom: f64,
    beta_limit_top: f64,
    beta_limit_bottom: f64,
    state: Vector,
    state_dot: Vector,
}

impl<S> PlaneBlock<S>
where
    S: ODESolver + VectorODESolver,
{
    pub fn new(
        solver: Arc<S>,
        model: &AerodynamicModel,
        init: &CoreInit,
        deflection: &[f64; 3],
        ctrl_limit: ControlLimit,
    ) -> Result<Self, FatalCoreError> {
        trace!(
            "create plane block with model: {}, init: {:?}, deflection: {:?}, ctrl_limit: {:?}",
            model.info().name,
            init,
            deflection,
            ctrl_limit
        );
        let control = ControllerBlock::new(solver.clone(), init.control, deflection, ctrl_limit);
        let lef = LeadingEdgeFlapBlock::new(
            solver.clone(),
            init.state.altitude,
            init.state.velocity,
            init.state.alpha,
        );
        // let integrator = VectorIntegrator::new(Into::<Vector>::into(init.state));
        let mut plane = MechanicalModel::new(model)?;
        plane.init()?;

        let init_state = Into::<Vector>::into(init.state);
        let init_state_dim = init_state.dim();
        Ok(PlaneBlock {
            control,
            lef,
            // integrator,
            solver: solver.clone(),
            plane: Arc::new(plane),
            extend: None,
            alpha_limit_top: ctrl_limit.alpha_limit_top,
            alpha_limit_bottom: ctrl_limit.alpha_limit_bottom,
            beta_limit_top: ctrl_limit.beta_limit_top,
            beta_limit_bottom: ctrl_limit.beta_limit_bottom,
            start_time: None,
            state: init_state,
            state_dot: Vector::zero(init_state_dim),
        })
    }

    pub fn update(
        &mut self,
        control: impl Into<Control>,
        t: f64,
    ) -> Result<CoreOutput, FatalCoreError> {
        trace!("update time: {}", t);

        if self.start_time.is_none() {
            self.start_time = Some(t);
            debug!("start time {}", t);
        }
        let t = (t - self.start_time.unwrap()).max(1e-3);
        // let state = &mut self.integrator.past();
        let mut state = self.state.clone();
        let control = self.control.update(control, t);

        let altitude = state[2];
        let velocity = state[6];
        let alpha = state[7];
        let d_lef = self.lef.update(altitude, velocity, alpha, t);

        state.data[7] = state[7].clamp(
            self.alpha_limit_bottom.to_radians(),
            self.alpha_limit_top.to_radians(),
        );

        state.data[8] = state[8].clamp(
            self.beta_limit_bottom.to_radians(),
            self.beta_limit_top.to_radians(),
        );

        let model_output = self.plane.step(&MechanicalModelInput::new(
            state.data.clone(),
            control,
            d_lef,
        ))?;

        trace!("model_output:\n{}", model_output);

        self.state_dot = model_output.state_dot.into();

        // let state = self
        //     .integrator
        //     .derivative_add(Into::<Vector>::into(model_output.state_dot), t);

        let plane = self.plane.clone();
        let dynamics = move |_t: f64, state: &Vector, _input: &Vector| -> Vector {
            let model_output = plane
                .step(&MechanicalModelInput::new(
                    state.data.clone(),
                    control,
                    d_lef,
                ))
                .unwrap();
            model_output.state_dot.into()
        };

        let state = VectorODESolver::solve(&*self.solver, &dynamics, t, &state, &Vector::zero(0));
        self.state = state.clone();

        let state = state.data;
        if state.iter().any(|x| x.is_nan()) {
            return Err(FatalCoreError::Nan);
        }

        let control = self.control.state();
        if Into::<Vec<f64>>::into(control).iter().any(|x| x.is_nan()) {
            return Err(FatalCoreError::Nan);
        }

        let extend = model_output.state_extend;
        if Into::<Vec<f64>>::into(extend).iter().any(|x| x.is_nan()) {
            return Err(FatalCoreError::Nan);
        }

        self.extend = Some(StateExtend::from(extend));

        let block_output = CoreOutput::new(
            State::from(state),
            Control::from(control),
            self.extend.unwrap(),
        );
        trace!("block_output:\n{}", block_output);

        Ok(block_output)
    }

    pub fn reset(&mut self, init: &CoreInit) {
        self.control.reset(init.control);

        let altitude = init.state.altitude;
        let velocity = init.state.velocity;
        let alpha = init.state.alpha;
        self.lef.reset(altitude, velocity, alpha);

        // self.integrator.reset();
        self.state = init.state.into();
    }

    pub fn state(&self) -> CoreOutput {
        // let state = &self.integrator.past();
        let state = self.state.clone();
        let control = self.control.state();

        CoreOutput::new(
            State::from(state.clone()),
            Control::from(control),
            self.extend.unwrap_or_default(),
        )
    }

    pub fn state_dot(&self) -> State {
        State::from(self.state_dot.clone())
    }

    pub fn delete_model(&self) {
        self.plane.delete();
    }
}

unsafe impl<S> Sync for PlaneBlock<S> where S: ODESolver + VectorODESolver {}

#[cfg(test)]
mod block_tests {
    use super::*;
    use crate::model::ControlLimit;
    use crate::optimizer::nelder_mead::NelderMeadOptions;
    use crate::plugin::{AerodynamicModel, AsPlugin};
    use crate::trim::{trim, TrimOutput, TrimTarget};
    use crate::utils::dev::test_logger_init;
    use crate::{
        components::{
            basic::step,
            flight::{multi_to_deg, MechanicalModel},
        },
        solver,
    };
    use csv::Writer;
    use log::{debug, trace};
    use std::cell::RefCell;
    use std::fs::File;
    use std::path::Path;
    use std::rc::Rc;
    use std::sync::Arc;
    use std::time::{Duration, Instant, SystemTime};

    const CL: ControlLimit = ControlLimit {
        thrust_cmd_limit_top: 19000.0,
        thrust_cmd_limit_bottom: 1000.0,
        thrust_rate_limit: 10000.0,
        ele_cmd_limit_top: 25.0,
        ele_cmd_limit_bottom: -25.0,
        ele_rate_limit: 60.0,
        ail_cmd_limit_top: 21.5,
        ail_cmd_limit_bottom: -21.5,
        ail_rate_limit: 80.0,
        rud_cmd_limit_top: 30.0,
        rud_cmd_limit_bottom: -30.0,
        rud_rate_limit: 120.0,
        alpha_limit_top: 45.0,
        alpha_limit_bottom: -20.0,
        beta_limit_top: 30.0,
        beta_limit_bottom: -30.0,
    };

    fn test_core_init() -> (AerodynamicModel, TrimOutput) {
        test_logger_init();
        let model = AerodynamicModel::new("./models/f16_model");
        assert!(matches!(model, Ok(_)));

        let model = model.unwrap();
        let res = model.plugin().install(&["./models/f16_model/data"]);
        assert!(matches!(res, Ok(Ok(_))));

        let plane = Rc::new(RefCell::new(MechanicalModel::new(&model).unwrap()));

        let trim_target = TrimTarget::new(15000.0, 500.0, None, None);
        let trim_init = None;
        let nm_options = Some(NelderMeadOptions {
            max_fun_evals: 50000,
            max_iter: 10000,
            tol_fun: 1e-6,
            tol_x: 1e-6,
        });

        (
            model,
            trim(plane.clone(), trim_target, trim_init, CL, None, nm_options).unwrap(),
        )
    }

    fn test_core_fin(model: AerodynamicModel) {
        let res = model.plugin().uninstall();
        assert!(matches!(res, Ok(Ok(_))));
    }

    #[test]
    fn test_control() {
        let (model, result) = test_core_init();
        debug!("{:#?}", result.control);

        let path = Path::new("output_control.csv");
        let file = File::create(&path).unwrap();
        let mut writer = Writer::from_writer(file);
        let start_time = SystemTime::now();
        writer
            .write_record(&["time(s)", "thrust", "ele", "ail", "rud"])
            .unwrap();

        let control_init = result.control;

        let solver = solver::rk::RK4Solver::new(0.01);
        let solver = Arc::new(solver);

        let mut control = ControllerBlock::new(solver.clone(), control_init, &[0.0, 0.0, 0.0], CL);

        loop {
            let current_time = SystemTime::now();
            let delta_time = current_time.duration_since(start_time).unwrap();

            let result = control.update(
                [
                    step(
                        control_init.thrust,
                        2.0 * control_init.thrust * 2.0,
                        1.0,
                        delta_time.as_secs_f64(),
                    ),
                    control_init.elevator,
                    control_init.aileron,
                    control_init.rudder,
                ],
                delta_time.as_secs_f64(),
            );
            trace!("time: {:?} \n{:?}\n", delta_time, result);

            let data: Vec<String> = Into::<Vec<f64>>::into(result)
                .iter()
                .map(|d| d.to_string())
                .collect();
            let mut record = vec![delta_time.as_secs_f32().to_string()];
            record.extend(data);
            writer.write_record(&record).unwrap();
            writer.flush().unwrap();
            if delta_time > Duration::from_secs_f32(10.0) {
                break;
            }
        }

        writer.flush().unwrap();

        test_core_fin(model)
    }

    #[test]
    fn test_plane() {
        let (model, result) = test_core_init();
        // set_time_scale(5.0).unwrap();

        let solver = solver::rk::RK4Solver::new(0.01);
        let solver = Arc::new(solver);

        let control: [f64; 4] = result.control.into();
        let f16_block =
            PlaneBlock::new(solver.clone(), &model, &result.into(), &[0.0, 0.0, 0.0], CL);
        let mut f16_block = f16_block.unwrap();

        let path = Path::new("output.csv");
        let file = File::create(&path).unwrap();
        let mut writer = Writer::from_writer(file);
        writer
            .write_record(&[
                "time(s)",
                "npos(ft)",
                "epos(ft)",
                "altitude(ft)",
                "phi(degree)",
                "theta(degree)",
                "psi(degree)",
                "velocity(ft/s)",
                "alpha(degree)",
                "beta(degree)",
                "p(degree/s)",
                "q(degree/s)",
                "r(degree/s)",
                "nx(g)",
                "ny(g)",
                "nz(g)",
                "mach",
                "qbar(lb/ft ft)",
                "ps(lb/ft ft)",
            ])
            .unwrap();

        let start_time = Instant::now();
        let mut next_write_time = start_time + Duration::from_millis(100);

        loop {
            let current_time = Instant::now();
            let delta_time = current_time.duration_since(start_time);
            let result = f16_block.update(control, delta_time.as_secs_f64()).unwrap();
            if current_time >= next_write_time {
                let state = multi_to_deg(&result.state.into());

                trace!("time: {:?} \n{:?}\n", delta_time, state);

                let mut state: Vec<f64> = state.data.clone();
                let extend: [f64; 6] = result.state_extend.into();
                state.extend_from_slice(&extend);

                let data: Vec<String> = state.iter().map(|d| d.to_string()).collect();
                let mut record = vec![delta_time.as_secs_f32().to_string()];
                record.extend(data);

                writer.write_record(&record).unwrap();
                writer.flush().unwrap();

                next_write_time += Duration::from_millis(100);
            }

            if delta_time >= Duration::from_secs_f32(15.0) {
                break;
            }
        }

        writer.flush().unwrap();

        test_core_fin(model)
    }
}
