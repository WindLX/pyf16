use crate::model::{Control, ControlLimit, CoreOutput, MechanicalModelInput, State, StateExtend};
use crate::plugin::{AerodynamicModel, AsPlugin};
use crate::utils::{error::FatalCoreError, Vector};
use crate::{
    components::{
        basic::VectorIntegrator,
        flight::{disturbance, MechanicalModel},
        group::Actuator,
    },
    model::CoreInit,
};
use log::{debug, trace};

pub(crate) struct ControllerBlock {
    actuators: Vec<Actuator>,
    deflection: Vec<f64>,
}

impl ControllerBlock {
    pub fn new(
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
            control_init.thrust,
            control_limit.thrust_cmd_limit_top,
            control_limit.thrust_cmd_limit_bottom,
            control_limit.thrust_rate_limit,
            1.0,
        );
        let elevator_ac = Actuator::new(
            control_init.elevator,
            control_limit.ele_cmd_limit_top,
            control_limit.ele_cmd_limit_bottom,
            control_limit.ele_rate_limit,
            20.2,
        );
        let aileron_ac = Actuator::new(
            control_init.aileron,
            control_limit.ail_cmd_limit_top,
            control_limit.ail_cmd_limit_bottom,
            control_limit.ail_rate_limit,
            20.2,
        );
        let rudder_ac = Actuator::new(
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
            control_input[i] = self.actuators[i].update(control_input[i], t)
            // }
        }
        trace!("correctional control input: \n{}", control_input);
        control_input
    }

    pub fn past(&self) -> Control {
        Control::from([
            self.actuators[0].past(),
            self.actuators[1].past(),
            self.actuators[2].past(),
            self.actuators[3].past(),
        ])
    }

    pub fn reset(&mut self) {
        for a in &mut self.actuators {
            a.reset()
        }
    }
}

pub struct PlaneBlock {
    start_time: Option<f64>,
    control: ControllerBlock,
    integrator: VectorIntegrator,
    plane: MechanicalModel,
    extend: Option<StateExtend>,
    alpha_limit_top: f64,
    alpha_limit_bottom: f64,
    beta_limit_top: f64,
    beta_limit_bottom: f64,
}

impl PlaneBlock {
    pub fn new(
        id: &str,
        model: &AerodynamicModel,
        init: &CoreInit,
        deflection: &[f64; 3],
        ctrl_limit: ControlLimit,
    ) -> Result<Self, FatalCoreError> {
        trace!(
            "create plane block with id: {}, model: {}, init: {:?}, deflection: {:?}, ctrl_limit: {:?}",
            id,
            model.info().name,
            init,
            deflection,
            ctrl_limit
        );
        let control = ControllerBlock::new(init.control, deflection, ctrl_limit);
        let integrator = VectorIntegrator::new(Into::<Vector>::into(init.state));
        let mut plane = MechanicalModel::new(model)?;
        plane.init(
            id,
            &MechanicalModelInput {
                state: init.state,
                control: init.control,
            },
        )?;
        Ok(PlaneBlock {
            control,
            integrator,
            plane,
            extend: None,
            alpha_limit_top: ctrl_limit.alpha_limit_top,
            alpha_limit_bottom: ctrl_limit.alpha_limit_bottom,
            beta_limit_top: ctrl_limit.beta_limit_top,
            beta_limit_bottom: ctrl_limit.beta_limit_bottom,
            start_time: None,
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
        let state = &mut self.integrator.past();
        let control = self.control.update(control, t);

        state.data[7] = state[7].clamp(
            self.alpha_limit_bottom.to_radians(),
            self.alpha_limit_top.to_radians(),
        );

        state.data[8] = state[8].clamp(
            self.beta_limit_bottom.to_radians(),
            self.beta_limit_top.to_radians(),
        );

        let model_output = self
            .plane
            .step(&MechanicalModelInput::new(state.data.clone(), control), t)?;

        trace!("model_output:\n{}", model_output);

        let state = self
            .integrator
            .derivative_add(Into::<Vector>::into(model_output.state_dot), t);

        let state = state.data;
        if state.iter().any(|x| x.is_nan()) {
            return Err(FatalCoreError::Nan);
        }

        let control = self.control.past();
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

    pub fn reset(&mut self) {
        self.control.reset();
        self.integrator.reset();
    }

    pub fn state(&self) -> Result<CoreOutput, FatalCoreError> {
        let state = &self.integrator.past();
        let control = self.control.past();

        Ok(CoreOutput::new(
            State::from(state.clone()),
            Control::from(control),
            self.extend.unwrap_or_default(),
        ))
    }

    pub fn delete_model(&mut self) {
        self.plane.delete()
    }
}

unsafe impl Sync for PlaneBlock {}

#[cfg(test)]
mod block_tests {
    use super::*;
    use crate::algorithm::nelder_mead::NelderMeadOptions;
    use crate::components::{
        basic::step,
        flight::{multi_to_deg, MechanicalModel},
    };
    use crate::model::ControlLimit;
    use crate::plugin::{AerodynamicModel, AsPlugin};
    use crate::trim::{trim, TrimOutput, TrimTarget};
    use crate::utils::logger::test_logger_init;
    use csv::Writer;
    use log::{debug, trace};
    use std::cell::RefCell;
    use std::fs::File;
    use std::path::Path;
    use std::rc::Rc;
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

        let mut control = ControllerBlock::new(control_init, &[0.0, 0.0, 0.0], CL);

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

        let control: [f64; 4] = result.control.into();
        let f16_block = PlaneBlock::new("123", &model, &result.into(), &[0.0, 0.0, 0.0], CL);
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
