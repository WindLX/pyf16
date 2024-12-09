use crate::solver::ODESolver;
use std::sync::Arc;
// use super::basic::Integrator;

#[derive(Clone)]
pub struct Actuator<S: ODESolver> {
    // integrator: Integrator,
    solver: Arc<S>,
    // feedback: f64,
    command_saturation_top: f64,
    command_saturation_bottom: f64,
    rate_saturation: f64,
    gain: f64,
    state: f64,
}

impl<S> Actuator<S>
where
    S: ODESolver,
{
    pub fn new(
        solver: Arc<S>,
        init: f64,
        command_saturation_top: f64,
        command_saturation_bottom: f64,
        rate_saturation: f64,
        gain: f64,
    ) -> Self {
        Self {
            // integrator: Integrator::new(init),
            solver,
            // feedback: 0.0,
            command_saturation_top,
            command_saturation_bottom,
            rate_saturation,
            gain,
            state: init,
        }
    }

    pub fn update(&mut self, input: f64, t: f64) -> f64 {
        // let r_1 = input.clamp(self.command_saturation_bottom, self.command_saturation_top);
        // let r_2 = r_1 - self.feedback;
        // let r_3 = self.gain * r_2;
        // let r_4 = r_3.clamp(-self.rate_saturation, self.rate_saturation);

        let csb = self.command_saturation_bottom;
        let cst = self.command_saturation_top;
        let rs = self.rate_saturation;
        let g = self.gain;

        let dynamics = move |_t: f64, state: f64, input: f64| -> f64 {
            let r_1 = input.clamp(csb, cst);
            let r_2 = r_1 - state;
            let r_3 = g * r_2;
            let r_4 = r_3.clamp(-rs, rs);
            r_4
        };

        let r_5 = self.solver.solve(&dynamics, t, self.state, input);
        self.state = r_5;

        // let r_5 = self.integrator.integrate(r_4, t);
        // self.feedback = r_5;
        let r_6 = r_5.clamp(self.command_saturation_bottom, self.command_saturation_top);
        r_6
    }

    pub fn state(&self) -> f64 {
        self.state
    }

    pub fn reset(&mut self, state: f64) {
        // self.feedback = 0.0;
        self.state = state;
        // self.integrator.reset();
    }

    // pub fn last(&self) -> f64 {
    //     self.last
    // }
}

#[cfg(test)]
mod components_tests {
    use crate::{
        components::{basic::step, group::Actuator},
        solver,
        utils::test_logger_init,
    };
    use log::trace;
    use std::sync::Arc;
    use std::time::{Duration, SystemTime};

    #[test]
    fn test_actuator() {
        test_logger_init();
        let solver = solver::rk::RK4Solver::new(0.01);
        let solver = Arc::new(solver);
        let mut i = Actuator::new(solver.clone(), -2.2441, 25.0, -25.0, 60.0, 20.2);
        let start_time = SystemTime::now();
        let mut r;
        loop {
            let current_time = SystemTime::now();
            let delta_time = current_time.duration_since(start_time).unwrap();
            r = i.update(
                step(0.0, 25.0, 3.0, delta_time.as_secs_f64()),
                delta_time.as_secs_f64(),
            );
            trace!("time: {:?} \n{:?}\n", delta_time, r);
            if delta_time > Duration::from_secs_f32(3.0) {
                break;
            }
        }
    }
}
