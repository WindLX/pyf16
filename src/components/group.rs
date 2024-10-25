use super::basic::Integrator;

#[derive(Clone)]
pub struct Actuator {
    integrator: Integrator,
    feedback: f64,
    command_saturation_top: f64,
    command_saturation_bottom: f64,
    rate_saturation: f64,
    gain: f64,
    last: f64,
}

impl Actuator {
    pub fn new(
        init: f64,
        command_saturation_top: f64,
        command_saturation_bottom: f64,
        rate_saturation: f64,
        gain: f64,
    ) -> Self {
        Self {
            integrator: Integrator::new(init),
            feedback: 0.0,
            command_saturation_top,
            command_saturation_bottom,
            rate_saturation,
            gain,
            last: 0.0,
        }
    }

    pub fn update(&mut self, value: f64, t: f64) -> f64 {
        self.last = value;
        let r_1 = value.clamp(self.command_saturation_bottom, self.command_saturation_top);
        let r_2 = r_1 - self.feedback;
        let r_3 = self.gain * r_2;
        let r_4 = r_3.clamp(-self.rate_saturation, self.rate_saturation);
        let r_5 = self.integrator.integrate(r_4, t);
        self.feedback = r_5;
        // let r_6 = r_5;
        let r_6 = r_5.clamp(self.command_saturation_bottom, self.command_saturation_top);
        r_6
    }

    pub fn past(&self) -> f64 {
        self.integrator.past()
    }

    pub fn reset(&mut self) {
        self.feedback = 0.0;
        self.integrator.reset();
    }

    // pub fn last(&self) -> f64 {
    //     self.last
    // }
}

#[cfg(test)]
mod components_tests {
    use crate::{
        components::{basic::step, group::Actuator},
        utils::test_logger_init,
    };
    use log::trace;
    use std::time::{Duration, SystemTime};

    #[test]
    fn test_actuator() {
        test_logger_init();
        let mut i = Actuator::new(-2.2441, 25.0, -25.0, 60.0, 20.2);
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
