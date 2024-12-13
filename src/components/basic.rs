use crate::{
    solver::ODESolver,
    utils::{Matrix, Vector},
};

pub fn step(init: f64, end: f64, step_time: f64, t: f64) -> f64 {
    if t < step_time {
        init
    } else {
        end
    }
}

// pub struct LinearModel<S: ODESolver> {
//     solver: S,
//     a: Matrix,
//     b: Matrix,
//     c: Matrix,
//     d: Matrix,
// }

// impl<S> LinearModel<S>
// where
//     S: ODESolver,
// {
//     pub fn new(a: Matrix, b: Matrix, c: Matrix, d: Matrix) -> Self {
//         Self { a, b, c, d }
//     }

//     pub fn a(&self) -> &Matrix {
//         &self.a
//     }

//     pub fn b(&self) -> &Matrix {
//         &self.b
//     }

//     pub fn c(&self) -> &Matrix {
//         &self.c
//     }

//     pub fn d(&self) -> &Matrix {
//         &self.d
//     }
// }

#[deprecated]
#[derive(Clone)]
pub struct Integrator {
    init: f64,
    last_time: f64,
    last_value: f64,
    past: f64,
}

impl Integrator {
    pub fn new(init: f64) -> Self {
        Self {
            init,
            last_value: init,
            past: init,
            last_time: 0.0,
        }
    }

    // pub fn integrate(&mut self, value: f64, t: f64) -> f64 {
    //     self.past += (t - self.last_time) * (value + self.last_value) * 0.5;
    //     self.last_value = value;
    //     self.last_time = t;
    //     self.past
    // }

    pub fn integrate(&mut self, value: f64, t: f64) -> f64 {
        let h = t - self.last_time;
        let k1 = self.last_value;
        let k2 = self.last_value + 0.5 * h * value;
        let k3 = self.last_value + 0.5 * h * value;
        let k4 = self.last_value + h * value;

        self.past += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        self.last_value = value;
        self.last_time = t;
        self.past
    }

    pub fn past(&self) -> f64 {
        self.past
    }

    pub fn reset(&mut self) {
        self.last_value = self.init;
        self.past = self.init;
        self.last_time = 0.0;
    }
}

#[deprecated]
#[derive(Clone)]
pub struct VectorIntegrator {
    init: Vector,
    last_time: f64,
    last_value: Vector,
    past: Vector,
}

impl VectorIntegrator {
    pub fn new(init: impl Into<Vector>) -> Self {
        let init = init.into();
        Self {
            init: init.clone(),
            last_value: init.clone(),
            past: init,
            last_time: 0.0,
        }
    }

    // pub fn integrate(&mut self, value: impl Into<Vector>, t: f64) -> Vector {
    //     let value = value.into();
    //     self.past += (value.clone() + self.last_value.clone()) * (t - self.last_time) * 0.5;
    //     self.last_value = value;
    //     self.last_time = t;
    //     self.past.clone()
    // }

    pub fn derivative_add(&mut self, derivative: impl Into<Vector>, t: f64) -> Vector {
        let derivative = derivative.into();
        let delta_t = t - self.last_time;
        self.past += derivative.clone() * delta_t;
        self.last_value = derivative;
        self.last_time = t;
        self.past.clone()
    }

    pub fn integrate(&mut self, value: impl Into<Vector>, t: f64) -> Vector {
        let value = value.into();
        let h = t - self.last_time;

        let k1 = self.last_value.clone();
        let k2 = self.last_value.clone() + value.clone() * 0.5 * h;
        let k3 = self.last_value.clone() + value.clone() * 0.5 * h;
        let k4 = self.last_value.clone() + value.clone() * h;

        self.past += (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
        self.last_value = value;
        self.last_time = t;
        self.past.clone()
    }

    pub fn past(&self) -> Vector {
        self.past.clone()
    }

    pub fn reset(&mut self) {
        self.last_value = self.init.clone();
        self.past = self.init.clone();
        self.last_time = 0.0;
    }
}

#[cfg(test)]
mod components_tests {
    use super::{Integrator, VectorIntegrator};
    use crate::utils::test_logger_init;
    use log::{info, trace};
    use std::time::{Duration, SystemTime};

    #[test]
    fn test_integrator() {
        test_logger_init();
        let mut i = Integrator::new(0.0);
        let start_time = SystemTime::now();
        let mut r;
        loop {
            let current_time = SystemTime::now();
            let delta_time = current_time.duration_since(start_time).unwrap();
            r = i.integrate(delta_time.as_secs_f64(), delta_time.as_secs_f64());
            trace!("time: {:?} \n{:?}\n", delta_time, r);
            if delta_time > Duration::from_secs_f32(1.0) {
                break;
            }
        }
        assert!((r - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_vector_integrator() {
        test_logger_init();
        let mut i = VectorIntegrator::new(vec![0.0, 0.0]);
        let start_time = SystemTime::now();
        let mut r;
        loop {
            let current_time = SystemTime::now();
            let delta_time = current_time.duration_since(start_time).unwrap();
            r = i.integrate(
                vec![delta_time.as_secs_f64(), 2.0 * delta_time.as_secs_f64()],
                delta_time.as_secs_f64(),
            );
            trace!("time: {:?} \n{:?}\n", delta_time, r);
            info!("{:?}", i.past);
            if delta_time > Duration::from_secs_f32(1.0) {
                break;
            }
        }

        assert!((r[0] - 0.5).abs() < 1e-3);
        assert!((r[1] - 1.0).abs() < 1e-3);
    }
}
