use super::{Dynamics, ODESolver, VectorDynamics, VectorODESolver};
use crate::utils::Vector;

#[derive(Clone)]
pub struct RK1Solver {
    delta_t: f64,
}

impl RK1Solver {
    pub fn new(delta_t: f64) -> Self {
        RK1Solver { delta_t }
    }
}

impl ODESolver for RK1Solver {
    fn solve(&self, dynamics: &Dynamics, t: f64, state: f64, input: f64) -> f64 {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_next = state + k1 * dt;

        state_next
    }
}

impl VectorODESolver for RK1Solver {
    fn solve(&self, dynamics: &VectorDynamics, t: f64, state: &Vector, input: &Vector) -> Vector {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_next = state.clone() + k1 * dt;

        state_next
    }
}

#[derive(Clone)]
pub struct RK2Solver {
    delta_t: f64,
}

impl RK2Solver {
    pub fn new(delta_t: f64) -> Self {
        RK2Solver { delta_t }
    }
}

impl ODESolver for RK2Solver {
    fn solve(&self, dynamics: &Dynamics, t: f64, state: f64, input: f64) -> f64 {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);

        let state_k2 = state + k1 * dt;
        let k2 = dynamics(t + dt, state_k2, input);
        let state_next = state + (k1 + k2) * dt / 2.0;

        state_next
    }
}

impl VectorODESolver for RK2Solver {
    fn solve(&self, dynamics: &VectorDynamics, t: f64, state: &Vector, input: &Vector) -> Vector {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);

        let state_k2 = state.clone() + k1.clone() * dt;
        let k2 = dynamics(t + dt, &state_k2, input);
        let state_next = state.clone() + (k1 + k2) * dt / 2.0;

        state_next
    }
}

#[derive(Clone)]
pub struct RK3Solver {
    delta_t: f64,
}

impl RK3Solver {
    pub fn new(delta_t: f64) -> Self {
        RK3Solver { delta_t }
    }
}

impl ODESolver for RK3Solver {
    fn solve(&self, dynamics: &Dynamics, t: f64, state: f64, input: f64) -> f64 {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_k2 = state + k1 * dt / 2.0;
        let k2 = dynamics(t + dt / 2.0, state_k2, input);
        let state_k3 = state - k1 * dt + k2 * 2.0 * dt;
        let k3 = dynamics(t + dt, state_k3, input);

        let state_next = state + (k1 + k2 * 4.0 + k3) * dt / 6.0;

        state_next
    }
}

impl VectorODESolver for RK3Solver {
    fn solve(&self, dynamics: &VectorDynamics, t: f64, state: &Vector, input: &Vector) -> Vector {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_k2 = state.clone() + k1.clone() * dt / 2.0;
        let k2 = dynamics(t + dt / 2.0, &state_k2, input);
        let state_k3 = state.clone() - k1.clone() * dt + k2.clone() * 2.0 * dt;
        let k3 = dynamics(t + dt, &state_k3, input);

        let state_next = state.clone() + (k1 + k2 * 4.0 + k3) * dt / 6.0;

        state_next
    }
}

#[derive(Clone)]
pub struct RK4Solver {
    delta_t: f64,
}

impl RK4Solver {
    pub fn new(delta_t: f64) -> Self {
        RK4Solver { delta_t }
    }
}

impl ODESolver for RK4Solver {
    fn solve(&self, dynamics: &Dynamics, t: f64, state: f64, input: f64) -> f64 {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_k2 = state + k1 * dt / 2.0;
        let k2 = dynamics(t + dt / 2.0, state_k2, input);
        let state_k3 = state + k2 * dt / 2.0;
        let k3 = dynamics(t + dt / 2.0, state_k3, input);
        let state_k4 = state + k3 * dt;
        let k4 = dynamics(t + dt, state_k4, input);

        let state_next = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0;

        state_next
    }
}

impl VectorODESolver for RK4Solver {
    fn solve(&self, dynamics: &VectorDynamics, t: f64, state: &Vector, input: &Vector) -> Vector {
        let dt = self.delta_t;

        let k1 = dynamics(t, state, input);
        let state_k2 = state.clone() + k1.clone() * dt / 2.0;
        let k2 = dynamics(t + dt / 2.0, &state_k2, input);
        let state_k3 = state.clone() + k2.clone() * dt / 2.0;
        let k3 = dynamics(t + dt / 2.0, &state_k3, input);
        let state_k4 = state.clone() + k3.clone() * dt;
        let k4 = dynamics(t + dt, &state_k4, input);

        let state_next = state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0;

        state_next
    }
}
