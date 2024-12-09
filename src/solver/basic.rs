use crate::utils::Vector;

pub type Dynamics = dyn Fn(f64, f64, f64) -> f64;

pub trait ODESolver: Clone {
    fn solve(&self, dynamics: &Dynamics, t: f64, state: f64, input: f64) -> f64;
}

pub type VectorDynamics = dyn Fn(f64, &Vector, &Vector) -> Vector;

/// A trait for ordinary differential equation (ODE) solvers.
/// An ODE solver is a type that can solve a system of ODEs given the current time, state, and input.
pub trait VectorODESolver: Clone {
    fn solve(&self, dynamics: &VectorDynamics, t: f64, state: &Vector, input: &Vector) -> Vector;
}
