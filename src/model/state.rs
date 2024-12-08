use crate::utils::Vector;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// What the `state` represent
/// npos (ft) epos (ft)
/// altitude (ft)
/// phi (rad) theta (rad) psi (rad)
/// velocity (ft/s)
/// alpha (rad) beta (rad)
/// p (rad/s) q (rad/s) r (rad/s)
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct State {
    pub npos: f64,
    pub epos: f64,
    pub altitude: f64,
    pub phi: f64,
    pub theta: f64,
    pub psi: f64,
    pub velocity: f64,
    pub alpha: f64,
    pub beta: f64,
    pub p: f64,
    pub q: f64,
    pub r: f64,
}

impl std::fmt::Display for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "npos: {:.2} ft, epos: {:.2} ft, altitude: {:.2} ft",
            self.npos, self.epos, self.altitude
        )?;
        writeln!(
            f,
            "phi: {:.4} rad, theta: {:.4} rad, psi: {:.4} rad",
            self.phi, self.theta, self.psi
        )?;
        writeln!(
            f,
            "velocity: {:.4} ft/s, alpha: {:.4} rad, beta: {:.4} rad",
            self.velocity, self.alpha, self.beta
        )?;
        writeln!(
            f,
            "p: {:.4} rad/s, q: {:.4} rad/s, r: {:.4} rad/s",
            self.p, self.q, self.r
        )
    }
}

impl From<&[f64]> for State {
    fn from(value: &[f64]) -> Self {
        Self {
            npos: value[0],
            epos: value[1],
            altitude: value[2],
            phi: value[3],
            theta: value[4],
            psi: value[5],
            velocity: value[6],
            alpha: value[7],
            beta: value[8],
            p: value[9],
            q: value[10],
            r: value[11],
        }
    }
}

impl From<[f64; 12]> for State {
    fn from(value: [f64; 12]) -> Self {
        Self {
            npos: value[0],
            epos: value[1],
            altitude: value[2],
            phi: value[3],
            theta: value[4],
            psi: value[5],
            velocity: value[6],
            alpha: value[7],
            beta: value[8],
            p: value[9],
            q: value[10],
            r: value[11],
        }
    }
}

impl Into<[f64; 12]> for State {
    fn into(self) -> [f64; 12] {
        [
            self.npos,
            self.epos,
            self.altitude,
            self.phi,
            self.theta,
            self.psi,
            self.velocity,
            self.alpha,
            self.beta,
            self.p,
            self.q,
            self.r,
        ]
    }
}

impl From<Vec<f64>> for State {
    fn from(value: Vec<f64>) -> Self {
        Self::from(&value[..])
    }
}

impl From<State> for Vec<f64> {
    fn from(value: State) -> Self {
        Vec::from(<State as Into<[f64; 12]>>::into(value))
    }
}

impl From<Vector> for State {
    fn from(value: Vector) -> Self {
        Self::from(&value[..])
    }
}

impl Into<Vector> for State {
    fn into(self) -> Vector {
        Vector::from(<State as Into<Vec<f64>>>::into(self))
    }
}

impl Into<HashMap<String, f64>> for State {
    fn into(self) -> HashMap<String, f64> {
        let mut map = HashMap::new();
        map.insert("npos".to_string(), self.npos);
        map.insert("epos".to_string(), self.epos);
        map.insert("altitude".to_string(), self.altitude);
        map.insert("phi".to_string(), self.phi);
        map.insert("theta".to_string(), self.theta);
        map.insert("psi".to_string(), self.psi);
        map.insert("velocity".to_string(), self.velocity);
        map.insert("alpha".to_string(), self.alpha);
        map.insert("beta".to_string(), self.beta);
        map.insert("p".to_string(), self.p);
        map.insert("q".to_string(), self.q);
        map.insert("r".to_string(), self.r);
        map
    }
}
