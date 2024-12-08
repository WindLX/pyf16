use crate::utils::Vector;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// What the `state_extend` represent
/// nx(g) ny(g) nz(g)
/// mach
/// qbar(lb/ft ft) ps(lb/ft ft)
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct StateExtend {
    pub nx: f64,
    pub ny: f64,
    pub nz: f64,
    pub mach: f64,
    pub qbar: f64,
    pub ps: f64,
}

impl std::fmt::Display for StateExtend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "nx: {:.4} g, ny: {:.4} g, nz: {:.4} g",
            self.nx, self.ny, self.nz
        )?;
        writeln!(f, "mach: {:.2}", self.mach)?;
        writeln!(
            f,
            "qbar: {:.2} lb/ft^2, ps: {:.2} lb/ft ft",
            self.qbar, self.ps
        )
    }
}

impl From<&[f64]> for StateExtend {
    fn from(value: &[f64]) -> Self {
        Self {
            nx: value[0],
            ny: value[1],
            nz: value[2],
            mach: value[3],
            qbar: value[4],
            ps: value[5],
        }
    }
}

impl From<[f64; 6]> for StateExtend {
    fn from(value: [f64; 6]) -> Self {
        Self {
            nx: value[0],
            ny: value[1],
            nz: value[2],
            mach: value[3],
            qbar: value[4],
            ps: value[5],
        }
    }
}

impl Into<[f64; 6]> for StateExtend {
    fn into(self) -> [f64; 6] {
        [self.nx, self.ny, self.nz, self.mach, self.qbar, self.ps]
    }
}

impl From<Vec<f64>> for StateExtend {
    fn from(value: Vec<f64>) -> Self {
        Self::from(&value[..])
    }
}

impl From<StateExtend> for Vec<f64> {
    fn from(value: StateExtend) -> Self {
        Vec::from(<StateExtend as Into<[f64; 6]>>::into(value))
    }
}

impl From<Vector> for StateExtend {
    fn from(value: Vector) -> Self {
        Self::from(&value[..])
    }
}

impl Into<Vector> for StateExtend {
    fn into(self) -> Vector {
        Vector::from(<StateExtend as Into<Vec<f64>>>::into(self))
    }
}

impl Into<HashMap<String, f64>> for StateExtend {
    fn into(self) -> HashMap<String, f64> {
        let mut map = HashMap::new();
        map.insert("nx".to_string(), self.nx);
        map.insert("ny".to_string(), self.ny);
        map.insert("nz".to_string(), self.nz);
        map.insert("mach".to_string(), self.mach);
        map.insert("qbar".to_string(), self.qbar);
        map.insert("ps".to_string(), self.ps);
        map
    }
}
