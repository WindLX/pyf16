use super::{control::Control, state::State, state_extend::StateExtend};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CoreInit {
    pub state: State,
    pub control: Control,
}

impl CoreInit {
    pub fn new(state: State, control: Control) -> Self {
        Self { state, control }
    }
}

impl Into<Vec<f64>> for CoreInit {
    fn into(self) -> Vec<f64> {
        let mut s: Vec<f64> = self.state.into();
        s.extend(Into::<Vec<f64>>::into(self.control));
        s
    }
}

impl std::fmt::Display for CoreInit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "State:  \n{}", self.state)?;
        writeln!(f, "Control:\n{}", self.control)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CoreOutput {
    pub state: State,
    pub control: Control,
    pub state_extend: StateExtend,
}

impl CoreOutput {
    pub fn new(state: State, control: Control, state_extend: StateExtend) -> Self {
        Self {
            state,
            control,
            state_extend,
        }
    }
}

impl Into<Vec<f64>> for CoreOutput {
    fn into(self) -> Vec<f64> {
        let mut s: Vec<f64> = self.state.into();
        s.extend(Into::<Vec<f64>>::into(self.control));
        s.extend(Into::<Vec<f64>>::into(self.state_extend));
        s
    }
}

impl std::fmt::Display for CoreOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "State:  \n{}", self.state)?;
        writeln!(f, "Control:\n{}", self.control)?;
        writeln!(f, "Extend: \n{}", self.state_extend)
    }
}
