use super::{control::Control, state::State};

/// The Input of the Model
#[derive(Debug, Clone)]
pub struct MechanicalModelInput {
    pub state: State,
    pub control: Control,
}

impl MechanicalModelInput {
    pub fn new(state: impl Into<State>, control: impl Into<Control>) -> Self {
        Self {
            state: state.into(),
            control: control.into(),
        }
    }
}

impl std::fmt::Display for MechanicalModelInput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "state:  \n{}", self.state)?;
        writeln!(f, "Control:\n{}", self.control)
    }
}
