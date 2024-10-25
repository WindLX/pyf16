use super::{state::State, state_extend::StateExtend};

/// The Ouput of the Model
/// the value's index in `state_dot` is as same of the `state` in `ModelInput`
#[derive(Debug, Clone, Copy)]
pub struct MechanicalModelOutput {
    pub state_dot: State,
    pub state_extend: StateExtend,
}

impl MechanicalModelOutput {
    pub fn new(state_dot: impl Into<State>, state_extend: impl Into<StateExtend>) -> Self {
        Self {
            state_dot: state_dot.into(),
            state_extend: state_extend.into(),
        }
    }
}

impl std::fmt::Display for MechanicalModelOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "State_dot(*/t):   \n{}", self.state_dot)?;
        writeln!(f, "State_extend:\n{}", self.state_extend)
    }
}
