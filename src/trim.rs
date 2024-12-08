use crate::model::{
    Control, ControlLimit, CoreInit, FlightCondition, MechanicalModelInput, State, StateExtend,
};
use crate::utils::{error::FatalCoreError, Vector};
use crate::{algorithm::nelder_mead::*, components::flight::MechanicalModel};
use log::trace;
use serde::{Deserialize, Serialize};
use std::{cell::RefCell, rc::Rc};

/// alpha is radians
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct TrimInit {
    pub control: Control,
    pub alpha: f64,
}

impl std::fmt::Display for TrimInit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "alpha:  {:.2}", self.alpha)?;
        write!(f, "control:\n{}", self.control)
    }
}

/// alpha has been convert to radians
impl Into<Vec<f64>> for TrimInit {
    fn into(self) -> Vec<f64> {
        let mut trim_init: Vec<_> = self.control.into();
        trim_init.push(self.alpha);
        trim_init
    }
}

/// A set of optimized initial values that are compared and verified in most cases
/// alpha have been convert to radians
impl Default for TrimInit {
    fn default() -> Self {
        Self {
            control: Control::from([5000.0, -0.09, 0.01, -0.01]),
            alpha: 8.49_f64.to_radians(),
        }
    }
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct TrimTarget {
    pub altitude: f64,
    pub velocity: f64,
    pub npos: f64,
    pub epos: f64,
}

impl std::fmt::Display for TrimTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "altitude: {:.2}, velocity: {:.2}",
            self.altitude, self.velocity
        )
    }
}

impl TrimTarget {
    pub fn new(altitude: f64, velocity: f64, npos: Option<f64>, epos: Option<f64>) -> Self {
        Self {
            altitude,
            velocity,
            npos: npos.unwrap_or_default(),
            epos: epos.unwrap_or_default(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrimOutput {
    pub state: State,
    pub control: Control,
    pub state_extend: StateExtend,
    pub nelder_mead_result: NelderMeadResult,
}

impl std::fmt::Display for TrimOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "state:  \n{}", self.state)?;
        writeln!(f, "control:\n{}", self.control)?;
        writeln!(f, "extend: \n{}", self.state_extend)?;
        writeln!(f, "nelder_mead_result: \n{}", self.nelder_mead_result)
    }
}

impl TrimOutput {
    pub fn new(
        state: State,
        control: Control,
        state_extend: StateExtend,
        nelder_mead_result: NelderMeadResult,
    ) -> Self {
        Self {
            state,
            control,
            state_extend,
            nelder_mead_result,
        }
    }
}

impl Into<CoreInit> for TrimOutput {
    fn into(self) -> CoreInit {
        CoreInit::new(self.state, self.control)
    }
}

/// Trim aircraft to desired altitude and velocity
/// fi_flag: true means hifi model
pub fn trim(
    plane: Rc<RefCell<MechanicalModel>>,
    trim_target: TrimTarget,
    trim_init: Option<TrimInit>,
    ctrl_limit: ControlLimit,
    flight_condition: Option<FlightCondition>,
    optim_options: Option<NelderMeadOptions>,
) -> Result<TrimOutput, FatalCoreError> {
    trace!(
        "trim target: {}, trim init: {}",
        trim_target,
        trim_init.unwrap_or_default()
    );

    // Initial Guess for free parameters
    // free parameters: two control values & angle of attack
    let x_0: Vec<f64> = trim_init.unwrap_or_default().into();

    let mut psi = 0.0;
    let mut psi_weight = 0.0;
    let mut q = 0.0;
    let mut theta_weight = 10.0;

    match flight_condition {
        Some(fc) => match fc {
            FlightCondition::WingsLevel => {}
            FlightCondition::Turning => {
                // turn rate, degrees/s
                psi = 1.0;
                psi_weight = 1.0;
            }
            FlightCondition::PullUp => {
                // pull-up rate, degrees/s
                q = 1.0;
                theta_weight = 1.0;
            }
            FlightCondition::Roll => {}
        },
        None => {}
    };

    let globals = vec![
        psi,
        q,
        theta_weight,
        psi_weight,
        trim_target.altitude,
        trim_target.velocity,
    ];

    let output = Rc::new(RefCell::new(Vec::<f64>::new()));
    let output_ = output.clone();

    let trim_func = move |x: &Vector| -> Result<f64, FatalCoreError> {
        trim_func(x, plane.clone(), ctrl_limit, output_.clone(), &globals)
    };

    let res = nelder_mead(Box::new(trim_func), Vector::from(x_0), optim_options)?;

    let o = Rc::into_inner(output).unwrap().into_inner();

    let mut state = State::from(&o[..12]);
    state.npos = trim_target.npos;
    state.epos = trim_target.epos;

    Ok(TrimOutput::new(
        state,
        Control::from(&res.x[..4]),
        StateExtend::from(&o[12..18]),
        res,
    ))
}

fn trim_func(
    x: &Vector,
    plane: Rc<RefCell<MechanicalModel>>,
    ctrl_limit: ControlLimit,
    output_vec: Rc<RefCell<Vec<f64>>>,
    globals: &Vec<f64>,
) -> Result<f64, FatalCoreError> {
    // global phi psi p q r phi_weight theta_weight psi_weight
    let psi = globals[0];
    let q = globals[1];
    let theta_weight = globals[2];
    let psi_weight = globals[3];
    let altitude = globals[4];
    let velocity = globals[5];

    // Implementing limits:
    // Thrust limits
    let thrust = x[0].clamp(
        ctrl_limit.thrust_cmd_limit_bottom,
        ctrl_limit.thrust_cmd_limit_top,
    );

    // Elevator limits
    let elevator = x[1].clamp(
        ctrl_limit.ele_cmd_limit_bottom,
        ctrl_limit.ele_cmd_limit_top,
    );

    // Aileron limits
    let alileron = x[2].clamp(
        ctrl_limit.ail_cmd_limit_bottom,
        ctrl_limit.ail_cmd_limit_top,
    );

    // Rudder limits
    let rudder = x[3].clamp(
        ctrl_limit.rud_cmd_limit_bottom,
        ctrl_limit.rud_cmd_limit_top,
    );

    // Angle of Attack limits
    let alpha = x[4].clamp(
        ctrl_limit.alpha_limit_bottom.to_radians(),
        ctrl_limit.alpha_limit_top.to_radians(),
    );

    let state = [
        0.0,              // npos (ft)
        0.0,              // epos (ft)
        altitude,         // altitude (ft)
        0.0,              // phi (rad)
        alpha,            // theta (rad)
        psi.to_radians(), // psi (rad)
        velocity,         // velocity (ft/s)
        alpha,            // alpha (rad)
        0.0,              // beta (rad)
        0.0,              // p (rad/s)
        q.to_radians(),   // q (rad/s)
        0.0,              // r (rad/s)
    ];

    let control = [thrust, elevator, alileron, rudder];

    // Create weight function
    // npos_dot epos_dot alt_dot phi_dot theta_dot psi_dot V_dot alpha_dpt beta_dot P_dot Q_dot R_dot
    let weight = Vector::from(vec![
        0.0,
        0.0,
        5.0,
        10.0,
        theta_weight,
        psi_weight,
        2.0,
        10.0,
        10.0,
        10.0,
        10.0,
        10.0,
    ]);

    let output = plane
        .borrow_mut()
        .trim(&MechanicalModelInput::new(state, control))?;

    let state_dot = Vector::from(Into::<Vec<f64>>::into(output.state_dot));
    let cost = weight.dot(&(state_dot.clone() * state_dot));

    let mut state_out = state.to_vec();
    let state_extend = Into::<Vec<f64>>::into(output.state_extend);
    state_out.extend_from_slice(&state_extend);
    *output_vec.borrow_mut() = state_out;

    Ok(cost)
}

#[cfg(test)]
mod trim_tests {
    use super::*;
    use crate::model::ControlLimit;
    use crate::plugin::{AerodynamicModel, AsPlugin};
    use crate::utils::test_logger_init;
    use crate::{algorithm::nelder_mead::NelderMeadOptions, components::flight::MechanicalModel};
    use std::{cell::RefCell, rc::Rc};

    const CL: ControlLimit = ControlLimit {
        thrust_cmd_limit_top: 19000.0,
        thrust_cmd_limit_bottom: 1000.0,
        thrust_rate_limit: 10000.0,
        ele_cmd_limit_top: 25.0,
        ele_cmd_limit_bottom: -25.0,
        ele_rate_limit: 60.0,
        ail_cmd_limit_top: 21.5,
        ail_cmd_limit_bottom: -21.5,
        ail_rate_limit: 80.0,
        rud_cmd_limit_top: 30.0,
        rud_cmd_limit_bottom: -30.0,
        rud_rate_limit: 120.0,
        alpha_limit_top: 45.0,
        alpha_limit_bottom: -20.0,
        beta_limit_top: 30.0,
        beta_limit_bottom: -30.0,
    };

    #[test]
    fn test_trim() {
        test_logger_init();
        let model = AerodynamicModel::new("./models/f16_model");
        assert!(matches!(model, Ok(_)));

        let model = model.unwrap();
        let res = model.plugin().install(&["./models/f16_model/data"]);
        assert!(matches!(res, Ok(Ok(_))));

        let plane = Rc::new(RefCell::new(MechanicalModel::new(&model).unwrap()));

        let trim_target = TrimTarget::new(15000.0, 500.0, None, None);
        let trim_init = None;
        let nm_options = Some(NelderMeadOptions {
            max_fun_evals: 50000,
            max_iter: 10000,
            tol_fun: 1e-10,
            tol_x: 1e-10,
        });

        let result = trim(plane.clone(), trim_target, trim_init, CL, None, nm_options).unwrap();

        let nm_result = result.nelder_mead_result;
        println!("{:#?}", result.state);
        println!("{:#?}", result.control);
        println!("{:#?}", result.state_extend);
        println!("{:#?} {:#?}", nm_result.x, nm_result.fval);
        println!("{:#?} {:#?}", nm_result.iter, nm_result.fun_evals);

        let res = model.plugin().uninstall();
        assert!(matches!(res, Ok(Ok(_))));
    }
}
