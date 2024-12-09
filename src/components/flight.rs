use crate::model::{
    MechanicalModelInput, MechanicalModelOutput, PlaneConstants, State, StateExtend, C,
};
use crate::plugin::{
    delete_handler_constructor, init_handler_constructor, step_handler_constructor,
    trim_handler_constructor, AerodynamicModel, AerodynamicModelDeleteFn, AerodynamicModelInitFn,
    AerodynamicModelStepFn, AerodynamicModelTrimFn, AsPlugin,
};
use crate::utils::{error::FatalCoreError, Vector};
use log::warn;

/// gravity ft/s^2
pub const G: f64 = 32.17;

/// Disturbance on the rudder surface
pub fn disturbance(deflection: f64, t: f64) -> f64 {
    if t >= 1.0 && t <= 3.0 {
        deflection
    } else if t >= 3.0 && t <= 5.0 {
        -deflection
    } else {
        0.0
    }
}

pub fn multi_to_deg(input: &Vector) -> Vector {
    assert!(input.dim() >= 12);
    let mut input = input.clone();
    let index = [3, 4, 5, 7, 8, 9, 10, 11];
    for i in 0..index.len() {
        input[index[i]] = input[index[i]].to_degrees();
    }
    input
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Atmos {
    pub mach: f64,
    pub qbar: f64,
    pub ps: f64,
}

impl Atmos {
    pub fn new(mach: f64, qbar: f64, ps: f64) -> Self {
        Self { mach, qbar, ps }
    }

    /// Function for mach and qbar
    pub fn atmos(altitude: f64, velocity: f64) -> Self {
        let rho0 = 2.377e-3;
        let tfac = 1.0 - 0.703e-5 * altitude;

        let mut temp = 519.0 * tfac;
        if altitude >= 35000.0 {
            temp = 390.0;
        }

        let mach = velocity / (1.4 * 1716.3 * temp).sqrt();
        let rho = rho0 * tfac.powf(4.14);
        let qbar = 0.5 * rho * velocity.powi(2);
        let mut ps = 1715.0 * rho * temp;

        if ps.abs() < 1.0e-6 {
            ps = 1715.0;
        }

        Atmos::new(mach, qbar, ps)
    }
}

impl Into<(f64, f64, f64)> for Atmos {
    fn into(self) -> (f64, f64, f64) {
        (self.mach, self.qbar, self.ps)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// rad
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    pub phi: f64,
    pub theta: f64,
    pub psi: f64,
    pub trigonal_phi: [f64; 2],
    pub trigonal_theta: [f64; 3],
    pub trigonal_psi: [f64; 2],
}

impl Orientation {
    /// in rad
    pub fn new(phi: f64, theta: f64, psi: f64) -> Self {
        let trigonal_phi = [phi.sin(), phi.cos()];
        let trigonal_theta = [theta.sin(), theta.cos(), theta.tan()];
        let trigonal_psi = [psi.sin(), psi.cos()];
        Self {
            phi,
            theta,
            psi,
            trigonal_phi,
            trigonal_theta,
            trigonal_psi,
        }
    }
}

impl From<&State> for Orientation {
    fn from(value: &State) -> Self {
        Self::new(value.phi, value.theta, value.psi)
    }
}

/// alpha: angle of attack in degrees
/// beta: sideslip angle in degrees
#[derive(Debug, Clone, Copy)]
pub struct AirAngles {
    pub alpha: f64,
    pub beta: f64,
    pub trigonal_alpha: [f64; 2],
    pub trigonal_beta: [f64; 3],
}

impl AirAngles {
    /// in rad
    pub fn new(alpha: f64, beta: f64) -> Self {
        let trigonal_alpha = [alpha.sin(), alpha.cos()];
        let trigonal_beta = [beta.sin(), beta.cos(), beta.tan()];
        Self {
            alpha: alpha.to_degrees(),
            beta: beta.to_degrees(),
            trigonal_alpha,
            trigonal_beta,
        }
    }

    pub fn derivation(
        &self,
        velocity: f64,
        velocity_dot: f64,
        sub_velocity: &Vector3,
        sub_velocity_dot: &Vector3,
    ) -> (f64, f64) {
        let u = sub_velocity.x;
        let v = sub_velocity.y;
        let w = sub_velocity.z;

        let u_dot = sub_velocity_dot.x;
        let v_dot = sub_velocity_dot.y;
        let w_dot = sub_velocity_dot.z;

        let alpha_dot = (u * w_dot - w * u_dot) / (u.powi(2) + w.powi(2));
        let beta_dot =
            (v_dot * velocity - v * velocity_dot) / (velocity.powi(2) * self.trigonal_beta[1]);
        (alpha_dot, beta_dot)
    }
}

impl From<&State> for AirAngles {
    fn from(value: &State) -> Self {
        Self::new(value.alpha, value.beta)
    }
}

/// p: Roll Rate: rolling moment is Lbar
/// q: Pitch Rate: pitching moment is M
/// r: Yaw Rate: yawing moment is N
#[derive(Debug, Clone, Copy)]
pub struct AngleRates {
    pub p: f64,
    pub q: f64,
    pub r: f64,
}

impl AngleRates {
    pub fn new(p: f64, q: f64, r: f64) -> Self {
        Self { p, q, r }
    }

    pub fn derivation(&self, c: &C, constants: &PlaneConstants, qbar: f64) -> Self {
        let b = constants.b;
        let s = constants.s;
        let c_bar = constants.c_bar;

        let h_eng = constants.h_eng;

        let j_y = constants.j_y;
        let j_xz = constants.j_xz;
        let j_z = constants.j_z;
        let j_x = constants.j_x;
        let l_total = c.c_l * qbar * s * b;
        let m_total = c.c_m * qbar * s * c_bar;
        let n_total = c.c_n * qbar * s * b;

        let denom = j_x * j_z - j_xz.powi(2);

        let p_dot = (j_z * l_total + j_xz * n_total
            - (j_z * (j_z - j_y) + j_xz.powi(2)) * self.q * self.r
            + j_xz * (j_x - j_y + j_z) * self.p * self.q
            + j_xz * self.q * h_eng)
            / denom;

        let q_dot = (m_total + (j_z - j_x) * self.p * self.r
            - j_xz * (self.p.powi(2) - self.r.powi(2))
            - self.r * h_eng)
            / j_y;

        let r_dot =
            (j_x * n_total + j_xz * l_total + (j_x * (j_x - j_y) + j_xz.powi(2)) * self.p * self.q
                - j_xz * (j_x - j_y + j_z) * self.q * self.r
                + j_x * self.q * h_eng)
                / denom;

        Self::new(p_dot, q_dot, r_dot)
    }
}

impl From<&State> for AngleRates {
    fn from(value: &State) -> Self {
        Self::new(value.p, value.q, value.r)
    }
}

/// return the dot of position and directional_velocity
fn navgation(
    velocity: f64,
    orientation: &Orientation,
    air_angles: &AirAngles,
) -> (Vector3, Vector3) {
    let ca = air_angles.trigonal_alpha[1];
    let cb = air_angles.trigonal_beta[1];
    let sa = air_angles.trigonal_alpha[0];
    let sb = air_angles.trigonal_beta[0];

    let ctheta = orientation.trigonal_theta[1];
    let cphi = orientation.trigonal_phi[1];
    let cpsi = orientation.trigonal_psi[1];
    let stheta = orientation.trigonal_theta[0];
    let sphi = orientation.trigonal_phi[0];
    let spsi = orientation.trigonal_psi[0];

    // directional velocities.
    let u = velocity * ca * cb;
    let v = velocity * sb;
    let w = velocity * sa * cb;

    let npos = u * (ctheta * cpsi)
        + v * (sphi * cpsi * stheta - cphi * spsi)
        + w * (cphi * stheta * cpsi + sphi * spsi);

    let epos = u * (ctheta * spsi)
        + v * (sphi * spsi * stheta + cphi * cpsi)
        + w * (cphi * stheta * spsi - sphi * cpsi);

    let altitude = u * stheta - v * (sphi * ctheta) - w * (cphi * ctheta);

    (Vector3::new(npos, epos, altitude), Vector3::new(u, v, w))
}

/// return dot of orientation
fn kinematics(orientation: &Orientation, angle_rates: &AngleRates) -> Vector3 {
    let ctheta = orientation.trigonal_theta[1];
    let cphi = orientation.trigonal_phi[1];
    let ttheta = orientation.trigonal_theta[2];
    let sphi = orientation.trigonal_phi[0];

    let phi_dot = angle_rates.p + ttheta * (angle_rates.q * sphi + angle_rates.r * cphi);
    let theta_dot = angle_rates.q * cphi - angle_rates.r * sphi;
    let psi_dot = (angle_rates.q * sphi + angle_rates.r * cphi) / ctheta;

    Vector3::new(phi_dot, theta_dot, psi_dot)
}

/// return dot of velocity and it's sub value
fn velocity_derivation(
    c: &C,
    constants: &PlaneConstants,
    velocity: f64,
    sub_velocity: &Vector3,
    orientation: &Orientation,
    angle_rates: &AngleRates,
    qbar: f64,
    thrust: &f64,
) -> (f64, Vector3) {
    let m = constants.m;
    let s = constants.s;

    let u = sub_velocity.x;
    let v = sub_velocity.y;
    let w = sub_velocity.z;

    let p = angle_rates.p;
    let q = angle_rates.q;
    let r = angle_rates.r;

    let stheta = orientation.trigonal_theta[0];
    let ctheta = orientation.trigonal_theta[1];
    let sphi = orientation.trigonal_phi[0];
    let cphi = orientation.trigonal_phi[1];

    let u_dot = r * v - q * w - G * stheta + qbar * s * c.c_x / m + thrust / m;
    let v_dot = p * w - r * u + G * ctheta * sphi + qbar * s * c.c_y / m;
    let w_dot = q * u - p * v + G * ctheta * cphi + qbar * s * c.c_z / m;
    (
        (u * u_dot + v * v_dot + w * w_dot) / velocity,
        Vector3::new(u_dot, v_dot, w_dot),
    )
}

fn accels(
    sub_velocity: Vector3,
    sub_velocity_dot: Vector3,
    orientation: &Orientation,
    angle_rates: &AngleRates,
) -> Vector3 {
    // const GRAV: f64 = 32.174;
    let vel_u = sub_velocity.x;
    let vel_v = sub_velocity.y;
    let vel_w = sub_velocity.z;
    let u_dot = sub_velocity_dot.x;
    let v_dot = sub_velocity_dot.y;
    let w_dot = sub_velocity_dot.z;
    let nx_cg = 1.0 / G * (u_dot + angle_rates.q * vel_w - angle_rates.r * vel_v)
        + orientation.trigonal_theta[0];
    let ny_cg = 1.0 / G * (v_dot + angle_rates.r * vel_u - angle_rates.p * vel_w)
        - orientation.trigonal_theta[1] * orientation.trigonal_phi[0];
    let nz_cg = -1.0 / G * (w_dot + angle_rates.p * vel_v - angle_rates.q * vel_u)
        + orientation.trigonal_theta[1] * orientation.trigonal_phi[1];

    Vector3::new(nx_cg, ny_cg, nz_cg)
}

pub fn get_lef(altitude: f64, velocity: f64, alpha: f64) -> f64 {
    let atmos = Atmos::atmos(altitude, velocity);
    let mut lef = 1.38 * alpha.to_degrees() - 9.05 * atmos.qbar / atmos.ps + 1.45;
    lef = lef.clamp(0.0, 25.0);
    lef
}

pub struct MechanicalModel {
    constants: PlaneConstants,
    model_trim_func: Box<AerodynamicModelTrimFn>,
    model_init_func: Box<AerodynamicModelInitFn>,
    model_step_func: Box<AerodynamicModelStepFn>,
    model_delete_func: Box<AerodynamicModelDeleteFn>,
}

impl MechanicalModel {
    pub fn new(model: &AerodynamicModel) -> Result<Self, FatalCoreError> {
        let constants = model
            .load_constants()
            .map_err(|e| FatalCoreError::from(e))?;
        let trim_handler = model
            .get_trim_handler()
            .map_err(|e| FatalCoreError::from(e))?;
        let init_handler = model
            .get_init_handler()
            .map_err(|e| FatalCoreError::from(e))?;
        let step_handler = model
            .get_step_handler()
            .map_err(|e| FatalCoreError::from(e))?;
        let delete_handler = model
            .get_delete_handler()
            .map_err(|e| FatalCoreError::from(e))?;
        let model_trim_func = trim_handler_constructor(trim_handler, model.info().name.clone());
        let model_init_func = init_handler_constructor(init_handler, model.info().name.clone());
        let model_step_func = step_handler_constructor(step_handler, model.info().name.clone());
        let model_delete_func =
            delete_handler_constructor(delete_handler, model.info().name.clone());
        Ok(Self {
            constants,
            model_trim_func,
            model_init_func,
            model_step_func,
            model_delete_func,
        })
    }

    pub fn init(&mut self) -> Result<(), FatalCoreError> {
        (self.model_init_func)().map_err(|e| FatalCoreError::from(e))
    }

    pub fn trim(
        &self,
        model_input: &MechanicalModelInput,
    ) -> Result<MechanicalModelOutput, FatalCoreError> {
        let state = &model_input.state;
        let control = &model_input.control;

        let orientation = Orientation::from(state);
        let air_angles = AirAngles::from(state);
        let angle_rates = AngleRates::from(state);
        let velocity = state.velocity.max(0.01);
        let altitude = state.altitude;

        let (mach, qbar, ps) = Atmos::atmos(altitude, velocity).into();
        let (position_dot, sub_velocity) = navgation(velocity, &orientation, &air_angles);
        let orientation_dot = kinematics(&orientation, &angle_rates);

        let c = (self.model_trim_func)(model_input).map_err(|e| FatalCoreError::from(e))?;

        let (velocity_dot, sub_velocity_dot) = velocity_derivation(
            &c,
            &self.constants,
            velocity,
            &sub_velocity,
            &orientation,
            &angle_rates,
            qbar,
            &control.thrust,
        );
        let (alpha_dot, beta_dot) =
            air_angles.derivation(velocity, velocity_dot, &sub_velocity, &sub_velocity_dot);
        let angle_rate_dot = angle_rates.derivation(&c, &self.constants, qbar);

        let n = accels(sub_velocity, sub_velocity_dot, &orientation, &angle_rates);

        let state_dot = State::from([
            position_dot.x,
            position_dot.y,
            position_dot.z,
            orientation_dot.x,
            orientation_dot.y,
            orientation_dot.z,
            velocity_dot,
            alpha_dot,
            beta_dot,
            angle_rate_dot.p,
            angle_rate_dot.q,
            angle_rate_dot.r,
        ]);
        let state_extend = StateExtend::from([n.x, n.y, n.z, mach, qbar, ps]);

        Ok(MechanicalModelOutput::new(state_dot, state_extend))
    }

    pub fn step(
        &self,
        model_input: &MechanicalModelInput,
    ) -> Result<MechanicalModelOutput, FatalCoreError> {
        let state = &model_input.state;
        let control = &model_input.control;

        let orientation = Orientation::from(state);
        let air_angles = AirAngles::from(state);
        let angle_rates = AngleRates::from(state);
        let velocity = state.velocity.max(0.01);
        let altitude = state.altitude;

        let (mach, qbar, ps) = Atmos::atmos(altitude, velocity).into();
        let (position_dot, sub_velocity) = navgation(velocity, &orientation, &air_angles);
        let orientation_dot = kinematics(&orientation, &angle_rates);

        let c = (self.model_step_func)(model_input).map_err(|e| FatalCoreError::from(e))?;
        let (velocity_dot, sub_velocity_dot) = velocity_derivation(
            &c,
            &self.constants,
            velocity,
            &sub_velocity,
            &orientation,
            &angle_rates,
            qbar,
            &control.thrust,
        );
        let (alpha_dot, beta_dot) =
            air_angles.derivation(velocity, velocity_dot, &sub_velocity, &sub_velocity_dot);
        let angle_rate_dot = angle_rates.derivation(&c, &self.constants, qbar);

        let n = accels(sub_velocity, sub_velocity_dot, &orientation, &angle_rates);

        let state_dot = State::from([
            position_dot.x,
            position_dot.y,
            position_dot.z,
            orientation_dot.x,
            orientation_dot.y,
            orientation_dot.z,
            velocity_dot,
            alpha_dot,
            beta_dot,
            angle_rate_dot.p,
            angle_rate_dot.q,
            angle_rate_dot.r,
        ]);
        let state_extend = StateExtend::from([n.x, n.y, n.z, mach, qbar, ps]);

        Ok(MechanicalModelOutput::new(state_dot, state_extend))
    }

    pub fn delete(&self) {
        let e = (self.model_delete_func)();
        if let Err(e) = e {
            warn!("{}", e)
        }
    }
}

unsafe impl Sync for MechanicalModel {}
unsafe impl Send for MechanicalModel {}
