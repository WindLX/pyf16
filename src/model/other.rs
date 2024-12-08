use serde::{Deserialize, Serialize};

/// Aerodynamic coefficient
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct C {
    pub c_x: f64,
    pub c_z: f64,
    pub c_m: f64,
    pub c_y: f64,
    pub c_n: f64,
    pub c_l: f64,
}

impl C {
    pub fn new(c_x: f64, c_z: f64, c_m: f64, c_y: f64, c_n: f64, c_l: f64) -> Self {
        Self {
            c_x,
            c_z,
            c_m,
            c_y,
            c_n,
            c_l,
        }
    }
}

/// Constants of a plane
/// m: mass slugs
/// b: span ft
/// s: planform area ft^2
/// c_bar: mean aero chord, ft
/// x_cg_r: reference center of gravity as a fraction of cbar
/// x_cg: center of gravity as a fraction of cbar
/// h_eng: turbine momentum along roll axis
/// j_y, j_xz, j_z, j_x: slug-ft^2
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PlaneConstants {
    pub m: f64,
    pub b: f64,
    pub s: f64,
    pub c_bar: f64,
    pub x_cg_r: f64,
    pub x_cg: f64,
    pub h_eng: f64,
    pub j_y: f64,
    pub j_xz: f64,
    pub j_z: f64,
    pub j_x: f64,
}

impl PlaneConstants {
    pub fn new(
        m: f64,
        b: f64,
        s: f64,
        c_bar: f64,
        x_cg_r: f64,
        x_cg: f64,
        h_eng: f64,
        j_y: f64,
        j_xz: f64,
        j_z: f64,
        j_x: f64,
    ) -> Self {
        Self {
            m,
            b,
            s,
            c_bar,
            x_cg_r,
            x_cg,
            h_eng,
            j_y,
            j_xz,
            j_z,
            j_x,
        }
    }
}

impl std::fmt::Display for PlaneConstants {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "m: {}(slugs), b: {}(ft), s: {}(ft^2)",
            self.m, self.b, self.s
        )?;
        write!(
            f,
            "\nc_bar: {}(ft), x_cg_r: {}, x_cg: {}, h_eng: {}",
            self.c_bar, self.x_cg_r, self.x_cg, self.h_eng
        )?;
        write!(
            f,
            "\nj_y: {}(slug-ft^2), j_xz: {}(slug-ft^2), j_z: {}(slug-ft^2), j_x: {}(slug-ft^2)",
            self.j_y, self.j_xz, self.j_z, self.j_x
        )
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ControlLimit {
    pub thrust_cmd_limit_top: f64,
    pub thrust_cmd_limit_bottom: f64,
    pub thrust_rate_limit: f64,
    pub ele_cmd_limit_top: f64,
    pub ele_cmd_limit_bottom: f64,
    pub ele_rate_limit: f64,
    pub ail_cmd_limit_top: f64,
    pub ail_cmd_limit_bottom: f64,
    pub ail_rate_limit: f64,
    pub rud_cmd_limit_top: f64,
    pub rud_cmd_limit_bottom: f64,
    pub rud_rate_limit: f64,
    pub alpha_limit_top: f64,
    pub alpha_limit_bottom: f64,
    pub beta_limit_top: f64,
    pub beta_limit_bottom: f64,
}

impl ControlLimit {
    pub fn new(
        thrust_cmd_limit_top: f64,
        thrust_cmd_limit_bottom: f64,
        thrust_rate_limit: f64,
        ele_cmd_limit_top: f64,
        ele_cmd_limit_bottom: f64,
        ele_rate_limit: f64,
        ail_cmd_limit_top: f64,
        ail_cmd_limit_bottom: f64,
        ail_rate_limit: f64,
        rud_cmd_limit_top: f64,
        rud_cmd_limit_bottom: f64,
        rud_rate_limit: f64,
        alpha_limit_top: f64,
        alpha_limit_bottom: f64,
        beta_limit_top: f64,
        beta_limit_bottom: f64,
    ) -> Self {
        Self {
            thrust_cmd_limit_top,
            thrust_cmd_limit_bottom,
            thrust_rate_limit,
            ele_cmd_limit_top,
            ele_cmd_limit_bottom,
            ele_rate_limit,
            ail_cmd_limit_top,
            ail_cmd_limit_bottom,
            ail_rate_limit,
            rud_cmd_limit_top,
            rud_cmd_limit_bottom,
            rud_rate_limit,
            alpha_limit_top,
            alpha_limit_bottom,
            beta_limit_top,
            beta_limit_bottom,
        }
    }
}

impl std::fmt::Display for ControlLimit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Thrust: cmd: ({:.2}, {:.2}), rate: {:.2}",
            self.thrust_cmd_limit_top, self.thrust_cmd_limit_bottom, self.thrust_rate_limit
        )?;
        writeln!(
            f,
            "Elevator: cmd: ({:.2}, {:.2}), rate: {:.2}",
            self.ele_cmd_limit_top, self.ele_cmd_limit_bottom, self.ele_rate_limit
        )?;
        writeln!(
            f,
            "Aileron: cmd: ({:.2}, {:.2}), rate: {:.2}",
            self.ail_cmd_limit_top, self.ail_cmd_limit_bottom, self.ail_rate_limit
        )?;
        writeln!(
            f,
            "Rudder: cmd: ({:.2}, {:.2}), rate: {:.2}",
            self.rud_cmd_limit_top, self.rud_cmd_limit_bottom, self.rud_rate_limit
        )?;
        writeln!(
            f,
            "Alpha: limit: ({:.2}, {:.2})",
            self.alpha_limit_top, self.alpha_limit_bottom
        )?;
        write!(
            f,
            "Beta: limit: ({:.2}, {:.2})",
            self.beta_limit_top, self.beta_limit_bottom
        )
    }
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub enum FlightCondition {
    WingsLevel,
    Turning,
    PullUp,
    Roll,
}

impl std::fmt::Display for FlightCondition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WingsLevel => write!(f, "wings level"),
            Self::Turning => write!(f, "turning"),
            Self::PullUp => write!(f, "pull up"),
            Self::Roll => write!(f, "roll"),
        }
    }
}

impl Default for FlightCondition {
    fn default() -> Self {
        FlightCondition::WingsLevel
    }
}
