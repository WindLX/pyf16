use crate::model::{Control, ControlLimit, PlaneConstants, State, C};
use libc::{c_double, c_int};

pub type FrModelLoadConstants = unsafe extern "C" fn(constants: *mut PlaneConstants) -> c_int;

pub type FrModelLoadCtrlLimits = unsafe extern "C" fn(ctrl_limits: *mut ControlLimit) -> c_int;

pub type FrModelTrim = unsafe extern "C" fn(
    state: *const State,
    control: *const Control,
    d_lef: c_double,
    c: *mut C,
) -> c_int;

pub type FrModelInit = unsafe extern "C" fn() -> c_int;

pub type FrModelStep = unsafe extern "C" fn(
    state: *const State,
    control: *const Control,
    d_lef: c_double,
    c: *mut C,
) -> c_int;

pub type FrModelDelete = unsafe extern "C" fn() -> c_int;
