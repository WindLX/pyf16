use crate::model::{Control, ControlLimit, PlaneConstants, State, C};
use libc::{c_char, c_int};

pub type FrModelLoadConstants = unsafe extern "C" fn(constants: *mut PlaneConstants) -> c_int;

pub type FrModelLoadCtrlLimits = unsafe extern "C" fn(ctrl_limits: *mut ControlLimit) -> c_int;

pub type FrModelTrim =
    unsafe extern "C" fn(state: *const State, control: *const Control, c: *mut C) -> c_int;

pub type FrModelInit =
    unsafe extern "C" fn(id: *const c_char, state: *const State, control: *const Control) -> c_int;

pub type FrModelStep = unsafe extern "C" fn(
    id: *const c_char,
    state: *const State,
    control: *const Control,
    t: f64,
    c: *mut C,
) -> c_int;

pub type FrModelDelete = unsafe extern "C" fn(id: *const c_char) -> c_int;
