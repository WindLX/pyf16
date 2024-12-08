use crate::components::group::Actuator;
use crate::components::{basic::Integrator, flight::Atmos};
use libc::{c_char, c_int};
use log::{debug, error, info, trace, warn, Level};
use std::ffi::CStr;

pub type FrPluginHook = unsafe extern "C" fn(argc: c_int, argv: *const *const c_char) -> c_int;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub enum LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
}

impl From<LogLevel> for Level {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::TRACE => Level::Trace,
            LogLevel::DEBUG => Level::Debug,
            LogLevel::INFO => Level::Info,
            LogLevel::WARN => Level::Warn,
            LogLevel::ERROR => Level::Error,
        }
    }
}

pub(in crate::plugin) type Logger = unsafe extern "C" fn(msg: *const c_char, level: LogLevel);
pub(in crate::plugin) type FrPluginLogRegister = unsafe extern "C" fn(lg: Logger);
pub(in crate::plugin) unsafe extern "C" fn logger_callback(msg: *const c_char, level: LogLevel) {
    let msg = CStr::from_ptr(msg);
    let msg = String::from_utf8_lossy(msg.to_bytes()).to_string();
    match level {
        LogLevel::TRACE => trace!("{}", msg),
        LogLevel::DEBUG => debug!("{}", msg),
        LogLevel::INFO => info!("{}", msg),
        LogLevel::WARN => warn!("{}", msg),
        LogLevel::ERROR => error!("{}", msg),
    }
}

pub(in crate::plugin) type AtmosFunc = unsafe extern "C" fn(altitude: f64, velocity: f64) -> Atmos;
pub(in crate::plugin) type FrPluginAtmosFuncRegister = unsafe extern "C" fn(func: AtmosFunc);
pub(in crate::plugin) unsafe extern "C" fn atmos_callback(altitude: f64, velocity: f64) -> Atmos {
    Atmos::atmos(altitude, velocity)
}

pub(in crate::plugin) type IntegratorNew = unsafe extern "C" fn(init: f64) -> *mut Integrator;
pub(in crate::plugin) type FrPluginIntegratorNewRegister =
    unsafe extern "C" fn(func: IntegratorNew);
pub(in crate::plugin) unsafe extern "C" fn integrator_new_callback(init: f64) -> *mut Integrator {
    let ptr = Box::new(Integrator::new(init));
    Box::into_raw(ptr)
}

pub(in crate::plugin) type IntegratorDrop = unsafe extern "C" fn(*mut Integrator);
pub(in crate::plugin) type FrPluginIntegratorDropRegister =
    unsafe extern "C" fn(func: IntegratorDrop);
pub(in crate::plugin) unsafe extern "C" fn integrator_drop_callback(integrator: *mut Integrator) {
    if integrator.is_null() {
        return;
    }
    drop(Box::from_raw(integrator));
}

pub(in crate::plugin) type IntegratorUpdate =
    unsafe extern "C" fn(integrator: *mut Integrator, value: f64, t: f64, result: *mut f64) -> i32;
pub(in crate::plugin) type FrPluginIntegratorUpdateRegister =
    unsafe extern "C" fn(func: IntegratorUpdate);
pub(in crate::plugin) unsafe extern "C" fn integrator_update_callback(
    integrator: *mut Integrator,
    value: f64,
    t: f64,
    result: *mut f64,
) -> i32 {
    match integrator.as_mut() {
        Some(integrator) => {
            *result = integrator.integrate(value, t);
            return 0;
        }
        None => return -1,
    }
}

pub(in crate::plugin) type IntegratorPast =
    unsafe extern "C" fn(integrator: *mut Integrator, result: *mut f64) -> i32;
pub(in crate::plugin) type FrPluginIntegratorPastRegister =
    unsafe extern "C" fn(func: IntegratorPast);
pub(in crate::plugin) unsafe extern "C" fn integrator_past_callback(
    integrator: *mut Integrator,
    result: *mut f64,
) -> i32 {
    match integrator.as_mut() {
        Some(integrator) => {
            *result = integrator.past();
            return 0;
        }
        None => return -1,
    }
}

pub(in crate::plugin) type IntegratorReset =
    unsafe extern "C" fn(integrator: *mut Integrator) -> i32;
pub(in crate::plugin) type FrPluginIntegratorResetRegister =
    unsafe extern "C" fn(func: IntegratorReset);
pub(in crate::plugin) unsafe extern "C" fn integrator_reset_callback(
    integrator: *mut Integrator,
) -> i32 {
    match integrator.as_mut() {
        Some(integrator) => {
            integrator.reset();
            return 0;
        }
        None => return -1,
    }
}

pub(in crate::plugin) type ActuatorNew = unsafe extern "C" fn(
    init: f64,
    command_saturation_top: f64,
    command_saturation_bottom: f64,
    rate_saturation: f64,
    gain: f64,
) -> *mut Actuator;
pub(in crate::plugin) type FrPluginActuatorNewRegister = unsafe extern "C" fn(func: ActuatorNew);
pub(in crate::plugin) unsafe extern "C" fn actuator_new_callback(
    init: f64,
    command_saturation_top: f64,
    command_saturation_bottom: f64,
    rate_saturation: f64,
    gain: f64,
) -> *mut Actuator {
    let actuator = Box::new(Actuator::new(
        init,
        command_saturation_top,
        command_saturation_bottom,
        rate_saturation,
        gain,
    ));
    Box::into_raw(actuator)
}

pub(in crate::plugin) type ActuatorDrop = unsafe extern "C" fn(*mut Actuator);
pub(in crate::plugin) type FrPluginActuatorDropRegister = unsafe extern "C" fn(func: ActuatorDrop);
pub(in crate::plugin) unsafe extern "C" fn actuator_drop_callback(actuator: *mut Actuator) {
    if actuator.is_null() {
        return;
    }
    drop(Box::from_raw(actuator));
}

pub(in crate::plugin) type ActuatorUpdate =
    unsafe extern "C" fn(actuator: *mut Actuator, value: f64, t: f64, result: *mut f64) -> i32;
pub(in crate::plugin) type FrPluginActuatorUpdateRegister =
    unsafe extern "C" fn(func: ActuatorUpdate);
pub(in crate::plugin) unsafe extern "C" fn actuator_update_callback(
    actuator: *mut Actuator,
    value: f64,
    t: f64,
    result: *mut f64,
) -> i32 {
    match actuator.as_mut() {
        Some(actuator) => {
            *result = actuator.update(value, t);
            return 0;
        }
        None => return -1,
    }
}

pub(in crate::plugin) type ActuatorPast =
    unsafe extern "C" fn(actuator: *mut Actuator, result: *mut f64) -> i32;
pub(in crate::plugin) type FrPluginActuatorPastRegister = unsafe extern "C" fn(func: ActuatorPast);
pub(in crate::plugin) unsafe extern "C" fn actuator_past_callback(
    actuator: *mut Actuator,
    result: *mut f64,
) -> i32 {
    match actuator.as_mut() {
        Some(actuator) => {
            *result = actuator.past();
            return 0;
        }
        None => return -1,
    }
}

pub(in crate::plugin) type ActuatorReset = unsafe extern "C" fn(actuator: *mut Actuator) -> i32;
pub(in crate::plugin) type FrPluginActuatorResetRegister =
    unsafe extern "C" fn(func: ActuatorReset);
pub(in crate::plugin) unsafe extern "C" fn actuator_reset_callback(actuator: *mut Actuator) -> i32 {
    match actuator.as_mut() {
        Some(actuator) => {
            actuator.reset();
            return 0;
        }
        None => return -1,
    }
}
