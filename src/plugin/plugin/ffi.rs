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
