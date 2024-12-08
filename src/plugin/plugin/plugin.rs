use super::ffi::{
    actuator_drop_callback, actuator_new_callback, actuator_past_callback, actuator_reset_callback,
    actuator_update_callback, atmos_callback, integrator_drop_callback, integrator_new_callback,
    integrator_past_callback, integrator_reset_callback, integrator_update_callback,
    logger_callback, FrPluginActuatorDropRegister, FrPluginActuatorNewRegister,
    FrPluginActuatorPastRegister, FrPluginActuatorResetRegister, FrPluginActuatorUpdateRegister,
    FrPluginAtmosFuncRegister, FrPluginHook, FrPluginIntegratorDropRegister,
    FrPluginIntegratorNewRegister, FrPluginIntegratorPastRegister, FrPluginIntegratorResetRegister,
    FrPluginIntegratorUpdateRegister, FrPluginLogRegister,
};
use crate::utils::error::FatalPluginError;
use libc::{c_char, c_int};
use libloading::Library;
use log::trace;
use serde::{Deserialize, Serialize};
use std::{ffi::CString, fs::read_to_string, path::Path};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PluginInfo {
    pub name: String,
    pub author: String,
    pub version: String,
    pub description: String,
}

impl PluginInfo {
    pub fn load<P: AsRef<Path>>(path: P) -> Result<PluginInfo, PluginError> {
        let content = read_to_string(path).map_err(|e| PluginError::Io(e))?;
        toml::from_str(&content).map_err(|e| PluginError::Info(e.message().to_string()))
    }
}

impl std::fmt::Display for PluginInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Name: {}", self.name)?;
        writeln!(f, "Author: {}", self.author)?;
        writeln!(f, "Version: {}", self.version)?;
        writeln!(f, "Description: {}", self.description)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PluginState {
    Enable,
    Disable,
    Failed,
}

impl Default for PluginState {
    fn default() -> Self {
        Self::Disable
    }
}

#[derive(Debug)]
pub struct Plugin {
    info: PluginInfo,
    lib: Library,
    state: PluginState,
}

impl Plugin {
    pub fn new<P: AsRef<Path>>(path: P) -> Result<Self, PluginError> {
        let p = path.as_ref().to_path_buf();
        let info = PluginInfo::load(p.join("info.toml"))?;
        trace!("plugin info loaded: {info:?}");
        let lib_path = if cfg!(target_os = "windows") {
            p.join(info.name.clone() + ".dll")
        } else if cfg!(target_os = "linux") {
            p.join(format!("lib{}.so", info.name))
        } else {
            return Err(PluginError::UnknownPlatform);
        };
        unsafe {
            let lib = Library::new(&lib_path).map_err(|e| PluginError::Lib(e))?;
            trace!("plugin lib loaded");
            Ok(Self {
                info,
                lib,
                state: PluginState::default(),
            })
        }
    }

    fn load_function<F>(&self, name: &str) -> Result<libloading::Symbol<'_, F>, PluginError> {
        unsafe {
            let f: libloading::Symbol<'_, F> = self.lib.get(name.as_bytes()).map_err(|e| {
                PluginError::Symbol(self.info.name.to_string(), name.to_string(), e)
            })?;
            trace!("function [{name}] load successfully");
            Ok(f)
        }
    }

    fn register_utils(&self) -> Result<(), PluginError> {
        let r = self.load_function::<FrPluginLogRegister>("frplugin_register_logger")?;
        unsafe {
            r(logger_callback);
        }

        let r = self.load_function::<FrPluginAtmosFuncRegister>("frplugin_register_atmos")?;
        unsafe {
            r(atmos_callback);
        }

        let r = self
            .load_function::<FrPluginIntegratorNewRegister>("frplugin_register_integrator_new")?;
        unsafe {
            r(integrator_new_callback);
        }
        let r = self
            .load_function::<FrPluginIntegratorDropRegister>("frplugin_register_integrator_drop")?;
        unsafe {
            r(integrator_drop_callback);
        }
        let r = self.load_function::<FrPluginIntegratorUpdateRegister>(
            "frplugin_register_integrator_update",
        )?;
        unsafe {
            r(integrator_update_callback);
        }
        let r = self
            .load_function::<FrPluginIntegratorPastRegister>("frplugin_register_integrator_past")?;
        unsafe {
            r(integrator_past_callback);
        }
        let r = self.load_function::<FrPluginIntegratorResetRegister>(
            "frplugin_register_integrator_reset",
        )?;
        unsafe {
            r(integrator_reset_callback);
        }

        let r =
            self.load_function::<FrPluginActuatorNewRegister>("frplugin_register_actuator_new")?;
        unsafe {
            r(actuator_new_callback);
        }
        let r =
            self.load_function::<FrPluginActuatorDropRegister>("frplugin_register_actuator_drop")?;
        unsafe {
            r(actuator_drop_callback);
        }
        let r = self
            .load_function::<FrPluginActuatorUpdateRegister>("frplugin_register_actuator_update")?;
        unsafe {
            r(actuator_update_callback);
        }
        let r =
            self.load_function::<FrPluginActuatorPastRegister>("frplugin_register_actuator_past")?;
        unsafe {
            r(actuator_past_callback);
        }
        let r = self
            .load_function::<FrPluginActuatorResetRegister>("frplugin_register_actuator_reset")?;
        unsafe {
            r(actuator_reset_callback);
        }

        Ok(())
    }

    fn call_hook_function(
        &self,
        name: &str,
        args: &[impl ToString],
    ) -> Result<Result<(), PluginError>, FatalPluginError> {
        let r = self.load_function::<FrPluginHook>(name);
        match r {
            Ok(r) => {
                let argc = args.len();
                let arg_string: Result<Vec<CString>, PluginError> = args
                    .iter()
                    .map(|s| {
                        trace!(
                            "call hook {hook_name} with arg: {arg}",
                            hook_name = name,
                            arg = s.to_string()
                        );
                        CString::new(s.to_string()).map_err(|_| {
                            PluginError::Args(self.info.name.to_string(), name.to_string())
                        })
                    })
                    .collect();

                let args: Vec<CString> = match arg_string {
                    Ok(args) => args.clone(),
                    Err(e) => return Ok(Err(e)),
                };

                let args: Vec<*const c_char> = args.iter().map(|s| s.as_ptr()).collect();
                let argv = args.as_ptr();
                unsafe {
                    let res = r(argc as c_int, argv as *const *const c_char);
                    if res < 0 {
                        return Err(FatalPluginError::inner(
                            &self.info.name,
                            res,
                            format!("when call {name}").as_str(),
                        ));
                    } else {
                        return Ok(Ok(()));
                    }
                }
            }
            Err(e) => return Ok(Err(e)),
        }
    }

    pub fn install(
        &self,
        args: &[impl ToString],
    ) -> Result<Result<(), PluginError>, FatalPluginError> {
        match self.register_utils() {
            Ok(_) => {
                trace!("registered all callbacks");
            }
            Err(e) => {
                trace!("{}", e);
            }
        }
        self.call_hook_function("frplugin_install_hook", args)
    }

    pub fn uninstall(&self) -> Result<Result<(), PluginError>, FatalPluginError> {
        self.call_hook_function("frplugin_uninstall_hook", &Vec::<String>::new())
    }
}

pub trait AsPlugin {
    fn plugin(&self) -> &Plugin;

    fn plugin_mut(&mut self) -> &mut Plugin;

    fn info(&self) -> PluginInfo {
        self.plugin().info.clone()
    }

    fn state(&self) -> PluginState {
        self.plugin().state
    }

    fn set_state(&mut self, state: PluginState) {
        self.plugin_mut().state = state;
    }

    fn load_function<F>(&self, name: &str) -> Result<libloading::Symbol<'_, F>, PluginError> {
        self.plugin().load_function::<F>(name)
    }
}

impl AsPlugin for Plugin {
    fn plugin(&self) -> &Plugin {
        self
    }

    fn plugin_mut(&mut self) -> &mut Plugin {
        self
    }
}

impl AsPlugin for Box<Plugin> {
    fn plugin(&self) -> &Plugin {
        self
    }

    fn plugin_mut(&mut self) -> &mut Plugin {
        self
    }
}

#[derive(Debug)]
pub enum PluginError {
    UnknownPlatform,
    Io(std::io::Error),
    Info(String),
    Lib(libloading::Error),
    Symbol(String, String, libloading::Error),
    Args(String, String),
}

impl std::error::Error for PluginError {}

impl std::fmt::Display for PluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PluginError::UnknownPlatform => write!(f, "unknown platform"),
            PluginError::Io(s) => write!(f, "fail to load plugin: {}", s),
            PluginError::Info(s) => write!(f, "fail to parse plugin info: {}", s),
            PluginError::Lib(s) => write!(f, "fail to load plugin library: {}", s),
            PluginError::Symbol(name, symbol, s) => {
                write!(
                    f,
                    "fail to load symbol {} in plugin: {} due to {}",
                    symbol, name, s
                )
            }
            PluginError::Args(name, s) => {
                write!(f, "invalid args when call symbol {} in plugin {}", s, name)
            }
        }
    }
}
