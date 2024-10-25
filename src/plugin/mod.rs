pub(crate) mod model;
pub(crate) mod plugin;

pub use model::{
    delete_handler_constructor, init_handler_constructor, step_handler_constructor,
    trim_handler_constructor, AerodynamicModel, AerodynamicModelDeleteFn, AerodynamicModelInitFn,
    AerodynamicModelStepFn, AerodynamicModelTrimFn,
};
pub use plugin::{AsPlugin, PluginError, PluginInfo, PluginState};
