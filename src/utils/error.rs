use std::error::Error;

/// inner error which occurred in a extern plugin
#[derive(Debug)]
pub struct PluginInner {
    name: String,
    result: i32,
    reason: String,
}

impl PluginInner {
    pub(crate) fn new(name: &str, result: i32, reason: &str) -> Self {
        Self {
            name: name.to_string(),
            result,
            reason: reason.to_string(),
        }
    }
}

impl std::error::Error for PluginInner {}

impl std::fmt::Display for PluginInner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "plugin {} fail with code {}: {}",
            self.name, self.result, self.reason
        )
    }
}

/// fatal error which occured in fly_ruler_core
/// Model: error occured in extern model
#[derive(Debug)]
pub enum FatalCoreError {
    NotInit(String),
    Controller(String),
    Plugin(FatalPluginError),
    Nan,
}

impl FatalCoreError {}

impl std::error::Error for FatalCoreError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::NotInit(_) => None,
            Self::Controller(_) => None,
            Self::Plugin(e) => Some(e),
            Self::Nan => None,
        }
    }
}

impl std::fmt::Display for FatalCoreError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotInit(s) => write!(f, "{s} not init"),
            Self::Controller(e) => write!(f, "controller for plane {} not found", e),
            Self::Plugin(_) => write!(f, "{}", self.source().unwrap()),
            Self::Nan => write!(f, "NaN value"),
        }
    }
}

impl From<FatalPluginError> for FatalCoreError {
    fn from(value: FatalPluginError) -> Self {
        Self::Plugin(value)
    }
}

/// fatal error which occured in fly_ruler_plugin
/// Symbol: fail to find target Symbol in dll/so
/// Inner: error occured in extern plugin
#[derive(Debug)]
pub enum FatalPluginError {
    Symbol(String),
    Inner(PluginInner),
}

impl FatalPluginError {
    pub fn symbol(msg: String) -> Self {
        Self::Symbol(msg)
    }

    pub fn inner(name: &str, result: i32, reason: &str) -> Self {
        Self::Inner(PluginInner::new(name, result, reason))
    }
}

impl std::error::Error for FatalPluginError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        if let Self::Inner(inner) = self {
            Some(inner)
        } else {
            None
        }
    }
}

impl std::fmt::Display for FatalPluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Symbol(msg) => write!(f, "(Symbol) {}", msg),
            Self::Inner(_) => write!(f, "(Inner) {}", self.source().unwrap()),
        }
    }
}
