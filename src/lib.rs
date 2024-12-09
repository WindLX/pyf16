pub mod binding;
pub mod block;
pub mod components;
pub mod model;
pub mod optimizer;
pub mod plugin;
pub mod solver;
pub mod trim;
pub mod utils;

use pyo3::prelude::*;

/// A Python module implemented in Rust. The name of this function must match
/// the `lib.name` setting in the `Cargo.toml`, else Python will not be able to
/// import the module.
#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // env_logger::init();
    pyo3_log::init();
    binding::register(m)?;
    Ok(())
}
