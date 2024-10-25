pub mod algorithm;
pub mod binding;
pub mod block;
pub mod components;
pub mod model;
pub mod plugin;
pub mod trim;
pub mod utils;

use pyo3::prelude::*;

#[pymodule]
#[pyo3(name = "pyf16")]
fn extension(m: &Bound<'_, PyModule>) -> PyResult<()> {
    env_logger::init();
    binding::register(m)?;
    Ok(())
}
