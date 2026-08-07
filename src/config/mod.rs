use std::path::Path;

use crate::logging::{Result, WrapErr};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {}

impl Config {
    /// Generates a populated [`Config`] from a `config.toml` file in the current directory.
    ///
    /// # Errors
    ///
    /// This function will return an error if `config.toml` does not exist.
    pub fn new(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).wrap_err("Failed to open config.toml")?;
        Ok(toml::from_str(&text).wrap_err("Failed to parse config.toml")?)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {}
    }
}
