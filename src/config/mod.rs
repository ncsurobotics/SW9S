use crate::logging::Result;
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
    pub fn new() -> Result<Self> {
        let text = std::fs::read_to_string("config.toml")?;
        Ok(toml::from_str(&text)?)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {}
    }
}
