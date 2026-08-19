use std::{fs::OpenOptions, io::Write, path::Path};

use crate::logging::{info, Result, WrapErr};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Config {}

impl Config {
    /// Generates a populated [`Config`] from the provided config file
    ///
    /// # Errors
    ///
    /// This function will return an error if the config file does not exist or cannot be parsed
    pub fn new(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).wrap_err("Failed to open config.toml")?;
        toml::from_str(&text).wrap_err("Failed to parse config.toml")
    }
    /// Save the default config to the provided file
    ///
    /// # Errors
    ///
    /// This function will return an error if the file cannot be written
    pub fn save_default(path: &Path, force: bool) -> Result<()> {
        let cfg = Self::default();
        let cfg_str = toml::to_string_pretty(&cfg)?;
        let mut file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .create_new(!force)
            .open(path)
            .wrap_err_with(|| format!("Could not open {:#?} for writing", path))?;
        file.write_all(cfg_str.as_bytes())?;
        info!("Wrote default config to {:#?}", path);
        Ok(())
    }
    /// Check the provided config file
    ///
    /// # Errors
    ///
    /// This function will return an error if the file does not exist or cannot be parsed
    pub fn check(path: &Path) -> Result<()> {
        Self::new(path)?;
        info!("{:#?} contains a valid config", path);
        Ok(())
    }
}
