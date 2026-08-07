pub use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[clap(author = "AquaPack Robotics", version, about)]
pub struct Args {
    /// Path to the config file
    #[arg(short, long, default_value = "config.toml")]
    pub config: PathBuf,
    pub missions: Vec<String>,
}
