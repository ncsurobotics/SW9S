//! Defines command-line arguments for mission execution and configuration.

/// Command-line parsing traits and derive macros used by the application.
pub use clap::{Args, Parser, Subcommand};
use std::path::PathBuf;

/// Command-line options for the SeaWolf 9 application.
#[derive(Parser, Debug)]
#[clap(author = "AquaPack Robotics", version, about)]
pub struct Cli {
    /// Command to execute.
    #[command(subcommand)]
    pub subcmd: Subcmd,
}

/// Available application commands.
#[derive(Subcommand, Debug)]
pub enum Subcmd {
    /// Run missions
    Run(RunArgs),
    /// Manage config file
    #[command(subcommand)]
    Cfg(CfgSubcmd),
}

/// Commands for validating or generating configuration files.
#[derive(Subcommand, Debug)]
pub enum CfgSubcmd {
    /// Validate configuration file
    Check {
        /// Path to the config file
        #[arg(default_value = "config.toml")]
        file: PathBuf,
    },
    /// Generate the default configuration
    Generate {
        /// Path to the config file
        #[arg(default_value = "config.toml")]
        file: PathBuf,
        /// Overwrite the existing config file if it exists
        #[arg(short, long)]
        force: bool,
    },
}

/// Configuration path and mission names for a run.
#[derive(Args, Debug)]
pub struct RunArgs {
    /// Path to the config file
    #[arg(short, long, default_value = "config.toml")]
    pub config: PathBuf,
    /// List of missions to run
    pub missions: Vec<String>,
}
