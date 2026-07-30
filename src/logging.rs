pub use color_eyre::eyre::{bail, eyre, Result};
pub use tracing::{error, info, instrument, trace, warn};

/// Sets up logging frameworks
/// Uses color_eyre for error handling and tracing for general logging
/// See install_tracing for tracing details
pub fn install() -> Result<()> {
    color_eyre::install()?;
    install_tracing()?;
    Ok(())
}

/// Sets up tracing with stdout, tokio-console, and error layers
fn install_tracing() -> Result<()> {
    use tracing_subscriber::{prelude::*, EnvFilter};

    let format_filter =
        EnvFilter::try_from_default_env().or_else(|_| EnvFilter::try_new("info"))?;
    let format_layer = tracing_subscriber::fmt::layer()
        .with_target(true)
        .with_filter(format_filter);

    let console_layer = console_subscriber::ConsoleLayer::builder()
        .with_default_env()
        .spawn();

    tracing_subscriber::registry()
        .with(console_layer)
        .with(format_layer)
        .with(tracing_error::ErrorLayer::default())
        .init();

    Ok(())
}
