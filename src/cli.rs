pub use clap::Parser;

#[derive(Parser, Debug)]
#[clap(author = "AquaPack Robotics", version, about)]
pub struct Args {
    pub missions: Vec<String>,
}
