default:
    @just --list

# Run pre-commit hooks on all files, including autoformatting
pre-commit-all:
    pre-commit run --all-files

# Run 'cargo run' on the project
run *ARGS:
    cargo run {{ARGS}}

# Cross-compile a self-contained musl release binary for Jetson Orin Nano
build-jetson:
    cargo build --locked --release --target aarch64-unknown-linux-musl

# Build and copy the release binary to the Jetson
deploy-jetson: build-jetson
    ssh -o ConnectTimeout=5 sw9@192.168.2.5 'mkdir -p ~/deploy && cat > ~/deploy/sw9s && chmod +x ~/deploy/sw9s' < target/aarch64-unknown-linux-musl/release/sw9s

# Run 'bacon' to run the project (auto-recompiles)
watch *ARGS:
	bacon --job run -- -- {{ ARGS }}
