/// Cross-compilation build script for Jetson Nano (aarch64-unknown-linux-gnu).
///
/// When building for the host architecture, this script does nothing.
/// When building for aarch64-unknown-linux-gnu, this script:
///   1. Downloads and unpacks the Jetson sysroot if it is missing.
///   2. Emits the env vars needed by downstream `-sys` crates (opencv, cuda, etc.).
///   3. Emits linker search paths and flags via cargo directives.
///
/// Linker / AR / rustflags for the target are set in `.cargo/config.toml`
/// because Cargo does not allow build scripts to override those values.
use std::{
    env,
    fmt::Write as FmtWrite,
    fs,
    io::{self, BufWriter},
    path::{Path, PathBuf},
    thread,
};

use anyhow::{anyhow, Context, Result};

// ---------------------------------------------------------------------------
// Config — mirrors config.toml used by the old jetson/ binary
// ---------------------------------------------------------------------------

#[derive(Debug, serde::Deserialize)]
struct Config {
    fetch_sysroot: Option<bool>,
    sysroot_url: String,
}

fn read_config() -> Result<Config> {
    // config.toml lives next to Cargo.toml (i.e. the workspace/package root).
    // CARGO_MANIFEST_DIR is set by Cargo before build.rs runs.
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let config_path = Path::new(&manifest_dir).join("sysroot-config.toml");
    let raw = fs::read_to_string(&config_path)
        .with_context(|| format!("Could not read {}", config_path.display()))?;
    toml::from_str(&raw).context("Failed to parse sysroot-config.toml")
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let target = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let target_triple = env::var("TARGET").unwrap_or_default();

    // Only do anything special when cross-compiling for the Jetson.
    if target_triple != "aarch64-unknown-linux-gnu" {
        return Ok(());
    }

    // Re-run if sysroot-config.toml changes.
    println!("cargo:rerun-if-changed=sysroot-config.toml");
    // Re-run if the sysroot disappears.
    println!("cargo:rerun-if-changed=../sysroot-jetson");

    let config = read_config()?;

    // Resolve the sysroot directory.
    // Convention (same as the old script): sysroot lives at ../sysroot-jetson
    // relative to the package manifest.
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let package_root = PathBuf::from(&manifest_dir);
    let workspace_root = package_root
        .parent()
        .ok_or_else(|| anyhow!("Package root has no parent"))?
        .canonicalize()
        .context("Failed to canonicalize workspace root")?;
    let sysroot = workspace_root.join("sysroot-jetson");

    ensure_sysroot(&sysroot, &config)?;

    let sysroot_str = sysroot
        .to_str()
        .ok_or_else(|| anyhow!("Sysroot path is not valid UTF-8"))?;

    // -----------------------------------------------------------------------
    // Flags shared across C, C++, and the linker.
    // -----------------------------------------------------------------------
    let shared_flags = format!(
        "-target aarch64-linux-gnu \
         -mcpu=cortex-a57 \
         -fuse-ld=lld \
         --sysroot={sysroot} \
         -L{sysroot}/usr/local/cuda-10.2/targets/aarch64-linux/lib/ \
         -L{sysroot}/usr/lib/aarch64-linux-gnu/",
        sysroot = sysroot_str,
    );

    // -----------------------------------------------------------------------
    // Env vars consumed by `cc` / `cmake` crates used by `-sys` dependencies.
    //
    // build.rs can export env vars to dependent crates via
    //   cargo:rustc-env=KEY=VALUE  (visible to rustc / the compiled crate)
    // but for env vars that the *build scripts of dependencies* need to see we
    // use the OUT_DIR-based approach: Cargo propagates DEP_* vars, but the
    // cleanest supported way to set env vars for downstream build scripts is
    // simply to emit them here so they appear in the build environment via
    // the `config.toml` `[env]` table — OR we rely on the fact that Cargo
    // re-exports `cargo:rustc-env` into the compile environment.
    //
    // For CC/CXX/CFLAGS that upstream `-sys` crate build scripts read, we
    // emit them as `cargo:rustc-env` so they are visible during compilation,
    // AND we document that users should also set them in `.cargo/config.toml`
    // [env] if a dep's *build.rs* needs them (build scripts don't inherit
    // cargo:rustc-env from their dependents' build scripts).
    // -----------------------------------------------------------------------

    // Tell cc-rs / cmake-rs which compiler to use for the target triple.
    println!("cargo:rustc-env=CC_aarch64_unknown_linux_gnu=clang");
    println!("cargo:rustc-env=CXX_aarch64_unknown_linux_gnu=clang++");
    println!("cargo:rustc-env=AR_aarch64_unknown_linux_gnu=llvm-ar");
    println!(
        "cargo:rustc-env=CFLAGS_aarch64_unknown_linux_gnu={}",
        shared_flags
    );
    println!(
        "cargo:rustc-env=CXXFLAGS_aarch64_unknown_linux_gnu={}",
        shared_flags
    );
    println!(
        "cargo:rustc-env=LDFLAGS_aarch64_unknown_linux_gnu={}",
        shared_flags
    );

    // -----------------------------------------------------------------------
    // OpenCV env vars (consumed by the opencv-sys / opencv crate).
    // -----------------------------------------------------------------------
    configure_opencv(&sysroot, sysroot_str)?;

    // -----------------------------------------------------------------------
    // Linker search paths — tell rustc where to find .so files at link time.
    // -----------------------------------------------------------------------
    println!(
        "cargo:rustc-link-search=native={}/usr/lib/aarch64-linux-gnu",
        sysroot_str
    );
    println!(
        "cargo:rustc-link-search=native={}/usr/local/cuda-10.2/targets/aarch64-linux/lib",
        sysroot_str
    );

    Ok(())
}

// ---------------------------------------------------------------------------
// Sysroot download
// ---------------------------------------------------------------------------

fn ensure_sysroot(sysroot: &Path, config: &Config) -> Result<()> {
    let sysroot_missing = !sysroot.exists();

    let should_fetch = match config.fetch_sysroot {
        Some(v) => v && sysroot_missing,
        None => sysroot_missing,
    };

    if should_fetch {
        eprintln!("cargo:warning=Sysroot not found — downloading from {}", config.sysroot_url);
        download_sysroot(sysroot, &config.sysroot_url)?;
        eprintln!("cargo:warning=Sysroot downloaded successfully");
    } else if sysroot_missing {
        return Err(anyhow!(
            "Sysroot not found at {} and fetching is disabled. \
             Set `fetch_sysroot = true` in config.toml or place the sysroot manually.",
            sysroot.display()
        ));
    } else {
        eprintln!("cargo:warning=Found existing sysroot at {}", sysroot.display());
    }

    Ok(())
}

fn download_sysroot(dest: &Path, url: &str) -> Result<()> {
    use reqwest::blocking::get;
    use tar::Archive;
    use xz::read::XzDecoder;

    eprintln!("cargo:warning=Downloading sysroot (this may take a while)...");

    let response = get(url).with_context(|| format!("Failed to GET {url}"))?;

    if !response.status().is_success() {
        return Err(anyhow!(
            "HTTP {} when downloading sysroot from {url}",
            response.status()
        ));
    }

    // Pipe: HTTP response -> XZ decoder -> tar unpacker.
    // Run in a dedicated thread to keep stack usage predictable.
    let dest = dest.to_path_buf();
    let reader = io::BufReader::new(response);
    let decoded = XzDecoder::new_multi_decoder(reader);

    thread::spawn(move || -> Result<()> {
        Archive::new(decoded)
            .unpack(&dest)
            .with_context(|| format!("Failed to unpack tarball into {}", dest.display()))
    })
    .join()
    .map_err(|_| anyhow!("Sysroot download thread panicked"))??;

    Ok(())
}

// ---------------------------------------------------------------------------
// OpenCV configuration
// ---------------------------------------------------------------------------

fn configure_opencv(sysroot: &Path, sysroot_str: &str) -> Result<()> {
    // Disable all automatic probes — we supply everything manually.
    println!("cargo:rustc-env=OPENCV_DISABLE_PROBES=pkg_config,cmake,vcpkg_cmake,vcpkg");

    // Collect every shared library in the aarch64 lib dir so opencv-sys knows
    // what to link against (same logic as the old script).
    let lib_dir = sysroot.join("usr/lib/aarch64-linux-gnu");
    let link_libs: String = std::fs::read_dir(&lib_dir)
        .with_context(|| format!("Failed to read {}", lib_dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().to_string())
        .filter(|name| name.ends_with(".so") && name.starts_with("lib"))
        .map(|name| ",".to_string() + &name[3..name.len() - 3]) // strip lib / .so
        .collect();

    println!("cargo:rustc-env=OPENCV_LINK_LIBS={}", link_libs);
    println!(
        "cargo:rustc-env=OPENCV_LINK_PATHS={}/usr/lib/aarch64-linux-gnu",
        sysroot_str
    );
    println!(
        "cargo:rustc-env=OPENCV_INCLUDE_PATHS={sysroot}/usr/include/opencv4,\
         {sysroot}/usr/include/opencv4/opencv2",
        sysroot = sysroot_str
    );

    Ok(())
}
