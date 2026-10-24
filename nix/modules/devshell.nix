{ inputs, ... }:
{
  perSystem = { config, self', pkgs, lib, ... }:
    let
      jetson = pkgs.pkgsCross.aarch64-multiplatform-musl;
    in
    {
      packages = lib.optionalAttrs pkgs.stdenv.isLinux {
        docker = pkgs.dockerTools.buildNixShellImage {
          name = "ghcr.io/ncsurobotics/sw9s-dev";
          tag = "latest";
          drv = config.devShells.default.overrideAttrs (old: {
            nativeBuildInputs = old.nativeBuildInputs ++ [ pkgs.git pkgs.openssh ];
          });
          shell = "${pkgs.bashInteractive}/bin/bash";
        };
      };

      devShells.default = pkgs.mkShell ({
        name = "sw9s-shell";
        inputsFrom = [
          self'.devShells.rust
          config.pre-commit.devShell # See ./nix/modules/pre-commit.nix
        ];
        packages = with pkgs; [
          just
          nixd # Nix language server
          bacon
        ] ++ lib.optionals pkgs.stdenv.isLinux [
          jetson.stdenv.cc
          jetson.openssl.dev
        ];
      } // lib.optionalAttrs pkgs.stdenv.isLinux {
        CC_aarch64_unknown_linux_musl = "${jetson.stdenv.cc}/bin/aarch64-unknown-linux-musl-gcc";
        CXX_aarch64_unknown_linux_musl = "${jetson.stdenv.cc}/bin/aarch64-unknown-linux-musl-g++";
        AR_aarch64_unknown_linux_musl = "${jetson.stdenv.cc}/bin/aarch64-unknown-linux-musl-ar";
        CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER = "${jetson.stdenv.cc}/bin/aarch64-unknown-linux-musl-gcc";
        PKG_CONFIG_ALLOW_CROSS = "1";
        PKG_CONFIG_PATH = "${jetson.openssl.dev}/lib/pkgconfig";
      });
    };
}
