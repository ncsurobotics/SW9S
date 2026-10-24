# Development container

On Linux, the `packages.<system>.docker` flake output packages the default
development shell, including the Jetson musl cross compiler, as a Docker image.
It also includes Git and SSH for fetching dependencies and deploying.

Build and load it:

```sh
nix build --builders '' .#docker --out-link result-docker
docker load < result-docker
```

The archive uses the full registry tag `ghcr.io/ncsurobotics/sw9s-dev:latest`,
so loading a new build updates the tag used by the push command:

```sh
docker push ghcr.io/ncsurobotics/sw9s-dev:latest
```

If you loaded an older archive named `sw9s-dev:latest`, retag it before pushing:

```sh
docker tag sw9s-dev:latest ghcr.io/ncsurobotics/sw9s-dev:latest
```

Start the development shell with this checkout mounted:

```sh
docker run --rm -it -v "$PWD:/build" ghcr.io/ncsurobotics/sw9s-dev:latest
```

Inside the container, run `just build-jetson` to cross compile. Build artifacts
are written to the checkout's `target/` directory. The image runs as UID/GID
1000:1000 by default; the mounted checkout must be writable by that user.
The source code and local `config.toml` are supplied by the mount, rather than
included in the image. Nix is only needed to build the image on the host.
