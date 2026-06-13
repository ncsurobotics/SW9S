# Zenoh configs

The vision client (`src/comms/zed_ros2.rs`) talks to ROS2 nodes running with
`rmw_zenoh` by subscribing to their Zenoh key expressions directly. It opens a
Zenoh session from the file named by `zed_ros2.zenoh_config` in `config.toml`;
when that key is unset it defaults to a client of the local router at
`tcp/localhost:7447`.

## Scenarios

### Remote access over Tailscale (`zenoh_remote.json5`)

The robot runs its ROS2 nodes with `rmw_zenoh` and a Zenoh router. A client on
the Tailscale VPN connects straight to that router:

1. Edit `zenoh_remote.json5` and replace `100.100.100.1` with the robot's
   Tailscale IP (`tailscale ip -4` on the robot).
2. Point the client at it in `config.toml`:

   ```toml
   [zed_ros2]
   zenoh_config = "config/zenoh_remote.json5"
   ```

The router must listen on a port reachable through Tailscale (the local router
config below listens on `0.0.0.0:7447`, which covers the Tailscale interface).

### Local network (`zenoh_router.json5`)

A single router on the robot is enough; every rmw_zenoh node and this client
connect to it. Run it with either:

```sh
ros2 run rmw_zenoh_cpp rmw_zenohd          # rmw_zenoh's bundled router
# or
zenohd -c /etc/zenoh/zenoh_router.json5    # standalone zenohd with this config
```

Leave `zenoh_config` unset in `config.toml`; the client then connects to
`tcp/localhost:7447`.

## Starting the router on boot

Copy `zenoh_router.json5` to `/etc/zenoh/zenoh_router.json5`, then create
`/etc/systemd/system/zenoh-router.service`:

```ini
[Unit]
Description=Zenoh router
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/bin/zenohd -c /etc/zenoh/zenoh_router.json5
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable it with:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-router.service
```

If the robot uses `rmw_zenohd` instead of standalone `zenohd`, swap the
`ExecStart` line for a wrapper that sources the ROS2 environment and runs
`ros2 run rmw_zenoh_cpp rmw_zenohd`.

Note: the Zenoh protocol version must match between this crate and the
router/rmw_zenoh build; all Zenoh 1.x releases interoperate.
