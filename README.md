# go2-nav2-wifi

3D mapping and 2D navigation for **Unitree Go2 Edu** over Wi-Fi.
Built-in LiDAR, Point-LIO on the robot, RViz on the laptop.

[![Go2 mapping demo](docs/media/go2-3d-slam-demo.jpg)](https://dancher00.github.io/go2-nav2-wifi/)

[Watch demo](https://dancher00.github.io/go2-nav2-wifi/) · 37 seconds · 2× speed

## Start 3D mapping

Requires Ubuntu, Docker, SSH access to the Go2 and a shared Wi-Fi network.

```bash
git clone https://github.com/dancher00/go2-nav2-wifi.git
cd go2-nav2-wifi
export GO2_HOST_IP=YOUR_LAPTOP_IP
export GO2_ROBOT_IP=YOUR_ROBOT_IP
./setup-lidar3d.sh --laptop
./setup-lidar3d.sh --jetson
GO2_RECORD=1 ./mapping.sh --3d
```

Keep the robot still until the map appears, then drive with the handheld remote.
Close RViz to stop and save the map to `ws/maps/lidar3d/`.

[Full 3D setup](docs/3D-QUICKSTART.md) · [2D setup](docs/RELAY-WIFI.md) · [Navigation](docs/NAVIGATION.md)

## Other modes

```bash
./mapping.sh              # 2D mapping
./mapping.sh --view       # Live LiDAR and camera
./mapping.sh --3d --plan  # Plan preview, no motion
```

3D saved-map relocalization and loop closure are not supported yet.

[Earlier demos](docs/DEMOS.md) · [Tests](.github/workflows/ci.yml) · [Releases](https://github.com/dancher00/go2-nav2-wifi/releases) · [MIT license](LICENSE)
