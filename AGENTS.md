# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble workspace. Packages live under `src/`: `bxi_slam_manager` coordinates runtime modes, `bxi_nav` contains Nav2 launch/configuration and maps, `bxi_nav_interfaces` defines messages and services, `Point-LIO` provides odometry/mapping, and `small_gicp_relocalization` handles relocalization. Livox integration is in `livox_ros_driver2`; avoid incidental edits to its `3rdparty/` tree or `Point-LIO/third_party/`. Package tests belong in each package's `test/` directory. Operational documentation is under `docs/`; root scripts install, start, and stop the system.

## Build, Test, and Development Commands

Run development commands on Ubuntu 22.04 with ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

Use `colcon build --packages-select nav` (or another package name) for focused builds. Start a local stack with `./start.sh` only when the production service is stopped; use `./stop.sh` afterward. `sudo ./install.sh` deploys into `/opt` and is not a routine development command.

## Coding Style & Naming Conventions

Match nearby code. Python uses four spaces, `snake_case` functions/modules, `PascalCase` classes, type hints, and `test_*.py` tests. C++ uses two-space continuation/style consistent with ROS 2, `PascalCase` types, and `camelCase` functions. Keep ROS package, topic, parameter, and YAML keys in `snake_case`. Run package-provided ament linters; `small_gicp_relocalization` also configures clang-format, clang-tidy, Black, and XML checks.

## Testing Guidelines

Add the smallest regression test beside the affected package. Tests use pytest through `ament_cmake_pytest` or the Python package, plus ament lint checks. There is no numeric coverage threshold; changes must pass relevant package tests and report `colcon test-result --verbose` cleanly.

## Commit & Pull Request Guidelines

History favors concise, imperative Conventional Commit subjects such as `fix(nav): improve recovery and obstacle safety` and `feat: align SLAM maps and Nav2 navigation`. Keep commits focused. Pull requests should explain behavior and risk, list tested commands/hardware, link issues, and include RViz screenshots or logs when navigation or mapping output changes.

## Deployment Safety

For builds on `192.168.88.162`, use only `/home/bxi/bxi_rc_build_tmp`: clean it before building, remove it afterward, and verify removal. Never modify remote `/opt` unless deployment was explicitly requested.
