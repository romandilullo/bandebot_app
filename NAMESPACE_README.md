# Namespace Configuration for Bandebot Launch Files

This document explains how to use the namespace feature added to the Bandebot launch files.

## Overview

All main launch files now support namespace configuration to allow running multiple robots or organizing nodes under specific namespaces:

- `run_bandebot.launch.py` - Main bandebot application launcher
- `run_mulita.launch.py` - Mulita base system launcher  
- `run_sim.launch.py` - Simulation environment launcher
- `remote_control.launch.py` - Remote control GUI launcher

## Configuration Methods

### Method 1: Launch Arguments (Recommended)

Use the `namespace` launch argument to set a namespace at runtime:

```bash
# Launch with namespace "robot1"
ros2 launch bandebot_app run_bandebot.launch.py namespace:=robot1

# Launch simulation with namespace "sim_robot"
ros2 launch mulita run_sim.launch.py namespace:=sim_robot

# Launch remote control with namespace "control_station"
ros2 launch bandebot_gui remote_control.launch.py namespace:=control_station

# Launch without namespace (default behavior)
ros2 launch bandebot_app run_bandebot.launch.py namespace:=""
```

## Multi-Robot Setup Example

To run multiple robots simultaneously:

### Terminal 1 - Robot 1:
```bash
ros2 launch bandebot_app run_bandebot.launch.py namespace:=robot1
```

### Terminal 2 - Robot 2:
```bash  
ros2 launch bandebot_app run_bandebot.launch.py namespace:=robot2
```

### Terminal 3 - Control Station:
```bash
ros2 launch bandebot_gui remote_control.launch.py namespace:=control
```

## Topic and Service Names

When using namespaces, all topics and services will be prefixed with the namespace:

### Without Namespace:
- `/cmd_vel`
- `/bandebot/app_state`
- `/mulita/moving_state`
- `/joint_states`

### With Namespace "robot1":
- `/robot1/cmd_vel`
- `/robot1/bandebot/app_state`
- `/robot1/mulita/moving_state`
- `/robot1/joint_states`

## Configuration File Locations

The namespace configuration files are located at:

- **Bandebot App**: `src/bandebot_app/config/namespace_config.yaml`
- **Mulita**: `src/mulita/config/namespace_config.yaml`
- **Bandebot GUI**: `src/bandebot_gui/config/namespace_config.yaml`

## Troubleshooting

1. **Build errors**: Make sure to rebuild after editing launch files:
   ```bash
   colcon build --symlink-install --packages-select bandebot_app mulita bandebot_gui
   ```

2. **Topic communication issues**: Ensure all communicating nodes use the same namespace

3. **Config file not found**: Verify the YAML config files exist and have correct syntax

4. **Remapping topics**: When using different namespaces for different components, you may need to remap topics:
   ```bash
   ros2 launch bandebot_gui remote_control.launch.py namespace:=gui /gui/cmd_vel:=/robot1/cmd_vel
   ```

## Advanced Usage

### Custom Configuration Files:

```bash
# Use custom config file
ros2 launch bandebot_app run_bandebot.launch.py namespace_config:=/path/to/custom_config.yaml
```

### Mixed Namespace Setup:

```bash
# Base system with namespace
ros2 launch mulita run_mulita.launch.py namespace:=base_robot

# Application without namespace (global scope)
ros2 launch bandebot_app run_bandebot.launch.py namespace:=""
```

## Notes

- Empty string (`""`) for namespace means no namespace (default ROS2 behavior)
- Namespace configuration files are optional - launch arguments take precedence
- All included launch files will inherit the namespace parameter
- The namespace parameter is passed to all child launch files automatically