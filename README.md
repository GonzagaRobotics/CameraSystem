# Camera System

Version: 0.1.0

Code name: N/A

## Description

Allows taking of pictures and video from the various cameras on the rover and sending them to wherever they need to go, such as the Control System.

## Dependencies

None

## Build and Run

Ensure that dependencies are either installed and sourced or are in the workspace.

```bash
colcon build --symlink-install
source install/local_setup.bash

ros2 launch launch/launch.py
```
