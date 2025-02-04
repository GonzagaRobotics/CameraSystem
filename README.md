# Camera System

Version: 0.1.0

Code name: N/A

## Description

Allows taking of pictures and video from the various cameras on the rover and sending them to wherever they need to go, such as the Control System.

## Dependencies

-   libdatachannel v0.22

## Build and Run

Ensure that dependencies are either installed and sourced or are in the workspace.

To install libdatachannel, run the following command: `git clone https://github.com/paullouisageneau/libdatachannel.git deps/libdatachannel/ --branch="v0.22" --recurse-submodules`

```bash
colcon build --symlink-install
source install/local_setup.bash

ros2 launch launch/launch.py
```
