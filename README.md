# Camera System

Version: 0.3.0

Code name: N/A

## Description

Allows streaming video from webcams to the Control System over WebRTC.

## Dependencies

-   Modern version of Chromium
-   Node.js v22
-   Typescript compiler (run `npm install -g typescript`)

## Building

Run `./build.sh` to install dependencies and build the system.

## Running

**IMPORTANT**

Always run the signaling server before a camera source. There can only be one signaling server running, while there can be multiple camera sources.

Use the `-s` flag to see the arguments for each launch file.

```bash
# Run just the signaling server
ros2 launch launch/signaler.launch.py

# Run a camera source
# The chromium_path argument is not optional
ros2 launch launch/source.launch.py chromium_path:=/path/to/chromium

# Run both the signaling server and a camera source
ros2 launch launch/combined.launch.py chromium_path:=/path/to/chromium
```

## Usage with the Control System

Once the signaling server and camera system are running, you can open the Control System and see the video feed from the rover.

See its README for more information.

## Troubleshooting

If the video feed does not appear within a few seconds, there are any number of issues that could be causing it. The console output of the Control System and signaling server are the best places to learn more about what is going wrong.

The easiest way to fix an issue is to close all systems and restart them.

If you're having trouble using the snap version of chromium-browser, consider using flatpak. Check to ensure flatpak Chromium is installed and this is the correct path before using.
```bash
./start.sh /var/lib/flatpak/app/org.chromium.Chromium/aarch64/stable/active/export/bin/org.chromium.Chromium
```
If you get the error `BROWSER: JSHandle@error` in your terminal output, you may have incorrectly set permissions on the camera file. Run the following line of code, replacing `/dev/video#` with the actual device file path of your camera (probably `/dev/video0`).
```bash 
sudo chmod 666 /dev/video#
```
