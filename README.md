# Camera System

Version: 0.2.0

Code name: N/A

## Description

Allows taking of pictures and video from the various cameras on the rover and sending them to wherever they need to go, such as the Control System.

Currently has basic live video streaming capabilities to the Control System.

## Dependencies

-   Modern version of Chromium
-   Node.js v22
-   NPM v11

## Building

Each part has its own npm dependencies that need to be installed.

```bash
# Run this in the root directory of the repository
(cd src/signaling/ && npm install && cd ../streamer/ && npm install)
```

## Running

**The order of this process is important.**

**Make sure port 8080 is open.**

Use the shell script to start the system. Make sure it has execute permissions.

```bash
# Run this in the root directory of the repository
./start.sh $CHROMIUM_PATH$

# Example using snap on linux
./start.sh /snap/bin/chromium
```

## Usage

Once the signaling server and camera system are running, you can open the Control System and see the video feed from the rover.

See its README for more information.

**Always start the Control System last. You may restart the Control System and it will reconnect automatically.**

## Troubleshooting

If the video feed does not appear within a few seconds, there are any number of issues that could be causing it. The console output of the Control System and signaling server are the best places to learn more about what is going wrong.

The easiest way to fix an issue is to close all systems and restart them in the correct order.

A USB webcam must be plugged in for the video feed to work. There is currently no way to choose which camera to use.

If you're having trouble using the snap version of chromium-browser, consider using flatpak. Check to ensure flatpak Chromium is installed and this is the correct path before using.
```bash
./start.sh /var/lib/flatpak/app/org.chromium.Chromium/aarch64/stable/active/export/bin/org.chromium.Chromium
```
If you get the error `BROWSER: JSHandle@error` in your terminal output, you may have incorrectly set permissions on the camera file. Run the following line of code, replacing `/dev/video#` with the actual device file path of your camera (probably `/dev/video0`).
```bash 
sudo chmod 666 /dev/video#
```
