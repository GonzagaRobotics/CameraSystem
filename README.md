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

The signaling server has its own dependencies that need to be installed.

```bash
# Run this in the root directory of the repository
(cd src/signaling/ && npm install)
```

## Running

**The order of this process is important.**

**Make sure port 8080 is open.**

First, start the signaling server.

```bash
# Run this in the root directory of the repository
node src/signaling/
```

Second, start the camera system. Do this in a different terminal.

**Ensure that there are no Chromium processes already running.**

```bash
# Run this in the root directory of the repository
chromium --use-fake-ui-for-media-stream src/streamer/index.html
```

If you look at the output of the signaling server, you should see `Connected: rover` appear.

## Usage

Once the signaling server and camera system are running, you can open the Control System and see the video feed from the rover.

See its README for more information.

**Always start the Control System last. You may restart the Control System and it will reconnect automatically.**

## Troubleshooting

If the video feed does not appear within a few seconds, there are any number of issues that could be causing it. The console output of the Control System and signaling server are the best places to learn more about what is going wrong.

The easiest way to fix an issue is to close all systems and restart them in the correct order.

A USB webcam must be plugged in for the video feed to work. There is currently no way to choose which camera to use.
