# Start the signaling server
node src/signaling/index.js &
SIGNALING_PID=$!

# Wait a second for it to start
sleep 1

# Start the streamer
node src/streamer/index.js $1 &
STREAMER_PID=$!

# Function to kill background processes
cleanup() {
  kill $SIGNALING_PID
  kill $STREAMER_PID
}

# Trap termination signals and call cleanup
trap cleanup EXIT

# Wait for background processes to finish
wait $SIGNALING_PID
wait $STREAMER_PID