# Check if node_modules exists in the signaler directory
if [ ! -d "signaler/node_modules" ]; then
    echo "Installing signaler dependencies"

    cd signaler
    npm install
    cd ..
fi

# Check if node_modules exists in the streamer directory
if [ ! -d "streamer/node_modules" ]; then
    echo "Installing streamer dependencies"

    cd streamer
    npm install
    cd ..
fi

# Build the signaler and streamer
(cd signaler && tsc) & (cd streamer && tsc)