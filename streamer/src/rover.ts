const urlSearchParams = new URLSearchParams(window.location.search);
const websocketPath =
    "ws://" +
    urlSearchParams.get("signaler") +
    ":" +
    urlSearchParams.get("port") +
    "/" +
    urlSearchParams.get("id");

const socket = new WebSocket(websocketPath);
let pc = new RTCPeerConnection({ iceServers: [] });
let makingOffer = false;

const codecs = RTCRtpSender.getCapabilities("video").codecs.filter(
    (codec) => codec.mimeType == "video/VP9" || codec.mimeType == "video/AV1"
);

if (codecs.find((codec) => codec.mimeType == "video/AV1") == undefined) {
    console.warn("WARNING: AV1 codec not found, using VP9 instead.");
}

codecs.sort((a, b) => {
    return a.mimeType == "video/AV1" ? -1 : 1;
});

console.log(`Codecs: ${codecs.map((codec) => codec.mimeType).join(", ")}`);

navigator.mediaDevices.enumerateDevices().then((devices) => {
    devices.forEach((device) => {
        console.log(`Device: ${device.kind} ${device.label}`);
    });
});

setup();

function setup() {
    setupLogListeners();
    addMedia();

    pc.onicecandidate = (event) => {
        if (event.candidate) {
            socket.send(
                JSON.stringify({
                    type: "candidate",
                    candidate: event.candidate,
                })
            );
        }
    };

    pc.onnegotiationneeded = async () => {
        renegotiate();
    };

    socket.onmessage = (ev) => {
        onMessage(JSON.parse(ev.data));
    };

    socket.onclose = () => {
        console.log("WebSocket closed");
    };

    socket.onerror = () => {
        console.error(`WebSocket error`);
    };
}

async function onMessage(message: any) {
    switch (message.type) {
        case "candidate": {
            console.log("Got candidate");
            await pc.addIceCandidate(message.candidate);
            break;
        }
        case "close": {
            // Recreate the peer connection
            console.log("Closing and setting up new connection");

            pc = new RTCPeerConnection({ iceServers: [] });
            setup();
        }
        default: {
            // As the polite peer, we always accept an offer, even during a collision

            await pc.setRemoteDescription(message);

            if (message.type == "offer") {
                console.log("Got offer");

                await pc.setLocalDescription();
                socket.send(JSON.stringify(pc.localDescription));
            } else if (message.type == "answer") {
                console.log("Got answer");
            } else {
                console.log(`Unknown message type: ${message.type}`);
            }
        }
    }
}

async function renegotiate() {
    console.log("Negotiating needed");

    try {
        makingOffer = true;

        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        socket.send(JSON.stringify(offer));
    } catch (err) {
        console.error(`${err}`);
    } finally {
        makingOffer = false;
    }
}

async function addMedia() {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({
            video: true,
            audio: false,
        });

        stream.getTracks().forEach((track) => {
            pc.addTrack(track, stream);
        });

        const transceiver = pc.getTransceivers()[0];
        transceiver.direction = "sendonly";
        transceiver.setCodecPreferences(codecs);

        const params = transceiver.sender.getParameters();

        params.encodings.forEach((encoding) => {
            encoding.maxBitrate = 64000;
            encoding.maxFramerate = 15;
        });

        transceiver.sender.setParameters(params);
    } catch (error) {
        console.error(`Error adding media: ${error}`);
    }
}

function setupLogListeners() {
    pc.onsignalingstatechange = () => {
        console.log(`Signaling state: ${pc.signalingState}`);
    };

    pc.oniceconnectionstatechange = () => {
        console.log(`ICE connection state: ${pc.iceConnectionState}`);
    };

    pc.onicegatheringstatechange = () => {
        console.log(`ICE gathering state: ${pc.iceGatheringState}`);
    };

    pc.onconnectionstatechange = () => {
        console.log(`Connection state: ${pc.connectionState}`);
    };

    pc.onicecandidateerror = (event) => {
        console.error(
            `ICE candidate error ${event.errorCode}: ${event.errorText}`
        );
    };
}
