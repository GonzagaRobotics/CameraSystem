import { WebSocket, WebSocketServer } from "ws";

const port = parseInt(process.argv[2]) || 8080;
const wss = new WebSocketServer({ port });

/**
 * The sockets for clients that provide video streams to the control system.
 */
const sources = new Map<string, WebSocket>();

/**
 * The websocket for the control system.
 */
let controlSocket: WebSocket | null = null;

/**
 * The id of the peer that the control system is connected to.
 */
let connectedId: string | null = null;

function formatSources(): string {
    const sourceIds = Array.from(sources.keys());

    return JSON.stringify({
        type: "sources",
        sources: sourceIds,
    });
}

function closeCurrentSource() {
    if (connectedId) {
        sources.get(connectedId)!.send(
            JSON.stringify({
                type: "close",
            })
        );
    }
}

function handleControlMessage(message: any) {
    if (message.type == "sourceChange") {
        closeCurrentSource();
        connectedId = message.to;

        return;
    }

    if (!connectedId) {
        console.error(`No source connected`);

        return;
    }

    sources.get(connectedId)!.send(JSON.stringify(message));
}

function handleSourceMessage(from: string, message: any) {
    if (!connectedId || connectedId != from) {
        return;
    }

    controlSocket!.send(JSON.stringify(message));
}

wss.on("connection", (socket, req) => {
    const id = req.url?.slice(1);

    if (!id || id.length == 0) {
        console.error(`No id provided by ${req.socket.remoteAddress}`);

        socket.close();
        return;
    }

    if (id == "control") {
        if (controlSocket) {
            console.error(`Control system already connected`);

            socket.close();
            return;
        }

        controlSocket = socket;

        socket.send(formatSources());
    } else {
        if (sources.has(id)) {
            console.error(`${id} already connected`);

            socket.close();
            return;
        }

        sources.set(id, socket);
    }

    socket.onerror = (err) => {
        console.error(`${id} error: ${err}`);
    };

    socket.onmessage = (raw) => {
        const message = JSON.parse(raw.data.toString());

        if (id == "control") {
            handleControlMessage(message);
        } else {
            handleSourceMessage(id, message);
        }
    };

    socket.onclose = () => {
        if (id == "control") {
            closeCurrentSource();
            controlSocket = null;
            connectedId = null;
        } else {
            sources.delete(id);

            controlSocket?.send(formatSources());
        }

        console.log(`${id} disconnected`);
    };

    console.log(`${id} connected`);
});

wss.on("error", (err) => {
    console.error(`Error: ${err}`);
});

wss.on("listening", () => {
    console.log(`Listening on port ${port}`);
});
