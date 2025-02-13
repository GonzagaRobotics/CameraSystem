import { WebSocketServer } from "ws";

const port = 8080;
const wss = new WebSocketServer({ port });

const clients = {};

wss.on("connection", (ws, req) => {
    const id = req.url.slice(1);

    if (!id) {
        console.error("Client id is required");
        ws.close();
        return;
    }

    if (clients[id]) {
        console.error(`Client with id ${id} already exists`);
        ws.close();
        return;
    }

    ws.on("error", console.error);

    ws.on("close", () => {
        delete clients[id];
        console.log("Disconnected: " + id);
    });

    ws.on("message", (data) => {
        // Broadcast message to all other clients
        for (const client of Object.values(clients)) {
            if (client !== ws) {
                client.send(data, { binary: false });
            }
        }
    });

    clients[id] = ws;

    console.log("Connected: " + id);
});

wss.on("error", console.error);

wss.on("listening", () => {
    console.log("Server is running on port " + port);
});
