import { createServer } from "http";
import { server } from "websocket";

const httpServer = createServer((req, res) => {
    console.log(`${req.method.toUpperCase()} ${req.url}`);

    const respond = (code, data, contentType = "text/plain") => {
        res.writeHead(code, {
            "Content-Type": contentType,
            "Access-Control-Allow-Origin": "*",
        });
        res.end(data);
    };

    respond(404, "Not Found");
});

const wsServer = new server({ httpServer });

wsServer.on("request", (req) => {
    console.log(`Connection from ${req.origin}`);

    const conn = req.accept(null, req.origin);

    conn.on("message", (data) => {
        if (data.type !== "utf8") return;

        console.log(`From ${req.origin} received: ${data.utf8Data}`);

        // Broadcast message to the other client
        wsServer.connections.forEach((connection) => {
            if (connection !== conn) {
                connection.send(data.utf8Data);
            }
        });
    });

    conn.on("close", () => {
        console.error(`${req.origin} disconnected`);
    });
});

const hostname = "127.0.0.1";
const port = 8000;

httpServer.listen(port, hostname, () => {
    console.log(`Signaling server is running on http://${hostname}:${port}`);
});
