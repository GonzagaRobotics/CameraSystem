import { launch } from "puppeteer-core";

const executablePath = process.argv[2];
const signalerPath = process.argv[3] || "localhost";
const signalerPort = process.argv[4] || "8080";
const sourceId = process.argv[5] || "source";

async function start() {
    const browser = await launch({
        args: ["--use-fake-ui-for-media-stream", "--no-sandbox"],
        headless: true,
        executablePath: executablePath,
    });

    const page = await browser.newPage();

    page.on("console", (message) => {
        console.log(message.text());
    });

    page.on("error", (error) => {
        console.error(error.message);
    });

    await page.goto(
        `file://${process.cwd()}/streamer/rover.html?signaler=${signalerPath}&port=${signalerPort}&id=${sourceId}`
    );
}

start();
