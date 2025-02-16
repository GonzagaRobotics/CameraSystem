import puppeteer from "puppeteer-core";

const executablePath = process.argv[2];

const browser = await puppeteer.launch({
    args: ["--use-fake-ui-for-media-stream", "--no-sandbox"],
    headless: true,
    executablePath: executablePath,
});

const page = await browser.newPage();

page.on("console", (message) => {
    console.log("BROWSER: " + message.text());
});

await page.goto(`file://${process.cwd()}/src/streamer/rover.html`);
