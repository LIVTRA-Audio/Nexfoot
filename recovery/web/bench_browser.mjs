/* SPDX-License-Identifier: Apache-2.0 */
// Local Windows bench automation. Starts an isolated Chrome profile and serves
// only the browser installer files on loopback. USB traffic uses real WebHID.
import { spawn } from "node:child_process";
import { mkdtemp, readFile } from "node:fs/promises";
import { createServer } from "node:http";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";

const pkg = resolve(process.argv[2]);
const servedFiles = new Map([
  ["/", { name: "index.html", type: "text/html" }],
  ["/nexfoot.js", { name: "nexfoot.js", type: "text/javascript" }],
]);
const server = createServer(async (req, res) => {
  const file = servedFiles.get(req.url);
  if (!file) return res.writeHead(404).end();
  try {
    const data = await readFile(new URL("browser/" + file.name, import.meta.url));
    res.writeHead(200, { "Content-Type": file.type }).end(data);
  } catch (error) {
    res.writeHead(500).end(error.message);
  }
});

await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
const url = "http://127.0.0.1:" + server.address().port;
const profile = await mkdtemp(join(tmpdir(), "nexfoot-web-bench-"));
const browser = spawn("C:/Program Files/Google/Chrome/Application/chrome.exe", [
  ...(process.argv.includes("--headed") ? [] : ["--headless=new"]), "--user-data-dir=" + profile, "--remote-debugging-port=0",
  "--remote-debugging-address=127.0.0.1", "--no-first-run", "--no-default-browser-check", "--disable-features=Translate", "--force-renderer-accessibility", url,
], { stdio: "ignore" });
let socket;
let seq = 0;
const responses = new Map();
const notification = new Map();

function send(method, params = {}) {
  const id = ++seq;
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      responses.delete(id);
      reject(new Error("CDP timed out for " + method));
    }, 12000);
    responses.set(id, message => {
      clearTimeout(timer);
      if (message.error) reject(new Error(message.error.message));
      else resolve(message.result);
    });
    socket.send(JSON.stringify({ id, method, params }));
  });
}

async function evaluate(expression) {
  const reply = await send("Runtime.evaluate", { expression, returnByValue: true });
  if (reply.exceptionDetails) throw new Error(reply.result.description);
  return reply.result.value;
}

try {
  let port;
  for (let attempt = 0; attempt < 100 && !port; attempt++) {
    const file = await readFile(join(profile, "DevToolsActivePort"), "utf8").catch(() => "");
    port = file.trim().split(/\s/)[0];
    if (!port) await delay(100);
  }
  if (!port) throw new Error("Chrome did not start.");
  const targets = await fetch("http://127.0.0.1:" + port + "/json/list").then(res => res.json());
  const page = targets.find(target => target.type === "page" && target.url.startsWith(url));
  if (!page) throw new Error("Installer tab missing.");
  socket = new WebSocket(page.webSocketDebuggerUrl);
  socket.addEventListener("message", event => {
    const message = JSON.parse(event.data);
    if (message.id) {
      responses.get(message.id)?.(message);
      responses.delete(message.id);
    } else {
      notification.get(message.method)?.(message.params);
    }
  });
  await new Promise((resolve, reject) => {
    socket.addEventListener("open", resolve, { once: true });
    socket.addEventListener("error", reject, { once: true });
  });
  await send("DOM.enable");
  for (let attempt = 0; attempt < 50 && !await evaluate("document.readyState === 'complete' && !!document.querySelector('#package')"); attempt++) {
    await delay(100);
  }
  const { root } = await send("DOM.getDocument");
  const { nodeId } = await send("DOM.querySelector", { nodeId: root.nodeId, selector: "#package" });
  await send("DOM.setFileInputFiles", { nodeId, files: [pkg] });
  await evaluate("document.querySelector('#package').dispatchEvent(new Event('change'))");
  for (let attempt = 0; attempt < 60 && await evaluate("document.querySelector('#install').disabled"); attempt++) {
    await delay(100);
  }
  const state = await evaluate("({ hid: !!navigator.hid, text: document.querySelector('#status').textContent, disabled: document.querySelector('#install').disabled })");
  console.log(state);
  if (state.disabled) throw new Error("Installer rejected the package or Chrome WebHID is unavailable.");
  if (process.argv.includes("--validate-only")) process.exitCode = 0;
  else {
    await send("Page.bringToFront");
    await send("DeviceAccess.enable");
    const selected = new Promise((resolve, reject) => {
      let pendingPrompt;
      const timeout = setTimeout(() => {
        if (pendingPrompt && !process.argv.includes("--manual-chooser")) {
          void send("DeviceAccess.cancelPrompt", { id: pendingPrompt }).catch(() => {});
        }
        reject(new Error("No matching web update mode device appeared in the browser chooser."));
      }, 15000);
      notification.set("DeviceAccess.deviceRequestPrompted", async ({ id, devices }) => {
        pendingPrompt = id;
        console.log("HID chooser candidate count:", devices.length);
        if (devices.length !== 1) return;
        clearTimeout(timeout);
        notification.delete("DeviceAccess.deviceRequestPrompted");
        try {
          await send("DeviceAccess.selectPrompt", { id, deviceId: devices[0].id });
          resolve();
        } catch (error) {
          reject(error);
        }
      });
    });
    await send("Runtime.evaluate", {
      expression: "document.querySelector('#install').click()", userGesture: true,
    });
    if (process.argv.includes("--manual-chooser")) selected.catch(() => {});
    else await selected;
    for (let attempt = 0; attempt < 360; attempt++) {
      const current = await evaluate("({ text: document.querySelector('#status').textContent, value: document.querySelector('#progress').value })");
      if (process.argv.includes("--manual-chooser") && (current.text.startsWith("Ready to install") || current.text.startsWith("Choose Nexfoot"))) {
        if (attempt === 359) throw new Error("Browser chooser timed out.");
        await delay(500);
        continue;
      }
      if (!current.text.startsWith("Installing")) {
        console.log(current);
        if (!current.text.startsWith("Flash verified")) throw new Error(current.text);
        break;
      }
      if (attempt === 359) throw new Error("Browser install timed out.");
      if (attempt % 10 === 0) console.log(current);
      await delay(500);
    }
  }
} finally {
  socket?.close();
  browser.kill();
  server.close();
}
