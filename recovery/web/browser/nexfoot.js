/* SPDX-License-Identifier: Apache-2.0 */
const APP_START = 0x3000;
const APP_END = 0x48000;
const SECTOR = 4096;
const ABI = "NEXWEB1";
const decoder = new TextDecoder("utf-8", { fatal: true });

export const UPDATE_FILTER = { vendorId: 0x4c4a, productId: 0x4157, usagePage: 0xff00, usage: 1 };

function check(condition, message) {
  if (!condition) throw new Error(message);
}

async function unzipEntry(blob, expectedBytes) {
  const reader = blob.stream().pipeThrough(new DecompressionStream("deflate-raw")).getReader();
  const data = new Uint8Array(expectedBytes);
  let offset = 0;
  try {
    for (;;) {
      const { done, value } = await reader.read();
      if (done) break;
      check(value.length <= data.length - offset, "Invalid ZIP entry size.");
      data.set(value, offset);
      offset += value.length;
    }
  } finally {
    await reader.cancel();
  }
  check(offset === data.length, "Truncated ZIP entry.");
  return data;
}

async function sha256(data) {
  const bytes = new Uint8Array(await crypto.subtle.digest("SHA-256", data));
  return Array.from(bytes, byte => byte.toString(16).padStart(2, "0")).join("");
}

export async function readPackage(blob) {
  check(blob.size <= 0x90000, "Nexfoot package is too large.");
  const bytes = new Uint8Array(await blob.arrayBuffer());
  const header = new DataView(bytes.buffer);
  const entries = new Map();
  let offset = 0;
  // The Nexfoot packer emits ordinary local ZIP headers with known member
  // lengths. Accept only those two entries and reject streaming/encrypted ZIPs.
  while (offset + 30 <= bytes.length && header.getUint32(offset, true) === 0x04034b50) {
    const flags = header.getUint16(offset + 6, true);
    const method = header.getUint16(offset + 8, true);
    const compressedSize = header.getUint32(offset + 18, true);
    const size = header.getUint32(offset + 22, true);
    const nameSize = header.getUint16(offset + 26, true);
    const extraSize = header.getUint16(offset + 28, true);
    const nameStart = offset + 30;
    const start = nameStart + nameSize + extraSize;
    check(start + compressedSize <= bytes.length, "Truncated Nexfoot ZIP.");
    const name = decoder.decode(bytes.subarray(nameStart, nameStart + nameSize));
    check((name === "manifest.json" || name === "firmware.bin") && !entries.has(name), "Unexpected ZIP entry.");
    check(flags === 0 && [0, 8].includes(method), "Unsupported Nexfoot ZIP format.");
    const limit = name === "manifest.json" ? 16384 : APP_END;
    check(size > 0 && size <= limit, "Nexfoot ZIP entry exceeds its size limit.");
    const body = blob.slice(start, start + compressedSize);
    if (method === 0) check(size === compressedSize, "Invalid stored ZIP entry.");
    entries.set(name, method === 0 ? new Uint8Array(await body.arrayBuffer()) : await unzipEntry(body, size));
    offset = start + compressedSize;
  }
  check(entries.size === 2 && offset + 4 <= bytes.length && header.getUint32(offset, true) === 0x02014b50,
    "Invalid Nexfoot package.");
  const manifest = JSON.parse(decoder.decode(entries.get("manifest.json")));
  const image = entries.get("firmware.bin");
  check(manifest.schema === "nexfoot-community-web-v1" && manifest.origin === "community" &&
    manifest.product === "Nexfoot" && manifest.board === "md400-bd19-v1" && manifest.mcu === "AC632N" &&
    manifest.flashBytes === 0x80000 && manifest.protectedBootBytes === APP_START && manifest.codeEnd === APP_END &&
    manifest.firmwareBytes === image.length && /^[A-Za-z0-9][A-Za-z0-9._+-]{0,63}$/.test(manifest.version),
    "This package is incompatible with Nexfoot browser update.");
  check(image.length > APP_START && image.length % SECTOR === 0 &&
    decoder.decode(image.subarray(0x10, 0x16)) === "AC632N" &&
    image.slice(-16).every(byte => byte === 0xff), "Invalid Nexfoot firmware image.");
  check(await sha256(image) === manifest.firmwareSha256, "Firmware digest mismatch.");
  check(await sha256(image.subarray(0, APP_START)) === manifest.protectedBootSha256,
    "Firmware recovery prefix digest mismatch.");
  return { manifest, image };
}

function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ (crc & 1 ? 0xedb88320 : 0);
  }
  return (~crc) >>> 0;
}

class UpdateSession {
  constructor(device) {
    this.device = device;
    this.sequence = 0;
  }

  async request(op, argument = 0, payload = new Uint8Array(0)) {
    const report = new Uint8Array(64);
    const data = new DataView(report.buffer);
    const seq = this.sequence++;
    report[0] = op;
    report[1] = payload.length;
    data.setUint16(2, seq, true);
    data.setUint32(4, argument, true);
    report.set(payload, 8);
    let listener;
    let timer;
    try {
      const reply = new Promise((resolve, reject) => {
        listener = event => {
          if (event.reportId === 0 && event.data.byteLength === 64 &&
              event.data.getUint8(0) === op && event.data.getUint16(2, true) === seq) resolve(event.data);
        };
        this.device.addEventListener("inputreport", listener);
        timer = setTimeout(() => reject(new Error("Nexfoot update reply timed out.")), 6000);
      });
      const [response] = await Promise.all([reply, this.device.sendReport(0, report)]);
      const status = response.getUint8(4);
      check(status === 0, `Nexfoot rejected command ${op} at 0x${argument.toString(16)} (status ${status}).`);
      return response;
    } finally {
      clearTimeout(timer);
      this.device.removeEventListener("inputreport", listener);
    }
  }
}

export async function installPackage(device, image, progress = () => {}) {
  check(device.vendorId === UPDATE_FILTER.vendorId && device.productId === UPDATE_FILTER.productId,
    "Choose a Nexfoot in browser update mode.");
  await device.open();
  try {
    const session = new UpdateSession(device);
    const info = await session.request(1);
    check(decoder.decode(new Uint8Array(info.buffer, info.byteOffset + 8, 7)) === ABI &&
      info.getUint32(16, true) === APP_START && info.getUint32(20, true) === APP_END,
      "Nexfoot reported an incompatible update protocol.");
    check(info.getUint32(24, true) === crc32(image.subarray(0, APP_START)),
      "The package targets a different Nexfoot recovery build.");
    const crc = new Uint8Array(4);
    new DataView(crc.buffer).setUint32(0, crc32(image.subarray(APP_START)), true);
    await session.request(2, image.length, crc);
    for (let sector = image.length - SECTOR; sector >= APP_START; sector -= SECTOR) {
      for (let offset = sector; offset < sector + SECTOR; offset += 56) {
        await session.request(3, offset, image.subarray(offset, Math.min(offset + 56, sector + SECTOR)));
      }
      progress((image.length - sector) / (image.length - APP_START));
    }
    await session.request(4);
    await session.request(5);
  } finally {
    if (device.opened) await device.close();
  }
}
