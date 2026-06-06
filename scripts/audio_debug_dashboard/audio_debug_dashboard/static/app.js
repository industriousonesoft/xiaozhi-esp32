const HEADER_SIZE = 40;
const MAGIC = 0x47424441;
const streams = { 1: "raw_mics", 2: "beam", 3: "afe", 4: "direction" };

class RingBuffer {
  constructor(capacity) {
    this.data = new Float32Array(capacity);
    this.capacity = capacity;
    this.write = 0;
    this.length = 0;
  }
  push(value) {
    this.data[this.write] = value;
    this.write = (this.write + 1) % this.capacity;
    this.length = Math.min(this.length + 1, this.capacity);
  }
  recent(count) {
    count = Math.min(count, this.length);
    const result = new Float32Array(count);
    let start = (this.write - count + this.capacity) % this.capacity;
    for (let i = 0; i < count; i++) result[i] = this.data[(start + i) % this.capacity];
    return result;
  }
}

const buffers = {
  mic0: new RingBuffer(48000 * 5),
  mic1: new RingBuffer(48000 * 5),
  mic2: new RingBuffer(48000 * 5),
  beam: new RingBuffer(48000 * 5),
  afe: new RingBuffer(16000 * 5),
};
const sampleRates = { mic0: 48000, mic1: 48000, mic2: 48000, beam: 48000, afe: 16000 };
let socket;
let connected = false;
let recording = false;
let controller = false;
let direction = { angle: 0, confidence: 0, active: false, valid: false };

function send(payload) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(payload));
}

function openSocket() {
  socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  socket.binaryType = "arraybuffer";
  socket.onmessage = event => {
    if (event.data instanceof ArrayBuffer) parsePacket(event.data);
    else handleMessage(JSON.parse(event.data));
  };
  socket.onclose = () => {
    connected = false;
    updateConnection();
    setTimeout(openSocket, 1000);
  };
}

function parsePacket(arrayBuffer) {
  const view = new DataView(arrayBuffer);
  if (view.byteLength < HEADER_SIZE || view.getUint32(0, true) !== MAGIC) return;
  const stream = view.getUint8(5);
  const frameCount = view.getUint16(28, true);
  const channels = view.getUint8(30);
  const payloadSize = view.getUint16(32, true);
  if (HEADER_SIZE + payloadSize !== view.byteLength) return;
  if (stream === 4) {
    direction = {
      angle: view.getFloat32(HEADER_SIZE, true),
      confidence: view.getFloat32(HEADER_SIZE + 4, true),
      active: !!view.getUint8(HEADER_SIZE + 8),
      valid: true,
    };
    updateDirectionText();
    return;
  }
  for (let frame = 0; frame < frameCount; frame++) {
    if (stream === 1 && channels === 3) {
      buffers.mic0.push(view.getInt16(HEADER_SIZE + (frame * 3) * 2, true));
      buffers.mic1.push(view.getInt16(HEADER_SIZE + (frame * 3 + 1) * 2, true));
      buffers.mic2.push(view.getInt16(HEADER_SIZE + (frame * 3 + 2) * 2, true));
    } else if (stream === 2) {
      buffers.beam.push(view.getInt16(HEADER_SIZE + frame * 2, true));
    } else if (stream === 3) {
      buffers.afe.push(view.getInt16(HEADER_SIZE + frame * 2, true));
    }
  }
}

function handleMessage(message) {
  if (message.kind === "role") {
    controller = message.controller;
    document.getElementById("roleText").textContent = controller ? "控制端" : "只读端";
    document.querySelectorAll("button, .params input").forEach(item => item.disabled = !controller);
  } else if (message.kind === "status") {
    connected = message.connected;
    recording = message.recording;
    document.getElementById("sessionValue").textContent = message.session_id || "--";
    document.getElementById("invalidPackets").textContent = message.invalid_packets;
    renderStats(message.streams || {});
    updateConnection();
    if (message.device?.dropped_packets !== undefined) {
      document.getElementById("deviceDrops").textContent = message.device.dropped_packets;
    }
  } else if (message.kind === "device") {
    if (message.params) updateParams(message.params);
    if (message.error) showError(message.error);
  } else if (message.kind === "recording") {
    document.getElementById("recordingPath").textContent = message.path || "--";
  } else if (message.kind === "error") {
    showError(message.error);
  }
}

function renderStats(stats) {
  const labels = { raw_mics: "RAW MIC", beam: "BEAM", afe: "AFE", direction: "DIRECTION" };
  document.getElementById("statsBody").innerHTML = Object.entries(labels).map(([key, label]) => {
    const item = stats[key] || { received: 0, missing: 0 };
    return `<tr><td>${label}</td><td>${item.received}</td><td>${item.missing}</td></tr>`;
  }).join("");
}

function updateConnection() {
  document.getElementById("statusText").textContent = connected ? "设备已连接，正在接收数据" : "未连接";
  const recordBtn = document.getElementById("recordBtn");
  recordBtn.textContent = recording ? "停止录制" : "开始录制";
}

function updateParams(params) {
  const form = document.getElementById("paramsForm");
  Object.entries(params).forEach(([name, value]) => {
    if (form.elements[name]) form.elements[name].value = value;
  });
}

function showError(error) {
  document.getElementById("errorText").textContent = error;
}

function drawWave(canvas, buffer, sampleRate) {
  const rect = canvas.getBoundingClientRect();
  const dpr = devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * dpr));
  const height = Math.max(1, Math.floor(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#243447";
  ctx.beginPath(); ctx.moveTo(0, height / 2); ctx.lineTo(width, height / 2); ctx.stroke();
  const seconds = Number(document.getElementById("windowSeconds").value);
  const values = buffer.recent(Math.floor(sampleRate * seconds));
  if (!values.length) return;
  let scale = 32768;
  if (!document.getElementById("fixedScale").checked) {
    scale = 256;
    for (const value of values) scale = Math.max(scale, Math.abs(value));
    scale *= 1.15;
  }
  ctx.strokeStyle = "#3ed6d0";
  ctx.beginPath();
  const samplesPerPixel = Math.max(1, values.length / width);
  for (let x = 0; x < width; x++) {
    const start = Math.floor(x * samplesPerPixel);
    const end = Math.min(values.length, Math.ceil((x + 1) * samplesPerPixel));
    let min = Infinity, max = -Infinity;
    for (let i = start; i < end; i++) { min = Math.min(min, values[i]); max = Math.max(max, values[i]); }
    const y1 = height / 2 - (max / scale) * height * .46;
    const y2 = height / 2 - (min / scale) * height * .46;
    ctx.moveTo(x, y1); ctx.lineTo(x, y2);
  }
  ctx.stroke();
}

function drawDirection() {
  const canvas = document.getElementById("direction");
  const rect = canvas.getBoundingClientRect();
  const dpr = devicePixelRatio || 1;
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  const cx = canvas.width / 2, cy = canvas.height * .82, radius = Math.min(cx * .85, canvas.height * .72);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = "#32455b"; ctx.lineWidth = 2 * dpr;
  ctx.beginPath(); ctx.arc(cx, cy, radius, Math.PI, 0); ctx.stroke();
  for (let deg = -90; deg <= 90; deg += 30) {
    const angle = (deg - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(angle) * radius * .9, cy + Math.sin(angle) * radius * .9);
    ctx.lineTo(cx + Math.cos(angle) * radius, cy + Math.sin(angle) * radius);
    ctx.stroke();
  }
  if (direction.valid) {
    const angle = (direction.angle - 90) * Math.PI / 180;
    ctx.strokeStyle = direction.active ? "#ffb84d" : "#6f7782";
    ctx.lineWidth = 4 * dpr;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(angle) * radius * .86, cy + Math.sin(angle) * radius * .86); ctx.stroke();
  }
}

function updateDirectionText() {
  document.getElementById("angleValue").textContent = direction.valid ? `${direction.angle.toFixed(1)}°` : "--°";
  document.getElementById("confidenceValue").textContent = `置信度 ${direction.confidence.toFixed(2)}`;
  const active = document.getElementById("activeValue");
  active.textContent = direction.active ? "有效声源" : "无有效声源";
  active.classList.toggle("inactive", !direction.active);
}

function animate() {
  if (!document.getElementById("pauseDisplay").checked) {
    Object.entries(buffers).forEach(([name, buffer]) => drawWave(document.getElementById(name), buffer, sampleRates[name]));
    drawDirection();
  }
  requestAnimationFrame(animate);
}

document.getElementById("connectBtn").onclick = () => {
  showError("");
  send({ action: "connect", device_ip: document.getElementById("deviceIp").value.trim() });
};
document.getElementById("disconnectBtn").onclick = () => send({ action: "disconnect" });
document.getElementById("recordBtn").onclick = () => send({ action: recording ? "record_stop" : "record_start" });
document.getElementById("resetParams").onclick = () => send({ action: "reset_params" });
document.getElementById("paramsForm").onsubmit = event => {
  event.preventDefault();
  const params = {};
  new FormData(event.target).forEach((value, key) => params[key] = Number(value));
  send({ action: "set_params", params });
};

renderStats({});
openSocket();
animate();
