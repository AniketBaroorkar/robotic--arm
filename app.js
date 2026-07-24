// Event names here must match ui.on_message / ui.send_message in main.py.
// Python -> UI : state, activity, activity_replay
// UI -> Python : hello, set_auto_pick, set_confidence, send_home, clear_error

const el = (id) => document.getElementById(id);

const connEl = el("conn");
const labelEl = el("label");
const confBar = el("conf-bar");
const confValue = el("conf-value");
const colourBar = el("colour-bar");
const colourValue = el("colour-value");
const gatesEl = el("gates");
const activityEl = el("activity");
const armBox = el("arm");
const armBadge = el("arm-badge");
const slider = el("confidence");
const sliderOut = el("confidence-value");
const errorEl = el("error");

// ----------------------------------------------------------- camera feed
// The detection brick serves its annotated preview at /embed on port 4912.
// It comes up after the page does, so retry until the iframe loads.
const previewUrl = `${location.protocol}//${location.hostname}:4912/embed`;
let previewTries = 0;

function loadPreview() {
  const frame = el("preview");
  frame.src = `${previewUrl}?t=${Date.now()}`;
  previewTries += 1;
}

el("preview").addEventListener("load", () => {
  el("preview-fallback").hidden = true;
  el("preview").classList.add("ready");
});

loadPreview();
const previewTimer = setInterval(() => {
  if (el("preview").classList.contains("ready") || previewTries > 60) {
    clearInterval(previewTimer);
    return;
  }
  loadPreview();
}, 2000);

// ------------------------------------------------------------- transport
if (typeof io === "undefined") {
  connEl.textContent = "socket.io failed to load";
  connEl.className = "conn bad";
  throw new Error("socket.io client script did not load");
}

const socket = io();

socket.on("connect", () => {
  connEl.textContent = "connected";
  connEl.className = "conn good";
  // Startup log lines are emitted before any browser exists, so ask for them.
  socket.emit("hello", true);
});

socket.on("disconnect", () => {
  connEl.textContent = "disconnected";
  connEl.className = "conn bad";
});

socket.on("connect_error", (err) => {
  connEl.textContent = "connect error";
  connEl.className = "conn bad";
  console.error("socket connect_error", err);
});

// ------------------------------------------------------------- controls
armBox.addEventListener("change", () => socket.emit("set_auto_pick", armBox.checked));
slider.addEventListener("input", () => {
  sliderOut.textContent = Number(slider.value).toFixed(2);
});
slider.addEventListener("change", () => socket.emit("set_confidence", Number(slider.value)));
el("home").addEventListener("click", () => socket.emit("send_home", true));
el("clear").addEventListener("click", () => socket.emit("clear_error", true));

// ----------------------------------------------------------------- state
socket.on("state", (msg) => {
  const s = typeof msg === "string" ? JSON.parse(msg) : msg;
  const d = s.detection;

  if (d) {
    labelEl.textContent = d.label;
    labelEl.className = "label active";
    el("cx").textContent = d.cx;
    el("cy").textContent = d.cy;
    setBar(confBar, confValue, d.confidence, `${Math.round(d.confidence * 100)}%`);
    if (d.colour === "unchecked") setBar(colourBar, colourValue, 0, "unchecked");
    else setBar(colourBar, colourValue, d.ratio, `${d.colour} ${Math.round(d.ratio * 100)}%`);
  } else {
    labelEl.textContent = "no fruit";
    labelEl.className = "label idle";
    el("cx").textContent = "\u2013";
    el("cy").textContent = "\u2013";
    setBar(confBar, confValue, 0, "0%");
    setBar(colourBar, colourValue, 0, "unchecked");
  }

  el("stable").textContent = `${s.stable} / ${s.required}`;
  el("cooldown").textContent = `${s.cooldown} s`;
  el("framesize").textContent = `${s.frame_w} x ${s.frame_h}`;

  armBox.checked = s.armed;
  armBadge.textContent = s.armed ? "armed" : "disarmed";
  armBadge.classList.toggle("on", s.armed);
  if (document.activeElement !== slider) {
    slider.value = s.threshold;
    sliderOut.textContent = Number(s.threshold).toFixed(2);
  }

  errorEl.textContent = s.error || "";
  errorEl.hidden = !s.error;

  renderGates(s.gates || []);
});

socket.on("activity", (msg) => addActivity(typeof msg === "string" ? JSON.parse(msg) : msg));

socket.on("activity_replay", (msg) => {
  const m = typeof msg === "string" ? JSON.parse(msg) : msg;
  activityEl.replaceChildren();
  (m.lines || []).forEach(addActivity);
});

// ---------------------------------------------------------------- render
function addActivity(m) {
  const li = document.createElement("li");
  const time = document.createElement("time");
  time.textContent = m.t;
  li.append(time, document.createTextNode(m.text));
  activityEl.prepend(li);
  while (activityEl.children.length > 40) activityEl.lastChild.remove();
}

function setBar(bar, out, ratio, text) {
  bar.style.width = `${Math.max(0, Math.min(1, ratio)) * 100}%`;
  out.textContent = text;
}

function renderGates(gates) {
  gatesEl.replaceChildren();
  if (!gates.length) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No frames received yet.";
    gatesEl.append(li);
    return;
  }
  for (const g of gates) {
    const li = document.createElement("li");
    li.className = g.ok ? "pass" : "fail";
    const dot = document.createElement("span");
    dot.className = "dot";
    const name = document.createElement("span");
    name.className = "gate-name";
    name.textContent = g.name;
    const detail = document.createElement("span");
    detail.className = "gate-detail";
    detail.textContent = g.detail;
    li.append(dot, name, detail);
    gatesEl.append(li);
  }
}
