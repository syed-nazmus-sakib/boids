// Boids — Craig Reynolds' flocking model.
// Each boid steers by three local rules within its perception radius:
//   separation — steer away from close neighbours
//   alignment  — match the average heading of neighbours
//   cohesion   — steer toward the average position of neighbours
// Nothing in a single boid "knows" about flocking; it emerges between them.
//
// Plain classic script (no modules) so index.html opens straight from the file system.

const canvas = document.querySelector("#scene");
const ctx = canvas.getContext("2d");

const config = {
  wSep: 1.7,
  wAlign: 1.0,
  wCoh: 0.9,
  perception: 55,
  separation: 26, // kept a sensible fraction of perception (see syncSeparation)
  maxSpeed: 3.0,
  maxForce: 0.15,
  count: 160,
  edges: "wrap",
  trails: false,
  vectors: false,
  vision: false,
  paused: false,
};

const mouse = { x: null, y: null, mode: "off", radius: 160 };

let flock = [];
let width = 0;
let height = 0;

// ---- small vector helpers (operate on {x, y}) --------------------------------

function mag(x, y) {
  return Math.hypot(x, y);
}

function setMag(x, y, m) {
  const len = Math.hypot(x, y) || 1e-6;
  return { x: (x / len) * m, y: (y / len) * m };
}

function limit(x, y, max) {
  const len = Math.hypot(x, y);
  if (len > max) {
    const s = max / len;
    return { x: x * s, y: y * s };
  }
  return { x, y };
}

// ---- setup -------------------------------------------------------------------

function makeBoid() {
  const angle = Math.random() * Math.PI * 2;
  const speed = 1 + Math.random() * config.maxSpeed;
  return {
    pos: { x: Math.random() * width, y: Math.random() * height },
    vel: { x: Math.cos(angle) * speed, y: Math.sin(angle) * speed },
    acc: { x: 0, y: 0 },
  };
}

function setCount(n) {
  config.count = n;
  while (flock.length < n) flock.push(makeBoid());
  if (flock.length > n) flock.length = n;
}

function resetFlock() {
  flock = [];
  setCount(config.count);
}

function resize() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  width = rect.width;
  height = rect.height;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

// Keep the separation radius proportional to perception so the two stay sensible.
function syncSeparation() {
  config.separation = Math.max(12, config.perception * 0.5);
}

// ---- simulation --------------------------------------------------------------

function step() {
  const P = config;
  const perceptionSq = P.perception * P.perception;
  const separationSq = P.separation * P.separation;

  // Pass 1: compute each boid's acceleration from a snapshot of the current frame.
  for (const b of flock) {
    let aliX = 0;
    let aliY = 0;
    let cohX = 0;
    let cohY = 0;
    let sepX = 0;
    let sepY = 0;
    let n = 0;
    let sepN = 0;

    for (const o of flock) {
      if (o === b) continue;
      const dx = b.pos.x - o.pos.x;
      const dy = b.pos.y - o.pos.y;
      const d2 = dx * dx + dy * dy;
      if (d2 > perceptionSq) continue;

      aliX += o.vel.x;
      aliY += o.vel.y;
      cohX += o.pos.x;
      cohY += o.pos.y;
      n += 1;

      if (d2 < separationSq) {
        const d = Math.sqrt(d2) || 1e-6;
        sepX += dx / d / d; // push harder the closer they are
        sepY += dy / d / d;
        sepN += 1;
      }
    }

    let ax = 0;
    let ay = 0;

    if (n > 0) {
      const ali = setMag(aliX / n, aliY / n, P.maxSpeed);
      const aliSteer = limit(ali.x - b.vel.x, ali.y - b.vel.y, P.maxForce);
      ax += aliSteer.x * P.wAlign;
      ay += aliSteer.y * P.wAlign;

      const coh = setMag(cohX / n - b.pos.x, cohY / n - b.pos.y, P.maxSpeed);
      const cohSteer = limit(coh.x - b.vel.x, coh.y - b.vel.y, P.maxForce);
      ax += cohSteer.x * P.wCoh;
      ay += cohSteer.y * P.wCoh;
    }

    if (sepN > 0) {
      const sep = setMag(sepX / sepN, sepY / sepN, P.maxSpeed);
      const sepSteer = limit(sep.x - b.vel.x, sep.y - b.vel.y, P.maxForce);
      ax += sepSteer.x * P.wSep;
      ay += sepSteer.y * P.wSep;
    }

    if (mouse.mode !== "off" && mouse.x !== null) {
      const dx = mouse.x - b.pos.x;
      const dy = mouse.y - b.pos.y;
      const d2 = dx * dx + dy * dy;
      if (d2 < mouse.radius * mouse.radius) {
        const sign = mouse.mode === "attract" ? 1 : -1;
        const want = setMag(dx * sign, dy * sign, P.maxSpeed);
        const steer = limit(want.x - b.vel.x, want.y - b.vel.y, P.maxForce * 2.5);
        ax += steer.x;
        ay += steer.y;
      }
    }

    b.acc.x = ax;
    b.acc.y = ay;
  }

  // Pass 2: integrate.
  for (const b of flock) {
    b.vel.x += b.acc.x;
    b.vel.y += b.acc.y;
    const v = limit(b.vel.x, b.vel.y, P.maxSpeed);
    b.vel.x = v.x;
    b.vel.y = v.y;
    b.pos.x += b.vel.x;
    b.pos.y += b.vel.y;
    wrapOrBounce(b);
  }
}

function wrapOrBounce(b) {
  if (config.edges === "wrap") {
    if (b.pos.x < 0) b.pos.x += width;
    else if (b.pos.x > width) b.pos.x -= width;
    if (b.pos.y < 0) b.pos.y += height;
    else if (b.pos.y > height) b.pos.y -= height;
  } else {
    const m = 6;
    if (b.pos.x < m) { b.pos.x = m; b.vel.x = Math.abs(b.vel.x); }
    else if (b.pos.x > width - m) { b.pos.x = width - m; b.vel.x = -Math.abs(b.vel.x); }
    if (b.pos.y < m) { b.pos.y = m; b.vel.y = Math.abs(b.vel.y); }
    else if (b.pos.y > height - m) { b.pos.y = height - m; b.vel.y = -Math.abs(b.vel.y); }
  }
}

// ---- rendering ---------------------------------------------------------------

function draw() {
  if (config.trails) {
    // Fade the previous frame instead of clearing, leaving motion streaks.
    ctx.fillStyle = "rgba(7, 11, 22, 0.16)";
    ctx.fillRect(0, 0, width, height);
  } else {
    ctx.clearRect(0, 0, width, height);
  }

  for (const b of flock) drawBoid(b);

  // Vision overlay on the first boid, to show what "neighbours" means.
  if (config.vision && flock.length) {
    const b = flock[0];
    ctx.save();
    ctx.strokeStyle = "rgba(65, 224, 200, 0.5)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(b.pos.x, b.pos.y, config.perception, 0, Math.PI * 2);
    ctx.stroke();
    ctx.strokeStyle = "rgba(255, 122, 89, 0.5)";
    ctx.beginPath();
    ctx.arc(b.pos.x, b.pos.y, config.separation, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
}

function drawBoid(b) {
  const angle = Math.atan2(b.vel.y, b.vel.x);
  const hue = ((angle * 180) / Math.PI + 360) % 360; // colour by heading
  const size = 6;

  ctx.save();
  ctx.translate(b.pos.x, b.pos.y);
  ctx.rotate(angle);
  ctx.fillStyle = `hsl(${hue}, 85%, 62%)`;
  ctx.beginPath();
  ctx.moveTo(size * 1.6, 0);
  ctx.lineTo(-size, size * 0.7);
  ctx.lineTo(-size, -size * 0.7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  if (config.vectors) {
    ctx.save();
    ctx.strokeStyle = "rgba(238, 243, 255, 0.35)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(b.pos.x, b.pos.y);
    ctx.lineTo(b.pos.x + b.vel.x * 6, b.pos.y + b.vel.y * 6);
    ctx.stroke();
    ctx.restore();
  }
}

// ---- loop --------------------------------------------------------------------

const countHud = document.querySelector("#countHud");
const fpsHud = document.querySelector("#fpsHud");
let lastFpsTime = performance.now();
let frames = 0;

function frame() {
  if (!config.paused) step();
  draw();

  frames += 1;
  const now = performance.now();
  if (now - lastFpsTime >= 500) {
    const fps = Math.round((frames * 1000) / (now - lastFpsTime));
    fpsHud.textContent = `${fps} fps`;
    countHud.textContent = `${flock.length} boids`;
    frames = 0;
    lastFpsTime = now;
  }
  requestAnimationFrame(frame);
}

// ---- UI wiring ---------------------------------------------------------------

function bindSlider(id, key, format = (v) => v) {
  const input = document.querySelector(`#${id}`);
  const out = document.querySelector(`#${id}Out`);
  const apply = () => {
    config[key] = Number(input.value);
    if (out) out.textContent = format(config[key]);
  };
  input.addEventListener("input", () => {
    apply();
    if (key === "perception") syncSeparation();
    if (key === "count") setCount(config.count);
  });
  apply();
}

bindSlider("wSep", "wSep", (v) => v.toFixed(2));
bindSlider("wAlign", "wAlign", (v) => v.toFixed(2));
bindSlider("wCoh", "wCoh", (v) => v.toFixed(2));
bindSlider("perception", "perception", (v) => String(Math.round(v)));
bindSlider("maxSpeed", "maxSpeed", (v) => v.toFixed(1));
bindSlider("count", "count", (v) => String(Math.round(v)));

document.querySelector("#edges").addEventListener("change", (e) => {
  config.edges = e.target.value;
});
document.querySelector("#mouseMode").addEventListener("change", (e) => {
  mouse.mode = e.target.value;
});

function bindToggle(id, key) {
  document.querySelector(`#${id}`).addEventListener("change", (e) => {
    config[key] = e.target.checked;
  });
}
bindToggle("tTrails", "trails");
bindToggle("tVectors", "vectors");
bindToggle("tVision", "vision");

const pauseBtn = document.querySelector("#pause");
pauseBtn.addEventListener("click", () => {
  config.paused = !config.paused;
  pauseBtn.textContent = config.paused ? "Resume" : "Pause";
});
document.querySelector("#reset").addEventListener("click", resetFlock);
document.querySelector("#addMore").addEventListener("click", () => {
  const slider = document.querySelector("#count");
  const next = Math.min(Number(slider.max), config.count + 50);
  slider.value = String(next);
  config.count = next;
  document.querySelector("#countOut").textContent = String(next);
  setCount(next);
});

canvas.addEventListener("pointermove", (e) => {
  const rect = canvas.getBoundingClientRect();
  mouse.x = e.clientX - rect.left;
  mouse.y = e.clientY - rect.top;
});
canvas.addEventListener("pointerleave", () => {
  mouse.x = null;
  mouse.y = null;
});
window.addEventListener("resize", resize);

// ---- go ----------------------------------------------------------------------

resize();
syncSeparation();
resetFlock();
requestAnimationFrame(frame);
