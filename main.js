(function () {
  'use strict';

  const { World, Body } = window.Physics;
  const canvas = document.getElementById('scene');
  const ctx = canvas.getContext('2d');
  const PALETTE = ['#4cc9f0', '#f72585', '#7209b7', '#f8961e', '#90be6d', '#4361ee', '#ffd166'];
  const WALL = 80; // thickness of invisible boundary walls

  const world = new World();
  let W = 0, H = 0, walls = [];
  let paused = false;
  let dragBody = null, grab = { x: 0, y: 0 }, pointer = { x: 0, y: 0 };
  let frames = 0, fpsTimer = 0, fps = 0;

  const el = {
    bodies: document.getElementById('bodies'),
    fps: document.getElementById('fps'),
    gravity: document.getElementById('gravity'),
    gravVal: document.getElementById('gravVal'),
    pause: document.getElementById('pauseBtn'),
  };

  const rand = (a, b) => a + Math.random() * (b - a);
  const pick = (arr) => arr[Math.floor(Math.random() * arr.length)];

  function resize() {
    const dpr = window.devicePixelRatio || 1;
    W = window.innerWidth;
    H = window.innerHeight;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    rebuildWalls();
  }

  // Static boundary: floor, ceiling and side walls around the visible canvas.
  function rebuildWalls() {
    for (const w of walls) world.remove(w);
    const opts = { isStatic: true, restitution: 0.2, friction: 0.6, color: '#2a2f45' };
    walls = [
      new Body({ ...opts, type: 'box', x: W / 2, y: H + WALL / 2, width: W + WALL * 4, height: WALL }),
      new Body({ ...opts, type: 'box', x: -WALL / 2, y: H / 2, width: WALL, height: H * 3 }),
      new Body({ ...opts, type: 'box', x: W + WALL / 2, y: H / 2, width: WALL, height: H * 3 }),
      new Body({ ...opts, type: 'box', x: W / 2, y: -WALL / 2, width: W + WALL * 4, height: WALL }),
    ];
    walls.forEach((w) => world.add(w));
  }

  function spawn(type, x, y) {
    const color = pick(PALETTE);
    const body = type === 'circle'
      ? new Body({ type: 'circle', x, y, radius: rand(12, 30), color, restitution: 0.4, friction: 0.4 })
      : new Body({ type: 'box', x, y, width: rand(30, 80), height: rand(30, 80), angle: rand(0, 0.6), color, restitution: 0.25, friction: 0.5 });
    world.add(body);
  }

  function addRamp(x, y, width, angle) {
    world.add(new Body({ type: 'box', isStatic: true, x, y, width, height: 18, angle, color: '#2a2f45', restitution: 0.2, friction: 0.6 }));
  }

  function pyramid() {
    const rows = 8, bw = 40, bh = 22, gap = 2;
    const cx = W / 2;
    for (let r = 0; r < rows; r++) {
      const count = rows - r;
      for (let c = 0; c < count; c++) {
        const x = cx + (c - (count - 1) / 2) * (bw + gap);
        const y = H - bh / 2 - r * (bh + gap);
        world.add(new Body({ type: 'box', x, y, width: bw, height: bh, color: PALETTE[r % PALETTE.length], restitution: 0.1, friction: 0.7 }));
      }
    }
  }

  function setupScene() {
    world.clear();
    rebuildWalls();
    addRamp(W * 0.3, H * 0.55, Math.min(360, W * 0.4), 0.35);
    for (let i = 0; i < 4; i++) spawn('box', W * 0.55 + rand(-40, 40), H * 0.15 + i * 50);
    for (let i = 0; i < 3; i++) spawn('circle', W * 0.2 + rand(-60, 60), H * 0.1 + i * 40);
  }

  function setPaused(value) {
    paused = value;
    el.pause.textContent = paused ? 'Resume' : 'Pause';
  }

  function onAction(action) {
    switch (action) {
      case 'box':     spawn('box', W / 2 + rand(-60, 60), H * 0.1); break;
      case 'circle':  spawn('circle', W / 2 + rand(-60, 60), H * 0.1); break;
      case 'random':  for (let i = 0; i < 10; i++) spawn(Math.random() < 0.5 ? 'box' : 'circle', rand(60, W - 60), rand(-300, -20)); break;
      case 'pyramid': pyramid(); break;
      case 'clear':   setupScene(); break;
      case 'pause':   setPaused(!paused); break;
    }
  }

  document.querySelectorAll('[data-action]').forEach((btn) => {
    btn.addEventListener('click', () => onAction(btn.dataset.action));
  });

  el.gravity.addEventListener('input', () => {
    world.gravity.y = Number(el.gravity.value);
    el.gravVal.textContent = el.gravity.value;
  });

  document.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && e.target === document.body) {
      e.preventDefault();
      setPaused(!paused);
    }
  });

  function pointerPos(e) {
    const r = canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }

  canvas.addEventListener('pointerdown', (e) => {
    const p = pointerPos(e);
    pointer = p;
    for (let i = world.bodies.length - 1; i >= 0; i--) {
      const b = world.bodies[i];
      if (!b.isStatic && b.containsPoint(p)) {
        dragBody = b;
        grab = { x: p.x - b.pos.x, y: p.y - b.pos.y };
        canvas.setPointerCapture(e.pointerId);
        return;
      }
    }
    spawn(Math.random() < 0.5 ? 'box' : 'circle', p.x, p.y);
  });

  canvas.addEventListener('pointermove', (e) => {
    pointer = pointerPos(e);
  });

  const endDrag = () => { dragBody = null; };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);

  // Drive a grabbed body towards the pointer with a velocity, so collisions still resolve.
  function updateDrag() {
    if (!dragBody) return;
    const b = dragBody;
    const tx = pointer.x - grab.x, ty = pointer.y - grab.y;
    if (paused) {
      b.pos.x = tx; b.pos.y = ty;
      b.vel.x = 0; b.vel.y = 0;
    } else {
      b.vel.x = (tx - b.pos.x) * 12;
      b.vel.y = (ty - b.pos.y) * 12;
    }
    b.angVel *= 0.9;
  }

  function drawBody(b) {
    ctx.beginPath();
    if (b.type === 'circle') {
      ctx.arc(b.pos.x, b.pos.y, b.radius, 0, Math.PI * 2);
      ctx.moveTo(b.pos.x, b.pos.y);
      ctx.lineTo(b.pos.x + Math.cos(b.angle) * b.radius, b.pos.y + Math.sin(b.angle) * b.radius);
    } else {
      const v = b.worldVerts();
      ctx.moveTo(v[0].x, v[0].y);
      for (let i = 1; i < v.length; i++) ctx.lineTo(v[i].x, v[i].y);
      ctx.closePath();
    }
    ctx.fillStyle = b.isStatic ? '#2a2f45' : b.color;
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.stroke();
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);
    for (const b of world.bodies) {
      if (b.isStatic && walls.includes(b)) continue; // keep boundary invisible
      drawBody(b);
    }
    if (dragBody) {
      ctx.beginPath();
      ctx.moveTo(dragBody.pos.x, dragBody.pos.y);
      ctx.lineTo(pointer.x, pointer.y);
      ctx.strokeStyle = 'rgba(76,201,240,0.6)';
      ctx.setLineDash([6, 6]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  let last = performance.now();
  function frame(now) {
    const dt = Math.min((now - last) / 1000, 1 / 30);
    last = now;

    updateDrag();
    if (!paused) world.step(dt);
    draw();

    frames++;
    fpsTimer += dt;
    if (fpsTimer >= 0.5) {
      fps = Math.round(frames / fpsTimer);
      frames = 0;
      fpsTimer = 0;
      el.fps.textContent = fps;
      el.bodies.textContent = world.bodies.length - walls.length;
    }
    requestAnimationFrame(frame);
  }

  window.addEventListener('resize', resize);
  resize();
  world.gravity.y = Number(el.gravity.value);
  setupScene();
  requestAnimationFrame((t) => { last = t; frame(t); });
})();
