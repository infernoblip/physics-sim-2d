const fs = require('fs');
const vm = require('vm');

const ctx = { window: {} };
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(process.argv[2] || 'physics.js', 'utf8'), ctx);
const { Body, World } = ctx.window.Physics;

function run(name, setup, steps, check) {
  const w = new World();
  setup(w);
  for (let i = 0; i < steps; i++) w.step(1 / 60);
  const ok = check(w);
  console.log((ok ? 'PASS ' : 'FAIL ') + name);
  return ok;
}

let all = true;

all = run('box stack settles', (w) => {
  w.add(new Body({ type: 'box', isStatic: true, x: 300, y: 500, width: 600, height: 40 }));
  for (let i = 0; i < 4; i++) w.add(new Body({ type: 'box', x: 300, y: 460 - i * 42, width: 80, height: 40 }));
}, 600, (w) => w.bodies.slice(1).every((b) => Math.abs(b.vel.y) < 5) && w.bodies[4].pos.y > 300 && w.bodies[4].pos.y < 360) && all;

all = run('circle rests on floor', (w) => {
  w.add(new Body({ type: 'box', isStatic: true, x: 300, y: 500, width: 600, height: 40 }));
  w.add(new Body({ type: 'circle', x: 300, y: 100, radius: 20 }));
}, 400, (w) => Math.abs(w.bodies[1].pos.y - (480 - 20)) < 2) && all;

all = run('box bounces off floor without tunnelling', (w) => {
  w.add(new Body({ type: 'box', isStatic: true, x: 300, y: 500, width: 600, height: 40 }));
  w.add(new Body({ type: 'box', x: 300, y: 100, width: 40, height: 40 }));
  w.bodies[1].vel.y = 800;
}, 300, (w) => w.bodies[1].pos.y <= 481 && w.bodies[1].pos.y > 400) && all;

all = run('circle rests on flat platform mid-air (no tunnelling)', (w) => {
  w.add(new Body({ type: 'box', isStatic: true, x: 300, y: 400, width: 400, height: 20, angle: 0 }));
  w.add(new Body({ type: 'circle', x: 300, y: 300, radius: 15 }));
}, 300, (w) => Math.abs(w.bodies[1].pos.x - 300) < 2 && Math.abs(w.bodies[1].pos.y - 375) < 2) && all;

console.log(all ? 'ALL PASS' : 'SOME FAILED');
process.exitCode = all ? 0 : 1;
