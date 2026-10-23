/* Basic 2D rigid body physics engine.
 * Shapes: circles and convex boxes (oriented rectangles).
 * Solver: sequential impulses with friction, restitution and positional correction.
 */
(function (global) {
  'use strict';

  const V = {
    dot: (a, b) => a.x * b.x + a.y * b.y,
    cross: (a, b) => a.x * b.y - a.y * b.x,
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
    scale: (a, s) => ({ x: a.x * s, y: a.y * s }),
    len: (a) => Math.hypot(a.x, a.y),
    norm: (a) => {
      const l = Math.hypot(a.x, a.y) || 1;
      return { x: a.x / l, y: a.y / l };
    },
    perp: (a) => ({ x: -a.y, y: a.x }),
  };

  // Velocity of a point at offset r from the centre of mass, for angular velocity w.
  function pointVelocity(body, r) {
    return { x: body.vel.x - body.angVel * r.y, y: body.vel.y + body.angVel * r.x };
  }

  // Angular velocity w crossed with vector r: w x r = (-w*r.y, w*r.x).
  function wcross(w, r) {
    return { x: -w * r.y, y: w * r.x };
  }

  let nextId = 1;

  class Body {
    constructor(opts) {
      this.id = nextId++;
      this.type = opts.type; // 'circle' | 'box'
      this.isStatic = !!opts.isStatic;
      this.pos = { x: opts.x, y: opts.y };
      this.vel = { x: 0, y: 0 };
      this.angle = opts.angle || 0;
      this.angVel = 0;
      this.restitution = opts.restitution ?? 0.3;
      this.friction = opts.friction ?? 0.5;
      this.color = opts.color || '#4cc9f0';

      const density = opts.density ?? 1;
      let area;
      if (this.type === 'circle') {
        this.radius = opts.radius;
        this.boundRadius = opts.radius;
        this.localVerts = null;
        area = Math.PI * opts.radius * opts.radius;
      } else {
        this.halfW = opts.width / 2;
        this.halfH = opts.height / 2;
        this.boundRadius = Math.hypot(this.halfW, this.halfH);
        this.localVerts = [
          { x: -this.halfW, y: -this.halfH },
          { x: this.halfW, y: -this.halfH },
          { x: this.halfW, y: this.halfH },
          { x: -this.halfW, y: this.halfH },
        ];
        area = opts.width * opts.height;
      }

      if (this.isStatic) {
        this.invMass = 0;
        this.invInertia = 0;
      } else {
        const mass = density * area;
        this.invMass = 1 / mass;
        const inertia =
          this.type === 'circle'
            ? 0.5 * mass * this.radius * this.radius
            : (mass * (opts.width * opts.width + opts.height * opts.height)) / 12;
        this.invInertia = 1 / inertia;
      }
    }

    worldVerts() {
      const c = Math.cos(this.angle), s = Math.sin(this.angle);
      return this.localVerts.map((v) => ({
        x: this.pos.x + v.x * c - v.y * s,
        y: this.pos.y + v.x * s + v.y * c,
      }));
    }

    containsPoint(p) {
      const d = V.sub(p, this.pos);
      if (this.type === 'circle') return V.len(d) <= this.radius;
      const c = Math.cos(this.angle), s = Math.sin(this.angle);
      const lx = c * d.x + s * d.y;
      const ly = -s * d.x + c * d.y;
      return Math.abs(lx) <= this.halfW && Math.abs(ly) <= this.halfH;
    }

    integrateVelocity(g, dt) {
      if (this.isStatic) return;
      this.vel.x += g.x * dt;
      this.vel.y += g.y * dt;
    }

    integratePosition(dt) {
      if (this.isStatic) return;
      this.pos.x += this.vel.x * dt;
      this.pos.y += this.vel.y * dt;
      this.angle += this.angVel * dt;
    }
  }

  // ---- Collision detection -------------------------------------------------
  // Every function returns an array of contacts {n, p, depth} where n points from A to B.

  function circleCircle(A, B) {
    const d = V.sub(B.pos, A.pos);
    const dist = V.len(d);
    const r = A.radius + B.radius;
    if (dist >= r) return null;
    const n = dist > 1e-9 ? V.scale(d, 1 / dist) : { x: 0, y: 1 };
    const depth = r - dist;
    return [{ n, depth, p: V.add(A.pos, V.scale(n, A.radius - depth / 2)) }];
  }

  // Outward unit normal of edge a->b of convex polygon P (centroid = P.pos).
  function outwardNormal(P, a, b) {
    let n = V.norm(V.perp(V.sub(b, a)));
    const mid = V.scale(V.add(a, b), 0.5);
    if (V.dot(n, V.sub(mid, P.pos)) < 0) n = V.scale(n, -1);
    return n;
  }

  // Best separating face of P against Q (largest separation = least penetration).
  function bestFace(P, Q) {
    const pv = P.worldVerts();
    const qv = Q.worldVerts();
    let best = null;
    for (let i = 0; i < pv.length; i++) {
      const a = pv[i], b = pv[(i + 1) % pv.length];
      const n = outwardNormal(P, a, b);
      let sep = Infinity;
      for (const q of qv) {
        const s = V.dot(n, V.sub(q, a));
        if (s < sep) sep = s;
      }
      if (!best || sep > best.sep) best = { sep, n, a };
    }
    return best;
  }

  function boxBox(A, B) {
    const f1 = bestFace(A, B);
    if (f1.sep > 0) return null;
    const f2 = bestFace(B, A);
    if (f2.sep > 0) return null;

    // Reference face is the one with less penetration (small bias towards A).
    const refIsA = f1.sep + 0.01 >= f2.sep;
    const R = refIsA ? A : B;
    const I = refIsA ? B : A;
    const f = refIsA ? f1 : f2;
    const n = f.n; // outward from reference body

    const contacts = [];
    for (const v of I.worldVerts()) {
      const s = V.dot(n, V.sub(v, f.a));
      if (s < 0) {
        contacts.push({ n: refIsA ? n : V.scale(n, -1), p: v, depth: -s });
      }
    }
    return contacts.length ? contacts : null;
  }

  // Circle C against polygon P. Returns normal pointing from P to C.
  function polyCircle(P, C) {
    const pv = P.worldVerts();
    const c = C.pos, r = C.radius;
    let inside = true;
    let bestSep = -Infinity, bestN = null;
    let minDist = Infinity, closest = null;

    for (let i = 0; i < pv.length; i++) {
      const a = pv[i], b = pv[(i + 1) % pv.length];
      const n = outwardNormal(P, a, b);
      const s = V.dot(n, V.sub(c, a));
      if (s > 0) inside = false;
      if (s > bestSep) { bestSep = s; bestN = n; }

      const ab = V.sub(b, a);
      const t = Math.max(0, Math.min(1, V.dot(V.sub(c, a), ab) / (V.dot(ab, ab) || 1)));
      const q = V.add(a, V.scale(ab, t));
      const d = V.len(V.sub(c, q));
      if (d < minDist) { minDist = d; closest = q; }
    }

    if (inside) return { n: bestN, depth: r - bestSep, p: c };
    if (minDist >= r) return null;
    return { n: V.norm(V.sub(c, closest)), depth: r - minDist, p: closest };
  }

  function collide(A, B) {
    if (A.type === 'circle' && B.type === 'circle') return circleCircle(A, B);
    if (A.type === 'box' && B.type === 'box') return boxBox(A, B);
    if (A.type === 'circle') {
      const r = polyCircle(B, A);
      return r ? [{ n: V.scale(r.n, -1), p: r.p, depth: r.depth }] : null;
    }
    const r = polyCircle(A, B);
    return r ? [{ n: r.n, p: r.p, depth: r.depth }] : null;
  }

  // ---- Solver --------------------------------------------------------------

  function solveContact(c) {
    const A = c.a, B = c.b, n = c.n;
    const rA = V.sub(c.p, A.pos), rB = V.sub(c.p, B.pos);

    // Normal impulse
    let rv = V.sub(pointVelocity(B, rB), pointVelocity(A, rA));
    let vn = V.dot(rv, n);
    const rnA = V.cross(rA, n), rnB = V.cross(rB, n);
    const kN = A.invMass + B.invMass + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB;
    if (kN <= 0) return;

    if (c.bias === undefined) {
      const e = Math.min(A.restitution, B.restitution);
      c.bias = vn < -2 ? -e * vn : 0; // restitution only for real impacts (avoids jitter)
    }
    let j = (c.bias - vn) / kN;
    const newAcc = Math.max(c.accN + j, 0);
    j = newAcc - c.accN;
    c.accN = newAcc;
    applyImpulse(A, B, rA, rB, V.scale(n, j));

    // Friction impulse
    rv = V.sub(pointVelocity(B, rB), pointVelocity(A, rA));
    const t = V.perp(n);
    const vt = V.dot(rv, t);
    const rtA = V.cross(rA, t), rtB = V.cross(rB, t);
    const kT = A.invMass + B.invMass + A.invInertia * rtA * rtA + B.invInertia * rtB * rtB;
    if (kT <= 0) return;
    let jt = -vt / kT;
    const maxF = Math.sqrt(A.friction * B.friction) * c.accN;
    const newT = Math.max(-maxF, Math.min(maxF, c.accT + jt));
    jt = newT - c.accT;
    c.accT = newT;
    applyImpulse(A, B, rA, rB, V.scale(t, jt));
  }

  function applyImpulse(A, B, rA, rB, P) {
    A.vel.x -= P.x * A.invMass;
    A.vel.y -= P.y * A.invMass;
    A.angVel -= A.invInertia * V.cross(rA, P);
    B.vel.x += P.x * B.invMass;
    B.vel.y += P.y * B.invMass;
    B.angVel += B.invInertia * V.cross(rB, P);
  }

  function correctPosition(c) {
    const slop = 0.5, percent = 0.4;
    const invSum = c.a.invMass + c.b.invMass;
    if (invSum === 0) return;
    const corr = (Math.max(c.depth - slop, 0) * percent) / invSum;
    const P = V.scale(c.n, corr);
    c.a.pos = V.sub(c.a.pos, V.scale(P, c.a.invMass));
    c.b.pos = V.add(c.b.pos, V.scale(P, c.b.invMass));
  }

  // ---- World ---------------------------------------------------------------

  class World {
    constructor() {
      this.bodies = [];
      this.gravity = { x: 0, y: 900 };
      this.iterations = 10;
      this.substeps = 4;
    }

    add(body) { this.bodies.push(body); return body; }

    remove(body) {
      const i = this.bodies.indexOf(body);
      if (i >= 0) this.bodies.splice(i, 1);
    }

    clear() { this.bodies.length = 0; }

    step(dt) {
      const h = dt / this.substeps;
      for (let s = 0; s < this.substeps; s++) this._substep(h);
    }

    _substep(dt) {
      const bodies = this.bodies;
      for (const b of bodies) b.integrateVelocity(this.gravity, dt);

      const contacts = [];
      for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
          const A = bodies[i], B = bodies[j];
          if (A.isStatic && B.isStatic) continue;
          const dx = B.pos.x - A.pos.x, dy = B.pos.y - A.pos.y;
          const rr = A.boundRadius + B.boundRadius;
          if (dx * dx + dy * dy > rr * rr) continue; // broadphase
          const cs = collide(A, B);
          if (!cs) continue;
          for (const c of cs) {
            contacts.push({ a: A, b: B, n: c.n, p: c.p, depth: c.depth, accN: 0, accT: 0, bias: undefined });
          }
        }
      }

      for (let k = 0; k < this.iterations; k++) {
        for (const c of contacts) solveContact(c);
      }

      for (const b of bodies) b.integratePosition(dt);
      for (const c of contacts) correctPosition(c);
    }
  }

  global.Physics = { Body, World, V };
})(window);
/* Basic 2D rigid body physics engine.
 * Shapes: circles and convex boxes (oriented rectangles).
 * Solver: sequential impulses with friction, restitution and positional correction.
 */
(function (global) {
  'use strict';

  const V = {
    dot: (a, b) => a.x * b.x + a.y * b.y,
    cross: (a, b) => a.x * b.y - a.y * b.x,
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
    scale: (a, s) => ({ x: a.x * s, y: a.y * s }),
    len: (a) => Math.hypot(a.x, a.y),
    norm: (a) => {
      const l = Math.hypot(a.x, a.y) || 1;
      return { x: a.x / l, y: a.y / l };
    },
    perp: (a) => ({ x: -a.y, y: a.x }),
  };

  // Velocity of a point at offset r from the centre of mass, for angular velocity w.
  function pointVelocity(body, r) {
    return { x: body.vel.x - body.angVel * r.y, y: body.vel.y + body.angVel * r.x };
  }

  // Angular velocity w crossed with vector r: w x r = (-w*r.y, w*r.x).
  function wcross(w, r) {
    return { x: -w * r.y, y: w * r.x };
  }

  let nextId = 1;

  class Body {
    constructor(opts) {
      this.id = nextId++;
      this.type = opts.type; // 'circle' | 'box'
      this.isStatic = !!opts.isStatic;
      this.pos = { x: opts.x, y: opts.y };
      this.vel = { x: 0, y: 0 };
      this.angle = opts.angle || 0;
      this.angVel = 0;
      this.restitution = opts.restitution ?? 0.3;
      this.friction = opts.friction ?? 0.5;
      this.color = opts.color || '#4cc9f0';

      const density = opts.density ?? 1;
      let area;
      if (this.type === 'circle') {
        this.radius = opts.radius;
        this.boundRadius = opts.radius;
        this.localVerts = null;
        area = Math.PI * opts.radius * opts.radius;
      } else {
        this.halfW = opts.width / 2;
        this.halfH = opts.height / 2;
        this.boundRadius = Math.hypot(this.halfW, this.halfH);
        this.localVerts = [
          { x: -this.halfW, y: -this.halfH },
          { x: this.halfW, y: -this.halfH },
          { x: this.halfW, y: this.halfH },
          { x: -this.halfW, y: this.halfH },
        ];
        area = opts.width * opts.height;
      }

      if (this.isStatic) {
        this.invMass = 0;
        this.invInertia = 0;
      } else {
        const mass = density * area;
        this.invMass = 1 / mass;
        const inertia =
          this.type === 'circle'
            ? 0.5 * mass * this.radius * this.radius
            : (mass * (opts.width * opts.width + opts.height * opts.height)) / 12;
        this.invInertia = 1 / inertia;
      }
    }

    worldVerts() {
      const c = Math.cos(this.angle), s = Math.sin(this.angle);
      return this.localVerts.map((v) => ({
        x: this.pos.x + v.x * c - v.y * s,
        y: this.pos.y + v.x * s + v.y * c,
      }));
    }

    containsPoint(p) {
      const d = V.sub(p, this.pos);
      if (this.type === 'circle') return V.len(d) <= this.radius;
      const c = Math.cos(this.angle), s = Math.sin(this.angle);
      const lx = c * d.x + s * d.y;
      const ly = -s * d.x + c * d.y;
      return Math.abs(lx) <= this.halfW && Math.abs(ly) <= this.halfH;
    }

    integrateVelocity(g, dt) {
      if (this.isStatic) return;
      this.vel.x += g.x * dt;
      this.vel.y += g.y * dt;
    }

    integratePosition(dt) {
      if (this.isStatic) return;
      this.pos.x += this.vel.x * dt;
      this.pos.y += this.vel.y * dt;
      this.angle += this.angVel * dt;
    }
  }

  // ---- Collision detection -------------------------------------------------
  // Every function returns an array of contacts {n, p, depth} where n points from A to B.

  function circleCircle(A, B) {
    const d = V.sub(B.pos, A.pos);
    const dist = V.len(d);
    const r = A.radius + B.radius;
    if (dist >= r) return null;
    const n = dist > 1e-9 ? V.scale(d, 1 / dist) : { x: 0, y: 1 };
    const depth = r - dist;
    return [{ n, depth, p: V.add(A.pos, V.scale(n, A.radius - depth / 2)) }];
  }

  // Outward unit normal of edge a->b of convex polygon P (centroid = P.pos).
  function outwardNormal(P, a, b) {
    let n = V.norm(V.perp(V.sub(b, a)));
    const mid = V.scale(V.add(a, b), 0.5);
    if (V.dot(n, V.sub(mid, P.pos)) < 0) n = V.scale(n, -1);
    return n;
  }

  // Best separating face of P against Q (largest separation = least penetration).
  function bestFace(P, Q) {
    const pv = P.worldVerts();
    const qv = Q.worldVerts();
    let best = null;
    for (let i = 0; i < pv.length; i++) {
      const a = pv[i], b = pv[(i + 1) % pv.length];
      const n = outwardNormal(P, a, b);
      let sep = Infinity;
      for (const q of qv) {
        const s = V.dot(n, V.sub(q, a));
        if (s < sep) sep = s;
      }
      if (!best || sep > best.sep) best = { sep, n, a };
    }
    return best;
  }

  function boxBox(A, B) {
    const f1 = bestFace(A, B);
    if (f1.sep > 0) return null;
    const f2 = bestFace(B, A);
    if (f2.sep > 0) return null;

    // Reference face is the one with less penetration (small bias towards A).
    const refIsA = f1.sep + 0.01 >= f2.sep;
    const I = refIsA ? B : A;
    const f = refIsA ? f1 : f2;
    const n = f.n; // outward from reference body

    const contacts = [];
    for (const v of I.worldVerts()) {
      const s = V.dot(n, V.sub(v, f.a));
      if (s < 0) {
        contacts.push({ n: refIsA ? n : V.scale(n, -1), p: v, depth: -s });
      }
    }
    return contacts.length ? contacts : null;
  }

  // Circle C against polygon P. Returns normal pointing from P to C.
  function polyCircle(P, C) {
    const pv = P.worldVerts();
    const c = C.pos, r = C.radius;
    let inside = true;
    let bestSep = -Infinity, bestN = null;
    let minDist = Infinity, closest = null;

    for (let i = 0; i < pv.length; i++) {
      const a = pv[i], b = pv[(i + 1) % pv.length];
      const n = outwardNormal(P, a, b);
      const s = V.dot(n, V.sub(c, a));
      if (s > 0) inside = false;
      if (s > bestSep) { bestSep = s; bestN = n; }

      const ab = V.sub(b, a);
      const t = Math.max(0, Math.min(1, V.dot(V.sub(c, a), ab) / (V.dot(ab, ab) || 1)));
      const q = V.add(a, V.scale(ab, t));
      const d = V.len(V.sub(c, q));
      if (d < minDist) { minDist = d; closest = q; }
    }

    if (inside) return { n: bestN, depth: r - bestSep, p: c };
    if (minDist >= r) return null;
    return { n: V.norm(V.sub(c, closest)), depth: r - minDist, p: closest };
  }

  function collide(A, B) {
    if (A.type === 'circle' && B.type === 'circle') return circleCircle(A, B);
    if (A.type === 'box' && B.type === 'box') return boxBox(A, B);
    if (A.type === 'circle') {
      const r = polyCircle(B, A);
      return r ? [{ n: V.scale(r.n, -1), p: r.p, depth: r.depth }] : null;
    }
    const r = polyCircle(A, B);
    return r ? [{ n: r.n, p: r.p, depth: r.depth }] : null;
  }

  // ---- Solver --------------------------------------------------------------

  function solveContact(c) {
    const A = c.a, B = c.b, n = c.n;
    const rA = V.sub(c.p, A.pos), rB = V.sub(c.p, B.pos);

    // Normal impulse
    let rv = V.sub(pointVelocity(B, rB), pointVelocity(A, rA));
    let vn = V.dot(rv, n);
    const rnA = V.cross(rA, n), rnB = V.cross(rB, n);
    const kN = A.invMass + B.invMass + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB;
    if (kN <= 0) return;

    if (c.bias === undefined) {
      const e = Math.min(A.restitution, B.restitution);
      c.bias = vn < -2 ? -e * vn : 0; // restitution only for real impacts (avoids jitter)
    }
    let j = (c.bias - vn) / kN;
    const newAcc = Math.max(c.accN + j, 0);
    j = newAcc - c.accN;
    c.accN = newAcc;
    applyImpulse(A, B, rA, rB, V.scale(n, j));

    // Friction impulse
    rv = V.sub(pointVelocity(B, rB), pointVelocity(A, rA));
    const t = V.perp(n);
    const vt = V.dot(rv, t);
    const rtA = V.cross(rA, t), rtB = V.cross(rB, t);
    const kT = A.invMass + B.invMass + A.invInertia * rtA * rtA + B.invInertia * rtB * rtB;
    if (kT <= 0) return;
    let jt = -vt / kT;
    const maxF = Math.sqrt(A.friction * B.friction) * c.accN;
    const newT = Math.max(-maxF, Math.min(maxF, c.accT + jt));
    jt = newT - c.accT;
    c.accT = newT;
    applyImpulse(A, B, rA, rB, V.scale(t, jt));
  }

  function applyImpulse(A, B, rA, rB, P) {
    A.vel.x -= P.x * A.invMass;
    A.vel.y -= P.y * A.invMass;
    A.angVel -= A.invInertia * V.cross(rA, P);
    B.vel.x += P.x * B.invMass;
    B.vel.y += P.y * B.invMass;
    B.angVel += B.invInertia * V.cross(rB, P);
  }

  function correctPosition(c) {
    const slop = 0.5, percent = 0.4;
    const invSum = c.a.invMass + c.b.invMass;
    if (invSum === 0) return;
    const corr = (Math.max(c.depth - slop, 0) * percent) / invSum;
    const P = V.scale(c.n, corr);
    c.a.pos = V.sub(c.a.pos, V.scale(P, c.a.invMass));
    c.b.pos = V.add(c.b.pos, V.scale(P, c.b.invMass));
  }

  // ---- World ---------------------------------------------------------------

  class World {
    constructor() {
      this.bodies = [];
      this.gravity = { x: 0, y: 900 };
      this.iterations = 10;
      this.substeps = 4;
    }

    add(body) { this.bodies.push(body); return body; }

    remove(body) {
      const i = this.bodies.indexOf(body);
      if (i >= 0) this.bodies.splice(i, 1);
    }

    clear() { this.bodies.length = 0; }

    step(dt) {
      const h = dt / this.substeps;
      for (let s = 0; s < this.substeps; s++) this._substep(h);
    }

    _substep(dt) {
      const bodies = this.bodies;
      for (const b of bodies) b.integrateVelocity(this.gravity, dt);

      const contacts = [];
      for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
          const A = bodies[i], B = bodies[j];
          if (A.isStatic && B.isStatic) continue;
          const dx = B.pos.x - A.pos.x, dy = B.pos.y - A.pos.y;
          const rr = A.boundRadius + B.boundRadius;
          if (dx * dx + dy * dy > rr * rr) continue; // broadphase
          const cs = collide(A, B);
          if (!cs) continue;
          for (const c of cs) {
            contacts.push({ a: A, b: B, n: c.n, p: c.p, depth: c.depth, accN: 0, accT: 0, bias: undefined });
          }
        }
      }

      for (let k = 0; k < this.iterations; k++) {
        for (const c of contacts) solveContact(c);
      }

      for (const b of bodies) b.integratePosition(dt);
      for (const c of contacts) correctPosition(c);
    }
  }

  global.Physics = { Body, World, V };
})(window);
