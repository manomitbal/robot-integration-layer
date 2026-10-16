// ---------------------------------------------------------------------------
// SHARED PLATFORM — kinematics
// Generic URDF parser + forward kinematics + damped-least-squares IK.
// Nothing in here knows which OEM it is talking to. In production this role is
// played by MoveIt 2 / the robot's own planner; the RIL only needs reachability
// checks and simple point-to-point plans to validate tasks against manifests.
// Matrices are row-major 4x4 arrays of length 16.
// ---------------------------------------------------------------------------
RIL.Kin = (() => {
  const I = () => [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
  function mul(a, b) {
    const o = new Array(16);
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
      o[r*4+c] = a[r*4]*b[c] + a[r*4+1]*b[4+c] + a[r*4+2]*b[8+c] + a[r*4+3]*b[12+c];
    }
    return o;
  }
  function fromXYZRPY(xyz, rpy) {
    const [r, p, y] = rpy;
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    // URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [
      cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, xyz[0],
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, xyz[1],
      -sp,   cp*sr,            cp*cr,            xyz[2],
      0, 0, 0, 1];
  }
  function axisAngle(ax, q) {
    const n = Math.hypot(ax[0], ax[1], ax[2]) || 1;
    const x = ax[0]/n, y = ax[1]/n, z = ax[2]/n, c = Math.cos(q), s = Math.sin(q), t = 1 - c;
    return [
      t*x*x + c,   t*x*y - s*z, t*x*z + s*y, 0,
      t*x*y + s*z, t*y*y + c,   t*y*z - s*x, 0,
      t*x*z - s*y, t*y*z + s*x, t*z*z + c,   0,
      0, 0, 0, 1];
  }
  const nums = (s, d) => (s == null ? d : s.trim().split(/\s+/).map(Number));
  const apply = (T, v) => [T[0]*v[0]+T[1]*v[1]+T[2]*v[2]+T[3], T[4]*v[0]+T[5]*v[1]+T[6]*v[2]+T[7], T[8]*v[0]+T[9]*v[1]+T[10]*v[2]+T[11]];
  const rot = (T, v) => [T[0]*v[0]+T[1]*v[1]+T[2]*v[2], T[4]*v[0]+T[5]*v[1]+T[6]*v[2], T[8]*v[0]+T[9]*v[1]+T[10]*v[2]];
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];

  class KinModel {
    // xmlDoc: a parsed URDF DOM (DOMParser in browser, xmldom in tests)
    constructor(xmlDoc, rootLink) {
      this.links = {};
      this.joints = {};
      this.childrenOf = {};
      const robot = xmlDoc.getElementsByTagName('robot')[0];
      this.name = robot.getAttribute('name');
      const L = robot.getElementsByTagName('link');
      for (let i = 0; i < L.length; i++) {
        const el = L[i], visuals = [];
        const V = el.getElementsByTagName('visual');
        for (let k = 0; k < V.length; k++) {
          const o = V[k].getElementsByTagName('origin')[0];
          const m = V[k].getElementsByTagName('mesh')[0];
          if (!m) continue;
          visuals.push({
            T: fromXYZRPY(nums(o && o.getAttribute('xyz'), [0,0,0]), nums(o && o.getAttribute('rpy'), [0,0,0])),
            mesh: m.getAttribute('filename'),
            scale: nums(m.getAttribute('scale'), [1,1,1]),
          });
        }
        this.links[el.getAttribute('name')] = { name: el.getAttribute('name'), visuals };
      }
      const J = robot.getElementsByTagName('joint');
      for (let i = 0; i < J.length; i++) {
        const el = J[i];
        const o = el.getElementsByTagName('origin')[0];
        const ax = el.getElementsByTagName('axis')[0];
        const lim = el.getElementsByTagName('limit')[0];
        const j = {
          name: el.getAttribute('name'),
          type: el.getAttribute('type'),
          parent: el.getElementsByTagName('parent')[0].getAttribute('link'),
          child: el.getElementsByTagName('child')[0].getAttribute('link'),
          T: fromXYZRPY(nums(o && o.getAttribute('xyz'), [0,0,0]), nums(o && o.getAttribute('rpy'), [0,0,0])),
          axis: nums(ax && ax.getAttribute('xyz'), [1,0,0]),
          lower: lim ? Number(lim.getAttribute('lower')) : -Math.PI,
          upper: lim ? Number(lim.getAttribute('upper')) : Math.PI,
          velocity: lim ? Number(lim.getAttribute('velocity')) : 1,
        };
        this.joints[j.name] = j;
        (this.childrenOf[j.parent] = this.childrenOf[j.parent] || []).push(j);
      }
      this.root = rootLink;
    }
    // Active (non-fixed) joints on the path root -> link, in order.
    chain(toLink) {
      const parentJoint = {};
      Object.values(this.joints).forEach(j => { parentJoint[j.child] = j; });
      const out = [];
      let l = toLink;
      while (l !== this.root) {
        const j = parentJoint[l];
        if (!j) throw new Error(`link ${toLink} not under ${this.root}`);
        out.unshift(j); l = j.parent;
      }
      return out;
    }
    // World transform of every link under root, given joint positions by name.
    fk(q, baseT = I()) {
      const out = { [this.root]: baseT };
      const walk = (link) => {
        (this.childrenOf[link] || []).forEach(j => {
          let T = mul(out[link], j.T);
          if (j.type !== 'fixed') T = mul(T, axisAngle(j.axis, q[j.name] || 0));
          out[j.child] = T;
          walk(j.child);
        });
      };
      walk(this.root);
      return out;
    }
  }

  // Solve for joint positions so that the TCP reaches target.p with the tool's
  // approach axis aligned to target.approach (5-DOF task; yaw about the
  // approach axis is left free, which is what a symmetric pick needs).
  function solveIK(model, tool, q0, target, baseT, opts = {}) {
    const joints = model.chain(tool.link).filter(j => j.type !== 'fixed');
    const names = joints.map(j => j.name);
    const iters = opts.iters || 250, lambda = opts.lambda || 0.08, wOri = opts.wOri || 0.6;
    const tcp = (q) => {
      const T = model.fk(q, baseT)[tool.link];
      return { p: apply(T, tool.offset), a: rot(T, tool.approach) };
    };
    const err = (q) => {
      const s = tcp(q);
      const ep = [target.p[0]-s.p[0], target.p[1]-s.p[1], target.p[2]-s.p[2]];
      const eo = cross(s.a, target.approach).map(x => x * wOri);
      return { e: ep.concat(eo), np: Math.hypot(...ep), no: Math.hypot(...eo) / wOri, dot: s.a[0]*target.approach[0]+s.a[1]*target.approach[1]+s.a[2]*target.approach[2] };
    };
    const tryFrom = (seed) => {
      const q = Object.assign({}, seed);
      for (let it = 0; it < iters; it++) {
        const E = err(q);
        if (E.np < 1e-3 && E.no < 2e-3 && E.dot > 0) return { ok: true, q, iters: it };
        // numerical Jacobian, 6 x n
        const n = names.length, h = 1e-5, Jm = [];
        for (let k = 0; k < n; k++) {
          const qq = Object.assign({}, q); qq[names[k]] += h;
          const E2 = err(qq);
          Jm.push(E.e.map((v, r) => (v - E2.e[r]) / h));      // d(-e)/dq = J
        }
        // dq = J^T (J J^T + l^2 I)^-1 e
        const A = [...Array(6)].map((_, r) => [...Array(6)].map((_, c) => {
          let s = 0; for (let k = 0; k < n; k++) s += Jm[k][r] * Jm[k][c];
          return s + (r === c ? lambda * lambda : 0);
        }));
        const y = gauss(A, E.e.slice());
        if (!y) break;
        let maxStep = 0;
        const dq = names.map((_, k) => { let s = 0; for (let r = 0; r < 6; r++) s += Jm[k][r] * y[r]; maxStep = Math.max(maxStep, Math.abs(s)); return s; });
        const scale = maxStep > 0.25 ? 0.25 / maxStep : 1;
        names.forEach((nm, k) => {
          const j = joints[k];
          q[nm] = Math.min(j.upper, Math.max(j.lower, q[nm] + dq[k] * scale));
        });
      }
      const E = err(q);
      return { ok: E.np < 2e-3 && E.no < 5e-3 && E.dot > 0, q, residual: E.np };
    };
    let best = tryFrom(q0);
    for (let r = 0; !best.ok && r < (opts.restarts ?? 6); r++) {
      const seed = {};
      names.forEach(nm => { seed[nm] = q0[nm] + (Math.random() - 0.5) * 0.9; });
      const res = tryFrom(seed);
      if (res.ok || (res.residual < (best.residual ?? 1e9))) best = res;
    }
    return best;
  }

  function gauss(A, b) {
    const n = b.length;
    for (let i = 0; i < n; i++) {
      let p = i;
      for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
      if (Math.abs(A[p][i]) < 1e-12) return null;
      [A[i], A[p]] = [A[p], A[i]]; [b[i], b[p]] = [b[p], b[i]];
      for (let r = i + 1; r < n; r++) {
        const f = A[r][i] / A[i][i];
        for (let c = i; c < n; c++) A[r][c] -= f * A[i][c];
        b[r] -= f * b[i];
      }
    }
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
      let s = b[i]; for (let c = i + 1; c < n; c++) s -= A[i][c] * x[c];
      x[i] = s / A[i][i];
    }
    return x;
  }

  return { I, mul, fromXYZRPY, axisAngle, apply, rot, KinModel, solveIK };
})();
