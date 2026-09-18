/*
 * solver.js — the engineering core of the truss analyser.
 *
 * Nothing in this file knows about the canvas or the DOM. It takes a model
 * (joints, members, supports, loads) and returns member forces, support
 * reactions, a stability classification and joint displacements.
 *
 * UNITS (used consistently everywhere):
 *   length  m        force  kN        E  GPa        A  mm²
 *   E [GPa] × A [mm²] = (E×10⁶ kN/m²)(A×10⁻⁶ m²) = E×A kN, so EA in kN is simply E·A.
 *
 * SIGN CONVENTION:
 *   Member force T > 0 is TENSION (member pulls on its joints),
 *   T < 0 is COMPRESSION (member pushes on its joints).
 *   Global axes: x to the right, y upwards. Reactions are positive in +x / +y.
 *
 * METHOD OF JOINTS AS A MATRIX
 *   Every joint is a pin, so each joint gives exactly two equations: ΣFx = 0, ΣFy = 0.
 *   With j joints that is 2j equations. The unknowns are the m member forces plus the
 *   r support reaction components. For joint i and a member k running from i to n, the
 *   unit vector from i towards n is (cx, cy) = ((xn − xi)/L, (yn − yi)/L).
 *   A tensile force T_k pulls joint i TOWARDS n, so it adds  +cx·T_k to ΣFx at i
 *   and +cy·T_k to ΣFy at i. It pulls joint n towards i, so it adds −cx·T_k and −cy·T_k at n.
 *   A reaction component R acting on joint i in x adds +1·R to ΣFx at i.
 *   External loads P are known, so they move to the right-hand side:
 *
 *        Σ (member terms) + Σ (reaction terms) + P = 0      →      A · x = −P
 *
 *   A is the (2j × (m + r)) "equilibrium matrix": one row per joint direction, one column
 *   per unknown. Each member column has at most four non-zeros (±cx, ±cy);
 *   each reaction column has a single 1.
 */
(function (root) {
  'use strict';

  // Pivots smaller than this are treated as zero. Matrix entries are direction cosines
  // (|value| ≤ 1) or 1, so an absolute tolerance is appropriate: a genuinely singular
  // system produces pivots of order 1e-16 from round-off, far below this.
  const PIVOT_TOL = 1e-9;

  /* ------------------------------------------------------------------------ *
   * Linear algebra, written from scratch.
   * ------------------------------------------------------------------------ */

  /**
   * Solve the square system M·x = b by Gaussian elimination with partial pivoting.
   *
   * Stage 1 — forward elimination. For each column c we:
   *   (a) pick the row at or below c with the largest |value| in column c and swap it up.
   *       This is "partial pivoting": dividing by the largest available number keeps
   *       round-off error small, and a pivot of ~0 tells us the matrix is singular.
   *   (b) subtract multiples of the pivot row from every row below it so that column c
   *       becomes zero under the diagonal. Doing the same operation to b keeps the
   *       system equivalent (we are only adding multiples of one equation to another).
   * After this, M is upper-triangular.
   *
   * Stage 2 — back substitution. The last equation has one unknown, so solve it; then
   * the second-last has one new unknown, and so on upwards.
   *
   * Cost is O(n³), which is trivial for trusses of a few hundred unknowns.
   * Returns { x, singular }. Inputs are copied, never modified.
   */
  function solveLinearSystem(M, b) {
    const n = M.length;
    const a = M.map(row => row.slice()); // working copy of the matrix
    const rhs = b.slice();               // working copy of the right-hand side

    for (let c = 0; c < n; c++) {
      // (a) Partial pivoting: find the largest magnitude entry in column c, rows c..n-1.
      let pivotRow = c;
      let best = Math.abs(a[c][c]);
      for (let r = c + 1; r < n; r++) {
        const v = Math.abs(a[r][c]);
        if (v > best) { best = v; pivotRow = r; }
      }
      if (best < PIVOT_TOL) return { x: null, singular: true };

      if (pivotRow !== c) {
        [a[c], a[pivotRow]] = [a[pivotRow], a[c]];
        [rhs[c], rhs[pivotRow]] = [rhs[pivotRow], rhs[c]];
      }

      // (b) Eliminate column c from every row below the pivot.
      for (let r = c + 1; r < n; r++) {
        const factor = a[r][c] / a[c][c];
        if (factor === 0) continue;       // row already has a zero here (common: A is sparse)
        for (let k = c; k < n; k++) a[r][k] -= factor * a[c][k];
        rhs[r] -= factor * rhs[c];
      }
    }

    // Back substitution on the upper-triangular system.
    const x = new Array(n).fill(0);
    for (let r = n - 1; r >= 0; r--) {
      let sum = rhs[r];
      for (let k = r + 1; k < n; k++) sum -= a[r][k] * x[k];
      x[r] = sum / a[r][r];
    }
    return { x, singular: false };
  }

  /**
   * Rank of a (possibly rectangular) matrix: the number of independent rows,
   * which equals the number of independent columns.
   *
   * Same forward elimination as above, but when a column has no usable pivot we
   * simply move on to the next column instead of giving up. Each pivot found is one
   * independent equation, so the pivot count is the rank.
   */
  function matrixRank(M) {
    const rows = M.length;
    if (rows === 0) return 0;
    const cols = M[0].length;
    const a = M.map(row => row.slice());
    let rank = 0;

    for (let c = 0; c < cols && rank < rows; c++) {
      let pivotRow = rank;
      let best = Math.abs(a[rank][c]);
      for (let r = rank + 1; r < rows; r++) {
        const v = Math.abs(a[r][c]);
        if (v > best) { best = v; pivotRow = r; }
      }
      if (best < PIVOT_TOL) continue; // this column depends on earlier ones

      [a[rank], a[pivotRow]] = [a[pivotRow], a[rank]];
      for (let r = rank + 1; r < rows; r++) {
        const factor = a[r][c] / a[rank][c];
        if (factor === 0) continue;
        for (let k = c; k < cols; k++) a[r][k] -= factor * a[rank][k];
      }
      rank++;
    }
    return rank;
  }

  /**
   * Basis for the null space of M: every vector v with M·v = 0.
   *
   * Reduce M to reduced row-echelon form (each pivot scaled to 1, with zeros both above
   * and below it). Columns without a pivot are "free": each free variable can be set to 1
   * (the others to 0) and the pivot variables then follow directly, giving one basis vector.
   * Used on Aᵀ to find mechanisms: displacements u that stretch no member (Aᵀ·u = 0).
   */
  function nullSpace(M) {
    const rows = M.length;
    if (rows === 0) return [];
    const cols = M[0].length;
    const a = M.map(row => row.slice());
    const pivotCols = [];
    let r = 0;
    for (let c = 0; c < cols && r < rows; c++) {
      let pivotRow = r, best = Math.abs(a[r][c]);
      for (let q = r + 1; q < rows; q++) if (Math.abs(a[q][c]) > best) { best = Math.abs(a[q][c]); pivotRow = q; }
      if (best < PIVOT_TOL) continue;
      [a[r], a[pivotRow]] = [a[pivotRow], a[r]];
      const p = a[r][c];
      for (let k = c; k < cols; k++) a[r][k] /= p;                 // scale pivot to 1
      for (let q = 0; q < rows; q++) {                             // clear the rest of the column
        if (q === r || a[q][c] === 0) continue;
        const f = a[q][c];
        for (let k = c; k < cols; k++) a[q][k] -= f * a[r][k];
      }
      pivotCols.push(c);
      r++;
    }
    const isPivot = new Set(pivotCols);
    const basis = [];
    for (let free = 0; free < cols; free++) {
      if (isPivot.has(free)) continue;
      const v = new Array(cols).fill(0);
      v[free] = 1;
      pivotCols.forEach((pc, i) => { v[pc] = -a[i][free]; });
      basis.push(v);
    }
    return basis;
  }

  function transpose(M) {
    if (M.length === 0) return [];
    return M[0].map((_, c) => M.map(row => row[c]));
  }

  /* ------------------------------------------------------------------------ *
   * Truss analysis.
   * ------------------------------------------------------------------------ */

  /** Number of reaction components a support provides, and in which directions. */
  function supportComponents(support) {
    if (support === 'pin') return ['x', 'y'];   // pin: restrains both directions
    if (support === 'roller') return ['y'];     // roller on horizontal ground: restrains y only
    return [];
  }

  /**
   * Build the equilibrium matrix A and load vector P for a model.
   *
   * model = {
   *   joints:  [{ id, x, y, support: 'none'|'pin'|'roller', fx, fy }],   (fx, fy: applied load, kN)
   *   members: [{ id, a, b }]                                             (a, b: joint ids)
   * }
   *
   * Row 2i is ΣFx at joint i, row 2i+1 is ΣFy at joint i.
   * Columns 0..m-1 are the member forces, columns m..m+r-1 the reaction components.
   */
  function buildEquilibrium(model) {
    const joints = model.joints;
    const members = model.members;
    const index = new Map(joints.map((jt, i) => [jt.id, i]));

    const reactions = [];
    joints.forEach((jt, i) => {
      for (const dir of supportComponents(jt.support)) reactions.push({ joint: i, jointId: jt.id, dir });
    });

    const nRows = 2 * joints.length;
    const nCols = members.length + reactions.length;
    const A = Array.from({ length: nRows }, () => new Array(nCols).fill(0));

    const geometry = members.map((mem, k) => {
      const i = index.get(mem.a);
      const n = index.get(mem.b);
      const dx = joints[n].x - joints[i].x;
      const dy = joints[n].y - joints[i].y;
      const L = Math.hypot(dx, dy);
      const cx = dx / L, cy = dy / L;       // unit vector from joint a towards joint b

      // Tension pulls joint a towards b (+c) and joint b towards a (−c).
      A[2 * i][k] += cx;
      A[2 * i + 1][k] += cy;
      A[2 * n][k] -= cx;
      A[2 * n + 1][k] -= cy;
      return { i, n, L, cx, cy };
    });

    reactions.forEach((rc, q) => {
      const col = members.length + q;
      A[2 * rc.joint + (rc.dir === 'x' ? 0 : 1)][col] = 1;
    });

    // Known applied loads. They sit on the left of ΣF = 0, so they become −P on the right.
    const P = new Array(nRows).fill(0);
    joints.forEach((jt, i) => {
      P[2 * i] = jt.fx || 0;
      P[2 * i + 1] = jt.fy || 0;
    });

    return { A, P, reactions, geometry, index };
  }

  /**
   * Classify the structure.
   *
   * Maxwell's counting rule compares equations (2j) with unknowns (m + r):
   *   m + r < 2j  → too few unknowns: definitely a mechanism.
   *   m + r = 2j  → possibly statically determinate.
   *   m + r > 2j  → too many unknowns: statically indeterminate (redundant).
   *
   * The count is NECESSARY but NOT SUFFICIENT: it cannot see geometry. Three parallel
   * rollers, or a well-braced panel next to a floppy one, can pass the count and still
   * collapse. The rank ρ of A settles it exactly (Pellegrino & Calladine, 1986):
   *   k = 2j − ρ          independent mechanisms (ways it can move with no member stretching)
   *   s = (m + r) − ρ     independent states of self-stress (degree of indeterminacy)
   * and the count is recovered as  (m + r) − 2j = s − k.
   * Only k = 0 AND s = 0 is statically determinate AND stable, which is exactly the case
   * where A is square and invertible, so the method of joints has a unique solution.
   */
  function classify(j, m, r, rank) {
    const equations = 2 * j;
    const unknowns = m + r;
    const k = equations - rank;
    const s = unknowns - rank;
    let status, title, detail;

    if (k === 0 && s === 0) {
      status = 'determinate';
      title = 'Statically determinate and stable';
      detail = `2j = ${equations} equations and m + r = ${unknowns} unknowns, and the equations are independent (rank ${rank}). ` +
        `Equilibrium alone fixes every member force, so the method of joints gives a unique answer.`;
    } else if (k === 0) {
      status = 'indeterminate';
      title = `Statically indeterminate to degree ${s}`;
      detail = `m + r = ${unknowns} is more than 2j = ${equations}. The truss is stable, but it has ${s} more unknown${s > 1 ? 's' : ''} than independent equilibrium equations, ` +
        `so equilibrium alone cannot decide how redundant members share the load. You would need compatibility (member stiffness, e.g. the stiffness method). ` +
        `Remove ${s} redundant member${s > 1 ? 's' : ''} or reaction${s > 1 ? 's' : ''} to solve it by the method of joints.`;
    } else {
      // k > 0: it can move. Say whether the count already predicted that or whether geometry is to blame.
      status = 'mechanism';
      const ways = `${k} independent way${k > 1 ? 's' : ''} to move without any member changing length`;
      if (unknowns < equations && s === 0) {
        title = 'Mechanism: not enough members or supports';
        detail = `m + r = ${unknowns} is less than 2j = ${equations}. With fewer unknowns than equations there are loads it cannot balance, so it would fold or slide. ` +
          `It has ${ways}. Add ${k} member${k > 1 ? 's' : ''} or reaction${k > 1 ? 's' : ''} in the right places.`;
      } else {
        title = 'Mechanism: unstable arrangement';
        detail = `The count gives m + r = ${unknowns} against 2j = ${equations}, which ${unknowns >= equations ? 'looks fine' : 'is short'}, but the equations are not independent (rank ${rank} < ${equations}). ` +
          `It has ${ways}, while ${s} unknown${s > 1 ? 's are' : ' is'} redundant elsewhere. Typical causes: all reactions parallel (e.g. only rollers), reactions meeting at one point, ` +
          `or members placed where they duplicate each other instead of where they are needed.`;
      }
    }
    return { status, title, detail, equations, unknowns, rank, k, s };
  }

  /**
   * Full analysis. `EA` is axial rigidity in kN (same for every member).
   * Never throws: degenerate input comes back as { ok: false, status, title, detail }.
   */
  function analyse(model, EA) {
    const joints = model.joints;
    const members = model.members;

    if (joints.length === 0) {
      return { ok: false, status: 'empty', title: 'Nothing to solve', detail: 'Place some joints and connect them with members, or load a preset.' };
    }

    // Degenerate geometry guards. The UI prevents these, but the solver must not trust it.
    const ids = new Set(joints.map(jt => jt.id));
    const byId = new Map(joints.map(jt => [jt.id, jt]));
    for (const mem of members) {
      if (!ids.has(mem.a) || !ids.has(mem.b)) {
        return { ok: false, status: 'invalid', title: 'Invalid member', detail: 'A member refers to a joint that does not exist.' };
      }
      const ja = byId.get(mem.a), jb = byId.get(mem.b);
      if (mem.a === mem.b || Math.hypot(jb.x - ja.x, jb.y - ja.y) < 1e-9) {
        return { ok: false, status: 'invalid', title: 'Zero-length member', detail: 'A member joins two joints at the same point, so it has no direction and cannot carry an axial force.', badMember: mem.id };
      }
    }
    for (let p = 0; p < joints.length; p++) {
      for (let q = p + 1; q < joints.length; q++) {
        if (Math.hypot(joints[p].x - joints[q].x, joints[p].y - joints[q].y) < 1e-9) {
          return { ok: false, status: 'invalid', title: 'Duplicate joints', detail: 'Two joints occupy the same point. Merge or move one of them.' };
        }
      }
    }

    const { A, P, reactions, geometry } = buildEquilibrium(model);
    const m = members.length, r = reactions.length, j = joints.length;
    const cls = classify(j, m, r, matrixRank(A));
    const base = { A, P, reactions, geometry, classification: cls, status: cls.status, title: cls.title, detail: cls.detail };

    if (cls.status !== 'determinate') {
      // For a mechanism, find one way it can move: Aᵀ·u = 0 means no member changes length
      // and no restrained direction moves. Returned per joint so the UI can draw it.
      if (cls.k > 0) {
        const modes = nullSpace(transpose(A));
        if (modes.length) {
          const v = modes[0];
          base.mechanism = joints.map((_, i) => ({ ux: v[2 * i], uy: v[2 * i + 1] }));
        }
      }
      return Object.assign({ ok: false }, base);
    }

    // A is square (2j × 2j) and full rank: solve A·x = −P.
    const eq = solveLinearSystem(A, P.map(v => -v));
    if (eq.singular) {
      // Cannot happen if the rank test passed, but stay defensive.
      return Object.assign({ ok: false }, base, { status: 'mechanism', title: 'Singular equilibrium matrix', detail: 'The equations could not be solved uniquely.' });
    }
    const x = eq.x;
    const forces = x.slice(0, m);
    const reactionValues = reactions.map((rc, q) => Object.assign({}, rc, { value: x[m + q] }));

    // Verification: how well does the solution satisfy every equation? ‖A·x + P‖∞ should be ~1e-15.
    let residual = 0;
    for (let row = 0; row < A.length; row++) {
      let sum = P[row];
      for (let c = 0; c < x.length; c++) sum += A[row][c] * x[c];
      residual = Math.max(residual, Math.abs(sum));
    }

    /*
     * DISPLACEMENTS by static–kinematic duality.
     * Member k's extension is e_k = (u_b − u_a)·c_k. Comparing with column k of A
     * (+c at a, −c at b) gives  e_k = −(column k of A)·u.  A reaction column is a 1 on a
     * restrained direction, whose displacement must be 0. So the compatibility equations are
     *
     *        Aᵀ · u = [ −e ; 0 ]      with  e_k = T_k · L_k / EA   (Hooke's law)
     *
     * The same matrix, transposed: this is the principle of virtual work in matrix form.
     * Because A is square and invertible here, Aᵀ is too, so u is unique.
     */
    const extensions = forces.map((T, k) => T * geometry[k].L / EA);
    const rhs = extensions.map(e => -e).concat(new Array(r).fill(0));
    const kin = solveLinearSystem(transpose(A), rhs);
    const displacements = kin.singular ? null : joints.map((_, i) => ({ ux: kin.x[2 * i], uy: kin.x[2 * i + 1] }));

    return Object.assign({ ok: true }, base, {
      forces, reactions: reactionValues, extensions, displacements, residual,
      lengths: geometry.map(g => g.L),
    });
  }

  const api = { solveLinearSystem, matrixRank, nullSpace, transpose, buildEquilibrium, classify, analyse, supportComponents };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.TrussSolver = api;
})(typeof window !== 'undefined' ? window : this);
