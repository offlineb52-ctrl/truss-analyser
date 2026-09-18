/*
 * tests.js — checks the solver against hand calculations.  Run:  node tests.js
 * Every expected value here was worked out on paper (method of sections / joints /
 * virtual work), independently of the code being tested.
 */
const S = require('./solver.js');
const P = require('./presets.js');

let failures = 0;
function near(actual, expected, label, tol = 1e-6) {
  const ok = Math.abs(actual - expected) <= tol * Math.max(1, Math.abs(expected));
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${label}: got ${actual.toFixed(6)}, expected ${expected.toFixed(6)}`);
}
function is(actual, expected, label) {
  const ok = actual === expected;
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${label}: got ${actual}, expected ${expected}`);
}
const J = (id, x, y, support = 'none', fx = 0, fy = 0) => ({ id, x, y, support, fx, fy });
const M = (id, a, b) => ({ id, a, b });
const EA = 200 * 1000; // steel, 1000 mm² → 200 000 kN
const force = (res, model, id) => res.forces[model.members.findIndex(m => m.id === id)];
const reaction = (res, jointId, dir) => res.reactions.find(r => r.jointId === jointId && r.dir === dir).value;

console.log('\n— Linear solver —');
{
  // 2x + y = 5, x + 3y = 10  →  x = 1, y = 3. Row order forces a pivot swap check too.
  const r = S.solveLinearSystem([[0, 1], [1, 0]], [3, 1]);
  near(r.x[0], 1, 'needs a row swap: x'); near(r.x[1], 3, 'needs a row swap: y');
  const q = S.solveLinearSystem([[2, 1], [1, 3]], [5, 10]);
  near(q.x[0], 1, '2x2: x'); near(q.x[1], 3, '2x2: y');
  is(S.solveLinearSystem([[1, 2], [2, 4]], [1, 2]).singular, true, 'singular matrix detected');
  is(S.matrixRank([[1, 2, 3], [2, 4, 6], [0, 1, 1]]), 2, 'rank of a 3x3 with a repeated row');
}

console.log('\n— Single bar, axial load (T = P, extension = PL/EA) —');
{
  const model = { joints: [J(1, 0, 0, 'pin'), J(2, 4, 0, 'roller', 50, 0)], members: [M(1, 1, 2)] };
  const r = S.analyse(model, EA);
  is(r.status, 'determinate', 'classification');
  near(r.forces[0], 50, 'bar tension');
  near(reaction(r, 1, 'x'), -50, 'pin reaction Rx');
  near(r.displacements[1].ux, 50 * 4 / EA, 'tip displacement = PL/EA');
}

console.log('\n— Symmetric triangle, 10 kN down at apex (span 4, rise 2) —');
{
  const model = {
    joints: [J(1, 0, 0, 'pin'), J(2, 4, 0, 'roller'), J(3, 2, 2, 'none', 0, -10)],
    members: [M(1, 1, 2), M(2, 1, 3), M(3, 2, 3)],
  };
  const r = S.analyse(model, EA);
  // Apex: 2·F·sin45° = 10 (compression) → F = −10/√2.  Bottom joint: F·cos45° balanced by tie → +5.
  near(force(r, model, 2), -10 / Math.SQRT2, 'left rafter (compression)');
  near(force(r, model, 3), -10 / Math.SQRT2, 'right rafter (compression)');
  near(force(r, model, 1), 5, 'tie (tension)');
  near(reaction(r, 1, 'y'), 5, 'left reaction');
  near(reaction(r, 2, 'y'), 5, 'right reaction');
  near(reaction(r, 1, 'x'), 0, 'no horizontal reaction');
  // Virtual work, unit load at apex: δ = Σ F·f·L / EA with f = F/10.
  const d = ((-10 / Math.SQRT2) ** 2 * 2 * Math.SQRT2 * 2 + 5 ** 2 * 4) / (10 * EA);
  near(r.displacements[2].uy, -d, 'apex deflection matches virtual work');
  // Supports are NOT symmetric kinematically: the pin holds the left end, the roller lets the
  // right end slide out by the tie's extension 5·4/EA. The apex therefore moves right by half that.
  near(r.displacements[2].ux, (5 * 4 / EA) / 2, 'apex moves right by half the tie extension');
  near(r.displacements[1].ux, 5 * 4 / EA, 'roller slides out by the tie extension');
  is(r.residual < 1e-12, true, `equilibrium residual tiny (${r.residual.toExponential(1)})`);
}

console.log('\n— Pratt preset: 6 panels × 2 m, height 2 m, 10 kN at each interior bottom joint —');
{
  const model = P.toSolverModel(P.PRESETS.pratt.build());
  const r = S.analyse(model, EA);
  is(r.status, 'determinate', 'classification');
  const byJoints = (a, b) => model.members.findIndex(m => (m.a === a && m.b === b) || (m.a === b && m.b === a));
  const jid = name => model.joints.find(jt => jt.name === name).id;
  near(reaction(r, jid('B0'), 'y'), 25, 'left reaction = 50/2');
  near(reaction(r, jid('B6'), 'y'), 25, 'right reaction = 50/2');
  // Section through panel 3 cuts T2–T3, T2–B3 and B2–B3. The first two meet at T2 (4,2), so take
  // moments about T2 for the left part: 25·4 − 10·2 = 80 = T·2 → +40.
  near(r.forces[byJoints(jid('B2'), jid('B3'))], 40, 'bottom chord B2–B3 (sections)');
  // Section through panel 4 cuts T3–T4, T4–B3 and B3–B4; the last two meet at B3 (6,0).
  // Moments about B3 for the left part: 25·6 − 10·4 − 10·2 = 90 = C·2 → top chord −45.
  near(r.forces[byJoints(jid('T3'), jid('T4'))], -45, 'top chord T3–T4 (sections)');
  // Joint T3 has no load and only one non-horizontal member → zero-force vertical.
  near(r.forces[byJoints(jid('B3'), jid('T3'))], 0, 'centre vertical is a zero-force member', 1e-9);
  // Pratt diagonals point down towards mid-span → tension. Panel 2 shear = 25 − 10 = 15 → T·sin45 = 15.
  near(r.forces[byJoints(jid('T1'), jid('B2'))], 15 * Math.SQRT2, 'Pratt diagonal T1–B2 in tension');

  // Mid-span deflection two independent ways. (1) the displacement solve Aᵀ·u = −e.
  // (2) the unit-load method: apply 1 kN down at B3 alone to get forces f, then δ = Σ F·f·L / EA.
  const unit = P.toSolverModel(P.PRESETS.pratt.build());
  unit.joints.forEach(jt => { jt.fx = 0; jt.fy = jt.name === 'B3' ? -1 : 0; });
  const f = S.analyse(unit, EA).forces;
  const delta = r.forces.reduce((sum, F, k) => sum + F * f[k] * r.lengths[k] / EA, 0);
  near(-r.displacements[model.joints.findIndex(jt => jt.name === 'B3')].uy, delta, `mid-span deflection = unit-load virtual work (${(delta * 1000).toFixed(3)} mm)`);
}

console.log('\n— Howe preset: diagonals flipped, so they go into compression —');
{
  const model = P.toSolverModel(P.PRESETS.howe.build());
  const r = S.analyse(model, EA);
  const jid = name => model.joints.find(jt => jt.name === name).id;
  const byJoints = (a, b) => model.members.findIndex(m => (m.a === a && m.b === b) || (m.a === b && m.b === a));
  is(r.status, 'determinate', 'classification');
  near(r.forces[byJoints(jid('B1'), jid('T2'))], -15 * Math.SQRT2, 'Howe diagonal B1–T2 in compression');
}

console.log('\n— Warren preset: global equilibrium —');
{
  const model = P.toSolverModel(P.PRESETS.warren.build());
  const r = S.analyse(model, EA);
  is(r.status, 'determinate', 'classification');
  const total = model.joints.reduce((s, jt) => s + jt.fy, 0);
  const sumR = r.reactions.filter(q => q.dir === 'y').reduce((s, q) => s + q.value, 0);
  near(sumR, -total, 'vertical reactions balance the applied loads');
}

console.log('\n— Stability classification —');
{
  // Square with no diagonal: j=4, m=4, r=3 → 7 < 8, mechanism.
  const sq = [J(1, 0, 0, 'pin'), J(2, 2, 0, 'roller'), J(3, 2, 2), J(4, 0, 2)];
  const ring = [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4), M(4, 4, 1)];
  let r = S.analyse({ joints: sq, members: ring }, EA);
  is(r.status, 'mechanism', 'square frame, no diagonal');
  is(r.classification.k, 1, '  ...one mechanism');
  // The mechanism is the square racking sideways: the top joints move horizontally together,
  // and the check Aᵀ·u = 0 (no member stretches) holds.
  const u = r.mechanism;
  near(u[2].ux, u[3].ux, '  ...mechanism mode: top joints sway together');
  is(Math.abs(u[2].ux) > 1e-6 && Math.abs(u[0].ux) < 1e-12 && Math.abs(u[1].uy) < 1e-12, true, '  ...mechanism mode: supports stay put, top sways');

  r = S.analyse({ joints: sq, members: ring.concat([M(5, 1, 3)]) }, EA);
  is(r.status, 'determinate', 'square frame + one diagonal');

  r = S.analyse({ joints: sq, members: ring.concat([M(5, 1, 3), M(6, 2, 4)]) }, EA);
  is(r.status, 'indeterminate', 'square frame + both diagonals');
  is(r.classification.s, 1, '  ...degree of indeterminacy 1');

  // Triangle on three rollers: m + r = 3 + 3 = 6 = 2j, but nothing resists sideways load.
  r = S.analyse({ joints: [J(1, 0, 0, 'roller'), J(2, 4, 0, 'roller'), J(3, 2, 2, 'roller')], members: [M(1, 1, 2), M(2, 1, 3), M(3, 2, 3)] }, EA);
  is(r.status, 'mechanism', 'count passes (6 = 6) but all reactions parallel');

  r = S.analyse({ joints: [J(1, 0, 0), J(2, 4, 0), J(3, 2, 2)], members: [M(1, 1, 2), M(2, 1, 3), M(3, 2, 3)] }, EA);
  is(r.status, 'mechanism', 'unsupported structure');
  is(r.classification.k, 3, '  ...three rigid-body freedoms (slide x, slide y, rotate)');

  // Three collinear joints (0,0)-(2,0)-(4,0) with a vertical load at the middle: rank-deficient.
  r = S.analyse({ joints: [J(1, 0, 0, 'pin'), J(2, 2, 0, 'none', 0, -10), J(3, 4, 0, 'pin')], members: [M(1, 1, 2), M(2, 2, 3)] }, EA);
  is(r.status !== 'determinate', true, `collinear members cannot carry a transverse load (${r.status})`);
}

console.log('\n— Degenerate input does not crash —');
{
  is(S.analyse({ joints: [], members: [] }, EA).status, 'empty', 'empty model');
  is(S.analyse({ joints: [J(1, 0, 0, 'pin'), J(2, 0, 0)], members: [M(1, 1, 2)] }, EA).status, 'invalid', 'zero-length member');
  is(S.analyse({ joints: [J(1, 0, 0, 'pin'), J(2, 1, 1)], members: [M(1, 1, 1)] }, EA).status, 'invalid', 'member from a joint to itself');
  is(S.analyse({ joints: [J(1, 0, 0, 'pin'), J(2, 0, 0, 'roller')], members: [] }, EA).status, 'invalid', 'duplicate joints');
  is(S.analyse({ joints: [J(1, 0, 0, 'pin')], members: [M(1, 1, 9)] }, EA).status, 'invalid', 'member to a missing joint');
}

console.log(failures === 0 ? '\nAll tests passed.' : `\n${failures} test(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
