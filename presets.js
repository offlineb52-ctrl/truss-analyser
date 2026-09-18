/*
 * presets.js — starting examples, plus the conversion from the editor's model to the
 * solver's model. Shared by the browser app and by tests.js.
 *
 * Editor joint: { id, name, x, y, support, load: { mag, angle } | null }
 *   load.angle is in degrees, anticlockwise from +x, so −90° points straight down.
 * Solver joint: { id, name, x, y, support, fx, fy }
 */
(function (root) {
  'use strict';

  /** Resolve a load into x/y components. Snap round-off (cos 90° ≈ 6e-17) to exact zero. */
  function loadComponents(load) {
    if (!load || !load.mag) return { fx: 0, fy: 0 };
    const t = load.angle * Math.PI / 180;
    const clean = v => (Math.abs(v) < 1e-12 ? 0 : v);
    return { fx: clean(load.mag * Math.cos(t)), fy: clean(load.mag * Math.sin(t)) };
  }

  function toSolverModel(model) {
    return {
      joints: model.joints.map(jt => Object.assign({ id: jt.id, name: jt.name, x: jt.x, y: jt.y, support: jt.support }, loadComponents(jt.load))),
      members: model.members.map(m => ({ id: m.id, a: m.a, b: m.b })),
    };
  }

  /*
   * Small builder: joints are named (B0, T1, ...) so members can be listed readably.
   */
  function builder() {
    const joints = [], members = [], byName = {};
    let id = 1;
    return {
      joint(name, x, y, support = 'none', load = null) {
        const jt = { id: id++, name, x, y, support, load };
        joints.push(jt); byName[name] = jt;
      },
      member(a, b) { members.push({ id: id++, a: byName[a].id, b: byName[b].id }); },
      done() { return { joints, members }; },
    };
  }

  const DOWN_10 = () => ({ mag: 10, angle: -90 });

  /*
   * All three presets: 12 m span, simply supported (pin left, roller right),
   * 10 kN downward at every interior bottom-chord joint, so j, m and r always satisfy
   * m + r = 2j and you can compare how the same loads flow through different web layouts.
   */
  const PRESETS = {
    // Pratt: verticals + diagonals sloping DOWN towards mid-span. Under gravity loads the
    // diagonals are in tension and the verticals in compression (good for steel: long
    // members in tension don't buckle).
    pratt: {
      label: 'Pratt',
      build() {
        const t = builder(), panels = 6, w = 2, h = 2;
        for (let i = 0; i <= panels; i++) {
          t.joint('B' + i, i * w, 0, i === 0 ? 'pin' : i === panels ? 'roller' : 'none', i > 0 && i < panels ? DOWN_10() : null);
        }
        for (let i = 1; i < panels; i++) t.joint('T' + i, i * w, h);
        for (let i = 0; i < panels; i++) t.member('B' + i, 'B' + (i + 1));        // bottom chord
        for (let i = 1; i < panels - 1; i++) t.member('T' + i, 'T' + (i + 1));    // top chord
        t.member('B0', 'T1'); t.member('B' + panels, 'T' + (panels - 1));         // inclined end posts
        for (let i = 1; i < panels; i++) t.member('B' + i, 'T' + i);              // verticals
        for (let i = 1; i < panels / 2; i++) t.member('T' + i, 'B' + (i + 1));    // left diagonals ↘
        for (let i = panels / 2 + 1; i < panels; i++) t.member('T' + i, 'B' + (i - 1)); // right diagonals ↙
        return t.done();
      },
    },

    // Howe: the mirror image of the Pratt web. Diagonals slope UP towards mid-span, so they
    // go into compression and the verticals into tension (historically: timber diagonals,
    // iron rods for the verticals).
    howe: {
      label: 'Howe',
      build() {
        const t = builder(), panels = 6, w = 2, h = 2;
        for (let i = 0; i <= panels; i++) {
          t.joint('B' + i, i * w, 0, i === 0 ? 'pin' : i === panels ? 'roller' : 'none', i > 0 && i < panels ? DOWN_10() : null);
        }
        for (let i = 1; i < panels; i++) t.joint('T' + i, i * w, h);
        for (let i = 0; i < panels; i++) t.member('B' + i, 'B' + (i + 1));
        for (let i = 1; i < panels - 1; i++) t.member('T' + i, 'T' + (i + 1));
        t.member('B0', 'T1'); t.member('B' + panels, 'T' + (panels - 1));
        for (let i = 1; i < panels; i++) t.member('B' + i, 'T' + i);
        for (let i = 1; i < panels / 2; i++) t.member('B' + i, 'T' + (i + 1));    // left diagonals ↗
        for (let i = panels / 2 + 1; i < panels; i++) t.member('B' + i, 'T' + (i - 1)); // right diagonals ↖
        return t.done();
      },
    },

    // Warren: no verticals, just alternating diagonals forming near-equilateral triangles
    // (3 m base, 2.5 m high). Diagonals alternate tension/compression along the span.
    warren: {
      label: 'Warren',
      build() {
        const t = builder(), bays = 4, w = 3, h = 2.5;
        for (let i = 0; i <= bays; i++) {
          t.joint('B' + i, i * w, 0, i === 0 ? 'pin' : i === bays ? 'roller' : 'none', i > 0 && i < bays ? DOWN_10() : null);
        }
        for (let i = 0; i < bays; i++) t.joint('T' + i, i * w + w / 2, h);
        for (let i = 0; i < bays; i++) t.member('B' + i, 'B' + (i + 1));
        for (let i = 0; i < bays - 1; i++) t.member('T' + i, 'T' + (i + 1));
        for (let i = 0; i < bays; i++) { t.member('B' + i, 'T' + i); t.member('T' + i, 'B' + (i + 1)); }
        return t.done();
      },
    },
  };

  const api = { PRESETS, toSolverModel, loadComponents };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.TrussPresets = api;
})(typeof window !== 'undefined' ? window : this);
