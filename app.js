/*
 * app.js — editor, canvas rendering and results panel.
 * All engineering lives in solver.js; this file only builds the model and draws the answer.
 */
(function () {
  'use strict';

  const { analyse } = window.TrussSolver;
  const { PRESETS, toSolverModel, loadComponents } = window.TrussPresets;

  const $ = id => document.getElementById(id);
  const canvas = $('canvas');
  const ctx = canvas.getContext('2d');
  const stage = $('stage');

  const HIT_JOINT_PX = 10;   // how close (screen px) the pointer must be to pick a joint
  const HIT_MEMBER_PX = 6;   // ... or a member
  const DRAG_START_PX = 3;   // movement before a click becomes a drag

  const state = {
    model: { joints: [], members: [] },
    nextId: 1,
    tool: 'select',
    selection: null,          // { type: 'joint' | 'member', id }
    hover: null,
    pending: null,            // member tool: joint id the next member starts from
    cursor: null,             // { sx, sy, wx, wy } — snapped world position under the pointer
    drag: null,
    pan: null,
    spaceHeld: false,
    view: { scale: 50, ox: 0, oy: 0 },   // screen = (ox + x·scale, oy − y·scale), CSS px
    settings: { E: 200, A: 1000, labels: true, reactions: true, ids: false, deflected: false, autoFactor: true, factor: 100, snap: 0.5 },
    result: null,
    deflFactor: null,
    undo: [], redo: [],
  };

  /* ====================================================================== *
   * Colours come from CSS custom properties so the canvas follows the theme.
   * ====================================================================== */
  let C = {};
  function readColours() {
    const cs = getComputedStyle(document.documentElement);
    const v = n => cs.getPropertyValue(n).trim();
    C = {
      canvas: v('--canvas'), ink: v('--ink'), muted: v('--muted'), faint: v('--faint'),
      gridMinor: v('--grid-minor'), gridMajor: v('--grid-major'), axis: v('--axis'),
      neutral: v('--member-neutral'), zero: v('--zero'), halo: v('--halo'), labelBg: v('--label-bg'),
      reaction: v('--reaction'), tension: v('--tension'), compression: v('--compression'),
    };
  }
  function hexToRgb(hex) {
    const h = hex.replace('#', '');
    const n = parseInt(h.length === 3 ? h.split('').map(c => c + c).join('') : h, 16);
    return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  }
  function mix(a, b, t) {
    const p = hexToRgb(a), q = hexToRgb(b);
    return `rgb(${p.map((v, i) => Math.round(v + (q[i] - v) * t)).join(',')})`;
  }
  /** Colour for a member force: hue from the sign, intensity from |F| / max|F|. */
  function forceColour(F, maxF) {
    if (isZeroForce(F, maxF)) return C.zero;
    const t = Math.min(1, Math.abs(F) / maxF);
    return mix(C.neutral, F > 0 ? C.tension : C.compression, 0.3 + 0.7 * t);
  }
  const isZeroForce = (F, maxF) => !(maxF > 0) || Math.abs(F) <= 1e-9 * Math.max(1, maxF);

  /* ====================================================================== *
   * Formatting — units are always shown next to numbers.
   * ====================================================================== */
  const MINUS = '−';
  function num(v, d = 2) {
    if (!isFinite(v)) return '—';
    if (Math.abs(v) < 0.5 * Math.pow(10, -d)) v = 0;
    const s = Math.abs(v).toFixed(d);
    return (v < 0 ? MINUS : '') + s;
  }
  function signed(v, d = 2) {
    const s = num(v, d);
    return s === (0).toFixed(d) || s.startsWith(MINUS) ? s : '+' + s;
  }
  function trimNum(v) { return String(Math.round(v * 1000) / 1000); }
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  /* ====================================================================== *
   * Model helpers
   * ====================================================================== */
  const joints = () => state.model.joints;
  const members = () => state.model.members;
  const jointById = id => joints().find(j => j.id === id);
  const memberById = id => members().find(m => m.id === id);
  const jointLabel = id => 'J' + (joints().findIndex(j => j.id === id) + 1);
  const memberLabel = id => 'M' + (members().findIndex(m => m.id === id) + 1);
  const samePoint = (a, b) => Math.abs(a.x - b.x) < 1e-9 && Math.abs(a.y - b.y) < 1e-9;
  const jointAt = (x, y, exceptId) => joints().find(j => j.id !== exceptId && samePoint(j, { x, y }));
  const memberBetween = (a, b) => members().find(m => (m.a === a && m.b === b) || (m.a === b && m.b === a));

  function addJoint(x, y) {
    const existing = jointAt(x, y);
    if (existing) return existing;            // never create duplicate joints
    const jt = { id: state.nextId++, x, y, support: 'none', load: null };
    joints().push(jt);
    return jt;
  }
  function addMember(a, b) {
    if (a === b || memberBetween(a, b)) return null;           // no self-members or duplicates
    if (samePoint(jointById(a), jointById(b))) return null;     // no zero-length members
    const m = { id: state.nextId++, a, b };
    members().push(m);
    return m;
  }
  function deleteJoint(id) {
    state.model.joints = joints().filter(j => j.id !== id);
    state.model.members = members().filter(m => m.a !== id && m.b !== id);
    if (state.pending === id) state.pending = null;
  }
  function deleteMember(id) {
    state.model.members = members().filter(m => m.id !== id);
  }
  /**
   * Dropping one joint exactly on another merges them: members are re-attached,
   * any that became zero-length or duplicated are removed. The stationary joint keeps
   * its support and load unless it had none.
   */
  function mergeJoints(fromId, intoId) {
    const from = jointById(fromId), into = jointById(intoId);
    if (into.support === 'none') into.support = from.support;
    if (!into.load || !into.load.mag) into.load = from.load;
    for (const m of members()) {
      if (m.a === fromId) m.a = intoId;
      if (m.b === fromId) m.b = intoId;
    }
    const seen = new Set();
    state.model.members = members().filter(m => {
      if (m.a === m.b) return false;
      const key = Math.min(m.a, m.b) + ':' + Math.max(m.a, m.b);
      if (seen.has(key)) return false;
      seen.add(key);
      return true;
    });
    state.model.joints = joints().filter(j => j.id !== fromId);
    if (state.selection && state.selection.id === fromId) state.selection = { type: 'joint', id: intoId };
  }

  /* ====================================================================== *
   * Undo / redo: whole-model snapshots (models are tiny, so this is simplest).
   * ====================================================================== */
  const snapshot = () => JSON.stringify({ model: state.model, nextId: state.nextId });
  function pushUndo(snap) {
    state.undo.push(snap);
    if (state.undo.length > 200) state.undo.shift();
    state.redo = [];
    updateUndoButtons();
  }
  function restore(snap) {
    const s = JSON.parse(snap);
    state.model = s.model;
    state.nextId = s.nextId;
    state.pending = null;
    if (state.selection && !(state.selection.type === 'joint' ? jointById(state.selection.id) : memberById(state.selection.id))) {
      state.selection = null;
    }
    selectionChanged();
    modelChanged();
  }
  function undo() { if (state.undo.length) { state.redo.push(snapshot()); restore(state.undo.pop()); updateUndoButtons(); } }
  function redo() { if (state.redo.length) { state.undo.push(snapshot()); restore(state.redo.pop()); updateUndoButtons(); } }
  function updateUndoButtons() {
    $('undoBtn').disabled = !state.undo.length;
    $('redoBtn').disabled = !state.redo.length;
  }

  /* ====================================================================== *
   * Update loop. Model edits mark the model dirty; one animation frame re-solves,
   * refreshes the panel and redraws. Dragging a joint therefore re-solves live.
   * ====================================================================== */
  let modelDirty = true, editorDirty = true, frameQueued = false;
  function modelChanged() { modelDirty = true; requestFrame(); }
  function selectionChanged() { editorDirty = true; requestFrame(); }
  function requestFrame() {
    if (frameQueued) return;
    frameQueued = true;
    requestAnimationFrame(() => {
      frameQueued = false;
      if (modelDirty) { solve(); renderStatus(); renderTables(); }
      if (editorDirty) renderEditor();
      if (modelDirty || editorDirty) syncEditor();
      modelDirty = editorDirty = false;
      renderLegend();
      draw();
    });
  }

  function solve() {
    const { E, A } = state.settings;
    const EA = E * A;                         // kN (GPa × mm² = kN, see solver.js)
    const validEA = isFinite(EA) && EA > 0;
    state.result = analyse(toSolverModel(state.model), validEA ? EA : 1);
    if (state.result.ok && !validEA) state.result.displacements = null;

    // Automatic deflection exaggeration: make the largest displacement about 10% of the
    // structure's size on screen, rounded down to a "nice" 1/2/5 × 10ⁿ so it reads clearly.
    state.deflFactor = null;
    const d = state.result.ok && state.result.displacements;
    if (d) {
      const maxU = Math.max(...d.map(u => Math.hypot(u.ux, u.uy)));
      if (maxU > 1e-12) {
        const xs = joints().map(j => j.x), ys = joints().map(j => j.y);
        const size = Math.max(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys), 1);
        const raw = 0.1 * size / maxU;
        const p = Math.pow(10, Math.floor(Math.log10(raw)));
        const nice = [5, 2, 1].map(k => k * p).find(v => v <= raw) || p;
        state.deflFactor = state.settings.autoFactor ? Math.max(1, nice) : state.settings.factor;
        state.maxDisp = maxU;
        state.maxDispJoint = joints()[d.findIndex(u => Math.hypot(u.ux, u.uy) === maxU)].id;
      }
    }
    if (state.settings.autoFactor && state.deflFactor) $('optFactor').value = state.deflFactor;
  }

  /* ====================================================================== *
   * View transform
   * ====================================================================== */
  const toScreen = (x, y) => [state.view.ox + x * state.view.scale, state.view.oy - y * state.view.scale];
  const toWorld = (sx, sy) => [(sx - state.view.ox) / state.view.scale, (state.view.oy - sy) / state.view.scale];
  function snapWorld(x, y, free) {
    if (free) return [Math.round(x * 1000) / 1000, Math.round(y * 1000) / 1000];
    const g = state.settings.snap;
    // Round to the grid, then clean floating-point dust (0.30000000000000004 → 0.3).
    return [+(Math.round(x / g) * g).toFixed(6), +(Math.round(y / g) * g).toFixed(6)];
  }

  function fitView() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const v = state.view;
    if (!joints().length) { v.scale = 50; v.ox = Math.round(w * 0.2); v.oy = Math.round(h * 0.65); requestFrame(); return; }
    const xs = joints().map(j => j.x), ys = joints().map(j => j.y);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    const padX = Math.min(110, w * 0.12), padTop = 90, padBottom = 130;   // room for loads, supports, reactions
    v.scale = Math.max(8, Math.min(160, Math.min((w - 2 * padX) / Math.max(x1 - x0, 1), (h - padTop - padBottom) / Math.max(y1 - y0, 1))));
    v.ox = w / 2 - (x0 + x1) / 2 * v.scale;
    v.oy = padTop + (h - padTop - padBottom) / 2 + (y0 + y1) / 2 * v.scale;
    requestFrame();
  }

  /* ====================================================================== *
   * Hit testing (in screen space, so it feels the same at any zoom)
   * ====================================================================== */
  function hitTest(sx, sy) {
    let best = null, bestD = HIT_JOINT_PX;
    for (const j of joints()) {
      const [x, y] = toScreen(j.x, j.y);
      const d = Math.hypot(x - sx, y - sy);
      if (d < bestD) { bestD = d; best = { type: 'joint', id: j.id }; }
    }
    if (best) return best;
    bestD = HIT_MEMBER_PX;
    for (const m of members()) {
      const a = jointById(m.a), b = jointById(m.b);
      const [x1, y1] = toScreen(a.x, a.y), [x2, y2] = toScreen(b.x, b.y);
      const dx = x2 - x1, dy = y2 - y1, len2 = dx * dx + dy * dy;
      const t = len2 ? Math.max(0, Math.min(1, ((sx - x1) * dx + (sy - y1) * dy) / len2)) : 0;
      const d = Math.hypot(sx - (x1 + t * dx), sy - (y1 + t * dy));
      if (d < bestD) { bestD = d; best = { type: 'member', id: m.id }; }
    }
    return best;
  }

  /* ====================================================================== *
   * Drawing
   * ====================================================================== */
  function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const w = stage.clientWidth, h = stage.clientHeight;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function draw() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = C.canvas;
    ctx.fillRect(0, 0, w, h);
    drawGrid(w, h);

    const res = state.result;
    const ok = res && res.ok;
    const maxF = ok ? Math.max(0, ...res.forces.map(Math.abs)) : 0;
    const sel = state.selection, hov = state.hover;

    // Selection / hover halos sit underneath the members.
    for (const m of members()) {
      const isSel = sel && sel.type === 'member' && sel.id === m.id;
      const isHov = hov && hov.type === 'member' && hov.id === m.id && state.tool !== 'joint';
      if (!isSel && !isHov) continue;
      line(m, isSel ? 11 : 9, isHov && state.tool === 'delete' ? mixAlpha(C.compression, 0.25) : C.halo);
    }

    // Members, coloured by force once solved.
    const badMember = res && res.badMember;
    members().forEach((m, k) => {
      let colour = C.ink, width = 2.25;
      if (ok) {
        const F = res.forces[k];
        colour = forceColour(F, maxF);
        width = isZeroForce(F, maxF) ? 1.75 : 2.5 + 2 * Math.abs(F) / maxF;
      } else if (m.id === badMember) {
        colour = C.compression;
      } else {
        colour = C.faint;
      }
      ctx.setLineDash(ok && isZeroForce(res.forces[k], maxF) ? [6, 4] : []);
      line(m, width, colour);
    });
    ctx.setLineDash([]);

    if (ok && state.settings.deflected && res.displacements && state.deflFactor) drawDeflected(res.displacements, state.deflFactor);
    if (res && res.mechanism) drawDeflected(res.mechanism, mechanismScale(res.mechanism));

    for (const j of joints()) drawSupport(j);
    if (ok && state.settings.reactions) drawReactions(res);
    for (const j of joints()) drawLoad(j);
    drawJoints();
    drawMemberLabels(res, maxF);
    drawPreview();
  }

  function mixAlpha(hex, a) { const [r, g, b] = hexToRgb(hex); return `rgba(${r},${g},${b},${a})`; }

  function line(m, width, colour) {
    const a = jointById(m.a), b = jointById(m.b);
    const [x1, y1] = toScreen(a.x, a.y), [x2, y2] = toScreen(b.x, b.y);
    ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }

  function drawGrid(w, h) {
    const s = state.view.scale;
    const [wx0, wy1] = toWorld(0, 0), [wx1, wy0] = toWorld(w, h);
    const minor = state.settings.snap;
    const major = [1, 2, 5, 10, 20, 50].find(v => v * s >= 45) || 100;
    const crisp = v => Math.round(v) + 0.5;

    const stroke = (step, colour) => {
      ctx.strokeStyle = colour; ctx.lineWidth = 1; ctx.beginPath();
      for (let x = Math.ceil(wx0 / step) * step; x <= wx1; x += step) { const sx = crisp(toScreen(x, 0)[0]); ctx.moveTo(sx, 0); ctx.lineTo(sx, h); }
      for (let y = Math.ceil(wy0 / step) * step; y <= wy1; y += step) { const sy = crisp(toScreen(0, y)[1]); ctx.moveTo(0, sy); ctx.lineTo(w, sy); }
      ctx.stroke();
    };
    if (minor * s >= 8) stroke(minor, C.gridMinor);
    stroke(major, C.gridMajor);

    // Axes through the origin.
    const [ax, ay] = toScreen(0, 0);
    ctx.strokeStyle = C.axis; ctx.beginPath();
    ctx.moveTo(crisp(ax), 0); ctx.lineTo(crisp(ax), h);
    ctx.moveTo(0, crisp(ay)); ctx.lineTo(w, crisp(ay));
    ctx.stroke();

    // Coordinate labels along the top and left edges, in metres.
    ctx.fillStyle = C.faint; ctx.font = '10px ' + monoFont();
    ctx.textBaseline = 'top'; ctx.textAlign = 'center';
    for (let x = Math.ceil(wx0 / major) * major; x <= wx1; x += major) {
      const sx = toScreen(x, 0)[0];
      if (sx > 30) ctx.fillText(trimNum(x), sx, 48);
    }
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    for (let y = Math.ceil(wy0 / major) * major; y <= wy1; y += major) {
      const sy = toScreen(0, y)[1];
      if (sy > 60 && sy < h - 60) ctx.fillText(trimNum(y), 6, sy);
    }
    ctx.textAlign = 'right'; ctx.textBaseline = 'top';
    ctx.fillText('m', w - 8, 48);
  }

  let _mono;
  function monoFont() { return _mono || (_mono = getComputedStyle(document.documentElement).getPropertyValue('--mono')); }

  function arrow(x1, y1, x2, y2, colour, width = 1.6, head = 8) {
    const ang = Math.atan2(y2 - y1, x2 - x1);
    ctx.strokeStyle = colour; ctx.fillStyle = colour; ctx.lineWidth = width; ctx.lineCap = 'butt';
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2 - Math.cos(ang) * head * 0.6, y2 - Math.sin(ang) * head * 0.6); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - head * Math.cos(ang - 0.4), y2 - head * Math.sin(ang - 0.4));
    ctx.lineTo(x2 - head * Math.cos(ang + 0.4), y2 - head * Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fill();
  }

  function textBox(text, x, y, colour, align = 'center', weight = '500') {
    ctx.font = `${weight} 11px ${monoFont()}`;
    const w = ctx.measureText(text).width + 8, h = 16;
    const left = align === 'center' ? x - w / 2 : align === 'left' ? x : x - w;
    ctx.fillStyle = C.labelBg;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(left, y - h / 2, w, h, 3); else ctx.rect(left, y - h / 2, w, h);
    ctx.fill();
    ctx.fillStyle = colour; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText(text, left + 4, y + 0.5);
  }

  function hatch(x0, x1, y) {
    ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = x0 + 3; x <= x1; x += 5) { ctx.moveTo(x, y); ctx.lineTo(x - 5, y + 5); }
    ctx.stroke();
  }

  function drawSupport(j) {
    if (j.support === 'none') return;
    const [x, y] = toScreen(j.x, j.y);
    ctx.strokeStyle = C.ink; ctx.fillStyle = C.canvas; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
    const top = y + 5, base = y + 19;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x - 10, base); ctx.lineTo(x + 10, base); ctx.closePath();
    ctx.fill(); ctx.stroke();
    if (j.support === 'pin') {
      ctx.lineWidth = 1.5; hatch(x - 14, x + 14, base);
    } else {
      ctx.lineWidth = 1.25;
      for (const dx of [-5.5, 5.5]) { ctx.beginPath(); ctx.arc(x + dx, base + 3.5, 3, 0, Math.PI * 2); ctx.fill(); ctx.stroke(); }
      ctx.lineWidth = 1.5; hatch(x - 14, x + 14, base + 7);
    }
  }

  const LOAD_LEN = 46;
  function drawLoad(j) {
    if (!j.load || !j.load.mag) return;
    const [x, y] = toScreen(j.x, j.y);
    const t = j.load.angle * Math.PI / 180;
    const dx = Math.cos(t), dy = -Math.sin(t);          // screen y points down

    // The arrow can either push into the joint (shaft on the far side) or hang from it
    // (shaft on the load side). Both mean the same force; pick the side with more clear
    // space so the arrow doesn't lie along a member, e.g. a downward load at a bottom-chord
    // joint of a Pratt truss would otherwise sit on top of the vertical.
    const busy = members().filter(m => m.a === j.id || m.b === j.id).map(m => {
      const o = jointById(m.a === j.id ? m.b : m.a);
      return Math.atan2(-(o.y - j.y), o.x - j.x);
    });
    if (j.support !== 'none') busy.push(Math.PI / 2);   // support symbol sits below
    const clearance = ang => busy.reduce((c, b) => Math.min(c, Math.abs(Math.atan2(Math.sin(ang - b), Math.cos(ang - b)))), Math.PI);
    const pushSide = Math.atan2(-dy, -dx), pullSide = Math.atan2(dy, dx);
    const pull = clearance(pullSide) > clearance(pushSide) + 1e-6;

    let tipX, tipY, tailX, tailY, lx, ly;
    if (pull) {
      tailX = x + dx * 7; tailY = y + dy * 7;
      tipX = tailX + dx * LOAD_LEN; tipY = tailY + dy * LOAD_LEN;
      lx = tipX + dx * 16; ly = tipY + dy * 12;
    } else {
      tipX = x - dx * 7; tipY = y - dy * 7;              // arrow ends just short of the joint
      tailX = tipX - dx * LOAD_LEN; tailY = tipY - dy * LOAD_LEN;
      lx = tailX - dx * 16; ly = tailY - dy * 12;
    }
    arrow(tailX, tailY, tipX, tipY, C.ink, 1.75, 9);
    textBox(`${trimNum(j.load.mag)} kN`, lx, ly, C.ink, 'center', '600');
  }

  function drawReactions(res) {
    for (const r of res.reactions) {
      if (Math.abs(r.value) < 1e-9) continue;
      const j = joints()[r.joint];
      const [x, y] = toScreen(j.x, j.y);
      const L = 34, label = `${num(Math.abs(r.value))} kN`;
      if (r.dir === 'y') {
        const y0 = y + (j.support === 'roller' ? 35 : 30);   // just below the support symbol
        if (r.value > 0) arrow(x, y0 + L, x, y0, C.reaction);
        else arrow(x, y0, x, y0 + L, C.reaction);
        textBox(label, x, y0 + L + 11, C.reaction);
      } else {
        const x0 = x - 18;
        if (r.value > 0) arrow(x0 - L, y + 12, x0, y + 12, C.reaction);
        else arrow(x0, y + 12, x0 - L, y + 12, C.reaction);
        textBox(label, x0 - L - 4, y + 12, C.reaction, 'right');
      }
    }
  }

  /**
   * A mechanism mode has no physical size (any multiple of it also works), so draw it at ~6% of
   * the structure. It is a first-order (small-displacement) mode: parts that rotate appear to
   * stretch slightly if drawn too large, so keep the amplitude modest.
   */
  function mechanismScale(mode) {
    const maxU = Math.max(...mode.map(u => Math.hypot(u.ux, u.uy)));
    const xs = joints().map(j => j.x), ys = joints().map(j => j.y);
    const size = Math.max(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys), 1);
    return maxU > 0 ? 0.06 * size / maxU : 0;
  }

  function drawDeflected(disp, factor) {
    const pos = joints().map((j, i) => toScreen(j.x + factor * disp[i].ux, j.y + factor * disp[i].uy));
    const idx = new Map(joints().map((j, i) => [j.id, i]));
    ctx.strokeStyle = mixAlpha(C.ink, 0.55);
    ctx.lineWidth = 1.25; ctx.setLineDash([5, 4]);
    ctx.beginPath();
    for (const m of members()) {
      const a = pos[idx.get(m.a)], b = pos[idx.get(m.b)];
      ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    }
    ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = ctx.strokeStyle;
    for (const p of pos) { ctx.beginPath(); ctx.arc(p[0], p[1], 2.2, 0, Math.PI * 2); ctx.fill(); }
  }

  function drawJoints() {
    const sel = state.selection, hov = state.hover;
    for (const j of joints()) {
      const [x, y] = toScreen(j.x, j.y);
      const isSel = sel && sel.type === 'joint' && sel.id === j.id;
      const isHov = hov && hov.type === 'joint' && hov.id === j.id;
      if (isSel || isHov) {
        ctx.fillStyle = isHov && state.tool === 'delete' ? mixAlpha(C.compression, 0.25) : C.halo;
        ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.fill();
      }
      ctx.lineWidth = 1.6; ctx.strokeStyle = C.ink;
      ctx.fillStyle = isSel ? C.ink : C.canvas;
      ctx.beginPath(); ctx.arc(x, y, 4.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      if (state.pending === j.id) {
        ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      }
      if (state.settings.ids) {
        ctx.font = `600 10px ${monoFont()}`; ctx.fillStyle = C.muted; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
        ctx.fillText(jointLabel(j.id), x - 7, y - 5);
      }
    }
  }

  function drawMemberLabels(res, maxF) {
    const ok = res && res.ok;
    const showForces = ok && state.settings.labels;
    if (!showForces && !state.settings.ids) return;
    members().forEach((m, k) => {
      const a = jointById(m.a), b = jointById(m.b);
      const [x1, y1] = toScreen(a.x, a.y), [x2, y2] = toScreen(b.x, b.y);
      const parts = [];
      if (state.settings.ids) parts.push(memberLabel(m.id));
      if (showForces) parts.push(signed(res.forces[k]));
      const F = ok ? res.forces[k] : 0;
      const colour = !showForces ? C.muted : isZeroForce(F, maxF) ? C.muted : F > 0 ? C.tension : C.compression;
      const text = parts.join('  ');
      ctx.font = `600 11px ${monoFont()}`;
      // Skip labels longer than the member is on screen (zoomed out): they would overlap
      // their neighbours. Every value is still in the table.
      if (ctx.measureText(text).width + 10 > Math.hypot(x2 - x1, y2 - y1)) return;
      textBox(text, (x1 + x2) / 2, (y1 + y2) / 2, colour, 'center', '600');
    });
  }

  function drawPreview() {
    const cur = state.cursor;
    if (!cur || state.drag || state.pan) return;
    const [gx, gy] = toScreen(cur.wx, cur.wy);
    const onJoint = state.hover && state.hover.type === 'joint';
    if (state.tool === 'member' && state.pending) {
      const p = jointById(state.pending);
      if (p) {
        const [px, py] = toScreen(p.x, p.y);
        let tx = gx, ty = gy;
        if (onJoint) { const h = jointById(state.hover.id); [tx, ty] = toScreen(h.x, h.y); }
        ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke(); ctx.setLineDash([]);
      }
    }
    if ((state.tool === 'joint' || state.tool === 'member') && !onJoint) {
      ctx.strokeStyle = C.muted; ctx.lineWidth = 1.25;
      ctx.beginPath(); ctx.arc(gx, gy, 4.5, 0, Math.PI * 2); ctx.stroke();
    }
  }

  /* ====================================================================== *
   * Pointer interaction
   * ====================================================================== */
  function localPoint(e) {
    const r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }
  function updateCursor(sx, sy, free) {
    const [wx, wy] = snapWorld(...toWorld(sx, sy), free);
    state.cursor = { sx, sy, wx, wy };
    $('coords').textContent = `x ${wx.toFixed(2)} m   y ${wy.toFixed(2)} m`;
  }
  function select(sel) {
    const same = (a, b) => (!a && !b) || (a && b && a.type === b.type && a.id === b.id);
    if (same(sel, state.selection)) return;
    state.selection = sel;
    selectionChanged();
    renderTables();
  }

  canvas.addEventListener('contextmenu', e => e.preventDefault());

  canvas.addEventListener('pointerdown', e => {
    canvas.setPointerCapture(e.pointerId);
    const [sx, sy] = localPoint(e);
    updateCursor(sx, sy, e.altKey);
    const hit = hitTest(sx, sy);

    // Right button cancels an in-progress member; middle button / space / right button pan.
    if (e.button === 2 && state.pending) { state.pending = null; requestFrame(); return; }
    if (e.button === 1 || e.button === 2 || state.spaceHeld) { startPan(sx, sy); return; }
    if (e.button !== 0) return;

    const { wx, wy } = state.cursor;
    const before = snapshot();
    const commit = () => { if (snapshot() !== before) { pushUndo(before); modelChanged(); } };

    switch (state.tool) {
      case 'select':
        if (hit && hit.type === 'joint') {
          select(hit);
          state.drag = { id: hit.id, sx, sy, before, moved: false };
        } else if (hit) {
          select(hit);
        } else {
          select(null);
          startPan(sx, sy);
        }
        break;

      case 'joint': {
        if (hit && hit.type === 'joint') { select(hit); break; }
        const jt = addJoint(wx, wy);
        select({ type: 'joint', id: jt.id });
        commit();
        break;
      }

      case 'member': {
        const target = hit && hit.type === 'joint' ? jointById(hit.id) : addJoint(wx, wy);
        if (state.pending == null || !jointById(state.pending)) {
          state.pending = target.id;
        } else if (state.pending === target.id) {
          state.pending = null;                 // clicked the start joint again: stop
        } else {
          const m = addMember(state.pending, target.id);
          if (m) select({ type: 'member', id: m.id });
          state.pending = target.id;            // keep chaining from the new joint
        }
        commit();
        requestFrame();
        break;
      }

      case 'support':
        if (hit && hit.type === 'joint') {
          const j = jointById(hit.id);
          j.support = { none: 'pin', pin: 'roller', roller: 'none' }[j.support];
          select(hit);
          commit();
        }
        break;

      case 'load':
        if (hit && hit.type === 'joint') {
          const j = jointById(hit.id);
          if (!j.load || !j.load.mag) j.load = { mag: 10, angle: -90 };
          select(hit);
          commit();
          requestAnimationFrame(() => { const el = $('ed-mag'); if (el) { el.focus(); el.select(); } });
        }
        break;

      case 'delete':
        if (hit && hit.type === 'joint') deleteJoint(hit.id);
        else if (hit) deleteMember(hit.id);
        if (hit) { if (state.selection && state.selection.id === hit.id) select(null); state.hover = null; commit(); }
        break;
    }
  });

  canvas.addEventListener('pointermove', e => {
    const [sx, sy] = localPoint(e);
    updateCursor(sx, sy, e.altKey);

    if (state.pan) {
      state.view.ox = state.pan.ox + (sx - state.pan.sx);
      state.view.oy = state.pan.oy + (sy - state.pan.sy);
      requestFrame();
      return;
    }
    if (state.drag) {
      const d = state.drag;
      if (!d.moved && Math.hypot(sx - d.sx, sy - d.sy) < DRAG_START_PX) return;
      d.moved = true;
      const j = jointById(d.id);
      if (j.x !== state.cursor.wx || j.y !== state.cursor.wy) {
        j.x = state.cursor.wx; j.y = state.cursor.wy;
        modelChanged();   // live re-solve while dragging
      }
      const other = jointAt(j.x, j.y, j.id);
      setHint(other ? `Release to merge ${jointLabel(j.id)} into ${jointLabel(other.id)}` : null);
      return;
    }
    const hit = hitTest(sx, sy);
    const changed = JSON.stringify(hit) !== JSON.stringify(state.hover);
    state.hover = hit;
    canvas.style.cursor = state.spaceHeld ? 'grab'
      : state.tool === 'select' ? (hit ? (hit.type === 'joint' ? 'move' : 'pointer') : 'grab')
      : state.tool === 'joint' || state.tool === 'member' ? 'crosshair'
      : hit && (state.tool === 'delete' || hit.type === 'joint') ? 'pointer' : 'default';
    if (changed || state.tool === 'joint' || state.tool === 'member') requestFrame();
  });

  function endPointer(e) {
    if (state.drag) {
      const d = state.drag;
      state.drag = null;
      if (d.moved) {
        const j = jointById(d.id);
        const other = jointAt(j.x, j.y, j.id);
        if (other) mergeJoints(j.id, other.id);
        pushUndo(d.before);
        selectionChanged();
        modelChanged();
      }
      setHint(null);
    }
    if (state.pan) { state.pan = null; canvas.style.cursor = state.tool === 'select' ? 'grab' : 'default'; }
    if (e && canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
  }
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);
  canvas.addEventListener('pointerleave', () => { if (!state.drag && !state.pan) { state.cursor = null; state.hover = null; requestFrame(); } });

  function startPan(sx, sy) {
    state.pan = { sx, sy, ox: state.view.ox, oy: state.view.oy };
    canvas.style.cursor = 'grabbing';
  }

  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const [sx, sy] = localPoint(e);
    const v = state.view;
    const k = Math.exp(-e.deltaY * (e.ctrlKey ? 0.01 : 0.0015));   // ctrlKey = trackpad pinch
    const scale = Math.max(6, Math.min(400, v.scale * k));
    const f = scale / v.scale;
    v.ox = sx - (sx - v.ox) * f;           // zoom about the pointer
    v.oy = sy - (sy - v.oy) * f;
    v.scale = scale;
    updateCursor(sx, sy, e.altKey);
    requestFrame();
  }, { passive: false });

  /* ====================================================================== *
   * Keyboard
   * ====================================================================== */
  const TOOL_KEYS = { v: 'select', j: 'joint', m: 'member', s: 'support', l: 'load', x: 'delete' };
  window.addEventListener('keydown', e => {
    const tag = document.activeElement && document.activeElement.tagName;
    const typing = tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA';
    if ($('matrixDialog').open) return;
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') {
      if (typing) return;
      e.preventDefault();
      e.shiftKey ? redo() : undo();
      return;
    }
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'y') { if (!typing) { e.preventDefault(); redo(); } return; }
    if (typing || e.metaKey || e.ctrlKey || e.altKey) return;

    const key = e.key.toLowerCase();
    if (TOOL_KEYS[key]) { setTool(TOOL_KEYS[key]); return; }
    if (key === 'f') { fitView(); return; }
    if (key === 'escape') { state.pending = null; select(null); requestFrame(); return; }
    if (key === ' ') { state.spaceHeld = true; canvas.style.cursor = 'grab'; e.preventDefault(); return; }
    if ((key === 'delete' || key === 'backspace') && state.selection) {
      e.preventDefault();
      deleteSelection();
    }
  });
  window.addEventListener('keyup', e => { if (e.key === ' ') state.spaceHeld = false; });

  function deleteSelection() {
    const sel = state.selection;
    if (!sel) return;
    pushUndo(snapshot());
    if (sel.type === 'joint') deleteJoint(sel.id); else deleteMember(sel.id);
    select(null);
    modelChanged();
  }

  /* ====================================================================== *
   * Tools, hints, presets
   * ====================================================================== */
  const HINTS = {
    select: 'Click a joint or member to inspect it. Drag joints to move them (drop onto another joint to merge). Drag empty space to pan, scroll to zoom.',
    joint: 'Click the grid to place a joint. Positions snap to the grid; hold Alt to place freely.',
    member: 'Click two joints to connect them. Clicking empty grid creates a joint there. Keep clicking to chain members; press Esc to stop.',
    support: 'Click a joint to cycle its support: none → pin (restrains x and y) → roller (restrains y only) → none.',
    load: 'Click a joint to add a 10 kN downward load, then set its magnitude and direction in the panel.',
    delete: 'Click a joint or member to delete it. Deleting a joint removes its members too.',
  };
  let hintOverride = null;
  function setHint(text) { hintOverride = text; $('hint').textContent = text || HINTS[state.tool]; }

  function setTool(tool) {
    state.tool = tool;
    if (tool !== 'member') state.pending = null;
    document.querySelectorAll('.tool').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.tool === tool)));
    setHint(hintOverride);
    canvas.style.cursor = tool === 'joint' || tool === 'member' ? 'crosshair' : 'default';
    requestFrame();
  }
  document.querySelectorAll('.tool').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));

  function loadPreset(key, record = true) {
    if (record) pushUndo(snapshot());
    state.model = PRESETS[key].build();
    state.model.joints.forEach(j => { delete j.name; });
    state.nextId = 1 + Math.max(0, ...state.model.joints.map(j => j.id), ...state.model.members.map(m => m.id));
    state.pending = null;
    state.selection = null;
    selectionChanged();
    modelChanged();
    fitView();
  }
  document.querySelectorAll('[data-preset]').forEach(b => b.addEventListener('click', () => loadPreset(b.dataset.preset)));

  $('undoBtn').addEventListener('click', undo);
  $('redoBtn').addEventListener('click', redo);
  $('fitBtn').addEventListener('click', fitView);
  $('clearBtn').addEventListener('click', () => {
    if (!joints().length) return;
    pushUndo(snapshot());
    state.model = { joints: [], members: [] };
    state.pending = null; state.selection = null;
    selectionChanged(); modelChanged();
  });

  /* ====================================================================== *
   * Panel: stability status
   * ====================================================================== */
  function renderStatus() {
    const res = state.result;
    const el = $('status');
    const cls = res.classification;
    const j = joints().length, m = members().length;
    const r = joints().reduce((n, jt) => n + (jt.support === 'pin' ? 2 : jt.support === 'roller' ? 1 : 0), 0);
    const warn = !res.ok && res.status !== 'empty';
    el.className = 'card status' + (warn ? ' warn' : '');

    let counts = '';
    if (j) {
      const cmp = 2 * j === m + r ? '=' : m + r < 2 * j ? '>' : '<';
      counts = `
        <div class="counts">
          <div><span>j</span> ${j}</div><div><span>m</span> ${m}</div><div><span>r</span> ${r}</div>
          <div class="span2"><span>2j = ${2 * j}  ${cmp}  m + r = ${m + r}</span>${cls ? `<span>rank A = ${cls.rank}</span>` : ''}</div>
          ${cls && (cls.k || cls.s) ? `<div class="span2"><span>mechanisms k = ${cls.k}</span><span>redundancies s = ${cls.s}</span></div>` : ''}
        </div>`;
    }
    const icon = res.ok ? '✓' : res.status === 'empty' ? '·' : '!';
    el.innerHTML = `
      <h2>Stability</h2>
      <div class="status-title"><span class="status-icon">${icon}</span><span>${esc(res.title)}</span></div>
      <p class="status-detail">${esc(res.detail)}</p>
      ${counts}`;
  }

  /* ====================================================================== *
   * Panel: results tables and checks
   * ====================================================================== */
  function renderTables() {
    const res = state.result;
    if (!res) return;
    const ok = res.ok;
    const maxF = ok ? Math.max(0, ...res.forces.map(Math.abs)) : 0;
    const sel = state.selection;
    const A = state.settings.A;

    if (!members().length) {
      $('memberTable').innerHTML = '<p class="empty">No members yet.</p>';
    } else {
      const rows = members().map((m, k) => {
        const a = jointById(m.a), b = jointById(m.b);
        const L = Math.hypot(b.x - a.x, b.y - a.y);
        const F = ok ? res.forces[k] : NaN;
        const zero = ok && isZeroForce(F, maxF);
        const kind = !ok ? '' : zero ? 'zero' : F > 0 ? 'T' : 'C';
        const sw = ok ? `<span class="swatch" style="background:${forceColour(F, maxF)}"></span>` : '';
        const stress = ok && A > 0 ? num(F * 1000 / A, 1) : '—';      // kN / mm² × 1000 = MPa
        const isSel = sel && sel.type === 'member' && sel.id === m.id;
        return `<tr data-member="${m.id}" class="${isSel ? 'selected' : ''}">
          <td>${sw}${memberLabel(m.id)}</td><td class="l">${jointLabel(m.a)}–${jointLabel(m.b)}</td>
          <td>${L.toFixed(2)}</td><td>${ok ? (zero ? '0.00' : signed(F)) : '—'}</td><td class="tc">${kind}</td><td>${stress}</td></tr>`;
      }).join('');
      $('memberTable').innerHTML = `<table><thead><tr><th>Member</th><th class="l">Joints</th><th>L (m)</th><th>F (kN)</th><th></th><th>σ (MPa)</th></tr></thead><tbody>${rows}</tbody></table>`;
      $('memberTable').querySelectorAll('tr[data-member]').forEach(tr => tr.addEventListener('click', () => select({ type: 'member', id: +tr.dataset.member })));
    }

    const supported = joints().filter(j => j.support !== 'none');
    if (!supported.length) {
      $('reactionTable').innerHTML = '<p class="empty">No supports. Use the Support tool (S) to add a pin and a roller.</p>';
    } else {
      const val = (jt, dir) => {
        if (!ok) return '—';
        const r = res.reactions.find(q => q.jointId === jt.id && q.dir === dir);
        return r ? signed(r.value) : '<span class="tc">free</span>';
      };
      const rows = supported.map(jt => `<tr data-joint="${jt.id}" class="${sel && sel.type === 'joint' && sel.id === jt.id ? 'selected' : ''}">
        <td>${jointLabel(jt.id)}</td><td class="l">${jt.support}</td><td>${val(jt, 'x')}</td><td>${val(jt, 'y')}</td></tr>`).join('');
      $('reactionTable').innerHTML = `<table><thead><tr><th>Joint</th><th class="l">Type</th><th>Rx (kN)</th><th>Ry (kN)</th></tr></thead><tbody>${rows}</tbody></table>`;
      $('reactionTable').querySelectorAll('tr[data-joint]').forEach(tr => tr.addEventListener('click', () => select({ type: 'joint', id: +tr.dataset.joint })));
    }

    // Independent global check: loads + reactions must have zero resultant force and moment.
    if (ok) {
      let fx = 0, fy = 0, mz = 0;
      const sm = toSolverModel(state.model);
      sm.joints.forEach(j => { fx += j.fx; fy += j.fy; mz += j.x * j.fy - j.y * j.fx; });
      res.reactions.forEach(r => {
        const j = sm.joints[r.joint];
        if (r.dir === 'x') { fx += r.value; mz -= j.y * r.value; } else { fy += r.value; mz += j.x * r.value; }
      });
      $('checks').innerHTML =
        `Global check (loads + reactions):<br><b>ΣFx</b> = ${num(fx, 6)} kN  <b>ΣFy</b> = ${num(fy, 6)} kN  <b>ΣM₀</b> = ${num(mz, 6)} kN·m<br>` +
        `Largest joint residual |A·x + P| = ${res.residual.toExponential(1)} kN`;
    } else {
      $('checks').innerHTML = '';
    }
    $('matrixBtn').disabled = !joints().length || !res.A;
  }

  /* ====================================================================== *
   * Panel: selection editor. Built once per selection (so typing never loses
   * focus) and then kept in sync by syncEditor().
   * ====================================================================== */
  let editSnap = null;   // model snapshot taken when an input gains focus, for undo
  function bindInput(el, onInput) {
    el.addEventListener('focus', () => { editSnap = snapshot(); });
    el.addEventListener('input', () => { onInput(el.value); });
    el.addEventListener('change', () => {
      if (editSnap && editSnap !== snapshot()) pushUndo(editSnap);
      editSnap = snapshot();
      syncEditor(true);
    });
  }

  function renderEditor() {
    const box = $('editor');
    const sel = state.selection;
    const obj = sel && (sel.type === 'joint' ? jointById(sel.id) : memberById(sel.id));
    if (!obj) {
      box.innerHTML = `<p class="empty">Nothing selected. Click a joint or member, or pick a tool above. Load a preset to see a solved truss straight away.</p>`;
      return;
    }

    if (sel.type === 'joint') {
      box.innerHTML = `
        <div class="ed-title"><strong id="ed-name">Joint ${jointLabel(obj.id)}</strong></div>
        <div class="field-row">
          <label class="field">x <span class="input-group"><input type="number" id="ed-x" step="${state.settings.snap}"><span class="suffix">m</span></span></label>
          <label class="field">y <span class="input-group"><input type="number" id="ed-y" step="${state.settings.snap}"><span class="suffix">m</span></span></label>
        </div>
        <div class="field-row"><div class="field">Support
          <div class="segmented" id="ed-support">
            <button data-s="none">None</button><button data-s="pin">Pin (x, y)</button><button data-s="roller">Roller (y)</button>
          </div></div></div>
        <div class="field-row">
          <label class="field">Load <span class="input-group"><input type="number" id="ed-mag" min="0" step="1"><span class="suffix">kN</span></span></label>
          <label class="field">Direction <span class="input-group"><input type="number" id="ed-angle" step="15"><span class="suffix">°</span></span></label>
        </div>
        <div class="field-row" style="justify-content:space-between">
          <div class="dir-buttons" aria-label="Load direction presets">
            <button class="btn" data-a="-90" title="Down (−90°)">↓</button><button class="btn" data-a="90" title="Up (90°)">↑</button>
            <button class="btn" data-a="180" title="Left (180°)">←</button><button class="btn" data-a="0" title="Right (0°)">→</button>
          </div>
          <button class="btn" id="ed-noload">Remove load</button>
        </div>
        <p class="note" style="margin-top:6px">Direction is measured anticlockwise from +x, so −90° is straight down.</p>
        <dl class="kv" id="ed-kv"></dl>
        <div class="eqs" id="ed-eqs"></div>
        <div class="ed-actions"><button class="btn danger" id="ed-delete">Delete joint</button></div>`;

      const moveTo = (x, y) => {
        const j = jointById(sel.id);
        if (!isFinite(x) || !isFinite(y)) return;
        j.x = x; j.y = y;
        const other = jointAt(x, y, j.id);
        if (other) { mergeJoints(j.id, other.id); selectionChanged(); }
        modelChanged();
      };
      bindInput($('ed-x'), v => { if (v !== '') moveTo(+v, jointById(sel.id).y); });
      bindInput($('ed-y'), v => { if (v !== '') moveTo(jointById(sel.id).x, +v); });

      const setLoad = (mag, angle) => {
        const j = jointById(sel.id);
        if (!j || !isFinite(mag) || !isFinite(angle) || mag < 0) return;
        j.load = mag > 0 ? { mag, angle } : null;
        modelChanged();
      };
      const curAngle = () => { const v = parseFloat($('ed-angle').value); return isFinite(v) ? v : -90; };
      bindInput($('ed-mag'), v => setLoad(v === '' ? 0 : +v, curAngle()));
      bindInput($('ed-angle'), v => { const j = jointById(sel.id); if (v !== '' && j.load) setLoad(j.load.mag, +v); });

      box.querySelectorAll('[data-a]').forEach(b => b.addEventListener('click', () => {
        const j = jointById(sel.id);
        pushUndo(snapshot());
        j.load = { mag: j.load && j.load.mag ? j.load.mag : 10, angle: +b.dataset.a };
        modelChanged();
      }));
      $('ed-noload').addEventListener('click', () => {
        const j = jointById(sel.id);
        if (!j.load) return;
        pushUndo(snapshot()); j.load = null; modelChanged();
      });
      box.querySelectorAll('[data-s]').forEach(b => b.addEventListener('click', () => {
        const j = jointById(sel.id);
        if (j.support === b.dataset.s) return;
        pushUndo(snapshot()); j.support = b.dataset.s; modelChanged();
      }));
    } else {
      box.innerHTML = `
        <div class="ed-title"><strong id="ed-name">Member ${memberLabel(obj.id)}</strong><span id="ed-ends"></span></div>
        <dl class="kv" id="ed-kv"></dl>
        <div class="ed-actions"><button class="btn danger" id="ed-delete">Delete member</button></div>`;
    }
    $('ed-delete').addEventListener('click', deleteSelection);
  }

  /** Refresh editor values and derived results without rebuilding the inputs. */
  function syncEditor(force) {
    const sel = state.selection;
    if (!sel) return;
    const res = state.result;
    const ok = res && res.ok;
    const setVal = (id, v) => { const el = $(id); if (el && (force || document.activeElement !== el)) el.value = v; };

    if (sel.type === 'joint') {
      const j = jointById(sel.id);
      if (!j) return;
      const i = joints().indexOf(j);
      $('ed-name').textContent = 'Joint ' + jointLabel(j.id);
      setVal('ed-x', trimNum(j.x)); setVal('ed-y', trimNum(j.y));
      setVal('ed-mag', j.load ? trimNum(j.load.mag) : 0);
      setVal('ed-angle', j.load ? trimNum(j.load.angle) : -90);
      $('ed-support').querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.s === j.support)));
      $('ed-noload').disabled = !j.load;

      const { fx, fy } = loadComponents(j.load);
      let kv = `<dt>Load components</dt><dd>Fx ${num(fx)}  Fy ${num(fy)} kN</dd>`;
      if (ok && res.displacements) {
        const u = res.displacements[i];
        kv += `<dt>Displacement</dt><dd>ux ${num(u.ux * 1000, 3)}  uy ${num(u.uy * 1000, 3)} mm</dd>`;
      }
      if (ok) {
        for (const r of res.reactions.filter(q => q.jointId === j.id)) kv += `<dt>Reaction R${r.dir}</dt><dd>${signed(r.value)} kN</dd>`;
      }
      $('ed-kv').innerHTML = kv;
      $('ed-eqs').innerHTML = res && res.A ? jointEquations(i) : '';
    } else {
      const m = memberById(sel.id);
      if (!m) return;
      const k = members().indexOf(m);
      const a = jointById(m.a), b = jointById(m.b);
      $('ed-name').textContent = 'Member ' + memberLabel(m.id);
      $('ed-ends').textContent = `${jointLabel(m.a)} → ${jointLabel(m.b)}`;
      const L = Math.hypot(b.x - a.x, b.y - a.y);
      const ang = Math.atan2(b.y - a.y, b.x - a.x) * 180 / Math.PI;
      let kv = `<dt>Length</dt><dd>${L.toFixed(3)} m</dd><dt>Angle from +x</dt><dd>${num(ang, 1)}°</dd>` +
        `<dt>Direction cosines</dt><dd>cx ${num((b.x - a.x) / L, 3)}  cy ${num((b.y - a.y) / L, 3)}</dd>`;
      if (ok) {
        const F = res.forces[k];
        const maxF = Math.max(...res.forces.map(Math.abs));
        const kind = isZeroForce(F, maxF) ? 'zero-force member' : F > 0 ? 'tension' : 'compression';
        kv += `<dt>Axial force</dt><dd>${signed(F)} kN (${kind})</dd>`;
        if (state.settings.A > 0) kv += `<dt>Stress F/A</dt><dd>${num(F * 1000 / state.settings.A, 1)} MPa</dd>`;
        if (res.displacements) kv += `<dt>Extension FL/EA</dt><dd>${num(res.extensions[k] * 1000, 4)} mm</dd>`;
      } else {
        kv += `<dt>Axial force</dt><dd>not solved</dd>`;
      }
      $('ed-kv').innerHTML = kv;
    }
  }

  /** Name of unknown number c: a member force or a reaction component. */
  function unknownName(c) {
    const m = members().length;
    if (c < m) return 'F<sub>' + memberLabel(members()[c].id) + '</sub>';
    const r = state.result.reactions[c - m];
    return `R<sub>${r.dir}</sub>`;
  }

  /** The two rows of A for joint i, written out as equations (and checked, if solved). */
  function jointEquations(i) {
    const res = state.result;
    const x = res.ok ? res.forces.concat(res.reactions.map(r => r.value)) : null;
    let html = `<h3>Equilibrium at ${jointLabel(joints()[i].id)} (rows ${2 * i + 1}–${2 * i + 2} of A)</h3>`;
    ['x', 'y'].forEach((dir, d) => {
      const row = res.A[2 * i + d];
      const P = res.P[2 * i + d];
      const terms = [], subs = [];
      row.forEach((a, c) => {
        if (Math.abs(a) < 1e-12) return;
        const mag = Math.abs(Math.abs(a) - 1) < 1e-12 ? '' : Math.abs(a).toFixed(3) + '·';
        terms.push([a < 0 ? MINUS : '+', mag + unknownName(c)]);
        if (x) subs.push([a < 0 ? MINUS : '+', `${Math.abs(a).toFixed(3)}×(${num(x[c])})`]);
      });
      if (Math.abs(P) > 1e-12) {
        terms.push([P < 0 ? MINUS : '+', num(Math.abs(P))]);
        if (x) subs.push([P < 0 ? MINUS : '+', num(Math.abs(P))]);
      }
      // Join signed terms as "a − b + c", dropping a leading "+".
      const join = list => list.map(([sg, t], n) => (n === 0 ? (sg === '+' ? '' : MINUS) : ` ${sg} `) + t).join('') || '0';
      let s = `<p class="eq">ΣF<sub>${dir}</sub> = 0: ${join(terms)} = 0`;
      if (x) {
        const sum = row.reduce((acc, a, c) => acc + a * x[c], P);
        s += `<br><span class="sub">${join(subs)} = ${num(sum, 6)} ✓</span>`;
      }
      html += s + '</p>';
    });
    return html;
  }

  /* ====================================================================== *
   * Legend overlay
   * ====================================================================== */
  function renderLegend() {
    const res = state.result;
    const el = $('legend');
    if (res && res.mechanism) {
      el.innerHTML = `<div class="defl">Dashed: one way this structure can move with <strong>no member changing length</strong> (a solution of Aᵀ·u = 0). ` +
        `First-order (small-displacement) mode; its size is arbitrary.</div>`;
      return;
    }
    if (!res || !res.ok || !members().length) { el.innerHTML = ''; return; }
    const maxF = Math.max(0, ...res.forces.map(Math.abs));
    let html = `
      <div class="bar" style="background:linear-gradient(to right, ${C.compression}, ${mix(C.neutral, C.compression, 0.3)} 49%, ${C.zero} 50%, ${mix(C.neutral, C.tension, 0.3)} 51%, ${C.tension})"></div>
      <div class="scale"><span class="c">${num(-maxF, 1)} C</span><span>0 kN</span><span class="t">+${num(maxF, 1)} T</span></div>
      <div>Axial force, kN · tension +, compression − · dashed = zero-force</div>`;
    if (state.settings.deflected) {
      if (state.deflFactor && res.displacements) {
        html += `<div class="defl">Deflected shape (dashed) <strong>× ${state.deflFactor.toLocaleString()}</strong>: exaggerated, not to scale. ` +
          `Max displacement ${num(state.maxDisp * 1000, 3)} mm at ${jointLabel(state.maxDispJoint)}.</div>`;
      } else {
        html += `<div class="defl">No deflection to show (enter valid E and A, or add loads).</div>`;
      }
    }
    el.innerHTML = html;
  }

  /* ====================================================================== *
   * Equilibrium matrix dialog
   * ====================================================================== */
  function showMatrix() {
    const res = state.result;
    if (!res || !res.A) return;
    const m = members().length;
    const names = res.A[0] ? res.A[0].map((_, c) => unknownName(c) + (c >= m ? `<sub>${jointLabel(res.reactions[c - m].jointId)}</sub>` : '')) : [];
    const x = res.ok ? res.forces.concat(res.reactions.map(r => r.value)) : null;
    const cell = (v, cls = '') => {
      const c = (Math.abs(v) < 1e-12 ? 'z ' : '') + cls;
      return `<td${c.trim() ? ` class="${c.trim()}"` : ''}>${Math.abs(v) < 1e-12 ? '0' : num(v, 3)}</td>`;
    };
    const selRows = state.selection && state.selection.type === 'joint' ? (() => { const i = joints().findIndex(j => j.id === state.selection.id); return [2 * i, 2 * i + 1]; })() : [];
    const selCol = state.selection && state.selection.type === 'member' ? members().findIndex(mm => mm.id === state.selection.id) : -1;

    const head = `<tr><th></th>${names.map((n, c) => `<th class="${c === selCol ? 'hl' : ''}">${n}</th>`).join('')}<th class="rhs">−P</th></tr>`;
    const body = res.A.map((row, r) => {
      const jl = jointLabel(joints()[r >> 1].id);
      const hl = selRows.includes(r) ? ' class="hl"' : '';
      return `<tr><th${hl}>${jl} ΣF<sub>${r % 2 ? 'y' : 'x'}</sub></th>` +
        row.map((v, c) => cell(v, c === selCol || selRows.includes(r) ? 'hl' : '')).join('') +
        cell(-res.P[r], 'rhs') + '</tr>';
    }).join('');
    const sol = x ? `<tr class="solution"><th>x (kN)</th>${x.map(v => `<td>${num(v, 2)}</td>`).join('')}<td class="rhs"></td></tr>` : '';

    const cls = res.classification;
    $('matrixBody').innerHTML = `
      <p>Each <strong>row</strong> is one equilibrium equation (ΣF<sub>x</sub> or ΣF<sub>y</sub> at a joint). Each <strong>column</strong> is one unknown: a member force F (tension +) or a reaction component R.
      Entry (row, column) is how much of that unknown acts in that equation. For a member it is ±c<sub>x</sub> or ±c<sub>y</sub>, the direction cosine pointing away from the joint along the member. For a reaction it is 1.
      The right-hand column is the applied load moved across: A·x = −P.</p>
      <p>Size ${res.A.length} × ${res.A[0] ? res.A[0].length : 0} (2j × (m + r)), rank ${cls ? cls.rank : '—'}. ${res.ok ? 'Square and full rank, so it has exactly one solution, shown in the bottom row.' : esc(res.title) + ', so there is no unique solution.'}</p>
      <div class="matrix-wrap"><table class="matrix"><thead>${head}</thead><tbody>${body}${sol}</tbody></table></div>`;
    $('matrixDialog').showModal();
  }
  $('matrixBtn').addEventListener('click', showMatrix);
  $('matrixClose').addEventListener('click', () => $('matrixDialog').close());
  $('matrixDialog').addEventListener('click', e => { if (e.target === $('matrixDialog')) $('matrixDialog').close(); });

  /* ====================================================================== *
   * Display & material options
   * ====================================================================== */
  const bindCheck = (id, key, resolve) => $(id).addEventListener('change', e => {
    state.settings[key] = e.target.checked;
    if (resolve) modelChanged(); else requestFrame();
  });
  bindCheck('optLabels', 'labels');
  bindCheck('optReactions', 'reactions');
  bindCheck('optIds', 'ids');
  bindCheck('optDeflected', 'deflected');
  $('optAuto').addEventListener('change', e => {
    state.settings.autoFactor = e.target.checked;
    $('optFactor').disabled = e.target.checked;
    if (!e.target.checked) state.settings.factor = state.deflFactor || state.settings.factor;
    modelChanged();
  });
  $('optFactor').addEventListener('input', e => {
    const v = parseFloat(e.target.value);
    if (isFinite(v) && v > 0) { state.settings.factor = v; modelChanged(); }
  });
  for (const [id, key] of [['optE', 'E'], ['optA', 'A']]) {
    $(id).addEventListener('input', e => { state.settings[key] = parseFloat(e.target.value); modelChanged(); });
  }
  $('optSnap').addEventListener('change', e => { state.settings.snap = parseFloat(e.target.value); selectionChanged(); requestFrame(); });

  /* ====================================================================== *
   * Start-up
   * ====================================================================== */
  readColours();
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readColours(); modelChanged(); });
  new ResizeObserver(() => { resizeCanvas(); requestFrame(); }).observe(stage);
  resizeCanvas();
  setTool('select');
  updateUndoButtons();
  loadPreset('pratt', false);   // something solved on screen within a second of opening
})();
