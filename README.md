# Truss Analyser

An interactive 2D pin-jointed truss analyser. You build a truss and apply supports and loads. It then solves for the axial force in every member by the **method of joints**, written as a matrix equation and solved with a hand-written Gaussian elimination.

Plain HTML, CSS and JavaScript. There's no framework, no build step and no maths library.

**Try it live:** https://offlineb52-ctrl.github.io/truss-analyser/
**Run it locally:** double-click `index.html`.
**Test the maths:** `node tests.js` (48 checks against hand calculations).

| File | What it is |
|---|---|
| `solver.js` | All of the engineering: builds the equilibrium matrix, checks stability, runs Gaussian elimination, computes displacements. No DOM code. |
| `presets.js` | Pratt, Howe and Warren trusses, plus conversion of loads (magnitude, angle) into components. |
| `app.js` | Editor, canvas drawing, results panel. It only draws; it contains no statics. |
| `tests.js` | Solver checked against the method of sections, joints and virtual work, all worked on paper. |

---

## 1. Assumptions

These are the standard ideal-truss assumptions. They're worth saying out loud in an interview, because the whole method depends on them.

1. **Joints are frictionless pins.** A pin can't transmit a moment, so each member is loaded only at its two ends.
2. **Loads are applied only at joints.** With no load along its length, a member is a *two-force member*: the forces at its ends must be equal, opposite and collinear. So the only thing a member can carry is **axial force** along its own line. That's why there's only one unknown per member.
3. **Self-weight is ignored**, or lumped to the joints.
4. **Small displacements, linear-elastic material.** Equilibrium is written on the undeformed geometry.

**Units:** metres, kilonewtons, E in GPa, A in mm². Stress comes out in MPa: kN/mm² × 1000.

**Sign convention:** tension is positive. A tensile member *pulls* on its joints, and a compressive member *pushes* on them. Reactions are positive in +x (right) and +y (up). Load direction is measured anticlockwise from +x, so −90° points straight down.

---

## 2. From equilibrium equations to a matrix

### One joint

Each joint is a particle in equilibrium, so it gives exactly two equations:

  ΣF<sub>x</sub> = 0,  ΣF<sub>y</sub> = 0

Take a member k running from joint *a* to joint *b*, of length L. Its unit vector from a to b is

  c = (c<sub>x</sub>, c<sub>y</sub>) = ((x<sub>b</sub> − x<sub>a</sub>)/L, (y<sub>b</sub> − y<sub>a</sub>)/L)

If the member is in tension T, it pulls joint *a* **towards b**, which is the +c direction. So it contributes **+c<sub>x</sub>T** to ΣF<sub>x</sub> at a and **+c<sub>y</sub>T** to ΣF<sub>y</sub> at a. It pulls joint *b* towards a, so it contributes **−c<sub>x</sub>T** and **−c<sub>y</sub>T** there.

Because tension is assumed positive for every member, you never have to guess directions. If a member is actually in compression, the solution simply comes out negative.

A reaction component R<sub>x</sub> at a joint contributes +1·R<sub>x</sub> to that joint's ΣF<sub>x</sub>. A pin provides R<sub>x</sub> and R<sub>y</sub>; a roller provides only R<sub>y</sub>.

### All joints at once

Stack the equations for all *j* joints: 2*j* equations. The unknowns are the *m* member forces and the *r* reaction components. Collect the coefficients into a matrix:

  **A · x = −P**

- **A** is the 2*j* × (*m* + *r*) *equilibrium matrix*. Each **row** is one equation (e.g. "ΣF<sub>y</sub> at J3"). Each **column** is one unknown.
- A member's column holds at most four numbers: +c<sub>x</sub>, +c<sub>y</sub> in joint a's rows and −c<sub>x</sub>, −c<sub>y</sub> in joint b's rows. A reaction's column holds a single 1.
- **x** lists the unknowns: [T<sub>1</sub> … T<sub>m</sub>, R<sub>1</sub> … R<sub>r</sub>].
- **P** lists the applied loads. They're known, so they move to the right-hand side with a minus sign.

The code for this is `buildEquilibrium()` in `solver.js`. In the app, click **"Show equilibrium matrix"** to see A for the current truss. Select a joint to see its two rows written out as equations, with the solved values substituted back in.

### Worked example (it's in `tests.js`)

Take a triangle with J1 at (0,0) pinned, J2 at (4,0) on a roller, and J3 at (2,2) carrying 10 kN downward.

The members are M1: J1→J2 with c = (1, 0), M2: J1→J3 with c = (0.707, 0.707), and M3: J2→J3 with c = (−0.707, 0.707).

```
             T1      T2      T3     R1x  R1y  R2y        −P
J1 ΣFx  [    1     0.707     0       1    0    0 ]     [  0 ]
J1 ΣFy  [    0     0.707     0       0    1    0 ]     [  0 ]
J2 ΣFx  [   −1       0     −0.707    0    0    0 ]  x = [  0 ]
J2 ΣFy  [    0       0      0.707    0    0    1 ]     [  0 ]
J3 ΣFx  [    0    −0.707    0.707    0    0    0 ]     [  0 ]
J3 ΣFy  [    0    −0.707   −0.707    0    0    0 ]     [ 10 ]
```

The solution is T2 = T3 = −7.07 kN (the rafters are in compression), T1 = +5 kN (the tie is in tension), R1x = 0, and R1y = R2y = 5 kN. You can check it by hand: at the apex, 2·T·sin45° = −10.

---

## 3. Solving it: Gaussian elimination (`solveLinearSystem`)

1. **Forward elimination.** For each column in turn, pick the row with the largest |value| in that column and swap it into the pivot position. This is *partial pivoting*. Then subtract multiples of that row from every row below it, so the column becomes zero under the diagonal. Adding a multiple of one equation to another doesn't change the solution, so the system stays equivalent. When this finishes, the matrix is upper-triangular.
2. **Back substitution.** The last equation now has one unknown, so solve it. The second-to-last then has one new unknown, and so on upwards.

**Why pivot?** Dividing by a tiny number magnifies round-off error. Choosing the largest available pivot keeps the arithmetic stable. It also means a pivot of about 0 (below 10⁻⁹ here) is a reliable sign that the matrix is singular.

**Cost** is O(n³). For a 24 × 24 system that's about 10⁴ operations, which is why the app can re-solve on every mouse movement while you drag a joint.

**Verification.** After solving, the app substitutes x back into every equation and reports the largest residual |A·x + P|, which is about 10⁻¹⁵ kN. It also runs an independent global check: ΣF<sub>x</sub>, ΣF<sub>y</sub> and ΣM about the origin, over loads plus reactions, must all be zero.

---

## 4. Stability: why 2j = m + r, and why it isn't enough

### The counting rule (Maxwell, 1864)

A system of linear equations has exactly one solution for *every* right-hand side only if it has as many independent equations as unknowns. We have 2*j* equations and *m* + *r* unknowns:

| Count | Meaning |
|---|---|
| **m + r < 2j** | **Mechanism.** There are more equations than unknowns, so some load vectors P can't be balanced by any set of member forces. The structure moves: it folds or slides. |
| **m + r = 2j** | **Possibly determinate.** A is square. If it's invertible, there's exactly one solution. |
| **m + r > 2j** | **Statically indeterminate.** There are more unknowns than equations, so there are infinitely many sets of forces in equilibrium. Statics alone can't decide how redundant members share the load; you need their stiffness as well. |

Check with the presets: the Pratt truss has j = 12, m = 21, r = 3, so 2j = 24 = m + r. ✓

### Why it's *necessary but not sufficient*

The count only asks *how many* equations there are, not whether they're *independent*. Geometry can make one equation a copy of another. Two examples (both are tests, and both are easy to build in the app):

- **A triangle on three rollers.** m + r = 3 + 3 = 6 = 2j, so the count says it's fine. But every reaction is vertical, so nothing resists a horizontal load. The x-equations can't all be satisfied.
- **Two collinear members with a sideways load at the middle joint.** That joint's ΣF<sub>y</sub> row is all zeros, because neither member has a y-component. The structure can't carry the load, whatever the count says. (A real cable would sag until it could, but that's a large-displacement effect outside linear statics.)

A structure can also have the right total count while one panel has an extra diagonal and a neighbouring panel has none. The count balances, but one part is redundant and another is a mechanism.

### The exact test: rank of A (`matrixRank`, `classify`)

The **rank ρ** of A is the number of genuinely independent equations. It's found by the same elimination as above, counting how many usable pivots appear. Then:

- **k = 2j − ρ** is the number of independent **mechanisms**: ways the joints can move without any member changing length.
- **s = (m + r) − ρ** is the number of independent **states of self-stress**: sets of member forces that balance with *zero* load. This is the degree of statical indeterminacy.

Subtracting the two gives **(m + r) − 2j = s − k**. This is Maxwell's rule in the form given by Calladine (1978) and Pellegrino & Calladine (1986). The simple count only tells you *s − k*. You need the rank to know *s* and *k* separately. The app solves only when **k = 0 and s = 0**. That's exactly the case where A is square and invertible, so the method of joints has one unique answer.

### What the app shows

- **Determinate:** forces, reactions and deflections.
- **Mechanism:** a plain-English explanation, and a dashed drawing of *how* it moves. That's a vector u with **Aᵀ·u = 0**, found by `nullSpace()` from the reduced row-echelon form. See section 5 for why the transpose appears.
- **Indeterminate:** the degree s, and an explanation that compatibility is needed.

---

## 5. Deflections: the same matrix, transposed

Forces don't need stiffness, but deflections do. Hooke's law gives each member's extension:

  e<sub>k</sub> = T<sub>k</sub> L<sub>k</sub> / EA

Geometry links extensions to joint displacements u. A member from a to b gets longer by the component of the relative displacement along its own axis:

  e<sub>k</sub> = (u<sub>b</sub> − u<sub>a</sub>) · c<sub>k</sub>

Compare this with column k of A (+c at a, −c at b): it's exactly **e = −Aᵀu** for the member columns. For reaction columns, the row reads "displacement in a restrained direction = 0". So:

  **Aᵀ · u = [−e ; 0]**

The matrix that turns forces into loads, transposed, turns displacements into extensions. This is the **principle of virtual work** in matrix form, and you can prove it in one line. For any equilibrium set x and any compatible u, the internal work equals the external work:

  Σ T<sub>k</sub> e<sub>k</sub> = −xᵀAᵀu = −(Ax)ᵀu = Pᵀu.

It's also why mechanisms are the null space of Aᵀ: if Aᵀu = 0, no member stretches.

For a determinate truss A is invertible, so Aᵀ is too, and the same Gaussian elimination gives u.

**Checked two ways in `tests.js`:** the Pratt mid-span deflection from Aᵀu = −e (4.773 mm) matches the classic unit-load method, δ = Σ T·t·L / EA, where t are the forces from a 1 kN load at mid-span.

The deflected shape is drawn **exaggerated**, and the factor is always shown (e.g. "× 500 (exaggerated, not to scale)"). The real deflections are a few millimetres on a 12 m span. The auto factor makes the largest displacement about 10% of the structure's size, rounded down to 1, 2 or 5 × 10ⁿ.

**Note:** in a determinate truss, E and A change the deflections but **not** the forces. Forces come from equilibrium alone. That's a good point to make in an interview.

---

## 6. The presets

All three have a 12 m span, pin + roller supports and 10 kN at each interior bottom joint, so you can compare how the same loads flow through different webs.

- **Pratt:** diagonals slope *down* towards mid-span. Under gravity loads they go into **tension**, and the verticals into compression. Long diagonals in tension can't buckle, which makes this efficient in steel. The centre vertical is a **zero-force member**. At the top-centre joint the two chord members are collinear and there's no load, so ΣF<sub>y</sub> forces the vertical to zero. Drag that joint upwards and it immediately picks up force.
- **Howe:** the mirror image. The diagonals are in **compression** and the verticals in tension. Historically this suited timber diagonals with iron tension rods.
- **Warren:** no verticals, just near-equilateral triangles. The diagonals alternate between tension and compression.

Hand checks, all in `tests.js`:
- Pratt reactions: 25 kN each, by symmetry.
- Pratt top chord at mid-span: take moments about B3 for the left part: 25·6 − 10·4 − 10·2 = 90 kN·m, and 90 / 2 m = **45 kN compression**.
- Pratt bottom chord in panel 3: take moments about T2: 25·4 − 10·2 = 80, and 80 / 2 = **40 kN tension**.
- Pratt end diagonal: panel shear 15 kN, so 15/sin 45° = **21.21 kN tension**. The Howe equivalent is −21.21 kN.

---

## 7. Robustness

- **Duplicate joints:** placing a joint on an existing one selects the existing joint instead. Dragging a joint onto another merges them, removing any member that becomes zero-length or duplicated.
- **Zero-length or duplicate members** can't be created. The solver also rejects them itself, because it doesn't trust the UI.
- **Unsupported structures** are reported as a mechanism with k = 3: two translations and one rotation. The rigid-body motion is drawn.
- **Singular or near-singular systems:** the rank test runs *before* solving, so the solver never divides by a zero pivot.
- **Invalid E or A:** forces are still solved (they don't depend on stiffness); only the deflections are hidden.

---

## 8. Limitations (and what I'd add next)

- **No buckling check.** Compression members usually fail by Euler buckling, P<sub>cr</sub> = π²EI/L², long before the material yields. A real design would check every member in compression.
- **Real joints aren't pins.** Bolted or welded gusset plates add secondary bending, so pin-jointed analysis is an idealisation.
- **Indeterminate trusses aren't solved.** The next step is the **stiffness method**: K = A·diag(EA/L)·Aᵀ, solve K·u = P, then find forces from the extensions. It reuses this same matrix and solver.
- **Uniform EA, loads only at joints, linear small-displacement theory.**

---

## 9. Questions you might be asked

**Why is there only one unknown per member?** Pins can't carry a moment and loads act only at joints, so each member is a two-force member. Its force must lie along its own axis.

**Why assume every member is in tension?** It gives one consistent convention with no guessing. A negative answer simply means compression.

**What does it mean if the matrix is singular?** The equations aren't independent. The structure can move without any member stretching (a mechanism), so no unique equilibrium solution exists.

**The count balanced but the app said it's unstable. How?** The count checks the *number* of equations, not their independence. The rank checks independence. Example: three rollers, where every reaction is vertical.

**Why partial pivoting?** Round-off is magnified when you divide by small pivots. Pivoting on the largest entry keeps the solve accurate, and it makes a near-zero pivot a reliable test for singularity.

**How do you know the answers are right?** Three ways. The residual |A·x + P| is about 10⁻¹⁵. The global ΣF and ΣM over loads and reactions come to zero. And 47 tests compare the solver with hand calculations: method of joints, method of sections, and virtual work.

**Why doesn't changing E change the forces?** In a determinate truss the forces come from equilibrium alone, and there are exactly as many equations as unknowns. Stiffness only matters when there are redundant members, and then compatibility decides how they share the load.

**Why does the displacement solve use Aᵀ?** Static–kinematic duality, which is virtual work. The coefficients that map member forces to joint forces also map joint displacements to member extensions, in the opposite direction.
