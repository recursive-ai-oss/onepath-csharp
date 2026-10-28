# ORCA solver

Normative description of `RecursiveAI.OnePath.Crowds.Orca`. A port that follows this text reproduces the same bits.
Source: van den Berg, Guy, Lin and Manocha, *Reciprocal n-body collision avoidance*, ISRR 2011 (sections 4, 5.1, 5.3, 5.4), plus the
choices listed under "Where the paper is silent".

All arithmetic is IEEE binary64 `+ - * /` and square root, evaluated as written (no fused multiply-add). Vectors are
`Vector2D` from the math package: `Dot(a, b) = a.x*b.x + a.y*b.y`, `Length = sqrt(Dot(a, a))`, `v / s` divides each component,
`v * s` multiplies each. Agent state is binary64: positions cross the library boundary as double with the usual limit
(magnitude below 2^21); an application that keeps tile-local float32 state converts at its boundary.

## Half-planes

A constraint is a half-plane `{ v : (v - P) . N >= 0 }` with a unit normal `N`. `SignedDistance(v) = (v - P) . N`; it is negative
when `v` violates the constraint, and then its magnitude is the penetration depth.

## Building a constraint

Everything is expressed relative to the agent being steered (it sits at the origin).

### The obstacle shape

The agent's neighbour is a segment `a-b` (a single point when `a == b`: another agent at relative position `p` is the point obstacle `p`
with the radius `r` = sum of both radii). The segment grown by `r` is the capsule `K`. The velocity obstacle for horizon `tau` is
`S = { s*k : k in K, s >= 1/tau }`, which is convex (equation 11 of the paper for segments, equation 4 for agents).

For a unit direction `n` the support function of `S` is finite exactly when `n.a <= -r` and `n.b <= -r` (the *domain*, an arc of
directions), and then equals `h(n) = (max(n.a, n.b) + r) / tau`. The signed distance of a velocity `x` from `S`
(positive outside, negative inside) is therefore

    d(x) = max over unit n in the domain of ( min(n.wa, n.wb) ) - r/tau,      wa = x - a/tau,  wb = x - b/tau

and the maximizer `n*` is the outward normal of `S` at its boundary point closest to `x`, namely `c = x - d(x) n*`. The paper's
`u` (equation 5) is `c - x = -d(x) n*`.

The maximum is found among these candidates, in this order; a candidate replaces the current best only when its value is strictly
greater, and candidates outside the domain (tolerance below) are skipped:

1. `wa / |wa|` (skipped when `wa = 0`);
2. `wb / |wb|` (only when `a != b`, skipped when `wb = 0`);
3. `+nn` and `-nn`, where `nn = (-e.y, e.x) / |e|`, `e = b - a` (only when `a != b` and `|e| > 0`);
4. the two *leg normals* of `a`, then (only when `a != b`) of `b`: for a point `p` with `rho2 = p.p`, `l = sqrt(rho2 - r*r)`,
   `c = -r*p`, `q = (-p.y*l, p.x*l)`: `n = ((c + q) / rho2)` and `n = ((c - q) / rho2)`, each component divided separately.
   These satisfy `n.p = -r`.

Value of a candidate: `min(n.wa, n.wb)`. Domain test for a candidate `n`: `n.a + r <= tolA` and `n.b + r <= tolB`, where
`tolA = 1e-9 * (|a| + r)` and `tolB = 1e-9 * (|b| + r)`. If no candidate survives, `n` is the unit vector toward the centre of
the obstacle, `-(a + b)/|a + b|` (or `(1, 0)` if `a + b = 0`), with its value computed the same way.
The result is `d = best - r/tau` and `n = bestN`.

### The agent construction in cases (what the code does for agents)

For another agent (`a = b`, `|a| > r`) the closest boundary point is computed case by case on the truncated cone of section 4 of the paper (the
cone from the origin around `a`, cut off at time `tau` by the disc of radius `r/tau` around `a/tau`), which is the same point the
support-function form above finds (a test checks that on random inputs). With `x` the relative optimization velocity:

1. `w = x - a/tau` (each component divided); `dot1 = w.a`.
2. **Cutoff case** when `dot1 < 0` and `dot1*dot1 > r*r * (w.w)`: `x` projects onto the arc. `n = w / |w|` and `u = n * (r/tau - |w|)`.
3. **Leg case** otherwise. `leg = sqrt(a.a - r*r)`. When `cross(a, w) = a.x*w.y - a.y*w.x` is greater than 0 the left leg is used, else the right leg.
   The left leg's unit direction is `dir = ((a.x*leg - a.y*r), (a.x*r + a.y*leg)) / a.a` and the normal `n = (-dir.y, dir.x)`; the right leg's is
   `dir = ((a.x*leg + a.y*r), (a.y*leg - a.x*r)) / a.a` and `n = (dir.y, -dir.x)`. `along = x.dir` and `u = dir * along - x`.
   (`/ a.a` divides each component; the cutoff normal divides each component of `w` by `|w|`.)
4. `P = vA + responsibility * u`, `N = n`, component-wise `u * share` added to the origin.

The tie `cross = 0` (relative velocity exactly along the axis) takes the right leg for both agents of a pair, so a symmetric pair turns the same way.
The construction for an overlapping pair, and the one for segments, stay as described in the support-function form.
The support-function text above remains the definition; the case form is its closed evaluation for agents.

### Overlap (collision) case

When the origin is within `r` of the segment (`distance^2 <= r*r`, the squared distance from the origin to the segment computed with
the clamped projection parameter) the velocity obstacle would be the whole plane. The paper assumes this does not happen; here the
obstacle is instead the capsule `K` cut off at one time step `dt`: `S = { k/dt : k in K }`, which is bounded and has no domain
restriction. The same formula applies with `tau` replaced by `dt` and every candidate admitted (step 4 is not used). The constraint
then asks for a relative velocity that leaves the overlap within one step.

### Agents (equation 6)

For agent A with neighbour B: `a = b = pB - pA`, `r = rA + rB`, `x = vA - vB` where `vA`, `vB` are the optimization velocities
(the current velocities, as recommended in section 5.2), `tau` the agent time horizon. With `u = -d n`:

    P = vA + responsibility * u,   N = n,   responsibility = 1/2 (B takes the other half)

### Static obstacles (section 5.4)

For a segment obstacle: `a`, `b` are its ends minus the agent's position, `r` is the agent's radius, `tau` is the obstacle horizon,
and the optimization velocity is zero (`x = 0`). The agent takes the whole responsibility: `P = u`, `N = n` (the tangent line at the
boundary point closest to the zero velocity). While the agent does not overlap the segment, the zero velocity is permitted, so the
obstacle constraints alone always admit a velocity. Obstacles block from both sides.

## The linear program (section 5.1)

Constraints are processed in list order (a fixed order instead of the paper's random order). Inputs: the list, the speed limit `R`,
the preferred velocity `p`. Result: the velocity inside the disc `|v| <= R` and every half-plane that is closest to `p`.

1. Start: `v = p` if `|p| <= R`, else `(p / |p|) * R`.
2. For `i = 0, 1, ...`: if `SignedDistance_i(v) >= 0`, continue. Otherwise the optimum lies on line `i`: with `d = (-N.y, N.x)`
   and `P = P_i`, the points `P + t d` inside the disc are `t in [-pd - s, -pd + s]` with `pd = P.d`,
   `s = sqrt(pd*pd + R*R - P.P)`; if `pd*pd + R*R - P.P < 0` the program is infeasible at `i`. For `j < i`:
   `denom = d . N_j`, `num = (P - P_j) . N_j`; if `|denom| <= 1e-12` the line is parallel to `j` and the program is infeasible at `i`
   when `num < 0` (otherwise `j` is ignored); else `t = -num/denom` raises the lower end when `denom > 0` and lowers the upper end
   when `denom < 0`; the program is infeasible at `i` as soon as lower > upper. The new `v = P + d * t` with `t` the projection
   of `p` on the line, `t = d . (p - P)`, clamped to the interval.
3. If every constraint passes, the result is `v` and the program was feasible; otherwise the first failing index `i` is
   reported and the fallback runs.

## The fallback (section 5.3)

Constraints before `hardCount` (obstacles, listed first) are hard. If the program failed at an index inside the hard prefix, the hard
prefix alone is relaxed by the procedure below with no hard constraints and the other constraints are ignored (this cannot happen with
constraints built by the functions above). Otherwise the procedure runs over all constraints with the first `hardCount` kept hard.

The paper looks for the point `(v*, d*)` above every plane `d_k(v) = -(signed distance of v to line k)` with minimal `d`; the
solution is the velocity maximizing the smallest signed distance `g_k(v) = (v - P_k).N_k` over the soft constraints. It is computed
incrementally, in order, as a program in `v` only:

1. `v` = the point inside the disc and the hard half-planes that maximizes `g` of the first soft constraint (the 2D program below with
   objective `N` of that constraint); `level = g(v)` for that constraint.
2. For each later soft constraint `j`: if `g_j(v) >= level`, continue. Otherwise the new optimum has `g_j(v) = level`, and each earlier
   soft constraint `k` contributes the half-plane `g_k(v) >= g_j(v)`, i.e. `v . (N_k - N_j) >= P_k.N_k - P_j.N_j`.
   With `m = N_k - N_j` (skipped when `m.m <= 1e-24`), `len = sqrt(m.m)`, `mh = m / len`, `rhs = P_k.N_k - P_j.N_j`: the half-plane
   has unit normal `mh` and point `mh * (rhs / len)`. Maximize `g_j` (objective direction `N_j`) over the disc, the hard half-planes
   and these half-planes, in the order hard, then `k` ascending; if that program is feasible set `v` to its result and
   `level = g_j(v)`.
3. The result is `v`.

The objective-direction program is the program of the previous section with the start `v = (N / |N|) * R` and, on each line, `t` at the
upper end when `d . N > 0`, the lower end when `d . N < 0`, and the projection of the current `v` clamped to the interval when `d . N = 0`.

## Neighbours and constraint order

The simulator builds the constraints of agent `i` from the state at the start of the step:

1. **Obstacles** within reach (`SegmentDistanceSquared <= (obstacleHorizon * maxSpeed + radius)^2`), in order of increasing squared
   distance from the agent to the segment, ties by obstacle index. These are the hard constraints.
2. **Agents** `j != i` with squared centre distance `< neighborDistance^2`, the `maxNeighbors` smallest in the order
   `(squared distance, index)`, listed in that order. Squared distance is `Dot(d, d)` of the position difference.

The set and order do not depend on the data structure used to find them. (The implementation sorts agents by grid cell
and index; the 3x3 block of cells around the agent, each cell at least `max(neighborDistance)` wide, contains every candidate.)

## The step

`Step(dt)`: (1) for every agent compute `v_new` from the start-of-step state, (2) for every agent `velocity = v_new` and then
`position = position + velocity * dt` (component-wise). Agents are independent in step 1, so any split by index, any
number of workers, gives the same bits. There is no clock. Zero agents is a no-op.

## Where the paper is silent (choices)

- **Overlap:** time-step cutoff capsule, above. (A black-box comparison with a reference implementation is pending.)
- **Agents that touch from outside** (`|a|` within rounding of `r`): the cone is nearly a half-plane and the normal is fixed by the leg or cutoff case
  that the position of `x` selects; the two forms of this document agree to the rounding of `sqrt(a.a - r*r)`, not bit for bit.
- **Order of constraints:** fixed (obstacles by distance, then agents by distance), not random. Equal distances break by index.
- **Fallback with ties:** the program above returns one of possibly several equally good velocities in a fixed way.
- **Exact head-on symmetry:** two agents meeting exactly head on with zero perturbation stop in front of each other (the permitted
  region shrinks symmetrically); real use perturbs the preferred velocity slightly, as the scenarios in the tests do.
- **Obstacle shape:** two-sided segments; a one-sided or convex-polygon variant belongs to the callers that know the mesh.
- **Parameters** (radius, speed, horizons, neighbour limits) are per agent and public; defaults belong to a higher tier.

## Checks

- The linear programs agree with vertex-enumeration brute force on over 10^5 random constraint sets (optimal when feasible, minimum
  largest penetration when not).
- The velocity-obstacle closest point and normal are checked against a sampled membership oracle for agents, segments and the overlap case.
- Simulations of antipodal circles, crossing blocks, a gap in a wall and a two-way corridor reach their goals; the deepest overlap
  is bounded: sparse scenes 0.1 radius; the dense centre of an antipodal circle 0.5 radius at 100-300 agents (observed 0.31 at 100) and 0.75 radius at 1000 agents (observed 0.55).
- `hash matrix`: the `orca/*` scenarios are equal on x64 JIT/AOT/scalar, WebAssembly, and arm64.

## One-sided boundary constraint (`ForBoundary`)

For a walkable area whose boundary the agent's centre may touch (the area was already shrunk by the agent radius), `OrcaConstraints.ForBoundary(a, b, clearance, tau)`
gives the half-plane for one boundary stretch `a`–`b` (relative to the agent; the walkable side is to the left of `a` → `b`, the winding of a counter-clockwise polygon).
With `e = b − a`, `s = −(a · e) / (e · e)` (the projection of the agent on the line, as a fraction), `n = (−e.y, e.x) / |e|` and `d = −(a · n)` (the signed distance of the agent
from the line, positive on the walkable side):

- there is **no** constraint when `e = 0`, when `s ≤ m` or `s ≥ 1 − m` with `m = (clearance + 1e-6) / |e|` (the agent is beside the stretch: grazing a corner is not
  a collision), or when `d < −1e-6` (the stretch faces away: it is the far side of a thin wall);
- otherwise the half-plane has normal `n` and point `−n · (d − clearance) / tau`, i.e. `v · n ≥ −(d − clearance) / tau`: an agent may approach the line at the speed that
  reaches `clearance` from it in `tau`, may always move along it or away from it, and is pushed away at `(clearance − d) / tau` when it is closer than `clearance`.

This is the paper's section 5.4 with the obstacle taken one-sided and without the corner legs, which the surface the agent moves on handles.
