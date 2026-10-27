# Subtask 3: Porting plan for the 3D-JPS + VO coupled planner

All equation numbers refer to the frozen Stage-2 document (阶段二：3D-JPS 与 VO 强耦合数学模型). Nothing here
changes a formula. Every item below is either a direct implementation of one or more equations, or an
engineering bridge between the model and ROS/PX4. Engineering bridges are marked **[ENG]** and listed in §6
for confirmation.

Only these parameters may be calibrated: v_max, a_max, d_min, τ₁, τ₂.
Every other constant in §0 is fixed once, before any experiment, and is never tuned.

---

## 0. Constants and where they come from

| Symbol | Meaning | Source / rule | Status |
|---|---|---|---|
| v_max, a_max | velocity / acceleration limit | calibratable; baseline ego uses 0.8 / 0.8, PX4 COMMAND cap 1.0 / 2.0 | calibrate |
| d_min | minimum surface clearance | calibratable | calibrate |
| τ₁, τ₂ | coupling window | eq. (32), from T_c, R_max, a_max, T_g, Δv* | calibrate (via (32)) |
| δ | grid resolution | 0.1 m (decision Q3) | fixed |
| r_q | UAV radius | P450 geometry | **to fix** |
| e_cut | corner-cutting margin | R6: bound from ψ_max, v_c²/a_max | **to fix** |
| r_infl | static inflation | r_q + d_min + e_cut (eq. 1) | derived |
| r_o | obstacle radius | scenario file (ground truth) | per scenario |
| v_c | cruise speed | v_c ≤ v_max (D1) | **to fix** (e.g. v_c = v_max) |
| Δt | local planning period | **[ENG]** 0.1 s | **to fix** |
| τ_h | local look-ahead horizon | D1 | **to fix** |
| m | risk smoothing margin | eq. (20) | **to fix** |
| L_s, L_min | normalisation | eq. (21): max(‖p_g − p₀‖, L_min) | **to fix** L_min |
| ψ_max | max heading change inside ℓ_w | eq. (27) | **to fix** |
| θ_max, v_min | turn limit / validity speed | (C-θ), eq. (34) | **to fix** |
| w_θ, w_a | Γ weights | eq. (34)(35) | **to fix** |
| ρ_g | exact-penalty weight | eq. (28): ρ_g > G_θ | **to fix**, check ρ_g > G_θ numerically |
| ε_w | change threshold | eq. (26) | **to fix** |
| T_c | perception-control delay | eq. (32); the measured tracking lag is ≈ 0.8 s (Subtask 2) | **to fix** |
| T_g | worst-case repair time | eq. (32); measure on this machine in step ③ | measure |
| Δv* | divergence bound | eq. (32); ≤ 2 v_c | **to fix** |

Worked example of eq. (32). These are not chosen values.
- Inputs: r_q = r_o = d_min = 0.3, so R_max = 0.9; a_max = 0.8; T_c = 0.8.
- τ₁ = 0.8 + √(2·0.9/0.8) = 2.30 s.
- With Δv* = v_c = 0.8: the second term is τ₁·exp(15·0.8/(8·0.8·2.30)) = 2.30·e^0.815 ≈ 5.2 s.
- With Δv* = 2v_c: τ₂ ≈ 11.7 s.
- τ₂ = max(τ₁ + T_g, that term).

---

## 1. Shared runtime structure (all three steps)

```
node jps_vo_planner_node (planner_mode ∈ {stub, jps, vo, coupled}, ablation switches §5)
  inputs : odom (/uav1/mavros/local_position/odom)        -> monitoring + first plan only
           cloud (/uav1/octomap_point_cloud_centers)      -> MapInput -> OccupancyGrid (§2)
           /gazebo/model_states (prefix dyn_obs_)         -> MapInput::obstacles()   (decision Q-VO)
           goal, control_state                            -> as in the stub
  outputs: /uav1/planning/bspline (order 3) + gate        -> unmodified traj_server_for_prometheus
           /uav1/jps_vo/{path, jump_points, guide, debug} -> RViz / logging

timers : global_timer  1 Hz + on demand (new goal, map change on path, D_k ≠ ∅)
         local_timer   1/Δt (step ② and ③)
state  : reference state (p_ref, v_ref) = currently published B-spline evaluated at now
         (Subtask-2 rule: every new trajectory starts at the reference; continuity < 0.01 m vs previous reference)
```

**[ENG-1] Mapping the vehicle state.** The model's p_A and v_prev (D1, eq. 15, 34) are taken from the
**reference state**, not from odom. Reason: PX4 tracks the reference with a lag of about 0.8 s. Planning from
odom would put a 0.6–0.9 m setpoint jump into every cycle, and the published commands would stop being
continuous. Odom is used to snap the first plan to the vehicle, and for logging and safety monitoring.

**[ENG-2] Turning a velocity into a B-spline.** The local layer outputs one velocity v* per cycle. Each cycle
publishes a short uniform cubic B-spline, built as follows:
- It starts at (p_ref, v_ref).
- Its velocity reaches v* at a rate of at most a_max. This holds automatically, because (C-a) already bounds
  ‖v* − v_prev‖ ≤ a_max Δt.
- It then holds v* until the horizon τ_h.
- It ends with a deceleration tail, so the vehicle stops safely if the next cycle never comes.

The control points are samples of this profile. This is the same construction as `buildStraightLine`, applied
per axis, so the convex hull keeps the speed below v_max.

```
bridge(p_ref, v_ref, v*, Δt_ctrl=0.1):
    t = 0, p = p_ref, v = v_ref, pts = [p_ref − v_ref·dt]           # Q_{-1}: vel(0) = v_ref
    while t < τ_h:          v ← v + clip(v* − v, a_max·dt); p ← p + v·dt; pts.append(p)
    while ‖v‖ > 0:          v ← v − clip(v, a_max·dt);       p ← p + v·dt; pts.append(p)   # stop tail
    pts += [p, p]                                                     # end state: vel = acc = 0
    publish Bspline{order 3, knots (j−3)·dt, start_time = now}, gate on
```

---

## 2. Downsampling octomap_point_cloud_centers to the 0.1 m grid (deliverable 3, implemented)

Code: `include/jps_vo_planner/occupancy_grid.h`, `src/occupancy_grid.cpp`, `src/map_input.cpp`.
Unit test: `test/test_occupancy_grid.cpp`; all checks pass.

```
MapInput.cloudCallback(msg):                 # whole occupied set of the octree, frame "world", 0.05 m centres
    latest = msg; dirty = true               # no work in the callback

MapInput.modelStatesCallback(msg):           # ~1 kHz, no header -> stamp = receive time (wall clock)
    for name with prefix "dyn_obs_": store p, v (twist), r_o (scenario), stamp
                                      history.append(p) at 10 Hz, keep mask_history (15 s)

MapInput.update(now):                        # called by the planner, at most rebuild_rate (5 Hz)
    if !dirty: return
    P = finite xyz of latest
    masks = capsules along each obstacle's history, radius r_o + mask_margin (0.15)
    grid.rebuild(P, masks)

OccupancyGrid.rebuild(P, masks):
    clear raw cells set last time; clear inflated layer
    for p in P:
        if p inside any capsule: continue    # moving obstacles and their octomap trails are not static (D2)
        n = floor((p − o) / δ)               # o = (−25, −25, 0.5), δ = 0.1  ≡ ego GridMap::posToIndex
        if n inside 500×500×25: raw[n] = 1   # 8 octomap voxels -> 1 cell (proved by the test)
    for each raw cell c, for each offset k with ‖k‖·δ ≤ r_infl:  inflated[c + k] = 1     # eq. (1) ball
    V_free = inside box ∧ ¬inflated          # z < 0.5 or z ≥ 3.0 or outside x/y: occupied, like ego
```

Properties verified offline:
- No octomap centre lies on a cell boundary, and every cell holds exactly 8 voxels.
- The index function is identical to ego's.
- The inflation is exactly a ball of radius r_infl.
- The capsule mask drops exactly the points inside it.
- The floor, the ceiling and the x/y box are treated as occupied.
- `nearestFree` matches a brute-force search.
- A rebuild of 50k voxels takes 11 ms.

---

## 3. Step-by-step port

### Step ① — 3D-JPS only (static global planning; local layer degenerate at λ ≡ 0)

```
JPS.search(grid, s, g):                                  # eqs. (2)–(10)
    D = {−1,0,1}³ \ {0};  c(d) = δ‖d‖                     # (2)
    legal(x,d) ⇔ ∀ m with m_i ∈ {0,d_i}: x+m ∈ V_free     # (3), no corner cutting
    h(n) = δ[(√3−√2)Δ(1) + (√2−1)Δ(2) + Δ(3)]            # (5), octile-3D
    OPEN ← {s with all 26 directions}; g(s)=0              # start expands all directions
    while OPEN:
        x ← pop min f = g + h                              # (4)
        if x == g: return path(x)
        for d' in natural(x, d) ∪ forced(x, d):            # (6), forced per (7)
            j = jump(x, d')                                # (8)
            if j ≠ ∅: relax g(j) ← g(x) + ‖p(j) − p(x)‖    # (9)
    return ∅

forced(x, d):                                            # literal implementation of (7), local window
    for each legal neighbour n ∉ N(x,d):
        L_via = len(p(x) → x → n)
        L_alt = shortest legal path p(x) → n in the 4×4×4 window around x, avoiding x  (tiny Dijkstra, cached
                by (d, occupancy bits of the window))
        if L_alt > L_via (or none): n is forced
    # Restricting π to a window can only declare MORE neighbours forced -> less pruning, never a lost optimum.

jump(x, d):                                              # (8), iterative along d
    loop: n = x + d; if !legal(x,d): return ∅
          if n == g or forced(n,d) ≠ ∅: return n
          if |S(d)| ≥ 2: for d' ≺ d: if jump(n,d') ≠ ∅: return n     # diagonal → lower-dim scans
          x = n

path → segments (t_i, ℓ_i) (10); every t_i is one of the 26 unit directions
```

Executing at λ ≡ 0 follows §4.2(a): the guidance cone degenerates to a ray, and the optimum is v = s·t_g at
the largest s allowed by (C-v)(C-a). It is computed in closed form, without the sampler:

```
local_cycle_step1():
    s_k  = projection of p_ref onto Π                    # (23) s_π
    t_g  = normalised ∫_{s_k}^{s_k+ℓ_w} t_Π ds,  ℓ_w = ψ_max v_c² / a_max      # (27)
    v*   = argmax_{s ≥ 0} s·t_g  s.t. ‖v‖ ≤ v_c, ‖v − v_prev‖ ≤ a_max Δt         # (C-v)(C-a), §4.2(a)
           plus the goal-stop limit: ‖v‖ ≤ √(2 a_max ‖p_g − p_ref‖)       [ENG-3]
    bridge(p_ref, v_ref, v*)                             # [ENG-2]
```

Replanning in step ①: on a new goal; when the map rebuild makes a cell of the remaining path non-free; and
every 1 s, the same period as ego's `thresh_replan_time`. This is map-change replanning, not the C2 mechanism.

**Verification ① (pass/fail):**

| ID | Check | Pass criterion |
|---|---|---|
| ①-a | offline: JPS vs plain 26-connected A* with the same legality (3) and costs (2), on 200 random maps (10–30 % occupancy) and on a recorded planning_easy grid | path cost identical (|ΔJ| < 1e-9) in 100 % of cases; JPS expands fewer nodes |
| ①-b | offline: every output segment direction ∈ the 26 unit vectors; all cells on all segments free | 100 % |
| ①-c | offline: h_D consistency h(n) ≤ c(d) + h(n') over all cells | no violation |
| ①-d | online, static world, 3 goals ≥ 6 m through the cylinder field | goal reached within 0.3 m (reference) in 3/3; OFFBOARD held; no contact |
| ①-e | online clearance | min over flight of (distance odom → nearest cylinder surface) ≥ d_min − 0.087 m (δ√3/2 discretisation) − measured lag margin; record the value |
| ①-f | timing | JPS search time logged per call; continuity vs previous reference < 0.01 m on every publish |

### Step ② — VO / collision-cone local avoidance (eqs. 11–17, 40), obstacle state from /gazebo/model_states

Two modes, both needed later as baselines:
- `vo_pure` (A2): v_pref = v_c·ĝ toward the goal (16).
- `vo_series` (A10, one-way series): v_pref = v_c toward the point at distance v_c·τ_h ahead on the JPS path Π.
  JPS does not receive any feedback.

```
vo_cycle():
    O = map_input.obstacles(now)                  # p_o, v_o, r_o ; R_o = r_q + r_o + d_min
    candidates V = sample of V_adm (15):          # [ENG-4] solver
        directions: 24 azimuth × 7 elevation (±45°) + v_pref dir + v_prev dir
        speeds:     {0, 0.25, 0.5, 0.75, 1.0}·v_c
        keep v with ‖v‖ ≤ v_c and ‖v − v_prev‖ ≤ a_max Δt; always include clip(v_pref), v_prev, 0
    for v in V:
        for o in O:  x = p_o − p_ref, w = v − v_o
                     inCC = w·x ≥ ‖w‖ √(‖x‖² − R_o²)                       # (12)
                     t_c  = (14) if inCC else ∞
        F ∋ v  ⇔  ∀o: t_c > τ  (VO^τ, τ = τ₂)   ∧  grid.segmentFree(p_ref, p_ref + τ_h v)   # (15) + (C-s) static
    if F ≠ ∅:  v* = argmin_{v∈F} ‖v − v_pref‖                               # (16)
               (variant (17): argmin_{V_adm} w_t / t_c(v) + ‖v_pref − v‖, ablation A2-with-ρ handled in step ③)
    else:      v* = argmax_{v∈V_adm} min_o t_c(v − v_o), tie: ‖v − v_pref‖  # (40)
    bridge(p_ref, v_ref, v*)
    if ‖x‖ ≤ R_o for some o: log "inside safety radius" (the cone (12) is undefined there)
```

**Verification ② (pass/fail):**

| ID | Check | Pass criterion |
|---|---|---|
| ②-a | offline: CC membership (12) vs brute-force closest approach (min over t∈[0,1000] of ‖x − w t‖ ≤ R) on 10⁵ random (x, w) | 100 % agreement outside ε-band of the cone surface |
| ②-b | offline: t_c (14) vs numeric root of ‖x − w t‖ = R | |Δt_c| < 1e-6 s |
| ②-c | offline: if F ≠ ∅ then v* ∈ F and no sampled v ∈ F is closer to v_pref | 100 % |
| ②-d | online: `model_states` twist vs finite difference (done in the smoke test: mean 0.006 m/s) and obstacle age < obstacle_timeout | every cycle |
| ②-e | online scenarios: head-on, 90° crossing, 45° crossing, each with one sphere r_o = 0.3 m at ‖v_o‖ ∈ {0.3, 0.5} m/s, 5 runs each | no contact (GT surface distance odom → sphere > 0) in all runs; record min surface distance (reference and odom) |
| ②-f | static safety | (C-s) static check never violated by the published reference; OFFBOARD held |
| ②-g | timing | local cycle time logged; ≤ Δt at 99th percentile |

### Step ③ — full coupling C1 / C2 / C3 (eqs. 19–40)

```
coupled_cycle():                                                 # order of §4.1 "一个规划周期内的数学操作顺序"
  # 1. coupling state (29) and edge risks (19)(20)
  t̂(q) = ‖q − p_ref‖ / v_c                                       # §2.1 lower bound
  for each edge e = (q, d̂, ℓ) of G_J with t̂(q) < τ₂:              # only these can have u > 0
      for o: x_o, w_o, t_cpa, d_cpa  (19);  g_o, ρ_o (20) with u from (31)
      ρ_e = 1 − ∏(1 − ρ_o)                                        # (20)
      ξ_e = min_o [τ_o + (1 − g_o) τ₂]                            # (29)
  σ_ref = (p_ref, t_g(s_k), v_c τ₂);  ξ = ξ(σ_ref)                # (29)(iv): reference motion, NOT v*
  λ = S(u(ξ))                                                    # (30)(31)

  # 2. global layer (C1 + C2)
  for edges: λ_e = S(u(ξ_e)); Γ_e (35) with t_prev = incoming direction
             w_e = ℓ_e/L_s + λ_e ρ_e + Γ_e + κ_e                 # (39); κ only on e_k (25)
  D_k = {e : |w_e^(k) − w_e^(k−1)| > ε_w}                         # (26)
  if D_k ≠ ∅ or goal/map changed:
      Π ← A* on G_J with weights w_e, h = h_D / L_s, state = (jump point, incoming dir)   # Prop. 1, R12
      [ENG-5] implemented as a fresh search on G_J; Prop. 1(c) proves the result equals the incremental repair
  C_Π(s) = ∫ ω, ω = w_e/ℓ_e                                       # (23)

  # 3. guidance (C3)
  t_g = (27) on Π at s_k

  # 4. local problem P_loc (37)(38) over the same candidate set as step ②   [ENG-4]
  for v in V:
      reject unless (C-v)(C-a)(C-θ)(C-s)                          # (C-s) dynamic part only if λ > 0; static part always [ENG-6]
      s_g(v) = max(0, ∠(v, t_g) − θ_c(λ)), θ_c = πλ, require θ_c + s_g ≤ π     # minimal slack satisfying (28)
      J_A = V_Π(p_ref + τ_h v)                                    # (23)(24)
      J_B = ‖q_pref − q(v)‖/L_s + ρ(σ(v))                         # (24), σ(v) = (p_ref, v̂, τ_h‖v‖)
      J̃(v) = (1−λ) J_A + λ J_B + Γ(v; ξ) + ρ_g s_g                 # (36)(37), Γ from (34)
  if feasible set (C-v)–(C-s) empty: v* = (40), κ_{e_k} = 1
  else v* = argmin J̃, s_g* = s_g(v*)
  bridge(p_ref, v_ref, v*)

  # 5. C2 channel 2
  κ_{e_k} = λ s_g*/π                                              # (25), e_k = edge containing s_k
```

**Verification ③ (pass/fail).** The degeneracy tests prove the code implements the frozen equations.

| ID | Check | Pass criterion |
|---|---|---|
| ③-a | offline replay, λ forced ≡ 0 | w_e = ℓ_e/L_s, D_k = ∅, Π identical to step ① JPS; v* identical to step-① closed form (§4.2(a)) on every recorded cycle |
| ③-b | offline replay, λ forced ≡ 1 | s_g* = 0 and Γ = 0 on every cycle; v* identical to step-② `vo_pure` with ρ (§4.2(b)) |
| ③-c | offline: λ(ξ) | λ = 0 for ξ ≥ τ₂, 1 for ξ ≤ τ₁, monotone, S' ≤ 15/8 (30) on a dense ξ grid |
| ③-d | offline: Prop. 1 | w_e ≥ ℓ_e/L_s on all edges; fresh search on G_J returns the minimum Σ w_e (vs brute-force Dijkstra on G_J) |
| ③-e | offline: exact penalty | on cycles where (guidance cone ∩ F) ≠ ∅, s_g* = 0; record numerically estimated G_θ and check ρ_g > G_θ |
| ③-f | logging invariants every cycle | ξ computed from σ_ref (not v*); κ ≤ λ_k; ρ, λ, κ ∈ [0,1] |
| ③-g | online: static-only run | behaviour equals step ① (λ stays 0 because ξ ≥ τ₂) |
| ③-h | online: scenarios of ②-e + one "obstacle sits on the JPS corridor" scenario | no contact; when the dynamic obstacle blocks the corridor, D_k ≠ ∅ and Π changes before the encounter (logged) |
| ③-i | timing | T_g (worst repair time) measured; check τ₂ ≥ τ₁ + T_g of (32); local cycle ≤ Δt at 99th percentile |

---

## 4. Logging contract (feeds the fixed Subtask-4 metrics)

One CSV row per local cycle:
`t, mode, plan_ms_global, plan_ms_local, p_ref, v_ref, p_odom, v*, λ, ξ, s_g*, κ, path_id, path_hash, n_jump_pts, min_surface_dist_ref, min_surface_dist_odom, turn_angle(v*, v_prev), lateral_dev_ref_to_Π`
- Path switch = `path_hash` change not caused by a goal change. This counts the oscillation metric.
- Deviation from the global path is computed on the reference (Subtask-2 decision).

---

## 5. Ablation switches (parameters only; no code edits per ablation)

| Switch | Values | Used by |
|---|---|---|
| `lambda_mode` | adaptive / zero / one / const:<c> | A1, A2, A3 |
| `use_rho_in_pure_vo` | true / false | A2 variants (16) vs (17) |
| `enable_c2` | true / false (w_e = ℓ_e/L_s, κ = 0, D_k = ∅) | A4, A10 |
| `enable_c3` | true / false (θ_c ≡ π, no (C-g)) | A5, A10 |
| `c3_soft_tracking` | w_t (adds w_t‖v − v_ref‖², drops (C-g)) | A6 |
| `enable_gamma` | true / false | A7 |
| `lambda_shape` | quintic_log / linear / logistic | A8 |
| `theta_c_mode` | linear / fixed:<θ> / sqrt | A9 |
| `xi_source` | reference / executed | A12 |

---

## 6. Troubleshooting checklist (deliverable 5)

**Grid alignment**
- G-1 The grid origin must equal ego's (−25, −25, ground_height). If `ground_height` or the map size changes in
  either launch file, change both. Test 1 of `test_occupancy_grid` must keep passing.
- G-2 The octomap resolution must stay 0.05, or any 0.1/k. Otherwise voxel centres can land on cell boundaries.
- G-3 The cloud frame must be `world` (diag line). Odom frame `map` = `world` was assumed; check with
  `rosrun tf tf_echo world map` (identity expected).
- G-4 The inflation ball has a discretisation error of up to δ√3/2 ≈ 0.087 m; d_min must cover it.
- G-5 Octomap trails of moving spheres: if JPS routes around phantom walls, check that the mask capsules
  cover them. Raise `mask_history`/`mask_margin` (engineering, not the model).
- G-6 Unknown space is free (octomap only reports occupied), as in ego. A path through unseen space can become
  blocked later; map-change replanning handles that.
- G-7 Start inside the inflation (the hover point is near a cylinder) → JPS has no start. Use
  `nearestFree(start)` and log the snap distance.

**ROS timestamps**
- T-1 `use_sim_time=false`: every stamp is wall clock. Never mix in `/clock`.
- T-2 `ModelStates` has no header, so the obstacle stamp is the receive time. An age > `obstacle_timeout`
  means Gazebo or the script stalled → obstacles are ignored (warned). Treat a run with that warning as invalid.
- T-3 B-spline `start_time` ≤ now when published (traj_server takes t < 0 as (0,0,0)).
- T-4 The cloud stamp age shown in diag should stay < 0.5 s. Larger means octomap is overloaded.

**Topic subscription / publication**
- S-1 `rosnode list` must not contain `uav1_ego_planner_node` when running jps_vo (shared output topics).
- S-2 `rostopic info /uav1/planning/bspline` must show exactly one publisher.
- S-3 `/gazebo/model_states` must contain `dyn_obs_*` names; the planner prefix must equal the script prefix.
- S-4 Gate: `/uav1/prometheus/command/ego_command_stop_pub` is true while flying, and false never appears
  outside a COMMAND exit.
- S-5 Manual RViz goals during an automated run go straight to the planner. Do not click while a run is active.

**Replanning trigger conditions**
- R-1 Global replan: new goal, a cell of the remaining path non-free after a rebuild, D_k ≠ ∅ (step ③), or
  1 s periodic. Log the reason per replan; a periodic replan must not change `path_hash` when nothing changed.
- R-2 The local cycle runs every Δt only in COMMAND with a goal; it stops at goal_tolerance on the reference.
- R-3 Fallback (40) active → κ_{e_k} = 1. Repeated fallback cycles at the same place indicate R10/R3 of Stage 2.
  Record it as a limitation, do not tune.

**B-spline message format**
- B-1 order = 3; knots size = control points + 4; knots[j] = (j − 3)·dt; uniform dt.
- B-2 The first three control points encode (p_ref, v_ref): vel(0) = (Q₂ − Q₀)/(2dt) = v_ref.
- B-3 The new start equals the previous reference at start_time (< 0.01 m), checked in every publish log.
- B-4 The last three control points are equal (stop with zero velocity) so a missed cycle cannot fly on.
- B-5 Control-point velocity differences ≤ v_max·dt and second differences ≤ a_max·dt² (convex hull bound).

---

## 7. C++ implementation status (2026-10-26)

All three steps are implemented in one node. Each step is a parameter configuration, not a separate binary:
- step ① = group g1 (λ ≡ 0);
- step ② = group g2 (λ ≡ 1, classic VO 16);
- step ③ = g6.

| File | Content |
|---|---|
| `include/src coupling.*` | eqs. (12)(14)(19)(20)(29)–(35), constants, switches |
| `include/src jps3d.*` | eqs. (2)–(10), forced neighbours by (7) in the 3×3×3 window, jump memo, state space (jump point × dir_in) |
| `include/src global_path.*` | Π, s_π, C_Π, V_Π (23), t_g (27), Hausdorff switch test |
| `include/src local_planner.*` | P_loc (36)–(38), fallback (40), classic VO (16), λ=0 closed form (§4.2a) |
| `include/src planner_core.*` | cycle order of §4.1, edge weights (39), D_k (26), κ (25), CSV record |
| `src/planner_node_coupled.cpp` | ROS integration, global thread, logs, runtime checks |
| `config/groups/g1..g6.yaml` | ablation switches |
| `scripts/send_goal.py`, `scripts/verify_run.py` | goal publisher; online verification rows + metrics M1–M6 from the logs |

Engineering decisions added during the implementation (none changes a Stage-2 formula):

| ID | Decision | Reason / evidence |
|---|---|---|
| tie rule | Eq. (7) tie handling for diagonal arrivals. `strict` (default): prune only if the alternative is strictly shorter, as in Harabor & Grastien [2]. `literal`: "≤" for all directions, as written. Parameter `jps/tie_rule`. | test_jps: strict = A* optimum on 200/200 random maps; literal on 181/200 (planning_easy routes identical). **Pending your decision.** |
| ENG-7 | ‖x‖ ≤ R (already inside the safety radius): t_c = 0 if approaching, ∞ otherwise | cone (12) undefined there |
| ENG-8 | G_J restricted to the start/goal bounding box + 5 m (full z) | halves the risk-weighted search time; optimum unchanged on the test routes |
| ENG-9 | no global path → J_A = ‖goal − q‖/L_s | J_A undefined without Π |
| ENG-10 | fallback with no statically free candidate: head for the nearest free cell | smoke run 1: braking left the reference stuck inside the inflation |
| ENG-11 | global step (D_k + search) in its own thread; the local cycle uses the latest path | risk-weighted 3D searches take 0.1–2.6 s; local cycle stays < 1 ms |
| ENG-4 refinement | incumbent with s_g > 0 projected onto the cone surface (and along its generator into V_adm) | 3-e: exact-penalty property 1208/1208 on the sampled set |
| pre-rotation | optional `pre_rotate_yaw` (default **off**, decision 1) | Prometheus takeoff forces yaw 0 (uav_controller.cpp:483); north routes start with the camera 90° off the route |
| jump memo / per-cell closure | exact JPS accelerations: memoised jump scans; per-cell closure when w does not depend on dir_in | 4.2 s → 0.32 s worst case; optimality re-verified |

Offline results (`devel/lib/jps_vo_planner/test_*`): all checks pass.
- ①-a/b/c.
- ②-a/b/c.
- ③-a: (i) 15/15, (ii) 400/400.
- ③-b 2000/2000.
- ③-c.
- ③-d 28/28 against the enumerated G_J.
- ③-e 1208/1208, with estimated G_θ = 5.17 < ρ_g = 10.
- ③-f.
- B-1…B-5.
