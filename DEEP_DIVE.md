# TorqueForge — Deep Dive

A complete architectural walkthrough of the codebase, the physics/math it implements,
how the artifacts (scenes, articulated bodies, rendering) are constructed, and a
demonstration Q&A for live presentation. Read this alongside the source; every
claim here cites a file and line so you can confirm it.

---

## 1. What TorqueForge Is

TorqueForge is an **interactive rotational-dynamics sandbox** built to show off
reduced-coordinate articulated-body simulation (Featherstone-style), loop-closure
constraints, and joint couplings (gears / screws / worm drives / racks), with a
raylib OpenGL renderer for real-time interaction.

- **Language / Std:** C++17
- **Math:** Eigen (dense linear algebra), GLM (rendering-side vector math)
- **Collision detection only:** Bullet3 — we use broadphase + narrowphase +
  persistent manifolds, but the dynamics/solver is ours.
- **Renderer:** raylib 5 + custom Phong-with-shadow-map shader, nlohmann::json
  for per-demo config persistence.
- **Asset import:** tinygltf, parsing the **KHR_physics_rigid_bodies** and
  **KHR_implicit_shapes** Khronos extensions that Blender's physics add-on exports.

The project layout mirrors this separation:

| Directory | Purpose |
| --------- | ------- |
| [featherstone/](featherstone/) | Core rigid & articulated-body dynamics, spatial algebra, PGS contact solver, loop-joint solver. |
| [renderer/](renderer/) | raylib renderer, shadow-map pass, free-roam camera, HUD, config JSON. |
| [loader/](loader/) | glTF parser that converts a Blender export into `SceneGraph` + `ArticulationForest`. |
| [command/](command/) | TCP command server + Python client for remote PID / torque control during demos. |
| [demos/](demos/) | Each scene's `main()`. Short, readable wrappers around `demo_common`. |
| [demos/common/](demos/common/) | Shared scene boot code: world init, GLTF→body creation, renderer wiring. |
| [scenes/](scenes/) | `.gltf` / `.glb` assets — wrecking ball, scissor lift, springs, worm drive, pendulum chain, etc. |
| [third_party/](third_party/) | raylib, Bullet3, GLM, tinygltf, JSON, etc. (vendored or fetched). |

---

## 2. High-Level Flow of One Demo

Every scene follows the same skeleton, so learning one teaches you all of them
(see [demos/demo_load_scene.cpp](demos/demo_load_scene.cpp),
[demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp)).

```
init_renderer(app_name, cam, world_aabb, light)      // renderer/rigidworld_renderer.cpp
init_world(gravity)                                  // demos/common/demo_common.cpp
load_gltf(scene_file, Scene&, GLTFParseOption)       // loader/gltf_parser.cpp  (optional)
build bodies:
    for each rigid in scene:     create_rigidbody_from_node(...)
    for each artgroup in scene:  create_articulated_body(...)
world->add_body(body)                                // registers + triggers build_tree()
renderer.run(
    [&](dt){ world->step(dt); renderer.update_body(...); },  // update cb
    [&](dt){ draw axes, gizmos, labels ... }                 // 3D-aid cb
)
```

The `run()` loop ([renderer/rigidworld_renderer.cpp:381](renderer/rigidworld_renderer.cpp#L381))
is a single `while (!WindowShouldClose())` that calls the user's update callback,
executes a shadow-map pass, a color pass, and a HUD/gizmo pass per frame.

---

## 3. Core Physics — Featherstone / CRBA

The heart of the project lives in [featherstone/articulatedbody.cpp](featherstone/articulatedbody.cpp).
Despite the README casually calling it "ABA," the implementation is actually
**CRBA (Composite Rigid-Body Algorithm) + dense Cholesky**, which is why its
live cost scales super-linearly at small N but is still extremely fast in
absolute terms.

### 3.1 The algorithm per step

For every articulated body in the world, on each `step(dt)`:

| Step | Function | Complexity | What it does |
| ---- | -------- | ---------- | ------------ |
| 1 | `compute_bias_RNEA()` ([articulatedbody.cpp:828](featherstone/articulatedbody.cpp#L828)) | **O(n)** | Inverse-Dynamics (Recursive Newton-Euler) with ddq=0 to produce bias forces `C(q,dq)` (Coriolis, centrifugal, gravity, passive spring). Forward pass builds `v`, `a`; backward pass reduces spatial force through each joint's motion subspace. |
| 2 | `compute_H()` ([articulatedbody.cpp:890](featherstone/articulatedbody.cpp#L890)) | **O(n²)** | Composite inertia sweep from leaves to root builds the joint-space inertia matrix `H`. Factorized via `H.llt()` (dense Cholesky), ~O(n³) but with a tiny constant. |
| 3 | `solve_ddq()` ([articulatedbody.cpp:78](featherstone/articulatedbody.cpp#L78)) | **O(n³)** (dense solve) | Assembles τ, subtracts bias, optionally folds in spring/damping (semi-implicit) via `compute_H_spring()`, handles loop/coupling constraints (Z, K) and returns joint-space accelerations. |
| 4 | `integrate_velocity / integrate_position` | **O(n)** | Semi-implicit Euler for `q`, `dq`, with angular/linear damping powers. |

So the actual recipe is **RNEA + CRBA + dense Cholesky** — a completely standard
reduced-coordinate scheme. Pure O(n) ABA is a straight substitution in step 3 but
is not implemented here; the benchmark's ~2x super-linear growth is dominated by
the `H.llt()` factorization at high N.

### 3.2 Spatial Algebra — the `spvec.hpp` vocabulary

Every quantity is a **6-D spatial vector** laid out as `[angular(3); linear(3)]`.
Types defined in [featherstone/spvec.hpp](featherstone/spvec.hpp):

| Type | Shape | Meaning |
| ---- | ----- | ------- |
| `MVector` | 6×1 | Motion (velocity / acceleration) |
| `FVector` | 6×1 | Force / momentum (cotangent of motion) |
| `MTransform` | 6×6 | Transformation for motion vectors: `[[R, 0], [S(p)R, R]]` |
| `Dyad` | 6×6 | Spatial inertia (mass + off-diagonal COM term + rotational inertia) |
| `MSubspace / FSubspace` | 6×k | Motion / force subspaces (joint axes) |
| `JDyad / GPower / Unitless` | k×k | Joint-space matrices |

Key helpers:
- `cross_mat(v)` — 3×3 skew-symmetric cross-product matrix.
- `m_transform(R, p_ref)` — build motion transform from rotation + translation.
- `derivative_cross(v)` — spatial cross-product operator used in Coriolis terms.
- `ortho_complement_QR(S)` — returns an orthogonal basis Z with `S^T Z = 0`.
  Used for coupled-DoF constraints.
- `InvOrPinvSolver` — tries `FullPivLU`, falls back to truncated SVD, guarded
  against NaN to avoid Eigen's `BDCSVD` infinite loop when effective masses
  become singular. See [featherstone/spvec.hpp](featherstone/spvec.hpp).

`BlockAccess` is a tiny helper that maps joint index → row/col range in `H`,
letting `compute_H` accumulate symmetric 6×6 composite-inertia blocks without
allocating sub-matrices.

### 3.3 Tree construction and loop detection — `build_tree()`

[`articulatedbody.cpp:443`](featherstone/articulatedbody.cpp#L443).

An articulated body is defined by (1) a set of `RigidBody` instances added via
`add_body()` and (2) a set of joints added via `add_joint()`. `build_tree()`
converts that into a sim-ready graph:

1. **BFS from root body (index 0).** Every time BFS visits a body through a
   new joint, that joint becomes a **tree joint**. Revisits become **loop
   joints** — the duplicate edge is deferred to the Sequential-Impulse loop
   solver instead of inverting the tree.
2. Populates:
   - `lambda[i]` = parent index
   - `mu[i]` = children indices
   - `nu[i]` = subtree indices (used by CRBA)
3. Allocates `H`, `K` (loop-constraint Jacobian), `Ks` (spring stiffness),
   `Ds` (damping), all sized to total DoF.
4. If any tree joint has non-zero spring params, `enable_springs` flips on,
   forcing the step solver onto the implicit spring path (see §3.6).
5. For every coupling declared via `add_constraint()`, computes the **orthogonal
   complement `Z`** via `ortho_complement_QR`. `Z` is a matrix whose columns
   span motion that *respects* the coupling, so the reduced solve becomes:
   `Z^T H Z · ddq_reduced = Z^T (τ − C − Ks·q − Ds·dq)`.

### 3.4 Joint types

Declared in [`articulatedbody.hpp`](featherstone/articulatedbody.hpp):

| Type | DoF | `move_joints` impl |
| ---- | --- | ------------------ |
| `Fixed` | 0 | identity transform between parent and child |
| `Revolute` | 1 | rotation about joint's Z axis |
| `Prismatic` | 1 | translation along joint's X axis |
| `Cylindrical` | 2 | rot about X + trans along X |
| `Spherical` | 3 | full 3-D rotation (represented internally so `project_position` can integrate cleanly) |

Each joint stores its motion subspace `S` (6×DoF), its inverse `S_inv`, a joint
basis transform, and `ks / ds` spring/damping vectors of the same width as DoF.

### 3.5 Couplings — gears, racks, worm drives, screws

Modelled not as joints but as **linear constraints between existing joint
DoFs**: `C · dq_block = 0`. `C` rows encode things like "angular rate of gear A
equals −radius-ratio × angular rate of gear B," or "screw translation equals
pitch × screw rotation."

`build_tree()` runs `Z = ortho_complement_QR(C)` once. From then on the solve
is done in the reduced DoF space, so gears never drift — the constraint is
exactly satisfied in every step, not just approximately via penalty.

### 3.6 Springs & damping — the `SpringParam` path

If any joint has `SpringParam{k, d}`, `solve_ddq` routes through
`compute_H_spring(dt)` ([articulatedbody.cpp:877](featherstone/articulatedbody.cpp#L877))
which builds a **semi-implicit Euler** effective mass matrix:

```
    H_eff = H + dt² · diag(Ks) + dt · diag(Ds)
    rhs   = τ − C − Ks · q − Ds · dq              (measured at start of step)
    ddq   = H_eff⁻¹ · rhs
```

The dt² · Ks and dt · Ds terms make very stiff springs (`k >> 1/dt²`) still
stable — we pay for the extra stability with a tiny lag, which is invisible
for interactive visualization.

For the chain pendulum specifically, springs are zero but damping is high
(`d = 80`, see [demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp))
giving us near-critical joint damping so the 30-link chain settles in ~10 s.

---

## 4. Rigid Bodies, Shapes, and Inertia Tensors

### 4.1 `RigidBody` and `Shape`

A `RigidBody::Config` carries name, shape, pose, density, dynamic-type
(Static / Dynamic / Kinematic). Shapes come from
[featherstone/spshapes.cpp](featherstone/spshapes.cpp):

- **Cuboid** — closed-form inertia  `I = (m/12) · diag(hy²+hz², hx²+hz², hx²+hy²)` (scaled by `4`).
- **Sphere** — `I = (2m/5) · r² · I3`.
- **Cylinder** — closed-form along the rotation axis.
- **ConvexHull** — tetrahedralize from an arbitrary interior origin; for each
  tetrahedron `(0, v0, v1, v2)` sum signed volume and contribute the canonical
  tetra inertia tensor, then parallel-axis shift to the hull COM.
- **CompoundShape** — volume-weighted aggregate of sub-shapes with per-sub
  `transform_dyad2` to translate each local `Ic6` to the compound's COM.

The 6×6 spatial inertia `Ic6` is built from the 3×3 mass moment tensor `Ic3`
plus the mass and COM offset so every sim operation lives in spatial algebra.

### 4.2 Bullet wrappers — `RigidWorld::Collider`

[`featherstone/rigidworld.cpp`](featherstone/rigidworld.cpp) translates each
`Shape` to a `btCollisionShape`:

- `Cuboid` → `btBoxShape`
- `Sphere` → `btSphereShape`
- `Cylinder` → `btCylinderShape`
- `ConvexHull` → `btConvexHullShape` populated from the hull vertices
- `CompoundShape` → `btCompoundShape` with per-sub transforms

Each `btCollisionObject`'s `userIndex` stores the body id and `userIndex2`
stores the sub-body index (or `-1` if the body is a loose rigid). The contact
solver later reads these back to decide whether to build a
`RigidBodyWrapper` or an `ArticulatedBodyWrapper`.

### 4.3 Adjacent-link collision filter

[`rigidworld.cpp` AdjacentLinkFilter](featherstone/rigidworld.cpp). For each
articulated body we walk its tree joints and, for any joint declared with
`disable_collision = true`, record the `(bodyA, bodyB)` pair in a per-artbody
adjacency map. Bullet's broadphase then asks our overlap filter callback
`needBroadphaseCollision(obj0, obj1)`; if both objects belong to the same
articulated body and their link IDs are adjacent in that map, we return
`false`, pruning the pair before narrowphase.

**Note:** this only prunes parent-child pairs. Non-adjacent links in a long
serial chain can still self-collide, which is why the 30-link pendulum uses a
small initial perturbation in the headless benchmark.

---

## 5. The World Step — `RigidWorld::step(dt)`

[`featherstone/rigidworld.cpp`](featherstone/rigidworld.cpp):

```
1.  collide()                         // Bullet broadphase + narrowphase, build persistent manifolds
2.  for each body:
        integrate_velocity(dt)        // implicit Euler for ω/v; solve_ddq for articulated
3.  contact_solver.initialize(...)    // build per-contact N_01 basis + tangential & normal effective mass
4.  loop_joint_solver.initialize(...) // build T_ortho constraint subspace + effective mass
5.  contact_solver.warm_start()
6.  loop_joint_solver.warm_start()
7.  for it in [0, 10):                // velocity iterations
        contact_solver.solve_velocity()
        loop_joint_solver.solve_velocity()
8.  project_velocity()                // articulated bodies re-project per-link ω/v from joint velocities
9.  for each body:
        integrate_position(dt)
10. for it in [0, 8):                 // position iterations (Baumgarte stabilization)
        contact_solver.solve_position()
        loop_joint_solver.solve_position()
11. project_position()                // articulated bodies re-project per-link pose from joint q
```

### 5.1 Contact solver — Projected Gauss-Seidel with warm start

[`featherstone/contact_solver.cpp`](featherstone/contact_solver.cpp).

For every contact point from Bullet's `btPersistentManifold`:

- Build a **local contact frame** `N_01 = [tx | ty | n]` where `n = -normalWorldOnB`.
- Compute tangential 2×2 effective mass (`t^T · (I0⁻¹+I1⁻¹) · t`) with
  `InvOrPinvSolver` for robustness; scalar normal effective mass.
- Persist `si_n` (normal) and `si_t` (2-D tangential) across frames via Bullet's
  `m_userPersistentData` and its destroyed-callback — classic **warm starting**.
- Each PGS pass:
  1. Solve tangential impulse `λ = −M_t⁻¹ · v_t`, clip to Coulomb cone
     `|si_t + λ| ≤ μ · si_n`, commit delta.
  2. Solve normal impulse, clamp `si_n ≥ 0` (no sticking), commit delta.
- Restitution is applied as a **velocity bias** `v_bias = e · v_approach` only
  when `v_approach < −restitution_threshold`, preventing bouncy jitter at rest.

The position pass applies **Baumgarte stabilization** with
`baumgarte = 0.2`, `slop = 0.001`, `max_correction = 0.4` — small enough to
never introduce visible pop, strong enough to zero out penetration in a few
frames.

### 5.2 Loop-joint solver — closing the kinematic loop

[`featherstone/loop_joint_solver.cpp`](featherstone/loop_joint_solver.cpp).

A loop joint is a joint that would have created a cycle in BFS. Each loop
carries a motion subspace `T_ortho` (the *constraint* subspace, orthogonal to
the joint's free motion subspace) computed once in `initialize()`.

- Velocity solve: `λ = −M_eff⁻¹ · T_orthoᵀ · (v_successor − v_predecessor)`,
  accumulate sequential impulse `si += λ`, apply equal-and-opposite 6-D
  impulses to both bodies via `ArticulatedBCPVelocity::apply_impulse`.
- Position solve: measure 6-D loop-closure displacement
  (`ArticulatedBCPPosition::loop_closure_displacement_ps`), split into angular
  axis-angle + linear parts, Baumgarte-scale each independently, project onto
  `T_ortho`, and distribute positional impulses.

This is exactly why the **scissor lift** works: the scissors form a kinematic
loop (`A-B-C-D-A`), BFS picks 3 joints as tree joints and deposits the 4th as
a loop joint, then SI enforces the closure without needing us to solve a
reduced coordinate system by hand.

---

## 6. Rendering Pipeline

[`renderer/rigidworld_renderer.cpp`](renderer/rigidworld_renderer.cpp) and
[`renderer/shaders/shadowmap.{vs,fs}`](renderer/shaders/).

### 6.1 Two-pass Phong with a hardware shadow map

- **Shadow pass** — render all renderables into a 2048×2048 depth-only FBO
  from the point of view of an orthographic directional light. The light camera
  is fit each frame to the world AABB via `DirectionalLightOBB()`: the OBB's
  third axis points along the light direction, and the `fovy` is set to cover
  the world's tight bounds.
- **Color pass** — render everything from the real camera, with uniforms for
  the light VP matrix, the shadow sampler, ambient color, light direction,
  and `viewPos`. The shadow lookup is 2×2 PCF inside `shadowmap.fs`.

### 6.2 Bodies

`add_body(...)` builds a renderable `Model` plus wireframe-collider `Model`s
for each `btCollisionShape`. Every frame, `update_body(key, q, t)` only
updates translation/quaternion — the GPU meshes are uploaded once.
`is_body_finite()` gates NaN state from ever reaching `DrawModelEx`, so
numerical blow-ups at extreme step sizes can't crash the renderer.

### 6.3 Camera

- **Orbit mode (default)** — target fixed at config's `cam.target`, position
  saved per-demo to `<app>_renderer_config.json`. No input — the scene is on
  rails so the camera stays on the interesting action.
- **Free roam** — toggled with `C`. The cursor is hidden and locked to screen
  center; `UpdateCameraFreeRoam` reads mouse delta each frame, converts to
  yaw/pitch deltas, clamps pitch to `±89°` to avoid gimbal lock, rebuilds
  forward/right/up, then handles `WASD + Space/Ctrl` translation. On re-toggle
  the cursor is released.
- **Light rotation** — optional, active when `opts.movable_light = true`.
  Keys `H/K` change azimuth, `U/J` change inclination, at ~1 rad/s.
- **Coordinate gizmo** — an anti-aliased 3-line indicator in the top-right,
  computed by projecting `{1,0,0}, {0,1,0}, {0,0,1}` through the *same*
  camera then offsetting.

### 6.4 Per-demo config persistence

The renderer saves `cam.position`, `cam.target`, `cam.up`, `fovy`, `projection`,
`light_dir`, `world_aabb`, `screen_width/height`, `fullscreen`, `fps` to
`<app_name>_renderer_config.json` on destructor. On next run, if the JSON is
found it **overrides** the code-provided defaults. This is why you can
nudge the camera once, close the window, and next launch is exactly where
you left it — see [demo_chain_pendulum_renderer_config.json](demo_chain_pendulum_renderer_config.json).

---

## 7. Asset Pipeline — glTF → ArticulationForest

[`loader/gltf_parser.cpp`](loader/gltf_parser.cpp).

Blender's physics add-on exports a glTF with extensions:

- `KHR_implicit_shapes` — box / sphere / cylinder / capsule primitives.
- `KHR_physics_rigid_bodies` — rigid-body motion properties, collider links,
  physics materials, joints (type + DoF limits + drives).
- `khr_physics_extra_props` — our own extension for `non_renderable` flags.
- `khr_physics_extra_constraint_props` — marks the "joint properties" helper
  nodes.

Parsing is a **3-pass** process:

1. **Label every node** — `label_physical_node()` computes a
   `NodePhysicsLabel` enum: `NonPhysical`, `StaticCompoundParent`,
   `DynamicCompoundParent`, `CompoundChildImplicit`, `DynamicTrivialConvexHull`,
   `JointProps`, `JointSpaceA/B`, etc. This is necessary because the *same* glTF
   node can mean wildly different things depending on whether it has mass,
   children, or is named `jointSpaceA`.
2. **Build colliders** — implicit shapes become `ImplicitShape`, convex hull
   nodes become baked `MeshData` via `build_convex_hull_shape()` (vertices are
   pre-multiplied with their parent transform so physics sees world-coordinates).
   Compound parents aggregate children into a single `vector<Collider>`.
3. **Collect articulation links** — for every `jointSpaceA` node we look up
   its partner `jointSpaceB` via the node-id reference, find the nearest
   `JointProps` sibling (used to get the user-friendly joint name), and build
   an `ArticulationLinkage` with both bodies, both joint frames, the joint
   type (with Blender's YZ / XZ axis-correction quaternion applied), and a
   `DofSprings` list if the joint has `drives`.

Finally, `build_and_flatten_articulation_links()` groups linkages into
articulation components via BFS:

- **Rooted components** — start from any parent body that is never itself a
  child; BFS outward, marking visited edges.
- **Cyclic components** (scissor lifts, wishbones) — prefer a static body as
  start so it becomes `art_group[0]`, otherwise fall back to any remaining
  parent. BFS from there; the one unvisited edge closing the loop will become
  a loop joint in `ArticulatedBody::build_tree`.

After parsing, the caller gets:

- `scene.graph` — flat list of `SceneNode` with world + local transforms and
  renderables (triangle meshes).
- `scene.art_forest` — list of `ArticulationTree`, each a vector of
  `ArticulationLinkage` in BFS order. Body IDs are remapped so they index
  into `scene.art_groups[i]` (a subset of `scene.graph` that belongs to that
  articulation).
- `scene.art_groups` — the per-articulation body subsets.
- `scene.rigidbody_group`, `scene.non_physical_group` — everything else.

---

## 8. Command Server — TCP control for demos

[`command/command_server.cpp`](command/command_server.cpp) opens a TCP port
that accepts simple newline-framed commands from a Python client
([command/command_server_client.py](command/command_server_client.py)).
Used in demos that need a closed-loop controller (e.g., PID applied to a
joint) without baking the controller into C++.

- Commands: `get_q <art_id> <joint_id>`, `get_dq ...`, `set_tau ...`,
  `set_q_target ...`. The server thread drains one command per `update_world_cb`
  tick, applies the effect (typically adding to `joint->taue`), then sends
  back the requested measurement.
- Robustness: PID output is clamped; velocity NaNs reset the controller state
  before they cascade into the solver. The `InvOrPinvSolver` on our side has
  a NaN guard so even pathological inputs don't hang Eigen's BDCSVD.

---

## 9. The Chain Pendulum — What, How, Why

Scene defined entirely in code ([demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp))
— no glTF asset needed.

### 9.1 Construction

```
  anchor (Static cube, 0.25 × 0.10 × 0.25)
    └─ revolute j0 at (0, −ANCHOR_HALF_Y, 0)
        └─ link_0 (Cuboid, 0.16 × 0.50 × 0.16, ρ=8)
            └─ revolute j1 at (0, −0.25, 0)
                └─ link_1 …
                    …
                        └─ link_{N−1}
```

Every link is a thin vertical cuboid with density 8 (~steel-ish). Every joint
is a revolute about the world Z axis (joint basis is identity). Every joint
gets `SpringParam{k = 0, d = JOINT_DAMPING = 80}` so the whole chain has
joint-space damping but no restoring springs.

**Why 80 and not smaller?** In a chain swinging as a near-rigid whip, the
*relative* joint velocities are small because adjacent links move together.
Joint damping therefore dissipates very little energy. A headless sweep
[(`demos/tune_chain_damping.cpp`)](demos/tune_chain_damping.cpp) ran the same
30-link chain for 12 seconds across `d ∈ {50, 80, 120, 180}` and tracked the
*peak* `|dq|` and `|q_top|` per second. `d=80` was the smallest value that
brought peak `|dq|` below ~0.05 rad/s within 10 seconds without visibly
over-damping the first swing.

### 9.2 Interaction

- `SPACE` — adds `+50` to `tree_joints[0]->taue(0)` for one frame: a whip-crack
  impulse on the top joint.
- `R` — rebuilds a fresh world + chain at the current `N`, resets joint q,
  reapplies the initial `INITIAL_ANGLE = 0.7 rad` perturbation on the top joint,
  `move_joints() + project_position()` to propagate poses.
- HUD prints an **EMA-smoothed step time** every 60 frames.

### 9.3 Benchmark harness

`print_scaling_table({5,10,20,30,40,50,60}, 200)` builds a fresh world per N,
takes one warm-up step, then times 200 steps with
`std::chrono::high_resolution_clock`. Output includes:

- absolute `µs/step`
- ratio vs N=5 (empirical scaling)
- an O(n³) prediction and an O(n) prediction for comparison

Observed on my machine: N=5 → 6.2 µs, N=60 → 119.1 µs. That's ~19× for a 12×
increase in N — super-linear but nowhere near cubic. The visible trend
confirms `H.llt()` and the O(n²) `compute_H` dominate at large N, while
per-step overhead (collide, warm-start, PGS) dominates at small N.

---

## 10. Demonstration Q&A (Low-Level & Practical)

A curated set of the kind of questions that come up in a live walkthrough.
Answers cite the exact function / file / line so you can pull it up on the spot.

### 10.1 Physics / Math

**Q: Is this truly O(n) Featherstone / ABA?**
Not in the form implemented. It's **CRBA + dense Cholesky**: `compute_H` is
O(n²), `H.llt()` is O(n³). Bias forces are RNEA, which is O(n).
See [articulatedbody.cpp:890](featherstone/articulatedbody.cpp#L890).
For the N≤60 regime we target, this is fine and in practice flatter than
cubic because per-step overhead dominates.

**Q: What integrator do you use?**
Semi-implicit (symplectic) Euler. For joints with springs/damping, the
effective-mass matrix absorbs `dt²·Ks + dt·Ds` so stiff passive joints are
unconditionally stable. See
[articulatedbody.cpp:877](featherstone/articulatedbody.cpp#L877).

**Q: How are spatial vectors laid out?**
`[angular(3); linear(3)]` — the "Featherstone ordering." Transforms are 6×6,
inertias are 6×6 `Dyad`s. This matches
[Featherstone 2008, *Rigid Body Dynamics Algorithms*] exactly.

**Q: How do you handle a loop like a scissor lift?**
BFS picks some spanning subset as tree joints, the rest become `loop_joints`.
At solve time, the loop's constraint is enforced by Sequential Impulses
against a constraint subspace `T_ortho`, with Baumgarte stabilization in the
position pass. See
[loop_joint_solver.cpp:59](featherstone/loop_joint_solver.cpp#L59).

**Q: How do gears / worm drives / screws work without drift?**
They're declared as **linear DoF couplings** `C·dq = 0`. We compute
`Z = orthoComplement(C)` once at tree-build time and solve in the reduced
space: `Zᵀ H Z · ddq_reduced = Zᵀ · rhs`. Because the coupling is baked into
the coordinate system, it can't drift — it's exactly satisfied every frame.

**Q: What collision detection stack is underneath?**
Bullet3, used for broadphase + narrowphase + persistent manifolds only. We
consume `btPersistentManifold::getContactPoint(i)` in `ContactSolver::initialize`
and do all of the dynamics ourselves. See
[contact_solver.cpp:19](featherstone/contact_solver.cpp#L19).

**Q: How does the contact solver work?**
Projected Gauss-Seidel with warm starting. Per-contact: build a contact
frame `[tx | ty | n]`, compute 2×2 tangential + scalar normal effective
mass, iterate 10 velocity iterations clipping tangential to a Coulomb cone
and normal to `si_n ≥ 0`, 8 position iterations with Baumgarte (`β=0.2`,
`slop=0.001`, `max=0.4`). Warm start persists sequential impulses via
Bullet's `m_userPersistentData`.

**Q: How is restitution implemented without jitter at rest?**
As a **velocity bias** `v_bias = e·v_approach` applied *only* when
`v_approach < −restitution_threshold`. Below that threshold the bias is
zero, so resting stacks stay at rest. See
[contact_solver.cpp:110](featherstone/contact_solver.cpp#L110).

**Q: What happens if the effective mass goes singular (edge-edge contact
with a DoF-missing normal)?**
`InvOrPinvSolver` catches it: `FullPivLU` of the 2×2 tangential mass, fallback
to SVD when it's rank-deficient; the normal scalar falls back to `1E4` as
inverse mass so the solver emits a big impulse but doesn't NaN. A warning
is logged to stdout. See
[contact_solver.cpp:99](featherstone/contact_solver.cpp#L99).

**Q: How do you prevent parent-child collisions inside an articulated body?**
Per-articulated-body adjacency map built from `disable_collision` joints;
consulted by a Bullet `btOverlapFilterCallback`. Pairs get culled before
narrowphase. See `AdjacentLinkFilter` in
[rigidworld.cpp](featherstone/rigidworld.cpp).

**Q: Why are PID outputs clamped and velocity NaNs reset?**
We saw cascading NaN when a user's PID gain was too high: velocity
blew up → `H` became singular → SVD went infinite loop. Defense in depth
is: clamp τ output at PID level, reset `dq` on NaN at integrator level,
guard `InvOrPinvSolver` at solve level, skip non-finite bodies at render
level.

### 10.2 Chain Pendulum Specifics

**Q: Why 30 links?**
README's scaling-benchmark claim targets "long serial chain (30+ links)."
30 is long enough that `compute_H`'s O(n²) is well above floor noise, but
short enough that Bullet's per-step overhead is still comparable — you see
the *transition region* of the cost curve, which is what makes the demo
interesting.

**Q: Why does the chain self-collide at large N?**
`disable_collision = true` only prunes adjacent parent-child pairs.
Link 5 and link 15, with enough slack, can overlap because Bullet's filter
says "yes they're allowed to touch." We cap the benchmark at N=60 and use a
tiny initial perturbation (0.03 rad) in the timed runs to keep
self-collision out of the measurement.

**Q: Why d=80 damping?**
The chain swings as a near-rigid whip, so relative joint velocities are
small — joint-space damping can't dissipate much energy per unit `d`. The
tuner ([demos/tune_chain_damping.cpp](demos/tune_chain_damping.cpp)) swept
`d ∈ {50, 80, 120, 180}` and reported the peak `|dq|` per second. d=80 was
the smallest value that brought peak `|dq|` to <~0.05 rad/s within 10 s
without killing the first swing.

**Q: How do you measure step time fairly?**
Build a fresh world + chain per `N`, take **one warm-up step** (touches
caches, triggers Bullet to allocate its overlap cache), then time
`M_steps = 200` steps of `world->step(dt)` with
`std::chrono::high_resolution_clock`. Divide and report µs/step. See
`benchmark_step_us()` in [demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp).

**Q: When does the animation stop?**
It never *terminates*, but motion decays to under the visible threshold at
~10 s because of joint damping. The step loop still runs — close the
window with ESC or the `X` button. `R` resets the pendulum to the initial
tilt.

### 10.3 Rendering / Camera

**Q: How are shadows done?**
Two-pass: an orthographic light camera writes a 2048×2048 depth texture
(no color attachment) from the light's POV; color pass samples that depth
via a 2×2 PCF lookup in
[renderer/shaders/shadowmap.fs](renderer/shaders/shadowmap.fs). The light
camera is fit each frame to the world AABB using
`DirectionalLightOBB(world_aabb, light_dir)` so the shadow map stays tight.

**Q: How does the free-roam camera work?**
Toggle with `C`. We hide the cursor, lock it to screen center, and read the
per-frame mouse delta. Pitch is derived from `asin(forward.y)`, yaw from
`atan2(forward.z, forward.x)`; deltas are added, pitch is clamped to ±89°
to avoid gimbal lock, then forward/right/up are rebuilt from the angles.
`WASD` moves along forward/right, `Space/Ctrl` moves along up. See
[rigidworld_renderer.cpp:772](renderer/rigidworld_renderer.cpp#L772).

**Q: Why does the camera remember where I put it?**
`~RigidWorldRenderer()` writes camera + world AABB + light dir to
`<app>_renderer_config.json`. Constructor reads it first and, if present,
lets it override code defaults. See
[rigidworld_renderer.cpp:107](renderer/rigidworld_renderer.cpp#L107).

**Q: How do you avoid gimbal lock?**
Pitch clamped to ±89° in `UpdateCameraFreeRoam`. For full 3-DoF spherical
joints inside the physics engine, we use quaternions, never Euler angles.

**Q: How are wireframe colliders drawn?**
For each body we built a second `Model` per-collider at `add_body()` time.
In the color pass, if the user pressed `G` to toggle visibility, we
`rlDisableBackfaceCulling()` and `DrawModelWiresEx` in black, then re-enable
culling. See [rigidworld_renderer.cpp:539](renderer/rigidworld_renderer.cpp#L539).

**Q: What if the physics produces NaN?**
`is_body_finite()` gates every `DrawModelEx` call. NaN bodies simply don't
draw, so the window never freezes or crashes when the physics misbehaves —
you just see a body disappear. The next `world->step` usually recovers; if
not, hit `R`.

### 10.4 Build / Tooling

**Q: Why is the build structure the way it is?**
CMake sets `RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_BINARY_DIR}/bin`, then each
demo target overrides it to `${CMAKE_BINARY_DIR}/bin/demos`. Shared libs go
to `lib/`. The top-level CMakeLists
([CMakeLists.txt](CMakeLists.txt)) requires `EIGEN3_INCLUDE_DIR` and
`GLM_INCLUDE_DIR`; everything else is pulled in through
[third_party/CMakeLists.txt](third_party/CMakeLists.txt).

**Q: Why two launcher scripts?**
[run_presentation.sh](run_presentation.sh) for Linux/macOS,
[run_presentation.ps1](run_presentation.ps1) for Windows. Both configure
and build release, then run each demo in a fixed sequence; the Windows
version just uses `& $Exe` in place of `eval "$cmd"`. The chain pendulum
is called inline because it takes `N` as **argv**, not stdin (unlike the
load-scene demos which pipe `0 / 1 / 2` for scene selection).

**Q: How do I enable NVIDIA GPU offload on a hybrid laptop?**
`run_presentation.sh` sets `__NV_PRIME_RENDER_OFFLOAD=1` and
`__GLX_VENDOR_LIBRARY_NAME=nvidia` before launching. It's a no-op on
machines without hybrid graphics.

**Q: How do I add a new demo?**
1. Create `demos/demo_<name>.cpp` with a `main()` that calls `init_renderer`,
   `init_world`, optionally `load_gltf`, builds bodies, and calls
   `renderer.run(update_cb, 3d_aid_cb, options)`.
2. CMake's glob picks it up automatically on next configure (`cmake -S . -B build`).
3. Add an entry to `run_presentation.sh` / `.ps1` if you want it in the
   presentation sequence.

### 10.5 Edge cases / war stories

**Q: What was the hardest bug to chase?**
NaN cascades. One wrong PID gain in a demo would blow up `dq`, which made
`H` singular, which sent Eigen's `BDCSVD` into an infinite loop (well-known
Eigen bug on NaN input). Fix required defense in depth — the
`InvOrPinvSolver` NaN guard is the key one.

**Q: Why track *peak* `|dq|` per second instead of instantaneous?**
Damping tuners that sampled instantaneous velocity at integer seconds
happened to catch zero-crossings and reported the chain was "settled" when
it was in fact still swinging hard. Peak-over-window is the only honest
metric.

**Q: Why `InvOrPinvSolver` instead of plain Eigen inverse?**
Three reasons: (1) effective-mass matrices go rank-deficient at edge-edge
contacts; (2) `FullPivLU` is faster than SVD when the matrix is regular;
(3) Eigen's `BDCSVD` hangs on NaN. `InvOrPinvSolver` = LU first, SVD fallback,
NaN guard, explicit Zero mode. See [spvec.hpp](featherstone/spvec.hpp).

---

## 11. Files to Read in Order (if you're new)

1. [featherstone/spvec.hpp](featherstone/spvec.hpp) — spatial algebra types.
2. [featherstone/rigidbody.hpp](featherstone/rigidbody.hpp) — `RigidBody`, `Config`.
3. [featherstone/articulatedbody.hpp](featherstone/articulatedbody.hpp) — articulated body API.
4. [featherstone/articulatedbody.cpp](featherstone/articulatedbody.cpp) §§ `build_tree`, `compute_H`, `compute_bias_RNEA`, `solve_ddq`.
5. [featherstone/rigidworld.cpp](featherstone/rigidworld.cpp) — the `step(dt)` sequence.
6. [featherstone/contact_solver.cpp](featherstone/contact_solver.cpp) + [featherstone/loop_joint_solver.cpp](featherstone/loop_joint_solver.cpp).
7. [renderer/rigidworld_renderer.cpp](renderer/rigidworld_renderer.cpp) — render loop.
8. [loader/gltf_parser.cpp](loader/gltf_parser.cpp) — asset pipeline.
9. [demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp) — end-to-end scene in 200 lines.

---

## 12. One-line TL;DR per Module

- `spvec.hpp` — 6-D spatial algebra + robust inverse solver.
- `rigidbody.{hpp,cpp}` — rigid body with COM, inertia, damping.
- `articulatedbody.{hpp,cpp}` — reduced-coordinate multibody: RNEA + CRBA + Cholesky, loops, springs, couplings.
- `rigidworld.{h,cpp}` — the per-step orchestration, Bullet wrappers, adjacency filter.
- `contact_solver.{h,cpp}` — PGS with warm start, Coulomb cone, Baumgarte.
- `loop_joint_solver.{h,cpp}` — SI on loop constraints with `T_ortho` + Baumgarte.
- `spshapes.{hpp,cpp}` — shape inertia tensors incl. tetrahedron-decomposed hulls.
- `rigidworld_renderer.{h,cpp}` — raylib, shadow-map, orbit + free-roam camera, config JSON.
- `gltf_parser.{h,cpp}` — Blender KHR_physics → `Scene` + `ArticulationForest`.
- `command_server.{h,cpp}` — TCP loop for external PID / τ control.
- `demo_*.cpp` — one `main()` per scene, all ~100–200 lines.
