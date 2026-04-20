# TorqueForge — The Pipeline, Explained

A friendly, end-to-end walkthrough of **how a scene becomes pixels** in TorqueForge, and **how the physics actually drives it**. We use the **chain pendulum** (a hanging chain of N rigid links) as the running example because every stage of the pipeline is visible in ~200 lines of code.

If you have read [DEEP_DIVE.md](DEEP_DIVE.md) this is the gentler sibling: fewer citations, more narrative, more "why does this step even exist."

---

## 0. Mental Model — The Three Worlds

TorqueForge is really three cooperating worlds that share one heartbeat (the main loop):

```
    ┌──────────────────┐      ┌──────────────────┐      ┌──────────────────┐
    │ 1. PHYSICS WORLD │      │ 2. COLLISION     │      │ 3. RENDER WORLD  │
    │                  │      │    WORLD         │      │                  │
    │  ArticulatedBody │◀────▶│   Bullet3 shapes │      │ raylib Models    │
    │  RigidBody       │      │   broadphase +   │      │ + shaders +      │
    │  joints, q, dq   │      │   narrowphase    │      │ shadow map       │
    │  Eigen math      │      │   manifolds      │      │ GLM math         │
    └──────────────────┘      └──────────────────┘      └──────────────────┘
            │                         │                          ▲
            │  contact manifolds      │                          │
            └─────────────────────────┘                          │
                       │                                         │
                       ▼                                         │
               PGS contact solver                                │
               loop-joint solver                                 │
                       │                                         │
                       ▼                                         │
               new q, new position ──────────────────────────────┘
```

- **Physics world** owns the truth: joint angles `q`, joint velocities `dq`, link transforms. It is Eigen-based 6-D spatial algebra.
- **Collision world** is Bullet3 — we borrow it *only* to tell us which pairs are touching and where. Bullet never integrates anything.
- **Render world** is raylib — it consumes per-frame link poses and draws them. It never reads the joint state, only the resulting world transforms.

Each frame: physics steps, collision detects, solver resolves, poses update, render draws. That's the whole story.

---

## 1. The File Map (Pendulum-Flavored)

| Stage | Code | What it does for the pendulum |
|-------|------|-------------------------------|
| Entry point | [demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp) | `main()`, CLI parsing, scene build, main loop callbacks |
| Shared scaffolding | [demos/common/demo_common.cpp](demos/common/demo_common.cpp) | `init_renderer()`, `init_world()`, Eigen↔GLM helpers |
| Articulated body | [featherstone/articulatedbody.cpp](featherstone/articulatedbody.cpp) | Holds the 30 links + 30 revolute joints, solves `ddq` |
| Rigid body | [featherstone/rigidbody.hpp](featherstone/rigidbody.hpp) | One link: pose, mass, inertia |
| Spatial algebra | [featherstone/spvec.hpp](featherstone/spvec.hpp) | 6-D motion/force vectors, 6×6 transforms, robust inverse |
| Shape + inertia | [featherstone/spshapes.cpp](featherstone/spshapes.cpp) | Cuboid inertia for each link |
| World orchestration | [featherstone/rigidworld.cpp](featherstone/rigidworld.cpp) | The 10-step `step(dt)` sequence |
| Contact solver | [featherstone/contact_solver.cpp](featherstone/contact_solver.cpp) | If links self-collide, this handles it |
| Renderer | [renderer/rigidworld_renderer.cpp](renderer/rigidworld_renderer.cpp) | Shadow pass, color pass, HUD, camera |
| Shaders | [renderer/shaders/shadowmap.fs](renderer/shaders/shadowmap.fs), [.vs](renderer/shaders/shadowmap.vs) | Phong + PCF shadows on the GPU |

---

## 2. The Big Picture — One Frame in Plain English

```
User presses Space
   │
   ▼
[main loop tick]
   │
   ├── 1. Poll input (SPACE → extra torque on joint 0)
   ├── 2. world->step(dt)
   │        ├── Bullet: who is touching whom?
   │        ├── Physics: compute joint accelerations ddq
   │        ├── Integrate velocities: dq += ddq · dt
   │        ├── Solve contacts + loop constraints (velocity pass)
   │        ├── Integrate positions: q += dq · dt
   │        └── Solve contacts + loops again (position correction)
   ├── 3. For each link: copy world transform into renderer
   ├── 4. Renderer draws
   │        ├── Shadow pass  (depth-only, from light's POV)
   │        ├── Color pass   (Phong + shadow sample, from camera)
   │        └── HUD / gizmos (camera axes, debug joint frames)
   └── 5. Swap buffers → your monitor shows the chain at a new angle
```

A single frame at 60 FPS has ~16.67 ms to do all of this. For N=30, the physics step takes ~50 µs — well under 1% of the frame. The rest is drawing.

---

## 3. Building the Pendulum (Construction Phase)

Open [demo_chain_pendulum.cpp:34-90](demos/demo_chain_pendulum.cpp#L34-L90). The function `build_chain(N, base_pos)` is where nothing moves yet — we are just describing the world.

### 3.1 One Anchor + N Links

```
  ┌───────────┐
  │  anchor   │  ← Static body (doesn't move, infinite mass conceptually)
  └─────┬─────┘
        │  joint j0 (revolute about world-Z)
        ▼
  ┌───────────┐
  │  link_0   │  ← Dynamic body, cuboid 0.16 × 0.50 × 0.16, density 8
  └─────┬─────┘
        │  joint j1
        ▼
  ┌───────────┐
  │  link_1   │
  └─────┬─────┘
        ⋮
  ┌───────────┐
  │  link_{N-1}│
  └───────────┘
```

Each link is a `RigidBody` holding:
- A **Cuboid shape** (gives us mass + inertia tensor via the closed-form formula in [spshapes.cpp](featherstone/spshapes.cpp) — a thin rod's inertia is approx `m · L² / 12` about its transverse axes).
- A pose `(translation, rotation)` in world coordinates.
- A density (8 kg/m³ means each link weighs ~0.1 kg).

### 3.2 Joints = "How Can These Move Relative to Each Other"

A **revolute joint** is the mechanical hinge. It has:
- A **motion subspace** `S` that says "the only relative motion allowed is rotation about Z." In 6-D spatial algebra, `S = [0,0,1,0,0,0]ᵀ`.
- A **frame on each body** (`bt0`, `bt1`) — the pin's position relative to each link.
- A single degree of freedom `q` (the hinge angle), with velocity `dq` and acceleration `ddq`.
- Optional **springs** and **damping** per DoF. We set `k=0` (no spring) and `d=80` (heavy viscous damping) so the chain eventually settles.

After `add_joint()` has been called N times, the articulated body looks like a linked list, but internally it is stored as a **tree** with parent indices `lambda[i]` and children `mu[i]`. This tree structure is what lets the solver scale well.

### 3.3 `build_tree()` — The Compile Step

The moment we call `world->add_body(ch.art)`, the articulated body runs `build_tree()` ([articulatedbody.cpp:443](featherstone/articulatedbody.cpp#L443)):

1. **BFS from the anchor.** Each time BFS reaches a new link, that joint is marked a **tree joint**. If BFS ever revisits a link (which doesn't happen for a simple chain, but *would* for a scissor lift), that extra joint becomes a **loop joint** — resolved later by impulses.
2. **Allocate the big matrices.** For N=30 revolutes, total DoF = 30. So `H` (joint-space inertia) is 30×30, `K` (loop Jacobian) is empty.
3. **Turn on the spring code path.** Because our joints have non-zero damping, `enable_springs = true`, which routes the solver through the implicit-Euler effective-mass path (more on this in §5.4).

At this point the pendulum exists but is perfectly vertical. Nothing will happen if we start stepping. So we cheat a little:

```cpp
ch.art->tree_joints[1]->q(0) = INITIAL_ANGLE;   // 0.7 rad ≈ 40°
ch.art->move_joints();                           // propagate joint q → link poses
ch.art->project_position();                      // compute link world transforms
```

Now joint 0 is tilted 0.7 rad off vertical. Every other joint is at 0. The chain hangs in an "L" shape, waiting for gravity.

---

## 4. Registering With the Renderer (Visual Build Phase)

The physics world has no idea what anything looks like. The renderer doesn't know about mass or torque. We wire them together in `register_chain_with_renderer()` ([demo_chain_pendulum.cpp:93-121](demos/demo_chain_pendulum.cpp#L93-L121)):

```cpp
Mesh collider   = build_mesh(Cuboid, link_half);  // wireframe for debug
Mesh renderable = build_mesh(Cuboid, link_half);  // solid for drawing
size_t key = g_renderer->add_body({collider}, renderable, position, rotation);
```

`add_body()` uploads a mesh to the GPU **once** and returns a key. Every subsequent frame, we just call `update_body(key, new_rotation, new_translation)` to move that mesh — no re-upload, no re-allocation.

The `ch.render_keys` vector stores the key for each of the 31 bodies (1 anchor + 30 links), in the same order as `ch.art->bodies`. So syncing is a trivial for-loop:

```cpp
for (size_t i = 0; i < ch.art->bodies.size(); ++i) {
    g_renderer->update_body(
        ch.render_keys[i],
        q(ch.art->bodies[i]->rotation),
        v3(ch.art->bodies[i]->translation));
}
```

**Design principle:** the renderer is a pure consumer of poses. It never calls back into physics. This makes it easy to replace either side.

---

## 5. Physics — What Happens Each `world->step(dt)`

This is the heart of the pipeline. `RigidWorld::step(dt)` ([rigidworld.cpp](featherstone/rigidworld.cpp)) is a 10-stage recipe. Let's walk through it with the pendulum in mind.

### 5.1 Collision Detection (Bullet)

```
collide() → Bullet broadphase + narrowphase → persistent manifolds
```

Bullet asks: "which pairs of shapes have overlapping AABBs?" (fast), then "of those, which actually touch, and where?" (slower). The output is a list of **contact points** — world-space position, normal, penetration depth.

For a swinging chain, most frames have **zero contacts** (the links don't touch each other). `disable_collision=true` on each joint tells Bullet to ignore parent-child pairs (link 5 vs link 6), because those are forced together by the hinge anyway — checking them would be wasteful and would produce spurious contacts.

### 5.2 Compute Joint Accelerations (the hard one)

For a free-floating rigid body, Newton gives you `F = ma` and you're done. But an articulated body has constraints: link 5 can't move independently of link 4 — they share a hinge. The classical way to handle this is the **Featherstone family** of algorithms.

TorqueForge uses a three-step solve per articulated body:

#### Step 1 — RNEA (Recursive Newton-Euler Algorithm), O(n)

Function: `compute_bias_RNEA()` at [articulatedbody.cpp:828](featherstone/articulatedbody.cpp#L828).

Goal: compute the **bias forces** `C(q, dq)` — everything that isn't the mass matrix × acceleration. This includes:
- **Gravity** pulling each link down.
- **Coriolis / centrifugal** terms from the fact that link 5 is rotating relative to link 4, which is rotating relative to link 3, etc.
- **Passive spring/damper forces** `−Ks·q − Ds·dq`.

The algorithm is a two-pass recursion over the tree:
- **Forward pass (root → leaves):** propagate velocity `v` and acceleration `a` outward. Each child gets the parent's motion plus its own joint's contribution.
- **Backward pass (leaves → root):** propagate force `f` inward. Each parent accumulates the net force its children exert on it.

Out pops `C ∈ ℝ^N`. For our pendulum, `C` essentially encodes "how much torque gravity is putting on each hinge."

#### Step 2 — CRBA (Composite Rigid-Body Algorithm), O(n²)

Function: `compute_H()` at [articulatedbody.cpp:890](featherstone/articulatedbody.cpp#L890).

Goal: build the **joint-space inertia matrix** `H ∈ ℝ^{N×N}`. `H[i][j]` tells you how much force on joint `j` you get per unit acceleration at joint `i`.

The method: walk the tree bottom-up. For each subtree, compute its **composite spatial inertia** (treating the subtree as one rigid body). Then for each ancestor joint, project that composite inertia through the chain of motion subspaces.

For a chain pendulum, `H` is **dense and banded**: joint 0 couples to every other joint because moving joint 0 swings everything below it.

#### Step 3 — Solve, O(n³)

Function: `solve_ddq()` at [articulatedbody.cpp:78](featherstone/articulatedbody.cpp#L78).

The equation of motion in joint space is:

```
    H(q) · ddq  =  τ  −  C(q, dq)
```

Where `τ` is the applied joint torque (e.g., the +50 when you press Space). We solve with **Cholesky factorization** (`H.llt()`) since `H` is symmetric positive-definite.

The result `ddq ∈ ℝ^N` is the acceleration of each joint. For the pendulum, typical values are single-digit rad/s² under gravity.

### 5.3 Integrate Velocities — Semi-Implicit Euler

```cpp
dq += ddq · dt;
```

That's it. One line, but this is the **symplectic (semi-implicit) Euler** step — it is stable for oscillating systems in a way that plain explicit Euler is not. You update velocity first, then use the *new* velocity to update position next. This is why energy doesn't artificially grow.

### 5.4 Stiff Springs? Use the Implicit Trick

If any joint has `k > 0` or `d > 0`, `solve_ddq()` routes through `compute_H_spring(dt)`:

```
    H_eff  =  H  +  dt²·diag(Ks)  +  dt·diag(Ds)
    rhs    =  τ  −  C  −  Ks·q  −  Ds·dq
    ddq    =  H_eff⁻¹ · rhs
```

The extra `dt² · Ks` and `dt · Ds` absorb stiff passive forces **implicitly**, so you don't need tiny `dt` to stay stable. The pendulum's `d=80` damping is comfortable for `dt=1/60 s` thanks to this.

### 5.5 Solve Contacts — Projected Gauss-Seidel

Even though the pendulum usually has no contacts, let's describe what would happen if two links touched:

For each contact point Bullet reported:
1. Build a **local frame** with normal and two tangents.
2. Compute **effective mass** in that frame (how much impulse is needed to change relative velocity by 1 m/s).
3. Iterate 10 times: solve tangential (friction, clipped to Coulomb cone), then normal (clamped ≥ 0, so things don't stick).
4. **Warm start:** store the final impulse; next frame, start from there instead of zero. This turns a 10-iteration problem into effectively a 2-iteration problem after the first touch.

### 5.6 Integrate Positions

```cpp
q += dq · dt;
```

For revolute joints this is scalar. For spherical joints (quaternion state), we integrate using the exponential map to stay on the unit quaternion manifold.

### 5.7 Position Correction — Baumgarte

After integrating, contacts and loop joints may have drifted by a tiny amount. 8 iterations of a position-level PGS pass nudge things back into place without imparting velocity. Parameters: `β=0.2`, `slop=0.001`, `max_correction=0.4`. Too strong → visible pop; too weak → drift. These values are tuned to be invisible.

### 5.8 Project World Poses

`project_position()` walks the tree from root and computes each link's **world-space transform** from the chain of joint `q` values. This is what the renderer reads next.

---

## 6. The Render Pipeline — Pixels Finally

Once physics has produced new world transforms for every link, the renderer takes over. See [rigidworld_renderer.cpp](renderer/rigidworld_renderer.cpp).

### 6.1 The Main Loop (`run(...)`)

```cpp
while (!WindowShouldClose()) {
    update_world_cb(frame_dt, frame_id);   // your callback: step physics, update keys
    shadow_pass();
    color_pass();
    hud_and_gizmo_pass();
    debug_draw_cb(frame_dt, frame_id);     // your callback: draw joint frames, labels
    SwapBuffers();
}
```

### 6.2 Shadow Pass — Render From the Light's POV

The sun is a directional light. To produce shadows, we first render the scene **from the sun's perspective** into a 2048×2048 depth-only texture. We do not write color here — we only care about "what's the closest thing the sun can see in each direction."

Two subtleties:
- **Fit the light frustum to the world AABB.** `DirectionalLightOBB(world_aabb, light_dir)` picks an orthographic box that just barely encloses the scene. Too big → low shadow resolution. Too small → things outside clip.
- **Render every renderable mesh** into this pass, regardless of whether it would be visible from the camera.

### 6.3 Color Pass — Render From the Camera

Now draw the actual pixels you see. For each mesh:
- The vertex shader ([shadowmap.vs](renderer/shaders/shadowmap.vs)) transforms vertices by the model matrix, then by the camera's view-projection. It also computes each vertex's position in the *light's* space so the fragment shader can do shadow lookup.
- The fragment shader ([shadowmap.fs](renderer/shaders/shadowmap.fs)) does:
  1. **Phong lighting**: diffuse `max(0, dot(n, L))` + ambient + a weak specular.
  2. **Shadow sample**: transform the fragment into light space, look up the depth texture at that position. If the stored depth is less than the fragment's depth (minus bias), this fragment is in shadow.
  3. **PCF (Percentage-Closer Filtering)**: sample 4 neighboring texels and average, producing softer shadow edges.

Each link of the pendulum gets shaded this way. You see a swinging chain with a soft shadow on the ground plane.

### 6.4 HUD & Gizmos

A final pass draws:
- **FPS counter / step-time HUD** in top-left.
- **Coordinate axis gizmo** in top-right (tiny red/green/blue lines projected through the same camera).
- **Debug joint frames** — in the pendulum demo, `debug_draw` draws a small RGB triad at every hinge, so you can *see* the revolute axis.

### 6.5 Camera

Two modes, toggled with `C`:
- **Orbit (default):** camera sits at a fixed offset, always looking at `cam.target`. No input. Position is saved to `demo_chain_pendulum_renderer_config.json` on exit so next launch is identical.
- **Free roam:** cursor is hidden and locked to screen center. Mouse delta → yaw/pitch (pitch clamped to ±89° to avoid gimbal lock). `WASD + Space/Ctrl` for translation.

---

## 7. A Whole Frame, Step-by-Step (Pendulum)

Let's say you just pressed `SPACE`. Here is frame N+1 in concrete detail:

```
t = 0.0 ms   Input polling
             IsKeyPressed(KEY_SPACE) == true
             → tree_joints[1]->taue(0) += 50.0   (applied torque on joint 0)

t = 0.1 ms   world->step(0.01667)
             │
             ├─ Bullet collide():      0 contacts this frame
             ├─ compute_bias_RNEA():   C has gravity + damping + Coriolis
             ├─ compute_H():           build 30×30 joint-space inertia
             ├─ solve_ddq():           H_eff · ddq = τ − C
             │                         τ = [50, 0, 0, …, 0]  (the kick)
             │                         ddq[0] ≈ large positive (kick on top)
             │                         ddq[i>0] small (rest pulled along via H)
             ├─ integrate_velocity:    dq += ddq · 0.01667
             ├─ contact_solver:        no-op
             ├─ loop_joint_solver:     no-op (no loops)
             ├─ integrate_position:    q += dq · 0.01667
             ├─ position correction:   no-op
             └─ project_position():    every link's world transform updated

t = 0.15 ms  tree_joints[1]->taue(0) = 0.0f    (kick was one-frame only)

t = 0.16 ms  Sync renderer
             for each of 31 bodies: update_body(key, rot, pos)

t = 0.2 ms   Shadow pass (GPU): 31 meshes rendered to depth texture

t = 3.0 ms   Color pass (GPU): 31 meshes rendered with Phong + PCF shadow

t = 3.5 ms   HUD pass (GPU): gizmo + FPS text + debug joint frames

t = 4.0 ms   SwapBuffers → monitor shows the chain mid-whip
```

Next frame, `taue(0)` is back to 0, but `dq[0]` is now non-zero (the kick gave it velocity), so the chain keeps whipping through its natural motion. Gravity and joint damping gradually dissipate the energy; after ~10 seconds the chain returns to hanging vertically.

Press `R` to reset: every joint is zeroed, `q[0]` gets the initial 0.7 rad tilt, `project_position()` re-propagates poses, and the chain starts swinging again.

---

## 8. What Makes the Physics Accurate

Three design choices keep the pendulum behaving like a real chain:

1. **Reduced coordinates.** We solve directly for joint angles `q`, not for each link's position with penalty constraints holding them together. This means links **cannot** fly apart no matter how stiff or fast the motion.
2. **Implicit spring/damper integration.** The `H + dt²·Ks + dt·Ds` trick lets us use stiff damping (`d=80`) without going unstable at `dt=1/60`.
3. **Symplectic Euler.** The velocity-then-position ordering is an energy-preserving integrator for oscillators — it doesn't artificially inflate swing amplitudes over long runs.

Meanwhile, contacts (when they occur) use **Projected Gauss-Seidel with warm start**, which is the industry-standard approach (used by Box2D, Bullet, PhysX) — it is fast, stable, and converges quickly for rigid-body stacks.

---

## 9. How Other Demos Differ

The pendulum is hand-built in code because it's simple. Most demos load `.gltf` files exported from Blender instead:

```
scene.gltf (KHR_physics_rigid_bodies + KHR_implicit_shapes)
   │
   ▼
loader/gltf_parser.cpp  —  3-pass parse
   ├─ label each glTF node (static? dynamic? joint-space marker?)
   ├─ build colliders (cuboid, sphere, cylinder, convex hull)
   └─ collect articulation links + BFS into trees
   │
   ▼
Scene { graph, art_forest, art_groups, rigidbody_group }
   │
   ▼
demos/common/demo_common.cpp
   ├─ create_rigidbody_from_node(...) for every free body
   └─ create_articulated_body(...) for every tree
   │
   ▼
world->add_body(...)  →  build_tree()  →  ready to step
```

From there, the main loop is identical to the pendulum. Only the *construction* changes.

---

## 10. TL;DR

- **Building**: rigid bodies + joints form an articulated tree. `build_tree()` figures out parent/child structure and what is a loop.
- **Physics per step**: RNEA (bias forces, O(n)) + CRBA (mass matrix, O(n²)) + Cholesky (solve, O(n³)) → `ddq`. Integrate semi-implicitly. Resolve contacts and loops with PGS + warm start.
- **Rendering**: physics updates world poses; renderer consumes them. Two GPU passes: shadow (depth-only from light) and color (Phong + PCF shadow from camera).
- **The pendulum** exercises everything except loop joints and contacts: you see the articulated-body solver in pure form. Joint damping + gravity → chain eventually hangs straight. Pressing SPACE injects a one-frame torque to relive the kick.

Read this alongside [DEEP_DIVE.md](DEEP_DIVE.md) for the citation-heavy version and [demos/demo_chain_pendulum.cpp](demos/demo_chain_pendulum.cpp) for the 200-line concrete implementation.
