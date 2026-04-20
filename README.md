# TorqueForge — Interactive Rotational Dynamics Visualization

**Computer Graphics — CSL7450 Course Project**

An articulated body physics engine for interactive rotational mechanics demos, built on the Featherstone Articulated Body Algorithm.

## Background

During our JEE prep, rotational mechanics was honestly one of the toughest topics to crack. Torque, angular momentum, moment of inertia tensors, coupled rotations — understanding these from flat textbook diagrams felt nearly impossible. We always wished we could just *see* these systems move in real time.

## Motivation

This challenge motivated us to explore computational approaches to rigid body dynamics visualization using the Featherstone Articulated Body Algorithm — an O(n) method widely adopted in robotics for simulating articulated multi-body systems.

## Objective

We present real-time, interactive 3D simulations that make rotational dynamics and multi-body constraint propagation directly observable and intuitive.

### We will try to build:
- Interactive demos of coupled mechanical systems (gear trains, linkages, suspensions)
- Real-time visualization of torque propagation and angular momentum transfer across joints
- Constraint force resolution in closed-loop kinematic chains

## Features

### Featherstone-based forward dynamics
Implements RNEA (Recursive Newton–Euler Algorithm) and CRBA (Composite Rigid Body Algorithm), as described in [Rigid Body Dynamics Algorithms](https://link.springer.com/book/10.1007/978-1-4899-7560-7) by Featherstone, at the core of the solver.

### Closed-loop constraint support
Supports closed-loop systems using simple joints as loop-closure constraints. Loop closure is enforced at the acceleration level, with Baumgarte-stabilized velocity and position terms.

### Joint and DoF coupling
Supports coupling across joints and degrees of freedom by modeling coupling as linear constraints between DoFs.

### Impulse-based contact resolution
Solves contacts using Projected Gauss–Seidel (PGS) with Sequential Impulses (SI) and impulse warm starting.

### Springs
Solves spring forces using a semi-implicit Euler integration scheme for numerical stability.

### glTF-based physics asset pipeline
Supports construction of a physics world from glTF physics assets exported from Blender.

## Supported Joints

| Joint Type | Dynamics Model | Loop Closure | Spring |
|---|---|---|---|
| Revolute | Motion subspace | ✅ | ✅ |
| Prismatic | Motion subspace | ✅ | ✅ |
| Cylindrical | Motion subspace | ✅ | ✅ |
| Spherical | Motion subspace | ✅ | ❌ |
| Gear | Coupling 2 revolutes | ❌ | ✅ |
| Rack and Pinion | Coupling revolute + prismatic | ❌ | ✅ |
| Screw | Coupling 2 DoFs of cylindrical | ❌ | ✅ |
| Worm | Coupling cylindrical + revolute | ❌ | ✅ |

## Demos

| Name | Articulated | Loop Closure | Spring | Coupling | Spherical Joint | PID Control |
|---|---|---|---|---|---|---|
| Scissor Lift | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Worm Screw Jack | ✅ | ✅ | ❌ | ✅ | ❌ | ✅ |
| Spring Scale | ✅ | ❌ | ✅ | ✅ | ❌ | ❌ |
| Double Wishbone Suspension | ✅ | ✅ | ✅ | ❌ | ✅ | ✅ |
| Collision Test | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| Spring Test | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ |
| Spherical Joint Test | ✅ | ✅ | ❌ | ❌ | ✅ | ❌ |

## Dependencies

- [Eigen v3.4.0](https://libeigen.gitlab.io/) — Core dynamics
- [GLM v1.0.1](https://github.com/g-truc/glm/tree/1.0.1) — Rendering math
- [raylib v5.5](https://github.com/raysan5/raylib/tree/5.5) — 3D rendering
- [BulletCollision v3.25](https://github.com/bulletphysics/bullet3/tree/3.25) — Collision detection only
- [nlohmann/json v3.12.0](https://github.com/nlohmann/json/tree/v3.12.0)
- [tinygltf v2.9.5](https://github.com/syoyo/tinygltf/tree/v2.9.5)

## Build

### Prerequisites

| | Windows | Linux (Ubuntu/Debian) |
|---|---|---|
| **Compiler** | Visual Studio 2022 | GCC 13+ or Clang |
| **CMake** | 3.20+ | `sudo apt install cmake` |
| **Eigen** | Download headers | `sudo apt install libeigen3-dev` |
| **GLM** | Download headers | `sudo apt install libglm-dev` |
| **X11/GL libs** | N/A | `sudo apt install libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev` |

### Build raylib from source (both platforms)

```bash
git clone --branch 5.5 --depth 1 https://github.com/raysan5/raylib.git /tmp/raylib-src
cd /tmp/raylib-src && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<project>/third_party/raylib -DBUILD_SHARED_LIBS=OFF ..
cmake --build . --parallel
cmake --install .
```

### Build Bullet from source (both platforms)

```bash
git clone --branch 3.25 --depth 1 https://github.com/bulletphysics/bullet3.git /tmp/bullet-src
cd /tmp/bullet-src && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<project>/third_party/bullet \
      -DBUILD_SHARED_LIBS=OFF -DUSE_DOUBLE_PRECISION=OFF \
      -DBUILD_BULLET2_DEMOS=OFF -DBUILD_CPU_DEMOS=OFF \
      -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF \
      -DBUILD_OPENGL3_DEMOS=OFF -DINSTALL_LIBS=ON ..
cmake --build . --parallel
cmake --install .
```

> Replace `<project>` with the absolute path to this project's root directory.

### Windows (Visual Studio 2022)

```bash
mkdir build && cd build
cmake -DEIGEN3_INCLUDE_DIR=<path-to-eigen> -DGLM_INCLUDE_DIR=<path-to-glm> .. -G "Visual Studio 17 2022"
cmake --build . --config Release
```

### Linux

```bash
mkdir build && cd build
cmake -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 -DGLM_INCLUDE_DIR=/usr/include ..
cmake --build . --parallel
```

## Run

### Combined run — all demos in sequence (recommended)

Launches every demo one after another, in fullscreen. Close the window or press `ESC` to advance to the next one.

**Linux / macOS**
```bash
./run_presentation.sh                # build (Release) then run all demos
./run_presentation.sh --no-rebuild   # skip build, just run
```

**Windows (PowerShell)**
```powershell
.\run_presentation.ps1               # build (Release) then run all demos
.\run_presentation.ps1 -NoRebuild    # skip build, just run
```

Both scripts walk through the same sequence:

| # | Demo | Highlights |
|---|---|---|
| 1 | Wrecking Ball | Free rigid bodies, impulse-based contact |
| 2 | Scissorlift | Closed kinematic chain / loop closure |
| 3 | Spring | Prismatic joint + spring-damper |
| 4 | Spherical Joint Chain | 3-DOF joints |
| 5 | Spring Scale | Torque balance |
| 6 | Chain Pendulum (N=30) | Featherstone scaling benchmark |

The Linux script exports `__NV_PRIME_RENDER_OFFLOAD=1` / `__GLX_VENDOR_LIBRARY_NAME=nvidia` automatically, so hybrid-GPU laptops use the dedicated card without extra flags.

### Run an individual demo

```bash
# List available demos
ls build/bin/demos/

# Run a single demo (from project root — renderer config JSONs resolve from CWD)
./build/bin/demos/demo_load_scene

# Scene-selecting demos read a digit from stdin
echo 0 | ./build/bin/demos/demo_load_scene_articulated   # scissorlift
echo 1 | ./build/bin/demos/demo_load_scene_articulated   # spring
echo 2 | ./build/bin/demos/demo_load_scene_articulated   # spherical joint chain

# Chain pendulum takes N as argv (default 30)
./build/bin/demos/demo_chain_pendulum 30

# Linux with NVIDIA dedicated GPU (hybrid laptops)
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ./build/bin/demos/demo_load_scene
```

### Controls
- **C** — Toggle free-roam camera
- **F** — Take screenshot
- **G** — Toggle collider/renderable visibility
- **ESC** — Exit

## Tech Stack

C++17 · Featherstone ABA · Eigen · Bullet Collision · raylib
