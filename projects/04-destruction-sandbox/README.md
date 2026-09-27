# Project 04: Procedural Destruction Sandbox

*Status: Milestone 1 Complete (Rigid Body Kinematics, Integrators & Math Foundation)*

---

## Executive Summary

**Project 04 — Procedural Destruction Sandbox** is a C++20 physical simulation and geometry engine focused on structural fracture, rigid body dynamics, collision detection manifolds, impulse-based constraint solving, and structural connectivity graph stress evaluation.

Building upon the foundations established in previous projects—ray-geometry math (Project 01), volumetric spatial grids and meshing (Project 02), and hardware-aware cache/SIMD/concurrency performance engineering (Project 03)—Project 04 introduces time-evolving physical dynamics, dynamic Voronoi volume partitioning, dynamic contact manifold generation, and cascading structural collapse.

---

## Technical Objectives

1. **Deterministic Math & Kinematics Foundation (Milestone 1)**: Provide robust 2D/3D vectors, 3x3 matrices, quaternions, rigid transforms, diagonal inertia tensors, rigid body state, force/torque accumulators, and Symplectic Euler integration.
2. **Procedural Geometry Fracturing**: Implement 2D/3D Voronoi partitioning and planar cell clipping to procedurally shatter convex polyhedral meshes into realistic fragment shards.
3. **Collision Detection & Manifold Generation**: Build a two-stage collision pipeline consisting of broadphase dynamic AABB tree spatial indexing and narrowphase GJK/EPA/SAT contact manifold extraction (contact points, normals, penetration depths).
4. **Impulse-Based Constraint Solver**: Develop a Sequential Impulse / Projected Gauss-Seidel solver resolving contact response, linear/angular friction, restitution, and Baumgarte position stabilization without energy gain or jitter.
5. **Structural Connectivity Graph**: Evaluate adhesive bonds and stress propagation across adjacent fragments to trigger progressive structural collapse when critical load thresholds are exceeded.
6. **Interactive Portfolio Demonstration**: Integrate subsystems into a real-time interactive 3D application demonstrating projectile impact, procedural mesh fracture, structural collapse, and debris stabilization.

---

## Progression from Projects 01–03

| Project | Core Domain | Key Capabilities Introduced | Relationship to Project 04 |
| :--- | :--- | :--- | :--- |
| **Project 01** | CPU Ray Tracer | Geometry intersections, BVH spatial partitioning, multithreading | Provides geometric ray/AABB math and spatial tree principles. |
| **Project 02** | Voxel Engine | Volumetric spatial grids, greedy meshing, async streaming queues | Provides dynamic geometry generation and chunk streaming concepts. |
| **Project 03** | Performance Lab | Micro-benchmarking, SIMD vectorization, cache locality, memory allocators | Provides high-performance SIMD math, lock-free queues, and arena allocation strategies. |
| **Project 04** | Destruction Sandbox | Physics dynamics, Voronoi fracturing, collision manifolds, impulse solvers, graph collapse | Combines geometry, spatial structures, and performance patterns to simulate dynamic physical destruction. |

---

## Mathematical & Physical Conventions (Milestone 1)

- **Coordinate System**: Right-handed 3D system. $+Y$ is World-Up, $+X$ is Right, $-Z$ is Forward. Units: meters ($\text{m}$), seconds ($\text{s}$), kilograms ($\text{kg}$), radians ($\text{rad}$).
- **Quaternion Convention**: $q = (w, x, y, z)$ with scalar real component $w$ and vector imaginary component $(x, y, z)$. Active column vector rotation via $v' = q \cdot v \cdot q^*$. Hamilton multiplication order.
- **Matrix Convention**: 3x3 row-major storage. Column vectors conceptually ($\mathbf{v}' = M \mathbf{v}$). Singular threshold $|\det(M)| < 10^{-7}$.
- **Rigid Transform**: $T = (t, q)$ where point transformation is $R(q) \cdot p + t$ and direction transformation is $R(q) \cdot d$.
- **Inertia Tensor**: Body-space diagonal inertia $I_{body} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ transformed to world-space via $I_{world}^{-1} = R I_{body}^{-1} R^T$.
- **Numerical Integrator**: Symplectic Euler (Semi-Implicit Euler):
  $$\mathbf{v}_{t+\Delta t} = \mathbf{v}_t + (m^{-1} \mathbf{F}_{accum}) \Delta t$$
  $$\mathbf{x}_{t+\Delta t} = \mathbf{x}_t + \mathbf{v}_{t+\Delta t} \Delta t$$
  $$\boldsymbol{\omega}_{t+\Delta t} = \boldsymbol{\omega}_t + (I_{world}^{-1} \boldsymbol{\tau}_{accum}) \Delta t$$
  $$q_{t+\Delta t} = \text{normalize}\left(q_t + \frac{1}{2} \omega_q q_t \Delta t\right)$$
- **Static Body Representation**: Infinite mass / static bodies set `mass = 0`, `inv_mass = 0`, `body_inv_inertia = (0,0,0)`, and `is_static = true`. Force and torque applications are ignored.

---

## Directory Structure

```text
projects/04-destruction-sandbox/
├── CMakeLists.txt              # Build configuration for library, tests, benchmarks, and demo
├── README.md                   # Project overview and roadmap tracking (this file)
├── docs/                       # Architectural documentation
│   ├── architecture.md         # Detailed technical architecture design
│   └── lessons.md              # Engineering design decisions and lessons learned
├── include/                    # Header files
│   └── destruction/
│       ├── math/               # Vec2, Vec3, Vec4, Mat3, Quat, Transform, MathUtils
│       ├── dynamics/           # RigidBody state, InertiaTensor, Integrator, PhysicsWorld
│       ├── fracture/           # Voronoi 2D/3D generators, planar mesh clippers (Planned M2)
│       ├── collision/          # Broadphase AABB tree, narrowphase GJK/EPA (Planned M3)
│       ├── solver/             # Sequential impulse solver, friction, PGS (Planned M4)
│       ├── graph/              # Structural connectivity graph, stress (Planned M4)
│       └── render/             # Scene visualizer and interactive camera (Planned M5)
├── src/                        # Subsystem implementations
├── tests/                      # Automated unit and invariant test suites
│   ├── test_dynamics_math.cpp  # Milestone 1 math and dynamics unit tests
│   └── val_dynamics_headless.cpp # Milestone 1 headless regression executable
├── benchmarks/                 # Micro-benchmarks for fracture and physics performance
└── assets/                     # Demo scene configurations and mesh presets
```

---

## Implementation Milestones

### Phase 0: Discovery, Scope & Architecture (COMPLETE)
- Authoritative roadmap review, technical domain scope definition, architectural design, directory structure, CMake targets, and risk identification documented.

### Milestone 1: Rigid Body Kinematics, Integrators & Math Foundation (COMPLETE)
- Implemented 2D/3D/4D vectors, 3x3 matrices, quaternions, rigid transforms, diagonal inertia tensors, and rigid body state containers.
- Implemented Symplectic Euler numerical integration and deterministic `PhysicsWorld` container with configurable gravity.
- Verified with unit test suite (`test_dynamics_math`) and headless regression executable (`val_dynamics_headless`).

### Milestone 2: Voronoi 2D/3D Partitioning & Dynamic Mesh Fracturing (PLANNED)
- Implement 2D/3D Voronoi site placement and planar bisector mesh clipping algorithms.
- Generate watertight, convex polyhedral fragment meshes with consistent material coordinates.
- Verify volume preservation, mass allocation, and manifold topological integrity.

### Milestone 3: Collision Detection & Contact Manifold Generation (PLANNED)
- Implement dynamic AABB tree broadphase spatial indexing for fast pair pruning.
- Implement narrowphase convex collision algorithms (GJK/EPA or SAT) generating contact manifolds (points, normal, depth).
- Verify contact manifold accuracy across primitive shapes (cubes, spheres, polyhedra).

### Milestone 4: Sequential Impulse Constraint Solver & Structural Graph (PLANNED)
- Implement Projected Gauss-Seidel / Sequential Impulse solver for contacts, friction, and Baumgarte position stabilization.
- Implement structural connectivity graph to track adhesive inter-fragment bonds and evaluate stress propagation.
- Verify resting stack stability without jitter/energy gain and progressive structural collapse under impact.

### Milestone 5: Integrated Sandbox Demo & Performance Benchmarking (PLANNED)
- Combine all subsystems into an interactive 3D destruction application.
- Add real-time projectile launch, wireframe/solid toggle, fracture site tuning, and camera controls.
- Export performance micro-benchmarks for fracture and physics frame time evaluation.

---

## Technical Dependencies

- **C++ Standard**: C++20 compliant compiler.
- **Build System**: CMake 3.20+ and Ninja.
- **Standard Library**: Standard containers, algorithms, atomic operations, timing routines.
- **Graphics Pipeline**: OpenGL 3.3 / GLFW / glad (matching Project 02 established conventions) for real-time visualization in Milestone 5, with headless PPM frame export support for automated continuous integration.

---

## Verification & Quality Discipline

- **Correctness First**: All algorithms verified with automated CTest suites (`test_dynamics_math`, `val_dynamics_headless`).
- **Determinism**: Fixed random seeds for Voronoi site placement and deterministic sub-stepping delta time ($\Delta t = 1/60\text{ s}$). State signature checksum validation (`0x40F6B6A4`).
- **Zero Energy Drift**: Physics integration validated against analytical energy and momentum conservation equations.
- **No Emojis**: Strict enforcement of clean professional documentation across all source files, headers, CLI logs, and reports.
