# Project 04: Procedural Destruction Sandbox

*Status: Milestone 5 Complete (Integrated Interactive Destruction Sandbox Demo & Performance Benchmarking)*

---

## Executive Summary

**Project 04 — Procedural Destruction Sandbox** is a C++20 physical simulation and geometry engine focused on structural fracture, rigid body dynamics, collision detection manifolds, impulse-based constraint solving, and structural connectivity graph stress evaluation.

Building upon the foundations established in previous projects—ray-geometry math (Project 01), volumetric spatial grids and meshing (Project 02), and hardware-aware cache/SIMD/concurrency performance engineering (Project 03)—Project 04 introduces time-evolving physical dynamics, dynamic Voronoi volume partitioning, dynamic contact manifold generation, sequential impulse constraint solving, and cascading structural collapse.

---

## Technical Objectives

1. **Deterministic Math & Kinematics Foundation (Milestone 1)**: Provide robust 2D/3D vectors, 3x3 matrices, quaternions, rigid transforms, diagonal inertia tensors, rigid body state, force/torque accumulators, and Symplectic Euler integration.
2. **Procedural Geometry Fracturing (Milestone 2)**: Implement 2D/3D Voronoi partitioning and planar cell clipping to procedurally shatter convex polyhedral meshes into realistic fragment shards with exact mass and volume conservation.
3. **Collision Detection & Contact Manifold Generation (Milestone 3)**: Build a two-stage collision pipeline consisting of broadphase dynamic AABB tree spatial indexing and narrowphase GJK/EPA/SAT contact manifold extraction (contact points, normals, penetration depths).
4. **Impulse-Based Constraint Solver & Structural Graph (Milestone 4)**: Develop a Sequential Impulse / Projected Gauss-Seidel solver resolving contact response, Coulomb friction, restitution, and split impulse position stabilization, combined with a structural connectivity graph evaluating inter-fragment support loads and failure disconnections.
5. **Interactive Portfolio Demonstration (Milestone 5)**: Integrate subsystems into a real-time interactive 3D application demonstrating projectile impact, procedural mesh fracture, structural collapse, and debris stabilization.

---

## Progression from Projects 01–03

| Project | Core Domain | Key Capabilities Introduced | Relationship to Project 04 |
| :--- | :--- | :--- | :--- |
| **Project 01** | CPU Ray Tracer | Geometry intersections, BVH spatial partitioning, multithreading | Provides geometric ray/AABB math and spatial tree principles. |
| **Project 02** | Voxel Engine | Volumetric spatial grids, greedy meshing, async streaming queues | Provides dynamic geometry generation and chunk streaming concepts. |
| **Project 03** | Performance Lab | Micro-benchmarking, SIMD vectorization, cache locality, memory allocators | Provides high-performance SIMD math, lock-free queues, and arena allocation strategies. |
| **Project 04** | Destruction Sandbox | Physics dynamics, Voronoi fracturing, collision manifolds, impulse solvers, graph collapse | Combines geometry, spatial structures, and performance patterns to simulate dynamic physical destruction. |

---

## Mathematical & Physical Conventions

- **Coordinate System**: Right-handed 3D system. $+Y$ is World-Up, $+X$ is Right, $-Z$ is Forward. Units: meters ($\text{m}$), seconds ($\text{s}$), kilograms ($\text{kg}$), radians ($\text{rad}$).
- **Quaternion Convention**: $q = (w, x, y, z)$ with scalar real component $w$ and vector imaginary component $(x, y, z)$. Active column vector rotation via $v' = q \cdot v \cdot q^*$. Hamilton multiplication order.
- **Matrix Convention**: 3x3 row-major storage. Column vectors conceptually ($\mathbf{v}' = M \mathbf{v}$). Singular threshold $|\det(M)| < 10^{-7}$.
- **Rigid Transform**: $T = (t, q)$ where point transformation is $R(q) \cdot p + t$ and direction transformation is $R(q) \cdot d$.
- **Voronoi Bisector Half-Space**: Pairwise bisector plane between site $\mathbf{p}_i$ and competing site $\mathbf{p}_j$:
  $$\mathbf{n} = \frac{\mathbf{p}_j - \mathbf{p}_i}{\|\mathbf{p}_j - \mathbf{p}_i\|}, \quad \mathbf{m} = \frac{1}{2}(\mathbf{p}_i + \mathbf{p}_j), \quad d = -\mathbf{n} \cdot \mathbf{m}$$
  Points $\mathbf{x}$ inside cell $i$ satisfy $\mathbf{n} \cdot \mathbf{x} + d \le 0$. Opposite cell $j$ uses opposite plane $-\mathbf{n}$ and $-d$.
- **Collision Pipeline**:
  - **Broadphase**: `DynamicAabbTree` surface-area cost binary tree emitting canonical overlapping candidate pairs $(A, B)$.
  - **Narrowphase GJK**: Simplex-based convex intersection test determining intersection status.
  - **Narrowphase EPA**: Expanding Polytope Algorithm determining exact penetration depth $d$ and normal $\mathbf{n}$ (pointing from A to B).
  - **Narrowphase SAT**: Independent Separating Axis Theorem cross-validation path.
  - **Contact Manifolds**: 4-point reduced manifolds maximizing contact polygon area coverage.
- **Sequential Impulse Solver**:
  - **Normal Impulses**: Effective normal mass $K_n = m_A^{-1} + m_B^{-1} + \mathbf{n} \cdot ((\mathbf{I}_A^{-1} (\mathbf{r}_A \times \mathbf{n})) \times \mathbf{r}_A + (\mathbf{I}_B^{-1} (\mathbf{r}_B \times \mathbf{n})) \times \mathbf{r}_B)$, clamped to $J_n \ge 0$.
  - **Friction Impulses**: Coulomb friction clamped to $\|\mathbf{J}_t\| \le \mu J_n$ using stable orthogonal tangent basis $(\mathbf{t}_1, \mathbf{t}_2)$.
  - **Position Stabilization**: Split impulses solving position bias $(\beta / \Delta t) \max(0, d - \text{slop})$ onto separate pseudo-velocities without inflating physical kinetic energy.
  - **Warm Starting**: Stable 64-bit contact key caching accumulated normal/tangent impulses across step frames with age pruning.
- **Structural Connectivity Graph**:
  - **Support Edges**: Evaluated from contact manifolds where reaction normal has upward component $-\mathbf{n} \cdot \hat{\mathbf{y}} > 0.3$.
  - **Load Propagation**: Deterministic iterative sweep distributing gravitational and transmitted weights across active edges.
  - **Structural Failure**: Overloaded edges exceeding capacity ($A \cdot \sigma_{\text{tensile}} \cdot k_{\text{mult}}$) break, and graph connectivity sweep recomputes supported/unsupported fragment sets from anchored roots.

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
│       ├── fracture/           # Plane, Polygon2D, Voronoi2D, Polyhedron, Clipper3D, Volume, Sites, Shard, Validator, Voronoi3D, ObjExporter
│       ├── collision/          # Aabb, Collider, DynamicAabbTree, Support, Gjk, Epa, Sat, ContactManifold, Narrowphase
│       ├── solver/             # SequentialImpulseSolver, ContactConstraint, WarmStartCache, SolverSettings
│       ├── graph/              # StructuralGraph, SupportEdge, StructuralNode, MaterialParams
│       └── render/             # Scene visualizer and interactive camera (Planned M5)
├── src/                        # Subsystem implementations
├── tests/                      # Automated unit and invariant test suites
│   ├── test_dynamics_math.cpp  # Milestone 1 math and dynamics unit tests
│   ├── val_dynamics_headless.cpp # Milestone 1 headless regression executable
│   ├── test_fracture_geometry.cpp # Milestone 2 2D/3D Voronoi fracture unit tests
│   ├── demo_fracture.cpp       # Milestone 2 headless OBJ export demo
│   ├── test_collision.cpp      # Milestone 3 collision detection unit tests
│   ├── demo_collision.cpp      # Milestone 3 headless collision demo
│   ├── test_solver_graph.cpp   # Milestone 4 solver and structural graph unit tests
│   └── demo_physics.cpp        # Milestone 4 headless physics simulation demo
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

### Milestone 2: Voronoi 2D/3D Partitioning & Dynamic Mesh Fracturing (COMPLETE)
- Implemented 2D/3D Voronoi site placement (`SiteGenerator`), 2D polygon clipping (`Polygon2D`), 3D convex polyhedron half-space clipping (`Clipper3D`), cap-face generation, and vertex welding.
- Implemented exact tetrahedral decomposition for convex polyhedral volume, mass, and centroid calculations.
- Implemented topological closed-manifold edge validator (`MeshValidator`), Wavefront OBJ exporter (`ObjExporter`), and headless fracture demo (`demo_fracture`).
- Verified volume and mass conservation ($0.0000\%$ relative error) across symmetric, multi-site, and translated source boxes (`test_fracture_geometry`).

### Milestone 3: Collision Detection & Contact Manifold Generation (COMPLETE)
- Implemented 3D AABBs, Box and ConvexPolyhedron colliders (`Collider`), and dynamic AABB tree broadphase (`DynamicAabbTree`).
- Implemented Minkowski difference support mappings (`support_minkowski`), GJK convex intersection engine (`Gjk`), EPA penetration depth & normal engine (`Epa`), and SAT cross-validation engine (`Sat`).
- Implemented 4-point reduced contact manifold generation (`ContactManifold`, `Narrowphase`) and headless collision demo (`demo_collision`).
- Verified zero false negatives, GJK vs SAT agreement, closed edge-manifold shard collisions, and 60-collider broadphase stress testing (`test_collision`).

### Milestone 4: Sequential Impulse Constraint Solver & Structural Graph (COMPLETE)
- Implemented Projected Gauss-Seidel / Sequential Impulse solver (`SequentialImpulseSolver`) resolving normal impulses, Coulomb friction, restitution, and split impulse position stabilization (`ContactConstraint`).
- Implemented deterministic 64-bit contact key warm starting with age pruning (`WarmStartCache`).
- Implemented structural connectivity graph (`StructuralGraph`) evaluating support edge eligibility, iterative load propagation, edge capacity failure, and structural connectivity sweeps.
- Verified analytical head-on elastic momentum conservation, off-center torque response, friction sliding bounds, resting stack stability, position split stabilization without kinetic energy bleed, and load capacity failure (`test_solver_graph`, `demo_physics`).

### Milestone 5: Integrated Sandbox Demo & Performance Benchmarking (COMPLETE)
- Implemented lightweight RAII OpenGL 3.3 Core Profile rendering pipeline (`destruction::render`), free-look camera (`Camera`), shader compilation (`Shader`), dynamic vertex/index buffers (`GlMesh`), and Blinn-Phong renderer (`Renderer`).
- Implemented real-time interactive destruction sandbox application (`destruction_sandbox`) featuring fixed-timestep physics accumulator ($1/60\text{ s}$), deterministic projectile launcher, Voronoi fracture trigger, cascading structural collapse, real-time diagnostic HUD/telemetry, and clean deterministic reset.
- Implemented comprehensive debug overlays: world AABBs, contact points, outward contact normals, structural support edges, broken support edges, wireframe colliders, and centers of mass.
- Implemented formal performance micro-benchmark suite (`bench_destruction`) covering Voronoi fracture scaling, dynamic AABB tree broadphase, narrowphase GJK/EPA, sequential impulse solver, structural graph load sweep, integrated physics step, and full fracture-to-collapse pipeline, exporting canonical CSV report (`destruction_release.csv`).
- Implemented focused integration tests (`test_sandbox_integration`) and automated 120-step headless simulation validation executable (`val_destruction_headless`) verifying finite state, zero NaNs, and deterministic checksum (`0xC8637A22`).

---

## Interactive Demo Controls

The `destruction_sandbox` application provides an interactive 3D demonstration:

| Key / Input | Action | Description |
| :--- | :--- | :--- |
| **W, A, S, D** | Move Camera | Free camera translation forward, left, backward, right |
| **Q, E** | Elevate Camera | Free camera translation down (Q) and up (E) |
| **Shift** | Boost Speed | 2.5x camera movement speed multiplier |
| **Right Mouse (Hold)** | Look Around | Free-look camera yaw and pitch rotation |
| **Space** | Launch Projectile | Fires high-velocity sphere projectile ($28\text{ m/s}$) along camera forward ray |
| **F** | Trigger Fracture | Manually fractures the central target tower block into Voronoi shards |
| **P** | Pause / Resume | Toggles physics simulation pause state |
| **O** | Single Step | Advances simulation by exactly one fixed physics step ($1/60\text{ s}$) while paused |
| **R** | Reset Scene | Restores scene, rigid bodies, colliders, and structural graph to initial deterministic state |
| **C** | Reset Camera | Restores camera position and orientation to default viewing angle |
| **1** | Toggle Wireframe | Toggles mesh polygon wireframe rendering mode |
| **2** | Toggle AABBs | Toggles world-space axis-aligned bounding box debug outlines |
| **3** | Toggle Contacts | Toggles contact points and outward-pointing contact normals |
| **4** | Toggle Support Graph | Toggles green (active) and red (broken) structural support graph edges |
| **H** | Toggle Help Overlay | Displays in-app keyboard and mouse navigation controls in console |
| **Esc** | Exit | Closes application cleanly and releases all OpenGL resources |

CLI options for `destruction_sandbox`:
- `--headless`: Runs simulation without creating an OpenGL window.
- `--timeout <steps>`: Automatically exits after simulating the specified number of physics steps.

---

## Benchmark Summary

Canonical Release benchmark results from `projects/04-destruction-sandbox/benchmarks/reports/destruction_release.csv`:

| Benchmark | Workload | Mean ($\mu\text{s}$) | Median ($\mu\text{s}$) | Min ($\mu\text{s}$) | Max ($\mu\text{s}$) | Throughput |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `voronoi_fracture` | 4 sites | 1012.52 | 941.30 | 906.70 | 1371.20 | 3,951 ops/s |
| `voronoi_fracture` | 8 sites | 4982.43 | 4956.05 | 4637.20 | 5488.50 | 1,606 ops/s |
| `voronoi_fracture` | 16 sites | 21687.97 | 21734.65 | 21132.30 | 22291.40 | 738 ops/s |
| `voronoi_fracture` | 32 sites | 109581.19 | 105538.85 | 101047.80 | 133387.10 | 292 ops/s |
| `broadphase_tree` | 16 colliders | 62.08 | 55.60 | 55.30 | 98.80 | 257,749 ops/s |
| `broadphase_tree` | 64 colliders | 278.22 | 260.05 | 258.10 | 508.80 | 230,032 ops/s |
| `broadphase_tree` | 128 colliders | 652.84 | 618.30 | 612.40 | 924.90 | 196,066 ops/s |
| `broadphase_tree` | 256 colliders | 1951.33 | 1778.30 | 1670.60 | 3233.30 | 131,193 ops/s |
| `narrowphase_gjk_epa` | 100 pairs | 2954.90 | 2875.70 | 2561.90 | 4396.20 | 33,842 ops/s |
| `impulse_solver` | 10-body stack | 1580.86 | 1519.60 | 1291.70 | 2272.00 | 6,326 ops/s |
| `structural_graph` | 20-node sweep | 36.81 | 36.60 | 36.40 | 45.80 | 543,272 ops/s |
| `integrated_physics` | 16-block step | 2354.48 | 2332.55 | 469.90 | 4071.10 | 6,796 ops/s |
| `fracture_to_collapse` | Shards & solve | 5168.94 | 5213.10 | 4847.90 | 5388.70 | 193 ops/s |

---

## Technical Dependencies

- **C++ Standard**: C++20 compliant compiler.
- **Build System**: CMake 3.20+ and Ninja.
- **Standard Library**: Standard containers, algorithms, atomic operations, timing routines.
- **Graphics Pipeline**: OpenGL 3.3 Core Profile / GLFW 3.4 / lightweight function loader (`gl_loader`) with zero external engine dependencies. Headless mode supported for automated validation and CI.

---

## Verification & Quality Discipline

- **Correctness First**: All algorithms verified with automated CTest suites: M1 math (`test_dynamics_math`, `val_dynamics_headless`), M2 fracture (`test_fracture_geometry`, `demo_fracture`), M3 collision (`test_collision`, `demo_collision`), M4 solver (`test_solver_graph`, `demo_physics`), and M5 integration (`test_sandbox_integration`, `val_destruction_headless`).
- **Determinism**: Fixed random seeds for Voronoi site placement and deterministic sub-stepping delta time ($\Delta t = 1/60\text{ s}$). Collision checksum validation (`0xA2F70000`) and M5 headless simulation checksum validation (`0xC8637A22`).
- **Zero Energy Drift**: Physics integration validated against analytical energy and momentum conservation equations. Split impulses eliminate position drift without kinetic energy inflation.
- **No Emojis**: Strict enforcement of clean professional documentation across all source files, headers, CLI logs, and reports.
