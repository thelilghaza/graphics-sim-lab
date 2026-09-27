# Project 04: Procedural Destruction Sandbox

*Status: Milestone 3 Complete (Collision Detection & Contact Manifold Generation)*

---

## Executive Summary

**Project 04 — Procedural Destruction Sandbox** is a C++20 physical simulation and geometry engine focused on structural fracture, rigid body dynamics, collision detection manifolds, impulse-based constraint solving, and structural connectivity graph stress evaluation.

Building upon the foundations established in previous projects—ray-geometry math (Project 01), volumetric spatial grids and meshing (Project 02), and hardware-aware cache/SIMD/concurrency performance engineering (Project 03)—Project 04 introduces time-evolving physical dynamics, dynamic Voronoi volume partitioning, dynamic contact manifold generation, and cascading structural collapse.

---

## Technical Objectives

1. **Deterministic Math & Kinematics Foundation (Milestone 1)**: Provide robust 2D/3D vectors, 3x3 matrices, quaternions, rigid transforms, diagonal inertia tensors, rigid body state, force/torque accumulators, and Symplectic Euler integration.
2. **Procedural Geometry Fracturing (Milestone 2)**: Implement 2D/3D Voronoi partitioning and planar cell clipping to procedurally shatter convex polyhedral meshes into realistic fragment shards with exact mass and volume conservation.
3. **Collision Detection & Contact Manifold Generation (Milestone 3)**: Build a two-stage collision pipeline consisting of broadphase dynamic AABB tree spatial indexing and narrowphase GJK/EPA/SAT contact manifold extraction (contact points, normals, penetration depths).
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
│       ├── solver/             # Sequential impulse solver, friction, PGS (Planned M4)
│       ├── graph/              # Structural connectivity graph, stress (Planned M4)
│       └── render/             # Scene visualizer and interactive camera (Planned M5)
├── src/                        # Subsystem implementations
├── tests/                      # Automated unit and invariant test suites
│   ├── test_dynamics_math.cpp  # Milestone 1 math and dynamics unit tests
│   ├── val_dynamics_headless.cpp # Milestone 1 headless regression executable
│   ├── test_fracture_geometry.cpp # Milestone 2 2D/3D Voronoi fracture unit tests
│   ├── demo_fracture.cpp       # Milestone 2 headless OBJ export demo
│   ├── test_collision.cpp      # Milestone 3 collision detection unit tests
│   └── demo_collision.cpp      # Milestone 3 headless collision demo
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

- **Correctness First**: All algorithms verified with automated CTest suites (`test_dynamics_math`, `val_dynamics_headless`, `test_fracture_geometry`, `demo_fracture`, `test_collision`, `demo_collision`).
- **Determinism**: Fixed random seeds for Voronoi site placement and deterministic sub-stepping delta time ($\Delta t = 1/60\text{ s}$). Collision checksum validation (`0xA2F70000`).
- **Zero Energy Drift**: Physics integration validated against analytical energy and momentum conservation equations.
- **No Emojis**: Strict enforcement of clean professional documentation across all source files, headers, CLI logs, and reports.
