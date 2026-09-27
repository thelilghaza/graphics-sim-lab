# Project 04: Procedural Destruction Sandbox

*Status: Planned (Phase 0 Documentation Complete — Milestone 1 Not Started)*

---

## Executive Summary

**Project 04 — Procedural Destruction Sandbox** is a C++20 physical simulation and geometry engine focused on structural fracture, rigid body dynamics, collision detection manifolds, impulse-based constraint solving, and structural connectivity graph stress evaluation.

Building upon the foundations established in previous projects—ray-geometry math (Project 01), volumetric spatial grids and meshing (Project 02), and hardware-aware cache/SIMD/concurrency performance engineering (Project 03)—Project 04 introduces time-evolving physical dynamics, dynamic Voronoi volume partitioning, dynamic contact manifold generation, and cascading structural collapse.

---

## Technical Objectives

1. **Procedural Geometry Fracturing**: Implement 2D/3D Voronoi partitioning and planar cell clipping to procedurally shatter convex polyhedral meshes into realistic fragment shards.
2. **Rigid Body Kinematics & Dynamics**: Compute position, orientation quaternions, linear/angular velocities, force/torque accumulators, and mass/inertia tensor properties for arbitrary polyhedral shards.
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
│       ├── math/               # Vectors, matrices, quaternions, inertia tensors
│       ├── dynamics/           # Rigid body state, integrators, mass properties
│       ├── fracture/           # Voronoi 2D/3D generators, planar mesh clippers
│       ├── collision/          # Broadphase AABB tree, narrowphase GJK/EPA contact manifolds
│       ├── solver/             # Sequential impulse solver, friction, position correction
│       ├── graph/              # Structural connectivity graph, stress propagation
│       └── render/             # Scene visualizer and interactive camera pipeline
├── src/                        # Subsystem implementations
├── tests/                      # Automated unit and invariant test suites
├── benchmarks/                 # Micro-benchmarks for fracture and physics performance
└── assets/                     # Demo scene configurations and mesh presets
```

---

## Planned Build Targets

- **`destruction_sandbox_lib`**: Core C++20 static library containing math, dynamics, fracture, collision, solver, and graph algorithms.
- **`sandbox_demo`**: Interactive 3D visualization and destruction simulation application.
- **`test_destruction_math`**: Unit tests for linear algebra, quaternions, and inertia tensors.
- **`test_destruction_dynamics`**: Unit tests for rigid body state integrators and momentum conservation.
- **`test_destruction_fracture`**: Unit tests for Voronoi site generation, planar clipping, and mesh volume preservation.
- **`test_destruction_collision`**: Unit tests for broadphase AABB overlap and narrowphase contact manifold extraction.
- **`test_destruction_solver`**: Unit tests for impulse response, stack resting stability, and friction.
- **`test_destruction_graph`**: Unit tests for structural connectivity graph load distribution and bond breaking.
- **`bench_destruction_fracture`**: Micro-benchmark measuring Voronoi partitioning execution scaling.
- **`bench_destruction_physics`**: Micro-benchmark measuring collision detection and impulse solver frame steps.

---

## Implementation Milestones

### Phase 0: Discovery, Scope & Architecture (COMPLETE)
- Authoritative roadmap review, technical domain scope definition, architectural design, directory structure, CMake targets, and risk identification documented.

### Milestone 1: Rigid Body Kinematics, Integrators & Math Foundation (PLANNED)
- Implement 3D vectors, matrices, quaternions, and mass/inertia tensor calculations for polyhedra.
- Implement numerical integrators (Semi-Implicit Euler, Verlet, RK4) and rigid body state management.
- Verify with analytical tests for momentum conservation and free-fall trajectories.

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
- **Graphics Pipeline**: OpenGL 3.3 / GLFW / glad (matching Project 02 established conventions) for real-time visualization, with headless PPM frame export support for automated continuous integration.

---

## Verification & Quality Discipline

- **Correctness First**: All algorithms verified with automated CTest suites before performance optimization.
- **Determinism**: Fixed random seeds for Voronoi site placement and deterministic sub-stepping delta time ($\Delta t = 1/60\text{ s}$).
- **Zero Energy Drift**: Physics integration validated against analytical energy conservation equations.
- **No Emojis**: Strict enforcement of clean professional documentation across all source files, headers, CLI logs, and reports.
