# Graphics & Simulation Lab

Welcome to the **Graphics & Simulation Lab**, a personal technical laboratory and systems engineering portfolio dedicated to first-principles computer graphics, volumetric voxel rendering, high-performance C++ micro-benchmarking, SIMD vectorization, lock-free queue concurrency, and memory allocator design.

---

## Vision & Core Philosophy

The primary objective of this repository is to build deep, first-principles systems engineering projects with rigorous code quality, empirical performance measurements, deterministic correctness verification, and architectural clarity.

### Core Architectural Principle
> **DO NOT build a giant shared engine/framework upfront.**

Each project in this repository is designed to be **independently buildable and self-contained**. Shared libraries (`libs/`) are extracted only after there is proven, multi-project architectural reuse.

---

## Project Index

| Project | Name | Status | Primary Focus |
| ------- | ---- | ------ | ------------- |
| **01** | [CPU Ray Tracer](projects/01-raytracer/) | Complete (`project-01-complete`) | First-principles ray tracing, BVH acceleration, materials, multithreaded rendering. |
| **02** | [Voxel Engine](projects/02-voxel-engine/) | Complete (`project-02-complete`) | Volumetric 3D spatial grids, naive/greedy meshing, dynamic chunk streaming, LOD, OpenGL viewer. |
| **03** | [Performance Lab](projects/03-performance-lab/) | Complete (`project-03-complete`) | Hardware-aware benchmarking, cache locality, SIMD vectorization, queue concurrency, memory allocators. |
| **04** | [Procedural Destruction Sandbox](projects/04-destruction-sandbox/) | Complete (`project-04-complete`) | Voronoi 2D/3D partitioning, rigid body dynamics, collision manifolds, impulse solvers, structural graph, interactive sandbox. |

---

## Completed Projects Overview

### Project 01 — CPU Ray Tracer
[Project 01 — CPU Ray Tracer Documentation](projects/01-raytracer/)

A deterministic, multithreaded CPU ray tracing engine built from first principles in C++20:
- **Ray-Geometry Intersections**: Sphere, AABB, and triangle ray intersection testing.
- **Surface & Material Shading**: Lambertian diffuse, metallic specular reflection, dielectric refraction with Snell's law and Schlick approximation, and positional light sources with shadow rays.
- **Acceleration & Parallelism**: Bounding Volume Hierarchy (BVH) spatial partitioning and multi-threaded tile rendering.
- **Camera & Artifacts**: Thin-lens depth of field, configurable aperture/focus distance, and anti-aliased image output.

### Project 02 — Voxel Engine
[Project 02 — Voxel Engine Documentation](projects/02-voxel-engine/)

A high-performance volumetric voxel engine handling large-scale terrain streaming and meshing:
- **World Architecture**: Contiguous $32^3$ chunk storage using compact 2-byte voxel payloads, supporting positive and negative world-space coordinates via deterministic noise generators.
- **Meshing Pipelines**: Naive face culling and surface-equivalent Greedy Meshing algorithms reducing vertex overhead by over 70%.
- **Multithreaded Streaming & Buffer Reuse**: Asynchronous worker thread pool for chunk generation and meshing with neighborhood snapshot isolation, stale-result rejection, and dynamic mesh buffer recycling.
- **Level of Detail (LOD) & Interactive Viewer**: 4-level LOD distance hierarchy, interactive OpenGL 3.3 viewer with dynamic camera controller, and real-time chunk streaming.

### Project 03 — Performance Lab
[Project 03 — Performance Lab Documentation](projects/03-performance-lab/) | [Visual Benchmark Dashboard](projects/03-performance-lab/benchmarks/report.html)

A hardware-aware micro-benchmarking laboratory for low-level CPU performance engineering:
1. **Milestone 1 — Benchmark Harness (`bench_harness`)**: High-precision monotonic timing (`std::chrono::steady_clock`), volatile optimization barriers (`do_not_optimize`), and automated CSV export infrastructure (`BenchRunner`).
2. **Milestone 2 — Cache Locality (`bench_cache_locality`)**: Comparative analysis of Array of Structures (AoS), Structure of Arrays (SoA), and Tiled AoSoA data layouts, strided access degradation, and working-set memory scaling (4 KiB to 64 MiB).
3. **Milestone 3 — SIMD Vectorization (`bench_simd_vectorization`)**: Intrinsic acceleration evaluating scalar reference, compiler auto-vectorization (`/O2`), SSE, and AVX2 across vector dot products, AXPY linear transforms, and batch 4-wide/8-wide ray-AABB geometry kernels.
4. **Milestone 4 — Concurrency & Synchronization Queues (`bench_lockfree_queues`)**: Empirical contention study comparing `MutexBoundedQueue`, lock-free `SpscQueue`, and sequence-number `MpmcBoundedQueue` across thread topology scaling (1P1C to 8P8C) and capacity variations.
5. **Milestone 5 — Memory Allocators (`bench_allocator_churn`)**: Allocation churn evaluation comparing `std::malloc`/`free`, `std::allocator`, preallocated `FixedBlockPool` (intrusive free-list), and `LinearArena` (monotonic bump allocation with bulk $O(1)$ reset) across fixed/variable churn, frame temporary memory, pool reuse, and capacity scaling.
- **Interactive Visual Dashboard**: Self-contained offline visual dashboard ([report.html](projects/03-performance-lab/benchmarks/report.html)) rendering embedded SVG bar charts and speedup metrics derived from 10 canonical CSV reports.

### Project 04 — Procedural Destruction Sandbox
[Project 04 — Procedural Destruction Sandbox Documentation](projects/04-destruction-sandbox/)

A C++20 physical simulation and geometry engine focused on structural fracture, rigid body dynamics, collision manifolds, impulse constraint solving, structural graph stress evaluation, and real-time interactive destruction:
1. **Milestone 1 — Math & Kinematics**: Symplectic Euler integration, quaternions, rigid transforms, and diagonal inertia tensors.
2. **Milestone 2 — Voronoi Fracturing**: 2D/3D Voronoi partitioning, planar half-space polyhedral clipping, cap-face construction, polar sorting, and exact volume/mass conservation ($0.0000\%$ error).
3. **Milestone 3 — Collision Detection**: Broadphase Dynamic AABB tree, narrowphase Minkowski support mapping, GJK convex intersection, EPA penetration depth/normal, SAT validation, and 4-point reduced contact manifolds.
4. **Milestone 4 — Impulse Solver & Structural Graph**: Sequential Impulse / Projected Gauss-Seidel solver with Coulomb friction, restitution, split impulse position stabilization, 64-bit warm starting, and structural connectivity graph evaluating support edge capacity failure.
5. **Milestone 5 — Interactive Sandbox & Benchmarking**: OpenGL 3.3 Core Profile renderer, interactive 3D sandbox (`destruction_sandbox`), debug overlays, canonical micro-benchmarking (`bench_destruction`), 120-step headless validation (`val_destruction_headless`), and contact stability verification (`test_contact_stability`).

---

## Current Repository Status

- **Projects Completed**: Project 01 (CPU Ray Tracer), Project 02 (Voxel Engine), Project 03 (Performance Lab), and Project 04 (Procedural Destruction Sandbox).
- **Completion Tags**: `project-01-complete`, `project-02-complete`, `project-03-complete`, `project-04-complete`.
- **Next Planned Project**: Project 05 — GPU Crater Simulator (not yet started).
- **Remote Push**: Local repository checkout only.

---

## Core Engineering Principles

1. **Deterministic Experiments**: Workload inputs, ray packets, terrain seeds, queue payloads, and allocation sequences are strictly deterministic to guarantee reproducible benchmark results.
2. **Empirical Benchmarking**: Performance conclusions are drawn strictly from empirical runtime timing measurements, avoiding unverified theoretical claims or compiler assumptions.
3. **Correctness Before Optimization**: All algorithms pass rigorous functional unit tests and mathematical output verification before timing data is recorded.
4. **Distinction of Measured Facts vs Interpretation**: Clear boundary between measured raw metrics (e.g., ops/sec, execution time, MB/s) and architectural interpretation.
5. **Independent Project Boundaries**: Each project maintains its own isolated build configuration, test suites, and documentation without premature coupling.

---

## Build & Quick Start

### Prerequisites
- **C++ Compiler**: Modern C++20 compliant compiler (MSVC 2022+, GCC 11+, or Clang 13+)
- **Build System**: [CMake](https://cmake.org/) (v3.20+) and [Ninja](https://ninja-build.org/)

### Building via CMake Presets

```bash
# Configure the default debug preset
cmake --preset default

# Build all available targets
cmake --build --preset default

# Run complete test suite via CTest (47 test targets)
ctest --preset default --output-on-failure
```

### Manual Configuration & Execution

```bash
# Configure out-of-tree Release build
cmake -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release

# Compile all targets
cmake --build build/release

# Run full CTest suite
ctest --test-dir build/release --output-on-failure

# Execute Project 04 Interactive Sandbox
./build/release/projects/04-destruction-sandbox/destruction_sandbox.exe
```

---

## Repository Structure

```text
graphics-sim-lab/
├── CMakeLists.txt              # Root build configuration
├── CMakePresets.json           # Standardized build presets
├── README.md                   # Repository overview (this file)
├── LICENSE                     # MIT License
├── docs/                       # Technical documentation & principles
│   ├── roadmap.md              # Detailed project roadmap
│   ├── architecture.md         # Repository architectural rules
│   └── engineering-principles.md # Core engineering principles
├── projects/                   # Independent engineering projects
│   ├── 01-raytracer/           # Project 01 — CPU Ray Tracer (Complete)
│   ├── 02-voxel-engine/        # Project 02 — Voxel Engine (Complete)
│   ├── 03-performance-lab/     # Project 03 — Performance Lab (Complete)
│   └── 04-destruction-sandbox/ # Project 04 — Procedural Destruction (Complete)
├── libs/                       # Shared components (extracted only on proven reuse)
└── tools/                      # Benchmark & build scripts
```

---

## License

This repository is licensed under the [MIT License](LICENSE).
