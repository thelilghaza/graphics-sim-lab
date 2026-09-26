# Project 01: CPU Ray Tracer

A software-based CPU ray tracer built from first principles in modern C++20.

---

## Current Status: Project 01 Complete (Milestones 1–5)

Project 01 is fully complete through Milestone 5. It features Axis-Aligned Bounding Boxes (AABB), Bounding Volume Hierarchy (BVH) spatial partitioning, thread-local parallel rendering with zero hot-path synchronization, intersection work instrumentation, 100% byte-for-byte output determinism across thread counts, and comprehensive performance benchmark analysis.

The next project on the repository roadmap is **Project 02 — Voxel Engine**.

---

## Features Implemented in Milestone 5
- **Axis-Aligned Bounding Box (AABB)**: Andrew Kensler slab ray-AABB intersection algorithm supporting minimum coordinate padding (`delta = 0.0001`) and arbitrary ray directions.
- **Bounding Volume Hierarchy (BVH)**: Binary acceleration hierarchy partitioning scene primitives along longest-axis centroids, reducing average ray-primitive intersection complexity from linear $O(N)$ to expected $O(\log N)$ for well-separated geometry.
- **Accelerated vs Naive Traversal Selection**: CLI option `--accel naive|bvh` (defaulting to `bvh`), preserving naive linear traversal as an empirical reference oracle for correctness regression testing.
- **Intersection Work Instrumentation**: `RenderStats` tracks `sphere_intersection_tests` and `aabb_tests` alongside `primary_samples`, `shadow_rays`, `secondary_rays`, and `total_rays`.
- **Multithreaded Parallel Rendering**: Disjoint row-partitioned thread pool (`--threads N`, where `0` selects hardware concurrency) using thread-local statistics and post-render aggregation without shared locks on the hot path.
- **100% Output Determinism Across Thread Counts**: Thanks to order-independent `SplitMix64` per-sample PRNG seeds (`make_sample_seed`), renders produced using 1, 2, 4, 8, or 16 threads are byte-for-byte identical (SHA256 verified).
- **Unit Test Suite**: 18 CTest targets including focused AABB interval tests, sphere bounding box tests, BVH construction, hierarchy bounds, and naive-vs-BVH equivalence tests.

---

## Architecture & Data Flow

```text
Image Buffer (Width x Height)
      │
      ├── Work Partitioning ──> Row Ranges [row_start, row_end) assigned to N Worker Threads
      │
Worker Threads (t = 0..N-1)
      │
      ├── Pixel (i, j), Sample s [s = 0..S-1]
      │     │
      │     ▼
      │   sample_seed = SplitMix64(global_seed, i, j, s)
      │   RNG sample_rng(sample_seed)
      │     │
      │     ▼
      │   Camera Ray Generation:
      │         ├── (aperture == 0.0) ==> Pinhole Ray from lookfrom
      │         └── (aperture > 0.0)  ==> Thin-Lens Ray from offset on lens disk
      │     │
      │     ▼
      │   Accelerated Traversal (BVH / Naive Selection):
      │         ├── BVH Mode   : Ray ──> Root AABB ──> Node AABB Test ──> Leaf Primitive Test ──> Closest Hit
      │         └── Naive Mode : Ray ──> Linear Primitive Loop (Reference Oracle)
      │     │
      │     ▼
      │   Recursive ray_color(...) Pipeline -> Thread-Local RenderStats[t]
      │         ├── (depth >= max_depth) ==> Return Color(0, 0, 0)
      │         ├── Hit: Specular / Diffuse Bounces (Lambertian Hybrid Direct + Indirect)
      │         └── Miss: Sky Gradient Background
      │
      ▼
Join Threads ──> Merge Thread-Local RenderStats[t] into Total Stats ──> Gamma 2.0 (sqrt) ──> Write PPM
```

---

## Build & Test Instructions

### Prerequisites
- Modern C++20 compiler (MSVC 2022/2026, GCC 11+, or Clang 13+)
- CMake (v3.20+) and Ninja

### Build Debug & Release Configurations
```bash
# Debug Build
cmake --preset default
cmake --build --preset default

# Release Build
cmake --preset release
cmake --build --preset release
```

### Run Unit Tests
```bash
ctest --preset default --output-on-failure
ctest --preset release --output-on-failure
```

### Run CLI Executable
```bash
# BVH Multithreaded Render (Demo Scene, 400x225, 16 SPP, 8 Threads)
./build/release/projects/01-raytracer/raytracer --scene demo --accel bvh --threads 8 --samples 16 --output image_bvh.ppm

# Naive Linear Traversal Benchmark Reference (1 Thread)
./build/release/projects/01-raytracer/raytracer --scene demo --accel naive --threads 1 --samples 16 --output image_naive.ppm

# Procedural Random Scene (50+ spheres, BVH Acceleration, Auto Threads)
./build/release/projects/01-raytracer/raytracer --scene random --accel bvh --threads 0 --samples 16 --seed 42 --output image_procedural.ppm
```

---

## Benchmark Summary (Milestone 5)

### 1. Naive vs BVH Acceleration (Procedural Scene, 50+ Spheres, Release, 1 Thread)
| Mode | Primary Samples | Total Rays | Sphere Tests | AABB Tests | Render Time (ms) | Speedup |
|---|---|---|---|---|---|---|
| **Naive Linear** | 1,440,000 | 5,061,874 | 956,161,773 | 0 | **5,346.64 ms** | 1.00x (ref) |
| **BVH Hierarchy**| 1,440,000 | 5,061,874 | 20,773,480 | 158,973,943 | **2,164.21 ms** | **2.47x** |

> **Key Observation**: BVH reduced primitive sphere intersection tests from **956 Million to 20.7 Million** (a **46x reduction**).

### 2. Thread Scaling & Determinism Verification (Procedural & Demo Scene, BVH, Release)
| Threads | Render Time (ms) | Total Rays / sec | Speedup vs BVH 1T | Speedup vs Naive 1T | Output SHA256 Hash Digest |
|---|---|---|---|---|---|
| **1 Thread**  | 2,164.21 ms | 2,338,896 | 1.00x | 2.47x | `2864CD572C42EB16DB4FF9FF6D369DF77D8F41CBB0A30E68A16593F36FB73257` |
| **2 Threads** | 1,494.92 ms | 3,386,043 | 1.45x | 3.58x | `2864CD572C42EB16DB4FF9FF6D369DF77D8F41CBB0A30E68A16593F36FB73257` |
| **4 Threads** |   711.98 ms | 7,109,612 | 3.04x | 7.51x | `2864CD572C42EB16DB4FF9FF6D369DF77D8F41CBB0A30E68A16593F36FB73257` |
| **8 Threads** |   539.84 ms | 9,376,645 | 4.01x | 9.90x | `2864CD572C42EB16DB4FF9FF6D369DF77D8F41CBB0A30E68A16593F36FB73257` |
| **16 Threads**|   345.29 ms | 14,659,907 | **6.27x** | **15.48x** | `2864CD572C42EB16DB4FF9FF6D369DF77D8F41CBB0A30E68A16593F36FB73257` |

---

## Milestone Roadmap
- [x] **Milestone 1**: Core math, Ray, Sphere intersection, Camera, PPM output, CLI, CTest harness.
- [x] **Milestone 2**: Surface normals, Lambertian materials, direct lighting, shadow rays, anti-aliasing, gamma correction, deterministic PRNG.
- [x] **Milestone 3**: Metal reflection, dielectric refraction, Schlick reflectance, TIR, recursive ray bounces, max depth limit.
- [x] **Milestone 4**: Order-independent SplitMix64 PRNG, camera orientation, thin-lens DOF, aperture, focus distance, SceneBuilder, procedural scenes, ray stats.
- [x] **Milestone 5**: AABB, BVH acceleration, multithreaded rendering loop, deterministic thread scaling, Debug vs Release matrix analysis.

**Project 01 is complete.** Next project: **Project 02 — Voxel Engine**.
