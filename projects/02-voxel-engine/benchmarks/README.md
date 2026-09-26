# Project 02 Benchmarks — Voxel Engine

Benchmark measurements and timing records for voxel storage, chunk access, and surface extraction are recorded here.

---

## Milestone 4 Benchmark Suite — Naive Exposed-Face Culling Baseline

The naive exposed-face mesher (`mesh_chunk`) evaluates all 32,768 voxels in a $32^3$ chunk and queries 6 neighbor directions per solid voxel via `WorldAccessor`. This benchmark establishes the unoptimized CPU baseline for comparison against future Greedy Meshing (Milestone 6).

### Benchmark Environment & Parameters
- **Compiler**: MSVC 19.51 (C++20, `-O2` / `/O2` Release Build)
- **Target**: `bench_naive_mesher` executable
- **Timing Source**: `std::chrono::high_resolution_clock` (high-precision microsecond timing over 200 iterations after 20 warmup runs per test).

### Measured Baseline Results (`milestone4_benchmark.txt`)

| Workload Description | Chunk Coord | Solid Voxels | Emitted Faces | Vertices | Indices | Avg Time (us) | Throughput (Meshes/sec) |
|---|---|---|---|---|---|---|---|
| **Empty Chunk** | `(0,0,0)` | 0 | 0 | 0 | 0 | 153.30 us | 6,523 / sec |
| **Single Solid Voxel** | `(0,0,0)` | 1 | 6 | 24 | 36 | 261.33 us | 3,826 / sec |
| **Full Solid $32^3$ Chunk** | `(0,0,0)` | 32,768 | 6,144 | 24,576 | 36,864 | 1,877.78 us | 532 / sec |
| **Deterministic Plane World** ($y=15$) | `(0,0,0)` | 16,384 | 4,096 | 16,384 | 24,576 | 1,118.43 us | 894 / sec |
| **Deterministic Sphere World** ($r=12$) | `(0,0,0)` | 7,153 | 2,646 | 10,584 | 15,876 | 653.71 us | 1,529 / sec |

### Architectural Observations
1. **Full Solid Chunk Face Count**: A solid $32^3$ chunk ($32,768$ voxels) surrounded by air emits exactly $6 \times 32 \times 32 = 6,144$ faces ($24,576$ vertices, $36,864$ indices), confirming all $6 \times 30 \times 32^2$ internal shared faces are culled correctly.
2. **Naive Quad Overhead**: Because adjacent faces are not merged, a full solid chunk requires 6,144 quads (24,576 vertices). Milestone 6 Greedy Meshing will merge coplanar quads into larger rectangles to dramatically reduce quad and vertex counts.
