# Project 02 Benchmarks — Voxel Engine

Benchmark measurements and timing records for voxel storage, chunk access, and surface extraction are recorded here.

---

## Milestone 4 Benchmark Suite — Naive Exposed-Face Culling Baseline

The naive exposed-face mesher (`mesh_chunk`) evaluates all 32,768 voxels in a $32^3$ chunk and queries 6 neighbor directions per solid voxel via `WorldAccessor`. This benchmark establishes the unoptimized CPU baseline for comparison against Greedy Meshing.

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

---

## Milestone 6 Benchmark Suite — Naive vs Greedy Mesher Performance Analysis

Milestone 6 evaluates the Greedy Meshing algorithm (`greedy_mesh_chunk`) against the Milestone 4 naive baseline across identical workloads.

### Benchmark Environment & Parameters
- **Build Configuration**: Release (`/O2` optimization, NDEBUG)
- **Compiler**: MSVC 19.51 (Visual Studio 2026 Developer Command Prompt)
- **Timing Source**: `std::chrono::high_resolution_clock`
- **Methodology**: 20 warmup runs followed by 200 repeated timed iterations per case.
- **Target Executable**: `bench_greedy_mesher`

### Measured Comparison Results (`milestone6_benchmark.txt`)

#### 1. Geometry Reduction & Generation Timing

| Workload Case | Naive Faces | Greedy Quads | Face Red. % | Naive Time (us) | Greedy Time (us) | Speedup Ratio | Time Diff (us) |
|---|---|---|---|---|---|---|---|
| **Empty Chunk** | 0 | 0 | 0.00% | 158.65 us | 957.46 us | 0.17x | +798.82 us |
| **Single Solid Voxel** | 6 | 6 | 0.00% | 257.32 us | 1,521.24 us | 0.17x | +1,263.92 us |
| **Full Solid $32^3$ Chunk** | 6,144 | 6 | **99.90%** | 1,798.72 us | 2,934.77 us | 0.61x | +1,136.05 us |
| **Deterministic Plane** ($y=15$) | 4,096 | 6 | **99.85%** | 1,043.95 us | 2,254.60 us | 0.46x | +1,210.65 us |
| **Deterministic Sphere** ($r=12$) | 2,646 | 1,050 | **60.32%** | 627.72 us | 1,923.97 us | 0.33x | +1,296.25 us |
| **Cross-Chunk Sphere** (chunk 0) | 375 | 168 | **55.20%** | 355.64 us | 2,136.93 us | 0.17x | +1,781.29 us |
| **Mixed Voxel Stripes** (types 1&2) | 4,096 | 34 | **99.17%** | 1,080.28 us | 2,310.54 us | 0.47x | +1,230.25 us |

#### 2. Detailed Vertex & Index Reductions

| Workload Case | Naive Vertices | Greedy Vertices | Vertex Red. % | Naive Indices | Greedy Indices | Index Red. % |
|---|---|---|---|---|---|---|
| **Empty Chunk** | 0 | 0 | 0.00% | 0 | 0 | 0.00% |
| **Single Solid Voxel** | 24 | 24 | 0.00% | 36 | 36 | 0.00% |
| **Full Solid $32^3$ Chunk** | 24,576 | 24 | **99.90%** | 36,864 | 36 | **99.90%** |
| **Deterministic Plane** ($y=15$) | 16,384 | 24 | **99.85%** | 24,576 | 36 | **99.85%** |
| **Deterministic Sphere** ($r=12$) | 10,584 | 4,200 | **60.32%** | 15,876 | 6,300 | **60.32%** |
| **Cross-Chunk Sphere** (chunk 0) | 1,500 | 672 | **55.20%** | 2,250 | 1,008 | **55.20%** |
| **Mixed Voxel Stripes** (types 1&2) | 16,384 | 136 | **99.17%** | 24,576 | 204 | **99.17%** |

### Benchmark Interpretation

1. **Geometric Reduction Outcome**:
   - Planar surfaces experience near-total collapse: a full solid $32^3$ chunk shrinks from 6,144 faces (24,576 vertices) to exactly 6 quads (24 vertices), achieving a 99.90% reduction.
   - Planar terrain surfaces ($y \le 15$) collapse by 99.85%.
   - Mixed-material planar surfaces with alternating 4-voxel stripes collapse by 99.17% (from 4,096 faces down to 34 quads), verifying material compatibility prevents invalid merges across material boundaries while still maximizing coplanar runs within each stripe.
   - Curved geometry (spheres) achieves 55% to 60% reduction, merging planar steps on quantized sphere bands into rectangular quads.

2. **Computational Cost Outcome**:
   - Greedy meshing generation time is longer than naive meshing across all workloads (~1.5 ms to 2.9 ms per chunk vs ~0.3 ms to 1.8 ms).
   - This occurs because naive meshing performs only local neighbor tests and direct emissions, while greedy meshing constructs six sets of 32 2D slice masks and executes 2D maximal rectangle search loops.
   - Trade-off analysis: Greedy meshing trades an additional ~1 ms of CPU meshing time per chunk for a 60% to 99.9% reduction in GPU vertex count, vertex shader invocations, and VBO memory bandwidth. In rendering-heavy workloads, this trade-off dramatically improves overall frame rate and GPU throughput.
