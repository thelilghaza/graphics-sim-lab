# Project 03: Performance Lab

A hardware-aware micro-benchmarking laboratory dedicated to CPU cache locality optimization, SIMD vectorization (AVX2 / SSE4.2 / NEON intrinsics), atomic lock-free queue concurrency, and memory allocation efficiency.

---

### Current Status: Milestone 3 Complete

Milestone 3 implements the SIMD Vectorization & Intrinsic Acceleration benchmark suite (`bench_simd_vectorization`) and unit test suite (`test_simd`). It empirically compares portable scalar reference, compiler auto-vectorized C++, explicit x86 SSE2/SSE4.2 SIMD, explicit AVX2 SIMD, and portable ARM NEON structures across vector arithmetic, vector transform, and packet geometry workloads.

---

## Implemented Milestone 1 Behavior

- **Micro-Benchmarking Harness (`BenchRunner`)**: Manages benchmark execution lifecycle: one-time setup -> 5 warmup iterations -> 100 timed iterations -> statistics computation -> plain-text terminal table -> CSV export -> teardown.
- **High-Precision Monotonic Timing**: Uses C++20 `std::chrono::steady_clock` to record iteration durations in microseconds without calendar clock distortion.
- **Statistical Aggregation (`compute_statistics`)**: Computes mean, median, sample standard deviation ($N-1$), minimum, maximum, operations per second ($\text{Ops/sec}$), and memory bandwidth ($\text{MB/s}$, $1 \text{ MB} = 10^6 \text{ bytes}$).
- **Compiler Optimization Barrier (`do_not_optimize`, `clobber_memory`)**: Portable inline assembly / volatile memory barriers (`compiler_barrier.hpp`) preventing compiler dead-code elimination around benchmark outputs on MSVC, GCC, and Clang.
- **Environment Metadata (`EnvironmentMetadata`)**: Auto-detects compiler ID and version, build configuration (Debug/Release), architecture (x86_64/ARM64), operating system (Windows/Linux/macOS), and hardware concurrency.
- **Structured CSV Exporter**: Writes benchmark metadata and statistical metrics to CSV format. Requires explicit `--overwrite` flag to prevent accidental report file destruction.
- **Command-Line Interface (`BenchCLI`)**: Supports `--help`, `--iterations N`, `--warmups N`, `--csv PATH`, `--overwrite`, and `--quiet`.
- **Validation Workloads (`bench_harness`)**: Verifies integer accumulation, floating-point accumulation, contiguous array traversal, and deterministic scalar transformations.

---

## Implemented Milestone 2 Behavior

- **Data Layout Models (`data_layouts.hpp`)**:
  - **AoS (Array of Structures)**: Contiguous `std::vector<RecordAoS>` (32 bytes per struct: 3D position, 3D velocity, mass, ID).
  - **SoA (Structure of Arrays)**: `RecordSoA` storing fields in 8 separate contiguous `std::vector` arrays.
  - **AoSoA (Array of Structures of Arrays)**: `RecordAoSoA<16>` tiling records into contiguous 16-element sub-arrays per tile.
- **Deterministic Checksum & Equivalence Verification**: All three layouts evaluate identical mathematical work ($(\text{pos} \cdot \text{vel}) \times \text{mass}$) and verify identical floating-point checksums prior to benchmarking.
- **Sequential Traversal Benchmark (`aos_sequential`, `soa_sequential`, `aosoa_sequential_tile16`)**: Evaluates 100,000 records (3.2 MB logical footprint). Working-set and throughput observations track data layout performance.
- **Constant-Access Stride Benchmark (`stride_1` to `stride_256`)**: Evaluates strided accesses over a contiguous float array. Fixed access count of 100,000 elements ensures identical operation counts across all strides. Stride 16 (64 bytes) marks the boundary of single cache-line stride spacing.
- **Working-Set Scaling Benchmark (`workingset_4KiB` to `workingset_64MiB`)**: Evaluates memory traversal throughput across dataset sizes spanning 15 orders of magnitude (4 KiB to 64 MiB), tracking observed throughput and bandwidth as memory footprint increases.
- **Automated CTest Suite (`test_cache_locality`)**: 5 unit test cases verifying AoS/SoA/AoSoA equivalence, deterministic initialization, stride array bounds safety, working-set calculation, and edge cases.

---

## Implemented Milestone 3 Behavior

- **SIMD Capability Layer (`simd_caps.hpp`)**:
  - Auto-detects runtime CPU features (AVX2, SSE4.2, SSE2) and compile-time architecture target (x86_64, ARM64).
  - Portable NEON headers and fallback paths compiled behind architecture guards (`#if defined(__ARM_NEON)`).
  - Safely gates intrinsic execution so hardware without AVX2 will not execute AVX2 instructions.
- **Kernel A: Vector Fused Arithmetic (`simd_kernel_dot.hpp`)**:
  - Computes $\text{sum} += a[i] \cdot b[i] + c[i]$ over 16K, 1M, and 16M float arrays.
  - Compares Scalar Ref, Compiler Opt, SSE2/SSE4.2 (`_mm_loadu_ps`, `_mm_mul_ps`, `_mm_add_ps`), AVX2 (`_mm256_loadu_ps`, `_mm256_fmadd_ps` / `_mm256_mul_ps` + `_mm256_add_ps`), and NEON (`vld1q_f32`, `vmlaq_f32`).
- **Kernel B: AXPY Vector Transform (`simd_kernel_axpy.hpp`)**:
  - Computes $y[i] = a \cdot x[i] + y[i]$ over 16K, 1M, and 16M float arrays.
  - Pre-allocated setup buffers passed outside timed region; reports logical bytes processed ($2 \times N \times \text{sizeof(float)}$).
- **Kernel C: Batch Ray-AABB Intersection (`simd_kernel_ray_box.hpp`)**:
  - 4-wide and 8-wide slab-style Ray-AABB intersection test processing 50,000 ray packets (200,000 rays).
  - SSE variant processes 4 rays simultaneously using `_mm_min_ps`, `_mm_max_ps`, `_mm_movemask_ps`.
  - AVX2 variant processes 8 rays simultaneously using `_mm256_min_ps`, `_mm256_max_ps`, `_mm256_movemask_ps`.
- **Auto-Vectorization Control & Generated Code Evidence**:
  - Scalar reference paths explicitly suppress MSVC auto-vectorization using `#pragma loop(no_vector)`.
  - Normal optimized C++ loops (`/O2`) enable compiler vectorizer.
  - Assembly inspection verified MSVC 19.51 generates auto-vectorized SIMD instructions for simple streaming loops while explicit intrinsics guarantee optimal SIMD register usage and FMA vectorization.
- **Tail Handling & Alignment**:
  - All SIMD implementations feature safe scalar remainder loops for array sizes not divisible by SIMD width (e.g. lengths 3, 7, 15, 17, 31, 33).
  - Unaligned loads (`_mm_loadu_ps`, `_mm256_loadu_ps`) prevent alignment fault UB while operating cleanly on standard contiguous memory allocations.
- **Numerical Tolerance Policy**:
  - Floating-point reduction sums accumulate in different order across SIMD vector lanes.
  - Unit tests enforce strict $10^{-4}$ tolerance for standard array sizes. Large multi-million float reductions use relative tolerance ($0.05$) accounting for single-precision IEEE 754 precision accumulation breakdown.
- **Automated CTest Suite (`test_simd`)**: 4 unit test cases verifying ISA capability detection, Kernel A length & tail handling, Kernel B length & tail handling, and Kernel C ray-box hit/miss equivalence.

---

## Planned Milestone Roadmap

- [x] **Phase 0**: Discovery, Technical Roadmap & Architecture Review
- [x] **Milestone 1**: Micro-Benchmarking Harness & High-Precision Timing Infrastructure
- [x] **Milestone 2**: Cache Locality & Data Layout Benchmarks (AoS vs SoA vs Stride Access)
- [x] **Milestone 3**: SIMD Vectorization & Intrinsic Acceleration (AVX2 / SSE4.2 Vector & Geometry Kernels)
- [ ] **Milestone 4**: Thread Contention & Lock-Free vs Mutex Synchronization Queues
- [ ] **Milestone 5**: Memory Allocator Churn & Arena / Bump Allocator Benchmarks

---

## Scope Boundaries & Explicit Non-Goals

- **No Queue / Allocator Work Yet**: Milestone 3 focuses strictly on SIMD intrinsics and vectorization. Lock-free queues and memory allocators are deferred to Milestones 4 and 5.
- **No Hardware Counter Claims**: Performance conclusions distinguish empirical timing and throughput metrics from hardware microarchitectural interpretations.
- **No Shared Libraries in `libs/`**: All code remains strictly isolated in `projects/03-performance-lab/`.

---

## Build & Test Instructions

```bash
# Configure build
cmake --preset default

# Build all Project 03 targets
cmake --build --preset default

# Run test suite via CTest
ctest --preset default --output-on-failure

# Execute benchmark executables
./build/release/projects/03-performance-lab/bench_cache_locality.exe
./build/release/projects/03-performance-lab/bench_simd_vectorization.exe
```
