# Project 03: Performance Lab

A hardware-aware micro-benchmarking laboratory dedicated to CPU cache locality optimization, SIMD vectorization (AVX2 / SSE4.2 / NEON intrinsics), atomic lock-free queue concurrency, and memory allocation efficiency.

---

### Current Status: Milestone 2 Complete

Milestone 2 implements the Cache Locality & Data Layout benchmark suite (`bench_cache_locality`) and unit test suite (`test_cache_locality`). It empirically examines AoS vs SoA vs AoSoA data layout performance, constant-access strided traversal (strides 1 to 256), and working-set size scaling (4 KiB to 64 MiB).

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
- **Sequential Traversal Benchmark (`aos_sequential`, `soa_sequential`, `aosoa_sequential_tile16`)**: Evaluates 100,000 records (3.2 MB logical footprint). In Release, SoA and AoSoA demonstrate superior memory throughput over AoS due to contiguous field streaming.
- **Constant-Access Stride Benchmark (`stride_1` to `stride_256`)**: Evaluates strided accesses over a contiguous float array. Fixed access count of 100,000 elements ensures identical operation counts across all strides. Stride 16 (64 bytes) marks the boundary of single cache-line stride spacing.
- **Working-Set Scaling Benchmark (`workingset_4KiB` to `workingset_64MiB`)**: Evaluates working-set memory scaling across 15 orders of magnitude, tracking throughput and bandwidth as working sets expand from L1/L2 cache fits into DRAM.
- **Automated CTest Suite (`test_cache_locality`)**: 5 unit test cases verifying AoS/SoA/AoSoA equivalence, deterministic initialization, stride array bounds safety, working-set calculation, and edge cases.

---

## Planned Milestone Roadmap

- [x] **Phase 0**: Discovery, Technical Roadmap & Architecture Review
- [x] **Milestone 1**: Micro-Benchmarking Harness & High-Precision Timing Infrastructure
- [x] **Milestone 2**: Cache Locality & Data Layout Benchmarks (AoS vs SoA vs Stride Access)
- [ ] **Milestone 3**: SIMD Vectorization & Intrinsic Acceleration (AVX2 / SSE4.2 Vector & Geometry Kernels)
- [ ] **Milestone 4**: Thread Contention & Lock-Free vs Mutex Synchronization Queues
- [ ] **Milestone 5**: Memory Allocator Churn & Arena / Bump Allocator Benchmarks

---

## Scope Boundaries & Explicit Non-Goals

- **No SIMD / Queue / Allocator Experiments Yet**: Milestone 2 focuses strictly on scalar cache locality and data layouts. Explicit SIMD intrinsics are intentionally deferred to Milestone 3.
- **No Hardware Counter Claims**: Benchmark reports record empirical execution times, throughput, and bandwidth without claiming exact hardware L1/L2 cache miss counts.
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

# Execute cache locality benchmark suite
./build/release/projects/03-performance-lab/bench_cache_locality.exe
```
