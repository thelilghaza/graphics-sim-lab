# Project 03: Performance Lab

A hardware-aware micro-benchmarking laboratory dedicated to CPU cache locality optimization, SIMD vectorization (AVX2 / SSE4.2 / NEON intrinsics), atomic lock-free queue concurrency, and memory allocation efficiency.

---

### Current Status: Milestone 1 Complete

Milestone 1 implements the core micro-benchmarking harness, high-precision timing framework, statistical aggregation model, compiler optimization barriers, plain-text terminal report generator, CSV exporter, and CLI interface. Baseline harness validation workloads (`bench_harness`) and unit tests (`test_benchmark_harness`) are verified. No cache, SIMD, lock-free queue, or allocator experiments have been implemented yet.

---

## Implemented Milestone 1 Behavior

- **Micro-Benchmarking Harness (`BenchRunner`)**: Manages benchmark execution lifecycle: one-time setup -> 5 warmup iterations -> 100 timed iterations -> statistics computation -> plain-text terminal table -> CSV export -> teardown.
- **High-Precision Monotonic Timing**: Uses C++20 `std::chrono::steady_clock` to record iteration durations in microseconds without calendar clock distortion.
- **Statistical Aggregation (`compute_statistics`)**: Computes mean, median, sample standard deviation ($N-1$), minimum, maximum, operations per second ($\text{Ops/sec}$), and memory bandwidth ($\text{MB/s}$, $1 \text{ MB} = 10^6 \text{ bytes}$).
- **Compiler Optimization Barrier (`do_not_optimize`, `clobber_memory`)**: Portable inline assembly / volatile memory barriers (`compiler_barrier.hpp`) preventing compiler dead-code elimination around benchmark outputs on MSVC, GCC, and Clang.
- **Environment Metadata (`EnvironmentMetadata`)**: Auto-detects compiler ID and version, build configuration (Debug/Release), architecture (x86_64/ARM64), operating system (Windows/Linux/macOS), and hardware concurrency.
- **Structured CSV Exporter**: Writes benchmark metadata and statistical metrics to CSV format. Requires explicit `--overwrite` flag to prevent accidental report file destruction.
- **Command-Line Interface (`BenchCLI`)**: Supports `--help`, `--iterations N`, `--warmups N`, `--csv PATH`, `--overwrite`, and `--quiet`.
- **Diagnostic Warning**: Automatically detects suspiciously short execution times ($< 0.02 \ \mu\text{s}$ per iteration) and outputs diagnostic warnings.
- **Validation Workloads (`bench_harness`)**: Verifies integer accumulation, floating-point accumulation, contiguous array traversal, and deterministic scalar transformations.
- **Automated CTest Suite (`test_benchmark_harness`)**: 9 unit test cases verifying config defaults, overrides, iteration counts, statistical formulas (mean, median, sample stddev), single-iteration bounds, CSV schema, and compiler barriers.

---

## Planned Milestone Roadmap

- [x] **Phase 0**: Discovery, Technical Roadmap & Architecture Review
- [x] **Milestone 1**: Micro-Benchmarking Harness & High-Precision Timing Infrastructure
- [ ] **Milestone 2**: Cache Locality & Data Layout Benchmarks (AoS vs SoA vs Stride Access)
- [ ] **Milestone 3**: SIMD Vectorization & Intrinsic Acceleration (AVX2 / SSE4.2 Vector & Geometry Kernels)
- [ ] **Milestone 4**: Thread Contention & Lock-Free vs Mutex Synchronization Queues
- [ ] **Milestone 5**: Memory Allocator Churn & Arena / Bump Allocator Benchmarks

---

## Scope Boundaries & Explicit Non-Goals

- **No Cache / SIMD / Queue / Allocator Experiments Yet**: Milestone 1 focuses strictly on harness infrastructure. Benchmark experiments begin in Milestone 2.
- **No Shared Libraries in `libs/`**: All code remains isolated in `projects/03-performance-lab/`.
- **No Third-Party Benchmarking Dependencies**: Harness relies on standard C++20 standard library facilities.
- **No GPU / Compute Shaders**: GPU performance profiling is deferred to Project 05 (Crater Simulator).

---

## Build & Test Instructions

```bash
# Configure build
cmake --preset default

# Build all Project 03 targets
cmake --build --preset default

# Run test suite via CTest
ctest --preset default --output-on-failure

# Execute baseline harness validation benchmark
./build/release/projects/03-performance-lab/bench_harness.exe
```
